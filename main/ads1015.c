#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "driver/gpio.h"   
#include "ads1015.h"
#include "motor_pid.h"

static const char *TAG = "sigmaDeltaADC";

i2c_master_bus_config_t i2c_mst_config = {
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .i2c_port = I2C_PORT,
    .scl_io_num = I2C_MASTER_SCL_IO,
    .sda_io_num = I2C_MASTER_SDA_IO,
    .glitch_ignore_cnt = 7,        
};

i2c_device_config_t dev_cfg = {
    .dev_addr_length = I2C_ADDR_BIT_LEN_7,
    .device_address = ADS1X15_ADDRESS, //addr connected gnd
    .scl_speed_hz = 400000,
};  

i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t dev_handle;

int init_i2c() {
    int ret = ESP_OK;
    ret |= (i2c_new_master_bus(&i2c_mst_config, &bus_handle));   
    ret |= (i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle));
    return ret;
};

/// @brief Read 16bit value from the specified register
/// @param reg Register address to read 
/// @param data_rd Read value 2 bytes long
/// @return ESP_OK if success, ESP_ERR_INVALID_ARG/ESP_ERR_TIMEOUT otherwise
int read_register(const uint8_t reg, uint8_t *data_rd) {
    int ret = ESP_OK;       
    ret |= i2c_master_transmit(dev_handle, &reg, 1, 500);
    ret |= i2c_master_receive(dev_handle, data_rd, 2, 500);
    return ret;
}

int write_register(uint8_t reg, uint16_t data_wr) {
    const uint8_t buffer[3] = {reg, data_wr >> 8, data_wr & 0xFF};

    return i2c_master_transmit(dev_handle, buffer, 3, 500);
}

int read_adc(uint16_t * result) {
  // Read the conversion results
  int ret = 0; 
  uint8_t buffer[2] = {0};
  ret |= read_register(ADS1X15_REG_POINTER_CONVERT, buffer);
  uint16_t adc_value = ((buffer[0] << 8) | buffer[1]);  

  uint16_t internal = adc_value>>4;
  if(internal > 2047) {
    ESP_LOGE(TAG, "incorrect adc reading %d - %d, returned 1 instead",buffer[0], buffer[1] );
    *result = 1;
  }
  *result = internal;
  return ret;  
}


int config_adc() {
  int ret = 0;
  uint16_t config =
      ADS1X15_REG_CONFIG_CQUE_1CONV |                                    
      ADS1X15_REG_CONFIG_CLAT_NONLAT |      
      ADS1X15_REG_CONFIG_CPOL_ACTVLOW |
      ADS1X15_REG_CONFIG_CMODE_TRAD |  
      ADS1X15_REG_CONFIG_PGA_0_256V |
      RATE_ADS1015_3300SPS |
      ADS1X15_REG_CONFIG_MUX_SINGLE_0 |
      ADS1X15_REG_CONFIG_MODE_CONTIN |
      ADS1X15_REG_CONFIG_OS_SINGLE;


      //"The conversion-ready function of the ALERT/RDY pin is enabled by setting the Hi_thresh register MSB to 1b and the Lo_thresh register MSB to 0b. "
  //set low and high threshold registers:
  ret |= write_register(ADS1X15_REG_POINTER_LOWTHRESH, 0);
  ret |= write_register(ADS1X15_REG_POINTER_HITHRESH, 0x8000);

  // Write config register to the ADC
  ret |= write_register(ADS1X15_REG_POINTER_CONFIG, config);
  
  return ret;
}

static TaskHandle_t xTaskToNotify = NULL;
const UBaseType_t xArrayIndex = 0;

static void adcpin_isr(void *arg) {     
    
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;          
    configASSERT( xTaskToNotify != NULL );
    vTaskNotifyGiveIndexedFromISR( xTaskToNotify,
                        xArrayIndex,
                        &xHigherPriorityTaskWoken );   
    
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }        
}

int set_RDY_interrupt() {
    configASSERT( xTaskToNotify == NULL );
    xTaskToNotify = xTaskGetCurrentTaskHandle();

    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE, 
        .mode = GPIO_MODE_INPUT,        
        .pin_bit_mask = (1ULL << GPIO_NUM_3),
    };
    int ret = 0;
    ret |= gpio_config(&io_conf);
    ret |= gpio_install_isr_service(0);
    ret |= gpio_isr_handler_add(GPIO_NUM_3, adcpin_isr, NULL);
    return ret;
}

void run_ads1015adc(void *args) {
    int ret = 0, printcounter=0, buffer_pos = 0;
    ret = init_i2c();  
    ret = set_RDY_interrupt();
    ret = config_adc();  

    if(ret != ESP_OK) {
        ESP_LOGE(TAG,"Error initializing, %d", ret);
        vTaskDelay(portMAX_DELAY);
    }
    
    
    float volt = 0, amp = 0, ampPower;
    uint16_t adc_value;

    while (true)
    {
        uint32_t ulNotificationValue = ulTaskNotifyTakeIndexed( xArrayIndex,
                                                   false,
                                                   portMAX_DELAY );        

        ret |= read_adc(&adc_value);

        executePid(adc_value);
        
        
        if(ret != ESP_OK) {
            ESP_LOGE(TAG, "Error reading %d", ret);
        }        
        if(ulNotificationValue < 1) {
            ESP_LOGE(TAG, "conversion task timeout %d", ulNotificationValue);
        }
        
        // if(printcounter++ > 500 && false) {                  
        //     volt = adcbuffer[buffer_pos].readvalue * 125e-7; //.256mv pp
        //     amp = volt * 220; //amp = volt * 220r
        //     ampPower = amp / 2.2; // tb9051
        //     ESP_LOGI(TAG, "adcval = %u, volt: %f, ampPower %f \n", adcbuffer[buffer_pos].readvalue, volt, ampPower);                        
        //     printcounter = 0;
        // }
        buffer_pos++;
        if(buffer_pos > 4000) {
            ESP_LOGI(TAG, "finished");
            vTaskDelay(portMAX_DELAY);
        }
    }    

}