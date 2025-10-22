#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "host/ble_hs.h"
#include "host/ble_uuid.h"
#include "services/gap/ble_svc_gap.h"
#include "services/gatt/ble_svc_gatt.h"

#include "storage.h"



// static const char *manuf_name = "Robot Arm";
// static const char *model_num = "hinch";

uint16_t pid_ble_values[setpoints_num * 3]; //pid controls * (p+i+d) * 16bit * setting^-1

static const ble_uuid128_t svc_uuid =
    BLE_UUID128_INIT(0x00, 0x00, 0x00, 0x10, 0x11, 0x11, 0x12, 0x12,
                     	0x22, 0x22, 0x21, 0x12, 0x33, 0x30, 0x34, 0x33);

static const ble_uuid128_t settings1_uuid =
    BLE_UUID128_INIT(0x00, 0x00, 0x00, 0x10, 0x11, 0x11, 0x11, 0x12,
                     	0x22, 0x22, 0x22, 0x22, 0x33, 0x30, 0x33, 0x33);

static int gatt_svr_access_setting(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg);

// static int gatt_svr_chr_access_device_info(uint16_t conn_handle, uint16_t attr_handle,
//                                 struct ble_gatt_access_ctxt *ctxt, void *arg);

static const struct ble_gatt_svc_def gatt_svr_svcs[] = {
    {
        /* Service: settings all */
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &svc_uuid.u,
        .characteristics = (struct ble_gatt_chr_def[])
        { //{
                // /* Characteristic: Heart-rate measurement */
                // .uuid = BLE_UUID16_DECLARE(GATT_HRS_MEASUREMENT_UUID),
                // .access_cb = gatt_svr_chr_access_heart_rate,
                // .val_handle = &hrs_hrm_handle,
                // .flags = BLE_GATT_CHR_F_NOTIFY,
            //}, 
            {
                /*setting bb*/
                .uuid = &settings1_uuid.u,
                .access_cb = gatt_svr_access_setting,
                .val_handle = pid_ble_values,
                .flags = BLE_GATT_CHR_F_READ | BLE_GATT_CHR_F_WRITE, //| BLE_GATT_CHR_F_NOTIFY,
                
            }, {
                0, /* No more characteristics in this service */
            },
        }
    },

    // {
    //     /* Service: Device Information */
    //     .type = BLE_GATT_SVC_TYPE_PRIMARY,
    //     .uuid = BLE_UUID16_DECLARE(GATT_DEVICE_INFO_UUID),
    //     .characteristics = (struct ble_gatt_chr_def[])
    //     { {
    //             /* Characteristic: * Manufacturer name */
    //             .uuid = BLE_UUID16_DECLARE(GATT_MANUFACTURER_NAME_UUID),
    //             .access_cb = gatt_svr_chr_access_device_info,
    //             .flags = BLE_GATT_CHR_F_READ,
    //         }, {
    //             /* Characteristic: Model number string */
    //             .uuid = BLE_UUID16_DECLARE(GATT_MODEL_NUMBER_UUID),
    //             .access_cb = gatt_svr_chr_access_device_info,
    //             .flags = BLE_GATT_CHR_F_READ,
    //         }, {
    //             0, /* No more characteristics in this service */
    //         },
    //     }
    // },

    {
        0, /* No more services */
    },
};

static int store_received() {    
    return store_nvs((uint64_t*)pid_stored_values, nvs_pid_keys, setpoints_num);    
}

static int apply_received() { return 0;};

static int gatt_svr_access_setting(uint16_t conn_handle, uint16_t attr_handle,
                               struct ble_gatt_access_ctxt *ctxt, void *arg)
{
    int err = 0;
    switch (ctxt->op) {
        case BLE_GATT_ACCESS_OP_READ_CHR:
            return os_mbuf_append(ctxt->om, &pid_ble_values, sizeof(pid_ble_values));
            
        case BLE_GATT_ACCESS_OP_WRITE_CHR:
            // if (ctxt->om->om_len > pid_value_num) {
            //     return BLE_ATT_ERR_INVALID_ATTR_VALUE_LEN;
            // }
            //uint8_t buf[4];
            err = os_mbuf_copydata(ctxt->om, 0, sizeof(pid_ble_values), pid_ble_values); //amount of bytes;
            
            //  MODLOG_DFLT(INFO, "ble received: ");
            //     for (int i = 0; i < pid_value_num; i++) {
            //     MODLOG_DFLT(INFO, "%02x ", pid_values[i]);
            // }
            for(int controller = 0; controller < setpoints_num; controller ++) {
                pid_stored_values[controller].kp = pid_ble_values[controller * setpoints_num];
                pid_stored_values[controller].ki = pid_ble_values[controller * setpoints_num + 1];
                pid_stored_values[controller].kd = pid_ble_values[controller * setpoints_num + 2];
            }

            store_received();
            apply_received();
            return err;
        }    
    
    return BLE_ATT_ERR_UNLIKELY;
}

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg)
{
    char buf[BLE_UUID_STR_LEN];

    switch (ctxt->op) {
    case BLE_GATT_REGISTER_OP_SVC:
        MODLOG_DFLT(DEBUG, "registered service %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                    ctxt->svc.handle);
        break;

    case BLE_GATT_REGISTER_OP_CHR:
        MODLOG_DFLT(DEBUG, "registering characteristic %s with "
                    "def_handle=%d val_handle=%d\n",
                    ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                    ctxt->chr.def_handle,
                    ctxt->chr.val_handle);
        break;

    case BLE_GATT_REGISTER_OP_DSC:
        MODLOG_DFLT(DEBUG, "registering descriptor %s with handle=%d\n",
                    ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                    ctxt->dsc.handle);
        break;

    default:
        assert(0);
        break;
    }
}

int gatt_svr_init(void)
{
    int rc;

    ble_svc_gap_init();
    ble_svc_gatt_init();

    rc = ble_gatts_count_cfg(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    rc = ble_gatts_add_svcs(gatt_svr_svcs);
    if (rc != 0) {
        return rc;
    }

    return 0;
}
