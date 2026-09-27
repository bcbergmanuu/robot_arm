.PHONY: host test ctest pytest firmware sim teleop gen-config check-config

host:
	cmake -S host -B build/host -DCMAKE_BUILD_TYPE=RelWithDebInfo >/dev/null
	cmake --build build/host -j

ctest: host
	ctest --test-dir build/host --output-on-failure

pytest: host
	uv run pytest -q pc/tests

test: ctest pytest

gen-config:
	uv run robotarm gen-config

# The C config table (compiled into the firmware and libsimaxis) must match config/arm.yaml.
check-config:
	@uv run robotarm gen-config --check >/dev/null || { \
		echo "error: components/axis_core/src/config_table.c is stale vs config/arm.yaml -- run 'make gen-config' (then rebuild; reflash the boards)" >&2; \
		exit 1; }

firmware: check-config host
	scripts/idf.sh build

sim: check-config host
	scripts/sim.sh

teleop: check-config host
	uv run robotarm teleop --bus tcp://127.0.0.1:29536
