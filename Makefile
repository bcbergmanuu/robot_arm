.PHONY: host test ctest pytest firmware sim teleop gen-config

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

firmware:
	scripts/idf.sh build

sim:
	uv run mjpython -m robotarm sim

teleop:
	uv run robotarm teleop --bus tcp://127.0.0.1:29536
