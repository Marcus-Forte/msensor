# msensor

C++23 sensor driver framework exposed over gRPC (port 50051). Abstract interfaces in `include/msensor/interface/`; concrete drivers in `src/<type>/` (lidar, imu, camera, adc, plus sim_* variants); gRPC services in `grpc/` (`SensorsServer` wires drivers → services); protos in `proto/`. `docs/arch.puml` has the diagram.

## Build / test
- Toolchain lives in the `mdnf1992/cpp-dev` image (devcontainer); OpenCV ≥4.10, jsoncpp, and gRPC are expected there. Presets use `/opt/toolchain/gcc.cmake`.
- Configure/build: `cmake --preset gcc && cmake --build --preset build-gcc` (output in `build/gcc`, Ninja). CI instead does plain `mkdir build && cd build && cmake .. && make && ctest` in an arm64 container.
- Tests (GTest, `test/src/`): `ctest --test-dir build/gcc`, or one test: `build/gcc/test/test_recorder --gtest_filter=Suite.Name`. Tests are only added when msensor is the top-level project.
- `test/draft.cc` (`draft` target) is a scratch executable, not a test.
- Submodules in `third_party/` (rplidar_sdk, Livox-SDK2) must be initialized: `git submodule update --init`.
- Format: configure with `-DFORMAT_CODE=ON`; the `format` target runs `clang-format -i` on sources as part of `ALL` (it rewrites files on build).

## Gotchas
- Hardware drivers (ads1115, icm-20948, rplidar, mid360, opencv camera) need real devices; use `sim_publisher` and `sim_*` drivers for local runs.
- `sensor_publisher` takes a JSON config (default `/usr/local/etc/publisher_config.json`, or the single CLI arg); template is `config/publisher_config.json`. The other `*_publisher` apps are older per-sensor executables (see `todo`: to be merged).
- `playback_publisher` replays a `.pbscan` recording as a sensor source over gRPC: `playback_publisher -f <file> -s <speed> [-p <port>]` (`speed` 1.0 = real time, 0 = max; default port 50051). It always starts paused; Space toggles playback, `r` rewinds and pauses, Right Arrow doubles speed, Left Arrow halves it (0.125x minimum; from unpaced `0`, it selects 64x), and Ctrl-C exits. It stays running at EOF and can replay through the existing sensor streams. It is backed by `msensor::RecordingSensorDriver` (`src/recorder/`), a push-based LiDAR+IMU source.
- Python client (`client/`, uses `uv`): generated code in `client/proto_gen/` must be regenerated after `.proto` changes with the `grpc_tools.protoc` command in `client/README.md` (the second `robot.proto` command there is stale; no such file here).
- `.clangd` points at `build/` for `compile_commands.json`, but presets output to `build/gcc`.

## Deployment (`deployment/`)
- Runtime image: `DockerfileRuntime` (multi-stage, build in `mdnf1992/cpp-dev`, run on `debian:trixie-slim`; aarch64 lib paths hardcoded). Build from repo root context; arm64 via `--platform linux/arm64`.
- `docker/` was moved to `deployment/`, but `DockerfileRuntime` (`COPY /src/docker/entrypoint.sh`) and `docker-compose.yml` (`dockerfile: docker/DockerfileRuntime`) still reference `docker/`; fix paths if touching them.
- Compose runs `sensor_publisher /config/publisher_config.json` with host network, privileged, `/dev` mounted, plus a `ptpd` grandmaster container. Entrypoint with no args only prints usage.
- Toolchain is gcc-16 (C++26); trixie's libstdc++6 is too old (`GLIBCXX_3.4.35 not found`), so the runtime image installs `libstdc++6` from the sid apt source. Don't copy libstdc++ from the build stage.
