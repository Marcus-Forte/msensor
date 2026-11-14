# The msensor library

A modular sensor driver framework that exposes hardware sensors over gRPC. Each sensor type has its own service, proto definition, and abstract interface — drivers are injected into services so the gRPC layer is decoupled from any specific hardware.

See `docs/arch.puml` for a PlantUML diagram.

## Usage

### Interfaces

Inherit the abstract interfaces in `include/interface/` and implement your own driver.  
See `src/<sensor_type>/` for examples.

### Server

Publisher executables (e.g. `sensor_publisher`, `sim_publisher`) instantiate concrete drivers, inject them into `SensorsServer`, and expose all four gRPC services on port **50051**. `SensorsServer::start()` throws `std::runtime_error` if the address cannot be bound; `wait()` blocks until `stop()`.

`sensor_publisher` now loads its sensor selection from a JSON file instead of individual CLI flags. By default it reads `/usr/local/etc/publisher_config.json` (the installed template is `config/publisher_config.json`), or you can pass a different file path as the only argument.

The config uses per-sensor objects; every key is optional. `sensor_publisher` builds the sensors with `msensor::makeSensors()` (`msensor/factory/sensor_factory.hh`), which throws on fatal problems (e.g. an enabled Mid360 without an existing config file, or a missing I2C bus) and makes the process print the message and exit with status 1.

| Key | Default | Meaning |
| --- | --- | --- |
| `rplidar.enable`, `rplidar.device` | `true`, `/dev/ttyUSB0` | RPLidar; a missing device only warns |
| `mid360.enable`, `mid360.config` | `false`, none | Mid360 (also provides the IMU); a relative `config` is resolved against the config file's directory |
| `mid360.accumulate_scan_count` | `100` | Point-cloud packets combined into each scan |
| `mid360.mode` | `"normal"` | `"normal"` or `"power_save"` (power save sets no scan pattern) |
| `mid360.scan_pattern` | `"non_repetitive"` | `"non_repetitive"`, `"repetitive"` or `"low_frame_rate"` |
| `icm20948.enable` | `false` | Skipped when the Mid360 supplies the IMU |
| `icm20948.i2c_bus` | the `ads1115.i2c_bus` value | I2C bus index |
| `ads1115.enable`, `ads1115.i2c_bus` | `false`, `1` | ADS1115 on I2C |
| `ads1115.gain` | `6.144` | Full-scale volts: 6.144, 4.096, 2.048, 1.024, 0.512, 0.256 |
| `ads1115.data_rate` | `8` | Samples per second: 8, 16, 32, 64, 128, 250, 475, 860 |
| `ads1115.channel` | `0` | Single-ended input, 0..3 |
| `ads1115.external_gain` | `(10+5.1)/5.1` | Voltage divider factor, positive |
| `camera.enable`, `camera.pipeline` | `true`, libcamera pipeline | OpenCV GStreamer camera |
| `camera.encoding` | `"raw"` | `"raw"` or `"jpeg"` (pipeline already delivers JPEG; passed through) |
| `camera.jpeg_quality` | `85` | 0..100, used when raw frames are JPEG-encoded |
| `sim.lidar`, `sim.imu`, `sim.camera`, `sim.adc` | `false` | Use the simulated driver instead of the hardware one of the same kind |

The older `use_sensors.*`, `i2c_bus`, `lidar_device` and `camera_pipeline` keys are still accepted. `config/sim_config.json` enables the four simulators and disables the hardware, for local runs without devices: `sensor_publisher config/sim_config.json`.

### C++ Remote Client

`SensorsRemoteClient` (in `grpc/`) connects to a running server and implements `ILidar` + `IImu`, so downstream code can consume remote sensors through the same interfaces as local ones.

### Python Client

A Python client is provided in `client/`. It connects to the gRPC services and renders data with [viser](https://viser.studio).

```bash
cd client
uv run client.py --server <host>:50051    # all streams
uv run client.py --lidar                  # lidar only
uv run client.py --imu --camera           # pick streams
```

See `client/README.md` for proto regeneration instructions.

## Docker

A `deployment/Dockerfile` is provided to offer small footprint images that allows one to run the sensor driver applications from inside a container. 

Make sure the hardware is correctly mapped to the container (e.g `--device /dev/i2c-1`, `--device /dev/ttyUSB*`,  `--network=host`, ...), or use `docker run -v /dev:/dev -v /run/udev:/run/udev --privileged` to allow complete device access.

A `docker-compose.yml` is provided in `docker` folder illustrating how to compose and deploy this app.

## Recorder

Use `remote_recorder` to store sensory data to a `pbscan` file.
Via docker use `docker run --rm -v ./:/app mdnf1992/msensor remote_recorder <server>`

Each entry of a `.pbscan` file is a native-endian `uint64` byte count followed by a serialized `sensors::RecordingEntry` (`proto/recording.proto`). `scan_checker -f <file>` counts the entries; `playback_publisher -f <file>` serves them again over gRPC.

## Benchmark

`bench_msensor` (built with the tests, not run by CTest) times the conversion, playback, recording and gRPC paths on synthetic scans and on the recordings given as arguments, e.g. `build/gcc/test/bench_msensor --passes 3 rooms.pbscan`. Every row prints a checksum of its output, so two builds can be compared for equivalence as well as speed.
