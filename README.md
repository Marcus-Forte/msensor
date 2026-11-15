# The msensor library

A modular sensor driver framework that exposes hardware sensors over gRPC. Each sensor type has its own service, proto definition, and abstract interface — drivers are injected into services so the gRPC layer is decoupled from any specific hardware.

See `docs/arch.puml` for a PlantUML diagram.

## Usage

### Interfaces

Inherit the abstract interfaces in `include/msensor/interface/` and implement your own driver.  
See `src/<sensor_type>/` for examples.

### Server

`sensor_publisher` (config-driven, see below) and `sim_publisher` (all four simulated drivers) build their drivers with `msensor::makeSensors()` and serve them through `msensor::SensorsServer`, which is constructed from a `msensor::Sensors` struct and optional `ServerOptions` and exposes the four gRPC services on port **50051**. Both stop cleanly on SIGINT/SIGTERM. `SensorsServer::start()` throws `std::runtime_error` if the address cannot be bound; `wait()` blocks until `stop()`.

`sensor_publisher` now loads its sensor selection from a JSON file instead of individual CLI flags. By default it reads `/usr/local/etc/publisher_config.json` (the installed template is `config/publisher_config.json`), or you can pass a different file path as the only argument. `--record <file>` (before or after the config path) starts a recording to that file whatever the config says; see [Recorder](#recorder).

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
| `recording.enable` | `false` | Record the LiDAR and IMU to a `.pbscan` on the sensor host (see [Recorder](#recorder)) |
| `recording.file` | `""` | Output file; relative paths are relative to the working directory; empty means `scan_<timestamp>.pbscan` |
| `recording.scan_queue`, `recording.imu_queue` | `256`, `8192` | Scans / IMU samples held while the disk is behind; positive; not limited by the gRPC caps |

The older `use_sensors.*`, `i2c_bus`, `lidar_device` and `camera_pipeline` keys are still accepted. `config/sim_config.json` enables the four simulators and disables the hardware, for local runs without devices: `sensor_publisher config/sim_config.json`.

### Sensor bus and several clients

Between the drivers and their consumers sits a small in-process publish/subscribe layer, `msensor::SensorBus` (`msensor/bus/`, CMake target `msensor::bus`). A driver keeps its single `setXCallback` consumer; that consumer is the sensor's `Topic<T>`, which wraps each sample in a `shared_ptr<const T>` once and pushes it into the bounded FIFO of every `Subscription<T>`. The driver's thread does nothing else: no protobuf conversion, no I/O, no waiting on a subscriber. A topic is attached to its driver only while it has subscribers, so a driver with no consumer behaves as it did before (the Mid360 discards packets, the simulators and the camera skip their work).

Every gRPC stream (LiDAR, IMU, camera) is a subscriber, so several clients can read the same stream at once, each with its own queue depth and its own drops. `ServerOptions::max_clients_per_stream` (default 8) limits the clients of each stream; one more gets `RESOURCE_EXHAUSTED`. `SensorsServer(Sensors, ServerOptions)` and the legacy positional constructor build a private bus; `SensorsServer(ServerOptions, std::shared_ptr<SensorBus>)` serves a bus that other in-process consumers share:

```cpp
auto bus = std::make_shared<msensor::SensorBus>(msensor::Sensors{.imu = imu, .lidar = lidar});
auto scans = bus->lidar()->subscribe(256);      // own FIFO, read with scans->tryPop() / pop(stop_token)
msensor::SensorsServer server({.listen_address = "0.0.0.0:50051"}, bus);
```

A bus takes over the driver's callback while it has subscribers, so do not also call `setXCallback` on a driver that is on a bus.

### C++ Remote Client

`SensorsRemoteClient` (in `grpc/`) connects to a running server and implements `ILidar` + `IImu`, so downstream code can consume remote sensors through the same interfaces as local ones.

Each stream request carries `max_queued_samples`, the number of samples the server may hold while the client is behind. When the queue is full the oldest sample is dropped, so the client gets the newest ones in order; a jump in `header.sequence_number` reveals a loss, and `SensorsRemoteClient` counts those jumps itself: `lostScans()` and `lostImuSamples()` return the totals, and a gap is logged as a warning at most once per second per stream. A forward jump of `n > 1` counts `n - 1` lost samples (modulo 2^32, so the wrap from `0xFFFFFFFF` to `0` is no loss); a jump back is a restart (playback rewind, server restart), not a loss; the first number of a stream means nothing, and a reconnect starts a new stream. Counting only observes: every received sample still reaches the callback, in order, on the reader thread. The server logs a drop at most once per second per stream and client, and the total when the stream ends. `0` selects the server default: IMU 200 (cap 4000), LiDAR 1 (cap 64), camera 1 (cap 8); larger values are lowered to the cap. Depth 1 is "latest wins", right for scans and frames; SLAM-style IMU consumers want a deep queue. The client takes the depths as an optional second constructor argument, `SensorsRemoteClient(address, {.scan_queue_size = 1, .imu_queue_size = 1000})`; both default to 0 (server default). `remote_recorder` asks for 16 scans and 1000 IMU samples.

### Python Client

A Python client is provided in `client/`. It connects to the gRPC services and renders data with [viser](https://viser.studio).

```bash
cd client
uv run client.py --server <host>:50051    # all streams
uv run client.py --lidar                  # lidar only
uv run client.py --imu --camera           # pick streams
```

See `client/README.md` for proto regeneration instructions.

## Logging

Library code reports through the small `msensor::log` facade (`msensor/log/log.hh`): `debug`, `info`, `warn` and `error` take a `std::format` string. The default threshold is `info`, set at startup with the `MSENSOR_LOG_LEVEL` environment variable (`debug`, `info`, `warn`, `error` or `off`) or at run time with `msensor::log::setLevel`. `msensor::log::setSink` redirects the messages (the default sink writes `[level] message` lines to stdout, or stderr for warn and error).

## Docker

A `deployment/Dockerfile` is provided to offer small footprint images that allows one to run the sensor driver applications from inside a container. 

Make sure the hardware is correctly mapped to the container (e.g `--device /dev/i2c-1`, `--device /dev/ttyUSB*`,  `--network=host`, ...), or use `docker run -v /dev:/dev -v /run/udev:/run/udev --privileged` to allow complete device access.

A `docker-compose.yml` is provided in the `deployment/` folder next to the Dockerfile illustrating how to compose and deploy this app.

## Recorder

Two ways to store sensory data to a `pbscan` file.

**On the sensor host** (no network in the path): `sensor_publisher` can record the LiDAR and IMU while it serves them, from the `recording` object of its config (`"recording": {"enable": true, "file": "run.pbscan"}`) or with `--record`:

```
sensor_publisher /config/publisher_config.json --record run.pbscan
```

The recorder (`msensor::BusRecorder`, `msensor/recorder/bus_recorder.hh`, CMake target `msensor::bus_recorder`) is one more subscriber of the sensor bus, with its own deep queues (default 256 scans and 8192 IMU samples, set by `recording.scan_queue` / `recording.imu_queue`) and its own writer thread, so a slow disk never slows a driver and gRPC clients are unaffected. Each wake-up writes the queued IMU samples before the queued scans. If a queue overflows, the oldest sample is dropped and counted, and a warning is logged right away (at most once per second per stream). On SIGINT/SIGTERM the publisher stops the server, drains what is queued, closes the file and prints the entries written and dropped per stream. It works with only a LiDAR or only an IMU. A recording of the sim config: `sensor_publisher config/sim_config.json --record /tmp/sim.pbscan`.

**Over the network**: use `remote_recorder <server> [output.pbscan]`. On exit it prints the saved entries and, next to them, the samples lost to sequence number gaps (see the remote client above).
Via docker use `docker run --rm -v ./:/app mdnf1992/msensor remote_recorder <server>`

Each entry of a `.pbscan` file is a native-endian `uint64` byte count followed by a serialized `sensors::RecordingEntry` (`proto/recording.proto`). `scan_checker -f <file>` counts the entries; `playback_publisher -f <file>` serves them again over gRPC.

## Benchmark

`bench_msensor` (built with the tests, not run by CTest) times the conversion, playback, recording and gRPC paths on synthetic scans and on the recordings given as arguments, e.g. `build/gcc/test/bench_msensor --passes 3 rooms.pbscan`. Every row prints a checksum of its output, so two builds can be compared for equivalence as well as speed.
