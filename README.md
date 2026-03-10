# The msensor library

A modular sensor driver framework that exposes hardware sensors over gRPC. Each sensor type has its own service, proto definition, and abstract interface — drivers are injected into services so the gRPC layer is decoupled from any specific hardware.

See `docs/arch.puml` for a PlantUML diagram.

## Usage

### Interfaces

Inherit the abstract interfaces in `include/interface/` and implement your own driver.  
See `src/<sensor_type>/` for examples.

### Server

Publisher executables (e.g. `all_publisher`, `sim_publisher`) instantiate concrete drivers, inject them into `SensorsServer`, and expose all four gRPC services on port **50051**.

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

A `DockerfileRuntime` is provided to offer small footprint images that allows one to run the sensor driver applications from inside a container. 

Make sure the hardware is correctly mapped to the container (e.g `--device /dev/i2c-1`, `--device /dev/ttyUSB*`,  `--network=host`, ...), or use `docker run -v /dev:/dev -v /run/udev:/run/udev --privileged` to allow complete device access.

A `docker-compose.yml` is provided in `docker` folder illustrating how to compose and deploy this app.

To use a specific "profile" (i.e. a group of services), use like 

For the mid360 app alone
```bash
docker compose --profile mid360 up
```

For the "robot" profile (msensor all publisher + robot service)
```bash
docker compose --profile robot up
```