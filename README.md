# The msensor

This library serves as an experiment for writing generic sensor drivers based on a simple interface.

By inheriting these common interfaces, the driver benefit from:

* Exposing your driver as a unified gRPC interface to be consumed remotely.
* Enabling your driver to serialize data to a file to be read back at a later time.

## Usage

### As an interface

You can inherit basic sensor interfaces located at `include/<sensor_type>/*.hh` and write your own corresponding driver. See `src/<sensor_type>/*.cc` for examples.

### Remote Driver

* Your driver can be programmed as a gRPC service. See `src/*_publisher` applications as example.
* A client (at another machine) application can instantiate a `grpc/sensors_remote_client.hh` class and subscribe to the IP
of a driver server to get sensor data.

### Client

A Python client is provided with the repository to facilitate testing the remote driver.
See `test/client` folder.

## Docker

A `DockerfileRuntime` is provided to offer small footprint images that allows one to run the sensor driver applications from inside a container. 

Make sure the hardware is correctly mapped to the container (e.g `--device /dev/i2c-1`, `--device /dev/ttyUSB*`,  `--network=host`, ...), or use `docker run -v /dev:/dev -v /run/udev:/run/udev --privileged` to allow complete device access.