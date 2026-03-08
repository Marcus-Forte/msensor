# Python client

Use the following command to generate proto and gRPC code if the `.proto` files are updated:

```bash
uv run python -m grpc_tools.protoc -I../../proto --python_out=proto_gen --pyi_out=proto_gen --grpc_python_out=proto_gen \
  ../../proto/lidar.proto ../../proto/imu.proto ../../proto/camera.proto ../../proto/adc.proto ../../proto/recording.proto
```

Then fix the generated imports to be package-relative:

```bash
sed -i 's/^import \(.*_pb2\) as/from . import \1 as/' proto_gen/*_pb2_grpc.py proto_gen/recording_pb2.py
```

```bash
uv run python -m grpc_tools.protoc -Iproto_gen=. --python_out=. --pyi_out=. --grpc_python_out=. robot.proto
```

Use `uv run client.py` to call the client listener.

The client uses `viser` to render sensor data in `http://localhost:8080`.