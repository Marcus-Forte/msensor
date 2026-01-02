# Python client

Use the following command to generate proto and grpc code if they need to be updated. 

```bash
uv run python -m grpc_tools.protoc -Iproto_gen=../../proto --python_out=. --pyi_out=. --grpc_python_out=. ../../proto/sensors.proto
```

```bash
uv run python -m grpc_tools.protoc -Iproto_gen=. --python_out=. --pyi_out=. --grpc_python_out=. robot.proto
```

Use `uv run client.py` to call the client listener.

The client uses `viser` to render sensor data in `http://localhost:8080`.