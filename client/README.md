# Python client

Use the following command to generate proto and gRPC code if the `.proto` files are updated:

```bash
uv run python -m grpc_tools.protoc -I../proto -Iproto_gen=../proto --python_out=proto_gen --grpc_python_out=proto_gen --pyi_out=proto_gen ../proto/*.proto
```

```bash
uv run python -m grpc_tools.protoc -Iproto_gen=. --python_out=. --pyi_out=. --grpc_python_out=. robot.proto
```

Note the presence and content of `__init__.py` in `proto_gen`

Use `uv run client.py` to call the client listener.

The client uses `viser` to render sensor data in `http://localhost:8080`. 