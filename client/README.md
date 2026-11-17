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
The LED at the top of the panel shows the connection to the sensor server (`connection.py`): green when connected,
amber while connecting, red when unreachable (retried every 1 to 2 s). It follows the gRPC channel state, so it does
not notice a server that stops answering without closing the connection (pulled cable). Tests:
`uv run python -m unittest`.

With no stream flag the client subscribes to IMU, LiDAR and camera. `--adc` polls the ADC and logs its readings;
they are not shown in the viewer.
