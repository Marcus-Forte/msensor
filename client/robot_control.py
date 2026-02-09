import queue
import threading
from dataclasses import dataclass

import grpc
import viser

# Generated from https://github.com/Marcus-Forte/grpc-robot/blob/main/proto/control.proto
from proto_gen import robot_pb2, robot_pb2_grpc


@dataclass
class RobotControlHandle:
    thread: threading.Thread
    _channel: grpc.Channel

    def close(self) -> None:
        """Close the underlying gRPC channel once the thread is finished."""
        self._channel.close()


def _stream_robot_control(
    stub: robot_pb2_grpc.RobotControlStub,
    key_queue: queue.Queue[str],
    stop_event: threading.Event,
) -> None:
    def key_request_iter():
        while True:
            if stop_event.is_set() and key_queue.empty():
                break
            try:
                key = key_queue.get(timeout=0.1)
            except queue.Empty:
                continue
            if not key:
                continue
            yield robot_pb2.KeyInput(key_value=key)

    try:
        stub.SendKeyboardStream(key_request_iter())
    except grpc.RpcError as exc:
        print(f"Robot control stream error: {exc.code().name} - {exc.details()}")


def start_robot_control(
    server: viser.ViserServer,
    robot_server_addr: str,
    stop_event: threading.Event,
) -> RobotControlHandle:
    """Attach a text input to the GUI and start the robot control stream."""
    if not robot_server_addr:
        raise ValueError("robot_server_addr must be provided")

    robot_input_queue: queue.Queue[str] = queue.Queue()
    control_robot_input = server.gui.add_text(
        "Control Robot (WSADX)",
        initial_value="",
        hint="Use WASD keys to control the robot.",
    )

    def handle_robot_input(event: viser.GuiEvent):
        text_value = getattr(event.target, "value", "") or ""
        if not text_value:
            return
        key = text_value[-1].lower()
        robot_input_queue.put(key)
        control_robot_input.value = ""
        print(f"Robot control input: {key}")

    control_robot_input.on_update(handle_robot_input)

    channel = grpc.insecure_channel(robot_server_addr)
    robot_control_stub = robot_pb2_grpc.RobotControlStub(channel)

    robot_thread = threading.Thread(
        target=_stream_robot_control,
        args=(robot_control_stub, robot_input_queue, stop_event),
        name="robot-control-thread",
    )
    robot_thread.start()

    return RobotControlHandle(thread=robot_thread, _channel=channel)
