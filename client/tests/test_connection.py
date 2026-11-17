"""Connection LED: state machine, HTML, and a real gRPC server going up, down and up again."""

import threading
import time
import unittest
from concurrent import futures

import grpc

from connection import ConnectionMonitor, ConnectionState, led_html, next_state

C = grpc.ChannelConnectivity
S = ConnectionState


def start_server(port: int = 0) -> tuple[grpc.Server, int]:
    server = grpc.server(futures.ThreadPoolExecutor(max_workers=1))
    port = server.add_insecure_port(f"127.0.0.1:{port}")
    server.start()
    return server, port


class StateMachineTest(unittest.TestCase):
    def test_transitions(self) -> None:
        self.assertIs(next_state(S.CONNECTING, C.READY), S.CONNECTED)
        self.assertIs(next_state(S.CONNECTED, C.IDLE), S.CONNECTING)  # server dropped: reconnecting
        self.assertIs(next_state(S.CONNECTING, C.TRANSIENT_FAILURE), S.DISCONNECTED)
        self.assertIs(next_state(S.DISCONNECTED, C.CONNECTING), S.DISCONNECTED)  # retries do not flicker
        self.assertIs(next_state(S.DISCONNECTED, C.IDLE), S.DISCONNECTED)
        self.assertIs(next_state(S.DISCONNECTED, C.READY), S.CONNECTED)
        self.assertIs(next_state(S.CONNECTED, C.SHUTDOWN), S.DISCONNECTED)

    def test_html_shows_state_and_escapes_the_address(self) -> None:
        text = led_html(S.CONNECTED, "<host>:1")
        self.assertIn(S.CONNECTED.color, text)
        self.assertIn("connected", text)
        self.assertIn("&lt;host&gt;:1", text)
        self.assertNotIn("<host>", text)


class MonitorTest(unittest.TestCase):
    def setUp(self) -> None:
        self.stop = threading.Event()
        self.states: list[ConnectionState] = []
        self.changed = threading.Condition()

    def tearDown(self) -> None:
        self.stop.set()

    def on_change(self, state: ConnectionState) -> None:
        with self.changed:
            self.states.append(state)
            self.changed.notify_all()

    def wait_for(self, state: ConnectionState, timeout: float = 6.0) -> float:
        start = time.monotonic()
        with self.changed:
            ok = self.changed.wait_for(lambda: self.states and self.states[-1] is state, timeout)
        self.assertTrue(ok, f"no {state} within {timeout} s; states {self.states}")
        return time.monotonic() - start

    def test_server_up_down_up(self) -> None:
        server, port = start_server()
        monitor = ConnectionMonitor(f"127.0.0.1:{port}", self.on_change)
        try:
            monitor.start(self.stop.is_set, self.stop.wait)
            self.assertIs(self.states[0], S.CONNECTING)
            self.wait_for(S.CONNECTED)
            server.stop(grace=None).wait()
            self.wait_for(S.DISCONNECTED)
            server, _ = start_server(port)
            self.assertLess(self.wait_for(S.CONNECTED), 4.0)  # short reconnect backoff
        finally:
            monitor.close()
            server.stop(grace=None)

    def test_no_server_is_unreachable(self) -> None:
        server, port = start_server()
        server.stop(grace=None).wait()  # a port nothing listens on
        monitor = ConnectionMonitor(f"127.0.0.1:{port}", self.on_change)
        try:
            monitor.start(self.stop.is_set, self.stop.wait)
            self.wait_for(S.DISCONNECTED)
        finally:
            monitor.close()


if __name__ == "__main__":
    unittest.main()
