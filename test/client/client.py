import argparse
import threading
import time
from typing import Optional, Sequence

import cv2
import grpc
import numpy as np
import viser
import viser.uplot

from proto_gen import sensors_pb2, sensors_pb2_grpc
from robot_control import RobotControlHandle, start_robot_control


DEFAULT_SERVER_ADDR = "localhost:50051"
DEFAULT_ADC_CHANNEL = 0
DEFAULT_ADC_PERIOD_SEC = 0.5


def to_numpy(points: Sequence[sensors_pb2.Point3]) -> np.ndarray:
    arr = np.zeros((len(points), 3), dtype=np.float32)
    for i, point in enumerate(points):
        arr[i, 0] = point.x
        arr[i, 1] = point.y
        arr[i, 2] = point.z
    return arr


def setup_imu_plots(server: viser.ViserServer):
    time_step = 1.0 / 60.0
    num_timesteps = 1000
    num_signals = 3

    x_data = time_step * np.arange(num_timesteps, dtype=np.float64)
    y_acc = np.zeros((num_signals, num_timesteps))
    y_gyr = np.zeros((num_signals, num_timesteps))
    data_acc = (x_data, *y_acc)
    data_gyro = (x_data, *y_gyr)

    uplot_handles: list[viser.GuiUplotHandle] = []
    uplot_handles.append(
        server.gui.add_uplot(
            data=data_acc,
            series=(
                viser.uplot.Series(label="time"),
                *[
                    viser.uplot.Series(
                        label=f"y{i}",
                        stroke=["red", "green", "blue"][i % 3],
                        width=2,
                    )
                    for i in range(num_signals)
                ],
            ),
            title="Accelerometer",
            scales={
                "x": viser.uplot.Scale(time=False, auto=True),
                "y": viser.uplot.Scale(auto=True),
            },
            legend=viser.uplot.Legend(show=True),
            aspect=2.0,
        )
    )

    uplot_handles.append(
        server.gui.add_uplot(
            data=data_gyro,
            series=(
                viser.uplot.Series(label="time"),
                *[
                    viser.uplot.Series(
                        label=f"y{i}",
                        stroke=["red", "green", "blue"][i % 3],
                        width=2,
                    )
                    for i in range(num_signals)
                ],
            ),
            title="Gyro",
            scales={
                "x": viser.uplot.Scale(time=False, auto=True),
                "y": viser.uplot.Scale(auto=True),
            },
            legend=viser.uplot.Legend(show=True),
            aspect=2.0,
        )
    )

    return {
        "x_data": x_data,
        "y_acc": y_acc,
        "y_gyr": y_gyr,
        "num_timesteps": num_timesteps,
        "uplot_handles": uplot_handles,
    }


def stream_imu(stub: sensors_pb2_grpc.SensorServiceStub, server: viser.ViserServer, stop_event: threading.Event):
    context = setup_imu_plots(server)
    request = sensors_pb2.SensorStreamRequest()
    sample = 0

    while not stop_event.is_set():
        try:
            for imu in stub.getImuData(request):
                if stop_event.is_set():
                    break
                sample += 1
                context["y_acc"][:, sample % context["num_timesteps"]] = (imu.ax, imu.ay, imu.az)
                context["uplot_handles"][0].data = (context["x_data"], *context["y_acc"])

                context["y_gyr"][:, sample % context["num_timesteps"]] = (imu.gx, imu.gy, imu.gz)
                context["uplot_handles"][1].data = (context["x_data"], *context["y_gyr"])
        except grpc.RpcError as exc:
            print(f"IMU stream error: {exc.code().name} - {exc.details()}")

    time.sleep(1.0)


def stream_lidar(stub: sensors_pb2_grpc.SensorServiceStub, server: viser.ViserServer, stop_event: threading.Event):
    request = sensors_pb2.SensorStreamRequest()
    last_timestamp = 0

    cloud = server.scene.add_point_cloud(
        name="/rplidar",
        points=np.empty((0, 3), dtype=np.float32),
        colors=(255, 0, 0),
        point_size=0.05,
        point_shape="rounded",
    )

    while not stop_event.is_set():
        try:
            for scan in stub.getLidarScan(request):
                print(f"Got points: {len(scan.points)} at {scan.timestamp}")
                delta_t = scan.timestamp - last_timestamp
                print(f" DeltaT: {delta_t} ms")
                last_timestamp = scan.timestamp
                cloud.points = to_numpy(scan.points)

        except grpc.RpcError as exc:
            print(f"LiDAR stream error: {exc.code().name} - {exc.details()}")
    time.sleep(1.0)


def stream_camera(stub: sensors_pb2_grpc.SensorServiceStub, server: viser.ViserServer, stop_event: threading.Event):
    request = sensors_pb2.CameraStreamRequest()
    frame_count = 0
    gui_image_handle: Optional[viser.GuiImageHandle] = None

    while not stop_event.is_set():
        try:
            for camera_data in stub.getCameraFrame(request):

                frame_count += 1
                print(
                    f"Got camera frame {frame_count}: {camera_data.width}x{camera_data.height} encoding={camera_data.encoding}"
                )

                # Decode the image data
                if camera_data.encoding == sensors_pb2.BGR8:
                    img_array = np.frombuffer(camera_data.image_data, dtype=np.uint8)
                    img = img_array.reshape((camera_data.height, camera_data.width, 3))
                    # Convert BGR to RGB for viser display
                    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                elif camera_data.encoding == sensors_pb2.GRAY8:
                    img_array = np.frombuffer(camera_data.image_data, dtype=np.uint8)
                    img = img_array.reshape((camera_data.height, camera_data.width))
                    # Convert grayscale to RGB for viser display
                    img_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
                elif camera_data.encoding == sensors_pb2.RGB8:
                    img_array = np.frombuffer(camera_data.image_data, dtype=np.uint8)
                    img_rgb = img_array.reshape((camera_data.height, camera_data.width, 3))
                elif camera_data.encoding == sensors_pb2.MJPEG:
                    img_array = np.frombuffer(camera_data.image_data, dtype=np.uint8)
                    img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
                    if img is None:
                        print("Failed to decode MJPEG frame")
                        continue
                    # Convert decoded BGR frame to RGB for viser display
                    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

                else:
                    print(f"Unsupported encoding: {camera_data.encoding}")
                    continue

                # Flip image to correct camera orientation (vertical + horizontal).
                img_rgb = np.flip(img_rgb, axis=(0, 1)).copy()

                # Display the current frame in the GUI rather than attaching it to the 3D scene.
                if gui_image_handle is None:
                    gui_image_handle = server.gui.add_image(
                        img_rgb,
                        label="Camera",
                        format="jpeg",
                    )
                else:
                    gui_image_handle.image = img_rgb

        except grpc.RpcError as exc:
            print(f"Camera stream error: {exc.code().name} - {exc.details()}")

    time.sleep(1.0)


def stream_adc(
    stub: sensors_pb2_grpc.SensorServiceStub,
    stop_event: threading.Event,
    channel: int,
    battery_level_handle: viser.GuiNumberHandle,
):
    request = sensors_pb2.AdcDataRequest(channel=channel)
    period_sec = DEFAULT_ADC_PERIOD_SEC
    print(f"Polling ADC channel {channel} every {period_sec}s. Press Ctrl+C to stop.")
    while not stop_event.is_set():
        try:
            resp = stub.getAdcData(request)
            print(f"ADC{channel}: {resp.sample:.4f} V @ {resp.timestamp}")
            battery_level_handle.value = resp.sample
        except grpc.RpcError as exc:
            print(f"ADC error: {exc.code().name} - {exc.details()}")
        stop_event.wait(period_sec)


def parse_args():
    parser = argparse.ArgumentParser(description="Subscribe to multiple sensor streams over gRPC.")
    parser.add_argument("--server", default=DEFAULT_SERVER_ADDR, help="gRPC target host:port")
    parser.add_argument(
        "--robot-server",
        help="gRPC target host:port for robot control (optional)",
    )
    parser.add_argument("--imu", action="store_true", help="Subscribe to the IMU stream")
    parser.add_argument("--lidar", action="store_true", help="Subscribe to the LiDAR stream")
    parser.add_argument("--camera", action="store_true", help="Subscribe to the camera stream")
    parser.add_argument("--adc", action="store_true", help="Poll the ADC")
    parser.add_argument("--adc-channel", type=int, default=DEFAULT_ADC_CHANNEL, help="ADC channel to poll")
    return parser.parse_args()


def main():
    args = parse_args()

    if not any([args.imu, args.lidar, args.camera, args.adc]):
        args.imu = args.lidar = args.camera = args.adc = True

    server = viser.ViserServer()
    server.scene.world_axes.visible = True
    server.scene.world_axes.axes_length = 1
    # TODO black BG
    bg_img = np.ndarray((3, 3, 3))
    server.scene.set_background_image(image=bg_img)

    stop_event = threading.Event()
    threads: list[threading.Thread] = []

    battery_level_handle = server.gui.add_number("Battery Voltage (V)", initial_value=0.0, step=0.1, disabled=True)

    robot_handle: Optional[RobotControlHandle] = None
    if args.robot_server:
        robot_handle = start_robot_control(server, args.robot_server, stop_event)
        threads.append(robot_handle.thread)

    with grpc.insecure_channel(args.server) as channel:
        stub = sensors_pb2_grpc.SensorServiceStub(channel)

        if args.imu:
            if server is None:
                raise RuntimeError("IMU visualization requires viser to be available.")
            t = threading.Thread(target=stream_imu, args=(stub, server, stop_event), name="imu-thread")
            t.start()
            threads.append(t)

        if args.lidar:
            if server is None:
                raise RuntimeError("LiDAR visualization requires viser to be available.")
            t = threading.Thread(target=stream_lidar, args=(stub, server, stop_event), name="lidar-thread")
            t.start()
            threads.append(t)

        if args.camera:
            if server is None:
                raise RuntimeError("Camera visualization requires viser to be available.")
            t = threading.Thread(target=stream_camera, args=(stub, server, stop_event), name="camera-thread")
            t.start()
            threads.append(t)

        if args.adc:
            t = threading.Thread(
                target=stream_adc,
                args=(stub, stop_event, args.adc_channel, battery_level_handle),
                name="adc-thread",
            )
            t.start()
            threads.append(t)

        try:
            while any(t.is_alive() for t in threads):
                for t in threads:
                    t.join(timeout=0.5)
        except KeyboardInterrupt:
            print("\nStopping sensor subscriptions...")
            stop_event.set()

    for t in threads:
        t.join()

    if robot_handle:
        robot_handle.close()


if __name__ == "__main__":
    main()
