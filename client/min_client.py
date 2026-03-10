from colormap import int2rgb

import cv2
import grpc
import queue
import time
import numpy as np
import viser
import viser.uplot
from typing import Optional, Sequence

from proto_gen import (
    camera_pb2,
    camera_pb2_grpc,
    imu_pb2,
    imu_pb2_grpc,
    lidar_pb2,
    lidar_pb2_grpc,
)


# SERVER_ADDR = "127.0.0.1:50051" # dev container
SERVER_ADDR = "192.168.178.129:50051" # Robot


def _to_viser_pointcloud_colors(points: Sequence[lidar_pb2.Point3]) -> np.ndarray:
    colors = np.empty((len(points), 3), dtype=np.uint8)
    for i, point in enumerate(points):
        rgb = int2rgb(point.intensity)
        colors[i, 0] = rgb[0] * 255
        colors[i, 1] = rgb[1] * 255
        colors[i, 2] = rgb[2] * 255
    return colors


def _to_viser_pointcloud(points: Sequence[lidar_pb2.Point3]) -> np.ndarray:
    arr = np.empty((len(points), 3), dtype=np.float32)
    for i, point in enumerate(points):
        arr[i, 0] = point.x
        arr[i, 1] = point.y
        arr[i, 2] = point.z
    return arr


def _decode_camera_frame(frame: camera_pb2.CameraStreamReply) -> Optional[np.ndarray]:
    if frame.encoding == camera_pb2.BGR8:
        img_array = np.frombuffer(frame.image_data, dtype=np.uint8)
        img = img_array.reshape((frame.height, frame.width, 3))
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    elif frame.encoding == camera_pb2.GRAY8:
        img_array = np.frombuffer(frame.image_data, dtype=np.uint8)
        img = img_array.reshape((frame.height, frame.width))
        img_rgb = cv2.cvtColor(img, cv2.COLOR_GRAY2RGB)
    elif frame.encoding == camera_pb2.RGB8:
        img_array = np.frombuffer(frame.image_data, dtype=np.uint8)
        img_rgb = img_array.reshape((frame.height, frame.width, 3))
    elif frame.encoding == camera_pb2.MJPEG:
        img_array = np.frombuffer(frame.image_data, dtype=np.uint8)
        img = cv2.imdecode(img_array, cv2.IMREAD_COLOR)
        if img is None:
            return None
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    else:
        return None

    return np.flip(img_rgb, axis=(0, 1)).copy()


def _setup_imu_plots(server: viser.ViserServer):
    history_size = 300
    time_step = 1.0 / 60.0
    x_data = time_step * np.arange(history_size, dtype=np.float64)
    y_acc = np.zeros((3, history_size), dtype=np.float32)

    acc_plot = server.gui.add_uplot(
        data=(x_data, *y_acc),
        series=(
            viser.uplot.Series(label="time"),
            viser.uplot.Series(label="ax", stroke="red", width=2),
            viser.uplot.Series(label="ay", stroke="green", width=2),
            viser.uplot.Series(label="az", stroke="blue", width=2),
        ),
        title="Accelerometer",
        scales={
            "x": viser.uplot.Scale(time=False, auto=True),
            "y": viser.uplot.Scale(auto=True),
        },
        legend=viser.uplot.Legend(show=True),
        aspect=2.0,
    )

    return x_data, y_acc, acc_plot


def main():

    # Start the Viser server and create a point cloud object to hold the LiDAR data
    server = viser.ViserServer(port=8081)
    # TODO black BG
    bg_img = np.ndarray((1, 3, 3))
    server.scene.set_background_image(image=bg_img)

    # Add a mock point cloud

    points = np.array([[0, 0, 0], [1, 1, 1], [2, 2, 2]], dtype=np.float32)
    colors = np.array([[255, 0, 0], [0, 255, 0], [0, 0, 255]], dtype=np.uint8)

    cloud = server.scene.add_point_cloud(
        name="/lidar",
        points=points,
        colors=colors,
        point_size=0.05,
        point_shape="rounded",
    )


    while True:
        time.sleep(1.0)

    ## Stream Pointclouds

    # print(f"Connecting to gRPC server: {SERVER_ADDR}")
    # with grpc.insecure_channel(SERVER_ADDR) as channel:
    #     request = lidar_pb2.LidarStreamRequest()
    #     last_timestamp: int | None = None

    #     stub = lidar_pb2_grpc.LidarServiceStub(channel)

    #     for scan in stub.getLidarScan(request):
    #         scan: lidar_pb2.PointCloud3

    #         ts = int(scan.timestamp) if scan.HasField("timestamp") else 0
    #         n = len(scan.points)
    #         if last_timestamp is None:
    #             dt_ms = 0
    #         else:
    #             dt_ms = ts - last_timestamp
    #         last_timestamp = ts

    #         print(f"LiDAR scan: points={n} timestamp={ts} dt={dt_ms}ms")
    #         cloud.points = _to_viser_pointcloud(scan.points)
    #         cloud.colors = _to_viser_pointcloud_colors(scan.points)

    ## Stream Camera Frames

    # with grpc.insecure_channel(SERVER_ADDR) as channel:
    #     stub = camera_pb2_grpc.CameraServiceStub(channel)
    #     gui_image_handle: Optional[viser.GuiImageHandle] = None

    #     for frame in stub.getCameraFrame(camera_pb2.CameraStreamRequest()):
    #         frame: camera_pb2.CameraStreamReply

    #         ts = int(frame.timestamp) if frame.HasField("timestamp") else 0
    #         print(f"Camera frame: timestamp={ts} resolution={frame.width}x{frame.height} encoding={frame.encoding}")
    #         img_rgb = _decode_camera_frame(frame)
    #         if img_rgb is None:
    #             print(f"Skipping unsupported or invalid frame encoding={frame.encoding}")
    #             continue

    #         if gui_image_handle is None:
    #             gui_image_handle = server.gui.add_image(
    #                 img_rgb,
    #                 label="Camera",
    #                 format="jpeg",
    #             )
    #         else:
    #             gui_image_handle.image = img_rgb

    ## Stream IMU data

    # print(f"Connecting to gRPC server: {SERVER_ADDR}")
    # x_data, y_acc, acc_plot = _setup_imu_plots(server)
    # with grpc.insecure_channel(SERVER_ADDR) as channel:
    #     stub = imu_pb2_grpc.ImuServiceStub(channel)
    #     request = imu_pb2.ImuStreamRequest()
    #     sample = 0

    #     for imu in stub.getImuData(request):
    #         sample += 1
    #         idx = (sample - 1) % y_acc.shape[1]
    #         y_acc[:, idx] = (imu.ax, imu.ay, imu.az)

    #         acc_plot.data = (x_data, *y_acc)

    #         ts = int(imu.timestamp) if imu.HasField("timestamp") else 0
    #         print(
    #             f"IMU sample {sample}: timestamp={ts} "
    #             f"acc=({imu.ax:.3f}, {imu.ay:.3f}, {imu.az:.3f})"
    #         )

    ## Voxel size slider → drives the bidi stream requests

    # voxel_queue: queue.Queue[float] = queue.Queue()

    # demo_slider = server.gui.add_slider(
    #     "Voxel Size",
    #     min=0.01,
    #     max=1,
    #     step=0.01,
    #     initial_value=0.1,
    #     hint="Voxel size sent to the subsampled LiDAR stream.",
    # )

    # def handle_demo_slider_update(event: viser.GuiEvent):
    #     voxel_size = float(event.target.value)
    #     voxel_queue.put(voxel_size)
    #     print(f"Voxel size changed: {voxel_size:.1f}")

    # demo_slider.on_update(handle_demo_slider_update)

    # # Stream Subsampled Pointclouds

    # def request_iterator():
    #     while True:
    #         voxel_size = voxel_queue.get()  # blocks until a value is available
    #         yield lidar_pb2.SubSampledLidarStreamRequest(voxel_size=voxel_size)

    # print(f"Connecting to gRPC server: {SERVER_ADDR}")
    # with grpc.insecure_channel(SERVER_ADDR) as channel:
    #     last_timestamp: int | None = None

    #     stub = lidar_pb2_grpc.LidarServiceStub(channel)

    #     for scan in stub.getSubSampledLidarScan(request_iterator()):
    #         scan: lidar_pb2.PointCloud3

    #         ts = int(scan.timestamp) if scan.HasField("timestamp") else 0
    #         n = len(scan.points)
    #         if last_timestamp is None:
    #             dt_ms = 0
    #         else:
    #             dt_ms = ts - last_timestamp
    #         last_timestamp = ts

    #         print(f"LiDAR scan: points={n} timestamp={ts} dt={dt_ms}ms")
    #         cloud.points = _to_viser_pointcloud(scan.points)
    #         cloud.colors = _to_viser_pointcloud_colors(scan.points)


if __name__ == "__main__":
    main()
