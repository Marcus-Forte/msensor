from colormap import int2rgb

import grpc
import numpy as np
import viser
from typing import Sequence

from proto_gen import lidar_pb2, lidar_pb2_grpc


DEFAULT_SERVER_ADDR = "127.0.0.1:50051"


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


def main() -> None:

    server = viser.ViserServer(port=8081)
    cloud = server.scene.add_point_cloud(
        name="/lidar",
        points=np.empty((0, 3), dtype=np.float32),
        colors=np.empty((0, 3), dtype=np.uint8),
        point_size=0.05,
        point_shape="rounded",
    )

    camera = server.scene.add_icosphere

    request = lidar_pb2.LidarStreamRequest()
    last_timestamp: int | None = None

    print(f"Connecting to gRPC server: {DEFAULT_SERVER_ADDR}")
    with grpc.insecure_channel(DEFAULT_SERVER_ADDR) as channel:
        stub = lidar_pb2_grpc.LidarServiceStub(channel)

        for scan in stub.getLidarScan(request):
            scan: lidar_pb2.PointCloud3

            ts = int(scan.timestamp) if scan.HasField("timestamp") else 0
            n = len(scan.points)
            if last_timestamp is None:
                dt_ms = 0
            else:
                dt_ms = ts - last_timestamp
            last_timestamp = ts

            print(f"LiDAR scan: points={n} timestamp={ts} dt={dt_ms}ms")
            cloud.points = _to_viser_pointcloud(scan.points)
            cloud.colors = _to_viser_pointcloud_colors(scan.points)


if __name__ == "__main__":
    main()
