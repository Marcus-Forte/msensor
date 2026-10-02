#pragma once

#include "camera.pb.h"
#include "imu.pb.h"
#include "lidar.pb.h"
#include "msensor/interface/ICamera.hh"
#include "msensor/interface/IImu.hh"
#include "msensor/interface/ILidar.hh"

/**
 * @brief Convert a gRPC point cloud message into an msensor point cloud.
 */
msensor::Scan3DI fromProtobuf(const sensors::PointCloud3 &msg);

/**
 * @brief Convert an msensor point cloud into a reusable gRPC message.
 */
void toProtobuf(const msensor::Scan3DI &scan, sensors::PointCloud3 &point_cloud);

/**
 * @brief Convert a gRPC IMU message into an msensor IMU sample.
 */
msensor::IMUData fromProtobuf(const sensors::IMUData &msg);

/**
 * @brief Convert an msensor IMU sample into a reusable gRPC message.
 */
void toProtobuf(const msensor::IMUData &imu_data, sensors::IMUData &grpc_data);

/**
 * @brief Converts a msensor camera frame to gRPC camera message.
 *
 * @param frame Camera Frame
 * @param reply Output message, which may be reused between calls.
 * @param quality JPEG quality [0-100]
 */
void toProtobuf(const msensor::CameraFrame &frame,
                sensors::CameraStreamReply &reply, int quality = 85);