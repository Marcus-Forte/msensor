#pragma once

#include "imu.pb.h"
#include "lidar.pb.h"
#include "msensor/interface/IImu.hh"
#include "msensor/interface/ILidar.hh"

namespace msensor {

/**
 * @brief Convert a gRPC point cloud message into an msensor point cloud.
 */
Scan3DI fromProtobuf(const sensors::PointCloud3 &msg);

/**
 * @brief Convert an msensor point cloud into a reusable gRPC message.
 */
void toProtobuf(const Scan3DI &scan, sensors::PointCloud3 &point_cloud);

/**
 * @brief Convert a gRPC IMU message into an msensor IMU sample.
 */
IMUData fromProtobuf(const sensors::IMUData &msg);

/**
 * @brief Convert an msensor IMU sample into a reusable gRPC message.
 */
void toProtobuf(const IMUData &imu_data, sensors::IMUData &grpc_data);

} // namespace msensor

#ifndef MSENSOR_NO_LEGACY_NAMES
// Legacy global names, for downstream code. Define MSENSOR_NO_LEGACY_NAMES to
// drop them.
using msensor::fromProtobuf;
using msensor::toProtobuf;
#endif
