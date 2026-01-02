#pragma once

#include "interface/IImu.hh"
#include "interface/ILidar.hh"
#include "sensors.pb.h"

/**
 * @brief Convert a gRPC point cloud message into an msensor point cloud.
 */
std::shared_ptr<msensor::Scan3DI> fromGRPC(const sensors::PointCloud3 &msg);
/**
 * @brief Convert a gRPC IMU message into an msensor IMU sample.
 */
msensor::IMUData fromGRPC(const sensors::IMUData &msg);