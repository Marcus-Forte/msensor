#pragma once

#include "interface/ICamera.hh"
#include "interface/IImu.hh"
#include "interface/ILidar.hh"
#include "sensors.pb.h"

/**
 * @brief Convert a gRPC point cloud message into an msensor point cloud.
 */
std::shared_ptr<msensor::Scan3DI> fromGRPC(const sensors::PointCloud3 &msg);

/**
 * @brief Convert an msensor point cloud to gRPC point cloud message.
 */
sensors::PointCloud3 toGRPC(const std::shared_ptr<msensor::Scan3DI> &msg);

/**
 * @brief Convert a gRPC IMU message into an msensor IMU sample.
 */
msensor::IMUData fromGRPC(const sensors::IMUData &msg);

/**
 * @brief Convert a msensor IMU message into a gRPC IMU message.
 */
sensors::IMUData toGRPC(msensor::IMUData msg);

/**
 * @brief Converts a msensor camera frame to gRPC camera message.
 *
 * @param Frame Camera Frame
 * @param quality JPEG quality [0-100]
 */
sensors::CameraStreamReply toGRPC(const msensor::CameraFrame &Frame,
                                  int quality = 85);