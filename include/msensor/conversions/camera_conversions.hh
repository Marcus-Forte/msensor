#pragma once

#include "camera.pb.h"
#include "msensor/interface/ICamera.hh"

namespace msensor {

/**
 * @brief Converts a msensor camera frame to gRPC camera message.
 *
 * @param frame Camera Frame
 * @param reply Output message, which may be reused between calls.
 * @param quality JPEG quality [0-100]; ignored for JPEG passthrough frames.
 */
void toProtobuf(const CameraFrame &frame, sensors::CameraStreamReply &reply,
                int quality = 85);

} // namespace msensor

#ifndef MSENSOR_NO_LEGACY_NAMES
// Legacy global name, for downstream code. Define MSENSOR_NO_LEGACY_NAMES to
// drop it.
using msensor::toProtobuf;
#endif
