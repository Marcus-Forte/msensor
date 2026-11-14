#pragma once

#include "adc.grpc.pb.h"
#include "msensor/interface/IAdc.hh"

namespace msensor {

/**
 * @brief Implements the ADC gRPC service.
 */
class AdcServiceImpl : public sensors::AdcService::Service {
public:
  AdcServiceImpl(std::shared_ptr<msensor::IAdc> adc);

  ::grpc::Status getAdcData(::grpc::ServerContext *context,
                            const ::sensors::AdcDataRequest *request,
                            ::sensors::AdcData *response) override;

private:
  std::shared_ptr<msensor::IAdc> adc_;
};

} // namespace msensor

#ifndef MSENSOR_NO_LEGACY_NAMES
// Legacy global names, for downstream code. Define MSENSOR_NO_LEGACY_NAMES to
// drop them.
using msensor::AdcServiceImpl;
#endif
