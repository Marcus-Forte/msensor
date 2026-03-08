#pragma once

#include "adc.grpc.pb.h"
#include "interface/IAdc.hh"

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
