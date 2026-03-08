
#include "adc_service.hh"

AdcServiceImpl::AdcServiceImpl(std::shared_ptr<msensor::IAdc> adc)
    : adc_(adc) {}

::grpc::Status
AdcServiceImpl::getAdcData(::grpc::ServerContext *context,
                           const ::sensors::AdcDataRequest *request,
                           ::sensors::AdcData *response) {
  if (!adc_) {
    return grpc::Status(grpc::StatusCode::UNAVAILABLE, "ADC not available");
  }

  const auto adc_value = adc_->readSingleEnded();
  if (!adc_value) {
    return grpc::Status(grpc::StatusCode::UNKNOWN, "ADC read failure");
  }

  response->set_sample(adc_value->voltage);
  response->set_timestamp(adc_value->timestamp);
  return ::grpc::Status::OK;
}
