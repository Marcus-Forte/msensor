add_library(msensor_ads1115 ads1115.cc)
add_library(msensor::ads1115 ALIAS msensor_ads1115)
target_link_libraries(msensor_ads1115 i2c msensor::IAdc)


add_library(msensor_sim_adc sim_adc.cc)
add_library(msensor::sim_adc ALIAS msensor_sim_adc)
target_link_libraries(msensor_sim_adc msensor::IAdc)
