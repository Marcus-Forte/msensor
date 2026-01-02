add_library(ads1115 ads1115.cc)
target_link_libraries(ads1115 i2c IAdc)


add_library(sim_adc sim_adc.cc)
target_link_libraries(sim_adc IAdc)
