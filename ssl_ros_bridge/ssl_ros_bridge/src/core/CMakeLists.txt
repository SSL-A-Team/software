add_library(${PROJECT_NAME}_core SHARED
    get_ip_addresses.cpp
    multicast_receiver.cpp
)
target_include_directories(${PROJECT_NAME}_core PUBLIC .)
target_link_libraries(${PROJECT_NAME}_core
  Boost::boost
  protobuf::libprotobuf
)
install(TARGETS ${PROJECT_NAME}_core DESTINATION lib)
