add_library(goldo_lidar_detect STATIC 
  src/lidar_detect.cpp
  )
  
target_include_directories(goldo_lidar_detect PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>  
  )
