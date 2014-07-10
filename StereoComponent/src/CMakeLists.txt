cmake_minimum_required(VERSION 2.8)

if (UNIX)
  include_directories(${CMAKE_CURRENT_BINARY_DIR} ${OpenCV_INCLUDE_DIR} ) 
  
else (UNIX)
  include_directories(${CMAKE_CURRENT_BINARY_DIR} ${OpenCV_INCLUDE_DIR}) 
 
endif (UNIX)


set(StereoCameraComponent_SRCS Settings.h CameraCalibration.h InterfaceStereoCamera.hpp StereoCamera.h StereoCameraAccess.h Settings.cpp CameraCalibration.cpp StereoCamera.cpp StereoCameraAccess.cpp)
#add_executable(StereoCameraComponent ${StereoCameraComponent_SRCS})
add_library(StereoCameraComponent STATIC ${StereoCameraComponent_SRCS})


#if (UNIX) 
#  target_link_libraries(StereoCameraComponent ${OpenCV_LIBS})
#endif (UNIX)
#if (WIN32)
#  target_link_libraries(StereoCameraComponent ${OpenCV_LIBS})
#endif (WIN32)