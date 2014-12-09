cmake_minimum_required(VERSION 2.8)


include_directories(${CMAKE_CURRENT_BINARY_DIR} ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} ) 
link_directories(${OpenCV_LIB_DIR} ${Boost_LIBRARY_DIRS})
  

set(StereoCameraComponent_SRCS 
					Settings.h 
					Results.h 
					commonStereoComponent.h
					CameraCalibration.h					
					InterfaceStereoCamera.hpp
					StereoCamera.h 
					StereoCameraAccess.h 
					TrackerPoint.h
					Settings.cpp 
					Results.cpp 
					CameraCalibration.cpp
					TrackerPoint.cpp 
					StereoCamera.cpp 
					StereoCameraAccess.cpp)


add_library(StereoCameraComponent STATIC ${StereoCameraComponent_SRCS})
set(LIBRARY_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/lib )

# copy the interface header to the library
file(COPY	InterfaceStereoCamera.hpp 
			CameraCalibration.h
			commonStereoComponent.h  
			StereoCameraAccess.h 
			TrackerPoint.h
			StereoCamera.h
			Settings.h
			Results.h
			Left_Setup_c270.xml
			Right_Setup_c270.xml
			DESTINATION ${CMAKE_HOME_DIRECTORY}/include)

message("output directory is: " ${LIBRARY_OUTPUT_PATH})

#add_executable(StereoCameraComponent ${StereoCameraComponent_SRCS})
#if (UNIX) 
#  target_link_libraries(StereoCameraComponent ${OpenCV_LIBS})
#endif (UNIX)
#if (WIN32)
#  target_link_libraries(StereoCameraComponent ${OpenCV_LIBS})
#endif (WIN32)