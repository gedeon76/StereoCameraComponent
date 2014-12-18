project(StereoCameraComponent)
cmake_minimum_required(VERSION 2.8)
add_definitions(-std=c++11)

# if(NOT CMAKE_BUILD_TYPE)
#   set(CMAKE_BUILD_TYPE Debug CACHE STRING
#       "Choose the type of build, options are: None Debug Release RelWithDebInfo MinSizeRel."
#       FORCE)
# endif(NOT CMAKE_BUILD_TYPE)

#define the path to the cmake modules
add_subdirectory(cmake)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules")
# set output directories
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_BINARY_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${CMAKE_HOME_DIRECTORY}/lib)


# find library packages
find_package(OpenCV REQUIRED)

#find the boost libraries
set(Boost_USE_STATIC_LIBS ON)
set(Boost_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
set(BOOST_ROOT $ENV{BOOSTDIR})

if (${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x86")
	set(ARCH "32")
	message("Arch: "${CMAKE_SYSTEM_PROCESSOR})
elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "x64")
	set(ARCH "64")
	message("Arch: "${CMAKE_SYSTEM_PROCESSOR})
endif()

if (WIN32)
	# look for vs2013 version
	if (${MSVC12})
			set(BOOST_LIBRARYDIR ${BOOST_ROOT}/lib${ARCH}-msvc-12.0)
			message("BOOST_LIBRARYDIR: " ${BOOST_LIBRARYDIR})
	endif()
	
endif()

find_package(Boost REQUIRED COMPONENTS thread date_time system filesystem signals)

if (Boost_FOUND)
	include_directories(${Boost_INCLUDE_DIRS})
	message("Boost include dir: " ${Boost_INCLUDE_DIRS})
	link_directories(${Boost_LIBRARY_DIRS})
	message("Boost libraries dir: " ${Boost_LIBRARY_DIRS})
endif()
	
# add source directory
add_subdirectory(src)



 
