project(ARNA_GUI)
cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)

#optimization/debug
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()
set(CMAKE_CXX_FLAGS "-Wall -Wextra -latomic")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

#export compile commands for language server
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

#include inc
include_directories(inc/)

# add rosbridge_lib
add_subdirectory(lib/rosbridge_lib)
include_directories(lib/rosbridge_lib/inc)

#find packages
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
find_package(OpenCV REQUIRED)
find_package(PkgConfig REQUIRED)
pkg_check_modules(GTK3 REQUIRED gtkmm-3.0)
pkg_check_modules(PC_OGG REQUIRED ogg)
pkg_check_modules(PC_THEORA REQUIRED theora)
pkg_check_modules(PC_THEORAENC REQUIRED theoraenc)
pkg_check_modules(PC_THEORADEC REQUIRED theoradec)

#include headers
include_directories(${OpenCV_INCLUDE_DIRS}
		    ${GTK3_INCLUDE_DIRS}
		    ${PC_OGG_INCLUDE_DIRS}
		    ${PC_THEORA_INCLUDE_DIRS}
		    ${PC_THEORAENC_INCLUDE_DIRS}
		    ${PC_THEORADEC_INCLUDE_DIRS}
)

#compiler flags
add_definitions(${GTK3_CFLAGS_OTHER}
		${PC_OGG_CFLAGS_OTHER}
 		${PC_THEORA_CFLAGS_OTHER}
		${PC_THEORAENC_CFLAGS_OTHER}
		${PC_THEORADEC_CFLAGS_OTHER}
)

#add executables and link libraries
add_executable(ARNA_TELE_VID src/ARNA_TELE_VID.cpp src/ARNA_THEORA.cpp src/joystick_listener.cpp)
target_link_libraries(ARNA_TELE_VID
		      ${GTK3_LIBRARIES}
		      rosbridge_lib
		      Threads::Threads
		      ${OpenCV_LIBRARIES}
		      ${PC_OGG_LIBRARIES}
 		      ${PC_THEORA_LIBRARIES}
 		      ${PC_THEORAENC_LIBRARIES}	
  		      ${PC_THEORADEC_LIBRARIES}
			  atomic
)

#add executables and link libraries
add_executable(ARNA_TELE_MOV src/ARNA_TELE_MOV.cpp src/joystick_listener.cpp)
target_link_libraries(ARNA_TELE_MOV
		      ${GTK3_LIBRARIES}
		      rosbridge_lib
		      Threads::Threads
			  atomic
)
