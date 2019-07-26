cmake_minimum_required(VERSION 3.5)
project(LaneDetection)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_FLAGS	"${CMAKE_C_FLAGS} -Wall -Wextra")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_FLAGS	"${CMAKE_CXX_FLAGS} -Wall -Wextra -no-pie")

#add_definitions(-DMEASSURE_EXECUTION_TIME)

find_package(Threads REQUIRED)
if(THREADS_HAVE_PTHREAD_ARG)
	compile_options(PUBLIC "-pthread")
endif()
if(CMAKE_THREAD_LIBS_INIT)
	link_libraries("${CMAKE_THREAD_LIBS_INIT}")
endif()

find_package(OpenCV REQUIRED)

#this can be made easier with GLOB or so
add_executable(LaneDetection
	src/ClientServer/Server.cpp
	src/ClientServer/Connection.cpp
	src/LaneDetection.cpp
	src/Main.cpp
	src/VideoProviders/VideoProvider.cpp
	src/VideoProviders/FileVideoProvider.cpp
	src/VideoProviders/CameraVideoProvider.cpp
	src/VideoConsumers/VideoConsumer.cpp
	src/VideoConsumers/VideoDisplay.cpp
	src/VideoConsumers/VideoStreamer.cpp
	src/VideoConsumers/VideoSocket.cpp
	src/VideoConsumers/VideoFileWriter.cpp
	src/ResultConsumers/ResultServer.cpp
	src/ResultConsumers/ResultConsumer.cpp
	src/Frame_ts.cpp 
	src/Frame_ts.h 
	src/PrecedingRoverDetection.cpp 
	src/PrecedingRoverDetection.h 
	src/PrecedingRover_ts.cpp
	src/PrecedingRover_ts.h
	)


target_link_libraries(LaneDetection ${OpenCV_LIBS})
target_link_libraries(LaneDetection /usr/local/lib/libpistache.a)
#target_link_libraries(LaneDetection /usr/lib/x86_64-linux-gnu/libzbar.so)
target_link_libraries(LaneDetection ${ZBAR_LIBRARIES} zbar)

add_executable(TestServer
	src/ClientServer/Server.cpp
	src/ClientServer/Connection.cpp
)
target_compile_definitions(TestServer PRIVATE TEST)

add_executable(TestClient
	src/ClientServer/Client.c
)
target_compile_definitions(TestClient PRIVATE TEST)

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(src/LaneDetectionAccessories)
