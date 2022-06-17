cmake_minimum_required(VERSION 3.8.2)
project(ProgettoAOS_client)

set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_C_COMPILER   /opt/gcc-arm-linux-gnueabihf/bin/arm-none-linux-gnueabihf-gcc)
#set(CMAKE_CXX_COMPILER /opt/gcc-arm-linux-gnueabihf/bin/arm-none-linux-gnueabihf-g++)

find_package (OpenCV REQUIRED)

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(ProgettoAOS_client main.cpp)
target_link_libraries(ProgettoAOS_client ${OpenCV_LIBS})