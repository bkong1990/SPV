project(test)  
find_package(OpenCV REQUIRED)  
set(SRC_LIST main.cpp CircularVoting.cpp)
add_executable(test ${SRC_LIST})
target_link_libraries(test ${OpenCV_LIBS})  
cmake_minimum_required(VERSION 2.8)
