cmake_minimum_required(VERSION 3.19)

project(digitalcurling3_client_examples
    VERSION 0.1.0
    LANGUAGES CXX
)

# use C++ 17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

# Digital Curling
add_subdirectory(extern/DigitalCurling3)

# boost.asio
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_RUNTIME OFF)
find_package(Boost 1.75 REQUIRED COMPONENTS date_time regex)

# Threads
find_package(Threads REQUIRED)

# source
add_subdirectory(rulebased)
add_subdirectory(stdio)
