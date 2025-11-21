cmake_minimum_required(VERSION 3.31)

project(adaptagrams-demo)

set(CMAKE_CXX_STANDARD 17)

set(THIRD_PARTY_DIR "${CMAKE_SOURCE_DIR}/third_party")
include_directories(${CMAKE_SOURCE_DIR}/include ${THIRD_PARTY_DIR}/include)

file(GLOB_RECURSE SOURCES "src/*.cpp" "examples/*.cpp")
add_executable(adaptagrams_demo ${SOURCES})

link_directories(${THIRD_PARTY_DIR}/lib)
set(CMAKE_BUILD_RPATH "${THIRD_PARTY_DIR}/lib")
set(CMAKE_INSTALL_RPATH "${THIRD_PARTY_DIR}/lib")

find_package(Threads REQUIRED)

target_link_libraries(adaptagrams_demo PRIVATE
    avoid
    cola
    dialect
    topology
    vpsc
    ${CMAKE_THREAD_LIBS_INIT}
)