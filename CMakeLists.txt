cmake_minimum_required(VERSION 3.31)
project(adaptagrams)

set(CMAKE_CXX_STANDARD 17)
# 手动设置JAVA_HOME
set(JAVA_HOME "/Library/Java/JavaVirtualMachines/jdk-17.jdk/Contents/Home")
set(LIBAVOID_STATIC ${CMAKE_SOURCE_DIR}/libavoid/.libs/libavoid.a)
set(LIBAVOID_DYNAMIC ${CMAKE_SOURCE_DIR}/libavoid/.libs/libavoid.0.dylib)

set(LIBCOLA_STATIC ${CMAKE_SOURCE_DIR}/libcola/.libs/libcola.a)
set(LIBCOLA_DYNAMIC ${CMAKE_SOURCE_DIR}/libcola/.libs/libcola.0.dylib)

set(LIBDIALECT_STATIC ${CMAKE_SOURCE_DIR}/libdialect/.libs/libdialect.a)
set(LIBDIALECT_DYNAMIC ${CMAKE_SOURCE_DIR}/libdialect/.libs/libdialect.0.dylib)

set(LIBTOPOLOGY_STATIC ${CMAKE_SOURCE_DIR}/libtopology/.libs/libtopology.a)
set(LIBTOPOLOGY_DYNAMIC ${CMAKE_SOURCE_DIR}/libtopology/.libs/libtopology.0.dylib)

set(LIBVPSC_STATIC ${CMAKE_SOURCE_DIR}/libvpsc/.libs/libvpsc.a)
set(LIBVPSC_DYNAMIC ${CMAKE_SOURCE_DIR}/libvpsc/.libs/libvpsc.0.dylib)
# 配置JNI头文件路径
include_directories(
        ${JAVA_HOME}/include
        ${JAVA_HOME}/include/darwin
        ${CMAKE_SOURCE_DIR}
)

link_directories(
        ${CMAKE_SOURCE_DIR}/libavoid/.libs
        ${CMAKE_SOURCE_DIR}/libcola/.libs
        ${CMAKE_SOURCE_DIR}/libdialect/.libs
        ${CMAKE_SOURCE_DIR}/libtopology/.libs
        ${CMAKE_SOURCE_DIR}/libvpsc/.libs
)

set(SRC_FILES
        cola/libnative/org_graph_outside_NativeLibrary.cpp
)
# 源文件
add_library(native_library SHARED
        cola/libnative/org_graph_outside_NativeLibrary.cpp
)

target_link_libraries(native_library PRIVATE
        ${LIBAVOID_STATIC}
        ${LIBCOLA_STATIC}
        ${LIBDIALECT_STATIC}
        ${LIBTOPOLOGY_STATIC}
        ${LIBVPSC_STATIC}
)