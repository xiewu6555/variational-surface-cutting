cmake_minimum_required(VERSION 3.10)

# Maybe stop from CMAKEing in the wrong place
if (CMAKE_BINARY_DIR STREQUAL CMAKE_SOURCE_DIR)
    message(FATAL_ERROR "Source and build directories cannot be the same. Go use the /build directory.")
endif()


# Collect all .cpp files in the subdirectories
file(GLOB_RECURSE CUTS_SRCS *.cpp)

# Create a single library for the project
add_library(
        cuts-core
        ${CUTS_SRCS}
        )

# 添加 vcpkg CHOLMOD 包含路径
target_include_directories(cuts-core PUBLIC
    ${CMAKE_PREFIX_PATH}/include/suitesparse
    ${CMAKE_PREFIX_PATH}/include
)

# 查找CHOLMOD包
find_package(CHOLMOD CONFIG)

target_link_libraries(cuts-core
    core
    rectangle-bin-pack
    flatten-core
)

# 有条件地链接CHOLMOD
if(TARGET SuiteSparse::CHOLMOD)
    target_link_libraries(cuts-core SuiteSparse::CHOLMOD)
else()
    # 如果找不到CHOLMOD target，尝试直接链接库文件
    if(CMAKE_BUILD_TYPE STREQUAL "Debug")
        if(EXISTS "${CMAKE_PREFIX_PATH}/debug/lib/cholmod.lib")
            target_link_libraries(cuts-core "${CMAKE_PREFIX_PATH}/debug/lib/cholmod.lib")
        endif()
    else()
        if(EXISTS "${CMAKE_PREFIX_PATH}/lib/cholmod.lib")
            target_link_libraries(cuts-core "${CMAKE_PREFIX_PATH}/lib/cholmod.lib")
        endif()
    endif()
endif()
