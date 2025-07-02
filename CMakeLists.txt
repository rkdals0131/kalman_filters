## kalman_filters library
set(PROJECT_NAME kalman_filters)

cmake_minimum_required(VERSION 3.1)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
project(${PROJECT_NAME})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})

# include_directories(include)

## Define library
add_library(${PROJECT_NAME}_lib STATIC
  src/ekf.cpp
  src/ukf.cpp
  src/system_model.cpp
)
target_include_directories(${PROJECT_NAME}_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)


install(TARGETS ${PROJECT_NAME}_lib
    EXPORT ${PROJECT_NAME}-targets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)


install(DIRECTORY include/
    DESTINATION include/${PROJECT_NAME}
    FILES_MATCHING PATTERN "*.hpp"
)

# Also install main headers to root include for easier access
install(FILES 
    include/system_model.hpp 
    include/ukf.hpp 
    include/ekf.hpp
    DESTINATION include
)

# Export library
install(EXPORT ${PROJECT_NAME}-targets
    FILE ${PROJECT_NAME}-targets.cmake
    NAMESPACE ${PROJECT_NAME}::
    DESTINATION lib/cmake/${PROJECT_NAME}
)

# Configure and install config file
include(CMakePackageConfigHelpers)
configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/cmake/${PROJECT_NAME}-config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    INSTALL_DESTINATION lib/cmake/${PROJECT_NAME}
)

install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}-config.cmake"
    DESTINATION lib/cmake/${PROJECT_NAME}
)
