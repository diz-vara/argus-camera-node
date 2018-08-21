cmake_minimum_required(VERSION 2.8.3)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

project(argus-camera-node)

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" "${CMAKE_MODULE_PATH}")
# System Dependencies
find_package(PkgConfig)


find_package(Argus REQUIRED)

  find_package(catkin REQUIRED 
    COMPONENTS roscpp image_transport sensor_msgs nodelet
    camera_calibration_parsers camera_info_manager
    )

  catkin_package(
    INCLUDE_DIRS include
    LIBRARIES argus_cam
    CATKIN_DEPENDS roscpp nodelet image_transport sensor_msgs
    camera_calibration_parsers camera_info_manager
    #DEPENDS GSTREAMER GST_APP
    )

  include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${ARGUS_INCLUDE_DIR}
    /home/nvidia/tegra_multimedia_api/include/
    /home/nvidia/tegra_multimedia_api/include/libjpeg-8b/
    )

    link_directories(
      /usr/lib/aarch64-linux-gnu/tegra/
    )

set(SRC_FILES
    src/argus-cam.cpp
    src/NvJpegEncoder.cpp
    src/NvElement.cpp
    src/NvElementProfiler.cpp
    src/NvLogging.cpp
)


  add_executable(argus_cam ${SRC_FILES})
  target_link_libraries(argus_cam
    ${catkin_LIBRARIES}
    stdc++fs
    pthread
    nvbuf_utils
    nvjpeg
    nveglstream_camconsumer
    ${ARGUS_LIBRARIES}
    )

  set_target_properties(argus_cam PROPERTIES OUTPUT_NAME argus-cam)

  # Install directives

  install(TARGETS argus_cam
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

  install(DIRECTORY include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
    PATTERN ".svn" EXCLUDE)


  install(FILES
    argus_cam.launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

  install(FILES examples/uncalibrated_parameters.ini
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/examples
    )

