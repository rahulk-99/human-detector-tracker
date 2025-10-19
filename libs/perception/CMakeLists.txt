# Perception library modules
cmake_minimum_required(VERSION 3.14)

# Find required dependencies
find_package(OpenCV QUIET)
if(NOT OpenCV_FOUND)
  message(WARNING "OpenCV not found - using stub implementation for Phase 0")
endif()

# Collect all source files
set(PERCEPTION_SOURCES
  # Detection module
  detection/BoundingBox.cpp
  detection/Detection.cpp
  detection/YOLODetector.cpp
  
  # Tracking module
  tracking/Track.cpp
  tracking/KalmanFilter.cpp
  tracking/KalmanTracker.cpp
  
  # Core module
  core/CameraModel.cpp
  core/CoordinateTransformer.cpp
  core/PerceptionPipeline.cpp
  
  # Utils module
  utils/Position3D.cpp
  utils/GeometryUtils.cpp
)

# Create perception library
add_library(perception STATIC ${PERCEPTION_SOURCES})

# Set C++17 standard
target_compile_features(perception PUBLIC cxx_std_17)

# Include directories
target_include_directories(perception
  PUBLIC
    ${PROJECT_SOURCE_DIR}/include
)

# Link OpenCV if found
if(OpenCV_FOUND)
  target_link_libraries(perception PUBLIC ${OpenCV_LIBS})
  target_compile_definitions(perception PUBLIC HAVE_OPENCV)
endif()

# Compiler warnings
target_compile_options(perception PRIVATE
  -Wall
  -Wextra
  -Wpedantic
  $<$<CONFIG:Debug>:-g>
  $<$<CONFIG:Release>:-O3>
)

# Installation rules
install(TARGETS perception
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/perception
  DESTINATION include
)

