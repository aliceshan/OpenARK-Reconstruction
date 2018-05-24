cmake_minimum_required(VERSION 2.8)

# Compiler requirements
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

project(tinyply LANGUAGES CXX)
find_package(PCL 1.2 REQUIRED)
find_package(OpenCV REQUIRED)

set(PROJECT_VERSION 2.0)

include_directories("${CMAKE_SOURCE_DIR}/source" ${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


#CGL
add_subdirectory("${CMAKE_SOURCE_DIR}/source/CGL")
include_directories("${CMAKE_SOURCE_DIR}/source/CGL/include")

# Library
add_library(tinyply source/tinyply.cpp source/halfEdgeMesh.cpp source/CGL/src/vector3D.cpp source/CGL/src/vector2D.cpp)

# Example
add_executable(colorizer source/colorizer.cpp)
target_link_libraries(colorizer PRIVATE tinyply ${PCL_LIBRARIES} ${OpenCV_LIBS})
# pkg-config support
set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
if(UNIX)
  configure_file(cmake/tinyply.pc.in ${generated_dir}/tinyply.pc @ONLY)
  install(FILES "${generated_dir}/tinyply.pc" DESTINATION lib/pkgconfig)
endif()

# CMake package config support
include(CMakePackageConfigHelpers)

set(config_install_dir "lib/cmake/${PROJECT_NAME}")
set(version_config "${generated_dir}/${PROJECT_NAME}ConfigVersion.cmake")
set(project_config "${generated_dir}/${PROJECT_NAME}Config.cmake")
set(targets_export_name "${PROJECT_NAME}Targets")
set(namespace "")

write_basic_package_version_file(
  "${version_config}"
  VERSION ${PROJECT_VERSION}
  COMPATIBILITY SameMajorVersion
)
configure_package_config_file(
  "cmake/Config.cmake.in"
  "${project_config}"
  INSTALL_DESTINATION "${config_install_dir}"
)

# Install
install(TARGETS tinyply EXPORT ${targets_export_name}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib)
install(FILES source/tinyply.h
        DESTINATION include)

install(FILES "${project_config}" "${version_config}"
        DESTINATION "${config_install_dir}")

install(EXPORT "${targets_export_name}"
        NAMESPACE "${namespace}"
        DESTINATION "${config_install_dir}")
