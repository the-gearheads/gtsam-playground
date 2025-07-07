cmake_minimum_required(VERSION 3.28..3.32)

project(gtsam-playground CXX)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if (MSVC)
  add_compile_definitions(_ENABLE_EXTENDED_ALIGNED_STORAGE)
endif()
add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")

set(CMAKE_CXX_STANDARD 20)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_INSTALL_RPATH "lib")

# redirect output binaries
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)

# use ccache if we got it
# find_program(CCACHE_PRESENT "ccache")
# if(CCACHE_PRESENT)
#   message("Using Ccache")
#   set(CMAKE_C_COMPILER_LAUNCHER "ccache")
#   set(CMAKE_CXX_COMPILER_LAUNCHER "ccache")
# endif()

set(BUILD_SHARED_LIBS OFF)

include(FetchContent)

find_package(OpenCV REQUIRED)

# Fetch Eigen -- From thirdparty-gtsam
message("Fetching Eigen")
set(EIGEN_BUILD_BLAS OFF)
set(EIGEN_BUILD_LAPACK OFF)
set(EIGEN_BUILD_CMAKE_PACKAGE ON)
FetchContent_Declare(
    Eigen3
    GIT_REPOSITORY    https://gitlab.com/libeigen/eigen.git
    GIT_TAG           0fb2ed140d4fc0108553ecfb25f2d7fc1a9319a1
    SYSTEM # Ignore warnings
    # Make gtsam (and our in-tree wpilib build) pick up this Eigen as our system Eigen
    OVERRIDE_FIND_PACKAGE
    EXCLUDE_FROM_ALL
)

# And build GTSAM -- From thirdparty-gtsam
message("Fetching Gtsam")
set(GTSAM_COMPILE_OPTIONS_PRIVATE_DEBUG CACHE INTERNAL FORCE)
set(GTSAM_COMPILE_OPTIONS_PRIVATE_RELWITHDEBINFO CACHE INTERNAL FORCE)
set(GTSAM_ENABLE_BOOST_SERIALIZATION OFF)
set(GTSAM_USE_BOOST_FEATURES OFF)
set(GTSAM_BUILD_PYTHON OFF)
set(GTSAM_BUILD_EXAMPLES_ALWAYS OFF)
set(GTSAM_BUILD_TESTS OFF)
set(GTSAM_FORCE_SHARED_LIB OFF)
set(GTSAM_FORCE_STATIC_LIB ON)
set(GTSAM_USE_SYSTEM_EIGEN ON)

FetchContent_Declare(
    gtsam
    GIT_REPOSITORY    https://github.com/borglab/gtsam
    GIT_TAG           a16dff37e6f3248bce67432754375d33f849388a
    EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(gtsam)


set(WPILIB_DIR "" CACHE PATH "Path to existing WPILib clone, if any")

set(WITH_SIMULATION_MODULES OFF)
set(WITH_TESTS OFF)
set(WITH_WPILIB ON)
set(NO_WERROR ON)
set(WITH_PROTOBUF OFF)
set(USE_SYSTEM_EIGEN ON) # use the same eigen that we're using for gtsam, just so we're using the same exact eigen everywhere


# this probably doesnt work right if the build dir isnt clean and wpilib_dir becomes defined
if(WPILIB_DIR STREQUAL "")
  # fetch it ourselves
  message("Fetching WPILib")
  FetchContent_Declare(
    allwpilib
    GIT_REPOSITORY https://github.com/wpilibsuite/allwpilib
    GIT_TAG        refs/tags/v2025.3.2
    GIT_SHALLOW 1
    EXCLUDE_FROM_ALL
  )
  FetchContent_MakeAvailable(allwpilib)
else()
  add_subdirectory(${WPILIB_DIR} ${CMAKE_BINARY_DIR}/allwpilib EXCLUDE_FROM_ALL)
endif()


add_library(gtsam-localizer
  src/localizer.cpp
  src/TagModel.cpp
  src/gtsam_utils.cpp
  src/config.cpp
  src/camera_listener.cpp
  src/odom_listener.cpp
  src/data_publisher.cpp
  src/config_listener.cpp
)

target_include_directories(gtsam-localizer PUBLIC
  ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(gtsam-localizer PUBLIC
  wpimath
  apriltag
  ntcore
  gtsam
  ${OpenCV_LIBS}
)

if (MSVC)
  target_link_libraries(gtsam-localizer PUBLIC dbghelp.lib) 
endif()


add_executable(gtsam-node
  src/gtsam_tags_node.cpp
)

target_link_libraries(gtsam-node
  gtsam-localizer
)

install(TARGETS gtsam-node)

set(gtest_build_tests OFF CACHE BOOL "" INTERNAL) # we dont wanna include python pointlessly (update: its still getting find_packaged somewhere)
set(gtest_build_samples OFF CACHE BOOL "" INTERNAL)
set(gtest_force_shared_crt ON CACHE BOOL "" INTERNAL) # for windows, says the original comment

FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/03597a01ee50ed33e9dfd640b249b4be3799d395.zip
  EXCLUDE_FROM_ALL
)
FetchContent_MakeAvailable(googletest)

enable_testing()

add_executable(
  localizer_test
  test/Test_Localizer.cpp
  test/Test_Config.cpp
)

target_link_libraries(
  localizer_test
  GTest::gtest_main gtsam-localizer
)
target_include_directories(localizer_test PRIVATE ${PROJECT_SOURCE_DIR}/src)

include(GoogleTest)
if(NOT CMAKE_CROSSCOMPILING)
  gtest_discover_tests(localizer_test)
endif()