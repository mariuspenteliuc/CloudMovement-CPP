# CMakeLists.txt

# Older versions of CMake are likely to work just fine but, since
# I don't know where to cut off I just use the version I'm using
cmake_minimum_required(VERSION "3.17")

# name of this example project
project(CloudMovement-CPP)

# set OpenCV_DIR variable equal to the path to the cmake
# files within the previously installed opencv program
set(OpenCV_DIR /Users/macbookair/Documents/uvt/disertatie/opencv/install/lib/cmake/opencv4)

# Tell compiler to use C++ 14 features which is needed because
# Clang version is often behind in the XCode installation
set(CMAKE_CXX_STANDARD 14)

# configure the necessary common CMake environment variables
# needed to include and link the OpenCV program into this
# demo project, namely OpenCV_INCLUDE_DIRS and OpenCV_LIBS
find_package( OpenCV REQUIRED )
find_package( OpenMP )

if (OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

# tell the build to include the headers from OpenCV
include_directories(${CMAKE_SOURCE_DIR}/source ${OpenCV_INCLUDE_DIRS} )

# specify the executable target to be built
add_executable(CloudMovement-CPP source/main.cpp source/OpticalFlow/OpticalFlowService.cpp source/Boids/Boid.cpp source/Boids/Vector.cpp source/Boids/Scene.cpp source/Helpers/ImageHelper.cpp source/Helpers/MathHelper.cpp)

# tell it to link the executable target against OpenCV
target_link_libraries(CloudMovement-CPP ${OpenCV_LIBS} ${OpenCV_INCLUDE_DIRS})