cmake_minimum_required(VERSION 2.8)
project(test)
# add eigen
include_directories("/usr/include/eigen3")

# add opencv 
find_package(OpenCV REQUIRED)
include_directories({$OpenCV_INCLUDE_DIRS})


# add sophus lib
find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

# add ceres
find_package(Ceres REQUIRED)
include_directories({$CERES_INCLUDE_DIRS})

# add g2o
list( APPEND CMAKE_MODULE_PATH /home/cuihang/g2o/cmake_modules )
set(G2O_ROOT /usr/local/include/g2o)
find_package(G2O)
include_directories({$G2O_INCLUDE_DIRS})


add_executable(test1 ./src/test1.cpp)
target_link_libraries(test1 Sophus::Sophus ${OpenCV_LIBS} ${CERES_LIBRARIES} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})

add_executable(ceresba ./src/ceres_ba.cpp)
target_link_libraries(ceresba Sophus::Sophus ${OpenCV_LIBS} ${CERES_LIBRARIES} ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY})
