cmake_minimum_required(VERSION 3.0)

project(VisND)
add_compile_options(-std=c++14)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(UI_RESOURCES resouce.qrc)
set(UI_FORMS visualize_nd.ui)

# Find includes in corresponding build directories
set(CMAKE_INCLUDE_CURRENT_DIR ON)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules/")
find_package(PCL 1.7.2 REQUIRED)
find_package(OpenCV 3.4 REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Boost REQUIRED)
find_package(Qt5 REQUIRED Widgets)
find_package(Qt5Widgets CONFIG REQUIRED)
find_package(ZLIB)
find_package(VTK REQUIRED)

# Fix a compilation bug under ubuntu 16.04 (Xenial)
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")

include_directories(
    ${OpenCV_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${ZLIB_INCLUDE_DIRS}
)
add_definitions(${PCL_DEFINITIONS})
link_directories(${PCL_LIBRARY_DIRS})

# Instruct CMake to run moc automatically when needed
set(CMAKE_AUTOMOC ON) # For meta object compiler
set(CMAKE_AUTORCC ON) # Resource files
set(CMAKE_AUTOUIC ON) # UI files

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY
    ${CMAKE_HOME_DIRECTORY}/bin
    )

add_executable(${PROJECT_NAME} main.cpp visualize_nd.cpp planedialog.cpp ${UI_RESOURCES})
target_link_libraries(${PROJECT_NAME}
                     ${PCL_LIBRARIES}
                     ${OpenCV_LIBS}
                     Qt5::Widgets
                     ${ZLIB_LIBRARIES}
                     ${VTK_LIBRARIES}
                     )
