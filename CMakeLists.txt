cmake_minimum_required(VERSION 3.5)
project(AKFSFSimulation)

set(CMAKE_CXX_STANDARD 11)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)

set(SRC_CPP
    src/main.cpp
    src/display.cpp
    src/beacons.cpp
    src/sensors.cpp
    src/simulation.cpp
    src/utils.cpp
    src/kalmanfilter.cpp
)

add_executable(${PROJECT_NAME}
        ${SRC_CPP}
        )

target_link_libraries(${PROJECT_NAME}
        SDL2
        SDL2_ttf
        Eigen3::Eigen
        )

add_custom_command(TARGET ${PROJECT_NAME} PRE_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_directory
    ${CMAKE_SOURCE_DIR}/data/ $<TARGET_FILE_DIR:${PROJECT_NAME}>)
    
    