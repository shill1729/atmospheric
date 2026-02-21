cmake_minimum_required(VERSION 3.20)
project(atmospheric LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(SFML 3 COMPONENTS Graphics Window System REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(atmospheric
    src/app/Application.cpp
    src/numerics/AdvectionDiffusionSolver.cpp
    src/numerics/ParticleSystem.cpp
    src/science/Fields.cpp
    src/science/SourceModel.cpp
    src/sim/Simulator.cpp
    src/main.cpp
)

target_include_directories(atmospheric
    PRIVATE
        include
)

target_link_libraries(atmospheric
    PRIVATE
        SFML::Graphics
        SFML::Window
        SFML::System
        Eigen3::Eigen
)

target_compile_options(atmospheric PRIVATE
    $<$<CXX_COMPILER_ID:MSVC>:/W4 /permissive->
    $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>
)
