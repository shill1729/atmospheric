cmake_minimum_required(VERSION 3.20)
project(atmospheric LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(SFML 3 COMPONENTS Graphics Window System REQUIRED)
find_package(Eigen3 REQUIRED NO_MODULE)

add_executable(atmospheric
    src/adjoint/AdjointSolver.cpp
    src/adjoint/SourceEstimator.cpp
    src/app/Application.cpp
    src/app/ApplicationEvents.cpp
    src/app/ApplicationRender.cpp
    src/core/RuntimeSettings.cpp
    src/core/Validation.cpp
    src/numerics/AdvectionDiffusionSolver.cpp
    src/numerics/ParticleSystem.cpp
    src/science/Fields.cpp
    src/science/SourceModel.cpp
    src/sim/SensorManager.cpp
    src/sim/SimulationController.cpp
    src/sim/Simulator.cpp
    src/ui/MenuModel.cpp
    src/ui/TopToolbar.cpp
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

include(CTest)
if (BUILD_TESTING)
    add_executable(atmospheric_self_check
        tests/self_check.cpp
        src/core/Validation.cpp
        src/numerics/AdvectionDiffusionSolver.cpp
        src/numerics/ParticleSystem.cpp
        src/science/Fields.cpp
        src/science/SourceModel.cpp
        src/sim/SensorManager.cpp
        src/sim/Simulator.cpp
    )

    target_include_directories(atmospheric_self_check
        PRIVATE
            include
    )

    target_link_libraries(atmospheric_self_check
        PRIVATE
            Eigen3::Eigen
    )

    target_compile_options(atmospheric_self_check PRIVATE
        $<$<CXX_COMPILER_ID:MSVC>:/W4 /permissive->
        $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>
    )

    add_test(NAME atmospheric_self_check COMMAND atmospheric_self_check)
endif()
