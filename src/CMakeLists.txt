cmake_minimum_required(VERSION 3.5)

project(madcar-ai LANGUAGES CXX)

# Chipmunk
add_library(chipmunk INTERFACE)
target_link_libraries(chipmunk INTERFACE ${CMAKE_CURRENT_SOURCE_DIR}/chipmunk_src/bin/libchipmunk.a)
target_include_directories(chipmunk INTERFACE chipmunk_src/include)

# Strategy
set(Sources
    solution/main.cpp
    solution/structures.cpp
    solution/common/vis_debug.cpp
    solution/logic/strategy.cpp
    solution/logic/montecarlo.cpp
    solution/logic/evaluator.cpp
    solution/simulation/cp_helpers.cpp
    solution/simulation/simulator.cpp)

add_compile_options(-Wall -Wextra -Wshadow -Wnon-virtual-dtor -Werror=return-type)
set(CMAKE_CXX_STANDARD 17)

add_executable(${PROJECT_NAME} ${Sources})
target_link_libraries(${PROJECT_NAME} csimplesocket loguru nljson chipmunk)
set_property(TARGET ${PROJECT_NAME} PROPERTY RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})

target_compile_definitions(${PROJECT_NAME} PRIVATE
    LOCAL_RUN
    ENABLE_LOG
#    ENABLE_VISUALISER
    )

if (CMAKE_BUILD_TYPE MATCHES "Debug")
    target_compile_definitions(${PROJECT_NAME} PRIVATE LOCAL_RUN
        ENABLE_LOG
        ENABLE_VISUALISER
    )
endif()