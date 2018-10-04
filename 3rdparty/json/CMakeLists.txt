cmake_minimum_required(VERSION 3.0)

project(nljson)

set(VERSION v3.2.0)

include(utils)
LoadDependency(https://github.com/nlohmann/json/releases/download/${VERSION}/json.hpp include/json.hpp)

add_library(${PROJECT_NAME} INTERFACE)
target_include_directories(${PROJECT_NAME} INTERFACE ${PROJECT_SOURCE_DIR}/include)
