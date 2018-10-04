cmake_minimum_required(VERSION 3.0)
project(loguru LANGUAGES CXX)

# Customize loguru version
set(version_url v1.8.0)

# Functon for load file from network with existence cheking
function(LoadDependency uri relpath)
    set(outmsg "Check ${relpath}...")
    if (EXISTS ${CMAKE_CURRENT_SOURCE_DIR}/${relpath})
        message(STATUS "${outmsg} Already exists")
    else()
        message(STATUS "${outmsg} Download ${relpath}")
        set(result_list "")
        set(code "")
        file(DOWNLOAD ${uri} ${CMAKE_CURRENT_SOURCE_DIR}/${relpath} STATUS result_list)
        list(GET result_list 0 code)
        if (${code})
            set(error "")
            list(GET result_list 1 error)
            message(SEND_ERROR "Error downloading ${relpath}: ${error}")
        else()
            message(STATUS "Done")
        endif()
    endif()
endfunction()
LoadDependency(https://raw.githubusercontent.com/emilk/loguru/${version_url}/loguru.hpp include/loguru.hpp)

# Create library
add_library(${PROJECT_NAME} STATIC src/loguru.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC include)
set_property(TARGET ${PROJECT_NAME} PROPERTY CXX_STANDARD 11)
if (UNIX)
    target_link_libraries(${PROJECT_NAME} PRIVATE dl pthread)
endif()

