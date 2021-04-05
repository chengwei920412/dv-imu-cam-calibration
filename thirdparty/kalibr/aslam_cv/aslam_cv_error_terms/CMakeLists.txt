cmake_minimum_required(VERSION 2.8.3)
project(aslam_cv_error_terms)

find_package(Boost REQUIRED COMPONENTS system )

add_library(${PROJECT_NAME} SHARED
        src/empty.cpp
        )

target_link_libraries(${PROJECT_NAME}
        ${Boost_LIBRARIES}
        aslam_backend_expressions
        )

target_include_directories(${PROJECT_NAME} PUBLIC
        include)
