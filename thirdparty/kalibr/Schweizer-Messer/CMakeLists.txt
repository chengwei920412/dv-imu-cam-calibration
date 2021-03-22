include_directories(sm_boost/include)
add_subdirectory(sm_boost)
add_dependencies(sm_boost sm_common)

include_directories(sm_common/include)
add_subdirectory(sm_common)

include_directories(sm_random/include)
add_subdirectory(sm_random)

include_directories(sm_eigen/include)
add_subdirectory(sm_eigen)
add_dependencies(sm_eigen sm_common sm_random)