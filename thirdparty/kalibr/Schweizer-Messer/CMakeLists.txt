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

include_directories(sm_timing/include)
add_subdirectory(sm_timing)
add_dependencies(sm_timing sm_common sm_random)

include_directories(sm_logging/include)
add_subdirectory(sm_logging)

include_directories(sm_property_tree/include)
add_subdirectory(sm_property_tree)
add_dependencies(sm_property_tree sm_common)