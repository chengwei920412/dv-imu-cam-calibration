cmake_minimum_required(VERSION 2.8.3)
project(suitesparse)

set(CMAKE_BUILD_TYPE Release)

include(ExternalProject)

set(VERSION 4.4.7)

ExternalProject_Add(suitesparse_src
  CMAKE_ARGS -DCMAKE_C_COMPILER=${CMAKE_C_COMPILER} -DCMAKE_CXX_COMPILER=${CMAKE_CXX_COMPILER}
  DOWNLOAD_COMMAND rm -f SuiteSparse-${VERSION}.tar.gz && wget http://faculty.cse.tamu.edu/davis/SuiteSparse/SuiteSparse-${VERSION}.tar.gz
  PATCH_COMMAND tar -xzf ../SuiteSparse-${VERSION}.tar.gz && rm -rf ../suitesparse_src-build/SuiteSparse && sed -i.bu 's,/usr/local/lib,${PROJECT_BINARY_DIR}/lib/,g' SuiteSparse/SuiteSparse_config/SuiteSparse_config.mk && sed -i.bu 's,/usr/local/include,${PROJECT_BINARY_DIR}/include/,g' SuiteSparse/SuiteSparse_config/SuiteSparse_config.mk && sed -i.bu 's,/usr/local/cuda,/nocuda,g' SuiteSparse/SuiteSparse_config/SuiteSparse_config.mk && mv SuiteSparse ../suitesparse_src-build/
  CONFIGURE_COMMAND ""
  BUILD_COMMAND cd SuiteSparse && make library -j8 -l8
  INSTALL_COMMAND cd SuiteSparse && mkdir -p lib && mkdir -p ${PROJECT_BINARY_DIR}/lib/ && mkdir -p ${PROJECT_BINARY_DIR}/include/ && mkdir -p include && make install
)

include_directories(${PROJECT_BINARY_DIR}/include/suitesparse/)

add_library(${PROJECT_NAME} src/export_lib_hack.cc)
target_link_libraries(${PROJECT_NAME} 
  ${PROJECT_BINARY_DIR}/lib/libamd.so
  ${PROJECT_BINARY_DIR}/lib/libbtf.so
  ${PROJECT_BINARY_DIR}/lib/libcamd.so
  ${PROJECT_BINARY_DIR}/lib/libccolamd.so
  ${PROJECT_BINARY_DIR}/lib/libcholmod.so
  ${PROJECT_BINARY_DIR}/lib/libcolamd.so
  ${PROJECT_BINARY_DIR}/lib/libcxsparse.so
  ${PROJECT_BINARY_DIR}/lib/libklu.so
  ${PROJECT_BINARY_DIR}/lib/libldl.so
  ${PROJECT_BINARY_DIR}/lib/librbio.so
  ${PROJECT_BINARY_DIR}/lib/libspqr.so
  ${PROJECT_BINARY_DIR}/lib/libsuitesparseconfig.so
  ${PROJECT_BINARY_DIR}/lib/libumfpack.so
  )
add_dependencies(${PROJECT_NAME} suitesparse_src)
target_include_directories(${PROJECT_NAME} PUBLIC
        include
        ${PROJECT_BINARY_DIR}/include)

install(DIRECTORY include
  DESTINATION include
)
install(DIRECTORY lib
  DESTINATION lib
)
install(DIRECTORY share
  DESTINATION lib
)

