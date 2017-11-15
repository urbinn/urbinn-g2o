CMAKE_MINIMUM_REQUIRED(VERSION 2.9)

# Compiler
SET(CMAKE_CXX_STANDARD 11)
SET(CMAKE_CXX_STANDARD_REQUIRED ON)

SET(LIBRARY_NAME "g2o")
SET(SOURCES "src/g2o.cpp")

LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules)
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR})

FIND_PACKAGE(Boost REQUIRED COMPONENTS python-py35)
FIND_PACKAGE(PythonLibs 3 REQUIRED)
FIND_PACKAGE(g2o REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(CSparse REQUIRED)
FIND_PACKAGE(Cholmod REQUIRED)

ADD_LIBRARY(${LIBRARY_NAME} SHARED ${SOURCES})
SET_TARGET_PROPERTIES(${LIBRARY_NAME} PROPERTIES PREFIX "")

INCLUDE_DIRECTORIES(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${G2O_INCLUDE_DIR}
    ${CSPARSE_INCLUDE_DIR}
    ${Cholmod_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${Boost_INCLUDE_DIRS}
    ${PYTHON_INCLUDE_DIRS}
)

LIST(APPEND G2O_LIBS
    cxsparse
    cholmod
    g2o_cli g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d
    g2o_core g2o_solver_csparse g2o_solver_structure_only g2o_types_sba
    g2o_types_slam3d g2o_csparse_extension g2o_solver_dense g2o_stuff 
    g2o_types_sclam2d g2o_solver_pcg g2o_types_data g2o_types_sim3
)

target_link_libraries(
    ${LIBRARY_NAME}
    ${G2O_LIBS}
    ${Boost_LIBRARIES}
    ${PYTHON_LIBRARIES}
)
