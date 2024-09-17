cmake_minimum_required(VERSION 3.12)
project(Sofa.RigidBodyDynamics VERSION 23.12.99 LANGUAGES CXX)

find_package(Sofa.Simulation.Core REQUIRED)
find_package(Sofa.Component.StateContainer REQUIRED)
find_package(Sofa.Component.Mapping.NonLinear REQUIRED)
find_package(Sofa.Component.Mass REQUIRED)
find_package(Sofa.Component.Collision.Geometry REQUIRED)
find_package(Sofa.GL.Component.Rendering3D REQUIRED)

# List all files
set(SOFA_RBD_SRC_DIR src/sofa/RigidBodyDynamics)
set(HEADER_FILES
    ${SOFA_RBD_SRC_DIR}/config.h.in
    ${SOFA_RBD_SRC_DIR}/Conversions.h
    ${SOFA_RBD_SRC_DIR}/GeometryConversions.h
    ${SOFA_RBD_SRC_DIR}/KinematicChainMapping.h
    ${SOFA_RBD_SRC_DIR}/KinematicChainMapping.inl
    ${SOFA_RBD_SRC_DIR}/URDFModelLoader.h
    )
set(HEADER_FILES_TO_MOC
    )
set(SOURCE_FILES
    ${SOFA_RBD_SRC_DIR}/init.cpp
    ${SOFA_RBD_SRC_DIR}/GeometryConversions.cpp
    ${SOFA_RBD_SRC_DIR}/KinematicChainMapping.cpp
    ${SOFA_RBD_SRC_DIR}/URDFModelLoader.cpp
    )
set(README_FILES
    README.md
    )

find_package(pinocchio 3.0 REQUIRED)

add_library(${PROJECT_NAME} SHARED ${HEADER_FILES} ${SOURCE_FILES} ${README_FILES})

target_link_libraries(${PROJECT_NAME} 
  Sofa.Simulation.Core
  Sofa.Component.StateContainer
  Sofa.Component.Mapping.NonLinear
  Sofa.Component.Mass
  Sofa.Component.Collision.Geometry
  Sofa.GL.Component.Rendering3D
  pinocchio::pinocchio
  )

sofa_create_package_with_targets(
    PACKAGE_NAME ${PROJECT_NAME}
    PACKAGE_VERSION ${PROJECT_VERSION}
    TARGETS ${PROJECT_NAME} AUTO_SET_TARGET_PROPERTIES
    INCLUDE_SOURCE_DIR "src"
    INCLUDE_INSTALL_DIR ${PROJECT_NAME}
    RELOCATABLE "plugins"
    )

# Tests
# If SOFA_BUILD_TESTS exists and is OFF, then these tests will be auto-disabled
cmake_dependent_option(SOFA_RIGIDBODYDYNAMICS_BUILD_TESTS "Compile the automatic tests" OFF "SOFA_BUILD_TESTS OR NOT DEFINED SOFA_BUILD_TESTS" OFF)
if(SOFA_RIGIDBODYDYNAMICS_BUILD_TESTS)
    enable_testing()
    add_subdirectory(tests)
endif()
