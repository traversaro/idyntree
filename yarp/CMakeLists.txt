# Copyright: (C) 2015 Fondazione Istituto Italiano di Tecnologia
# Authors: Silvio Traversaro
# CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT

project(iDynTree_YARP CXX)

SET(iDynTree_YARP_source src/DynTree.cpp
                         src/yarp_kdl.cpp)

SET(iDynTree_YARP_header include/iCub/iDynTree/DynTree.h
                  include/iCub/iDynTree/yarp_kdl.h)

SOURCE_GROUP("Source Files" FILES ${iDynTree_YARP_source})
SOURCE_GROUP("Header Files" FILES ${iDynTree_YARP_header})

# share headers with all iDynTree targes
set_property(GLOBAL APPEND PROPERTY IDYNTREE_TREE_INCLUDE_DIRS "${PROJECT_SOURCE_DIR}/include")
get_property(IDYNTREE_TREE_INCLUDE_DIRS GLOBAL PROPERTY IDYNTREE_TREE_INCLUDE_DIRS)

include_directories(${IDYNTREE_TREE_INCLUDE_DIRS})
include_directories(${orocos_kdl_INCLUDE_DIRS})

include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/bin"
                          LIB_DIRS "${CMAKE_INSTALL_PREFIX}/lib"
                          DEPENDS IDYNTREE_ENABLE_RPATH
                          USE_LINK_PATH)


add_library(idyntree-yarp ${iDynTree_YARP_source} ${iDynTree_YARP_header})

set_target_properties(idyntree-yarp PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                           SOVERSION ${${VARS_PREFIX}_VERSION}
                                           PUBLIC_HEADER "${iDynTree_YARP_header}")


include_directories(include/iCub/iDynTree)

target_include_directories(idyntree-yarp PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
                                           "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>"
                                            ${GSL_INCLUDE_DIRS}
                                            ${YARP_INCLUDE_DIRS}
                                            ${EIGEN3_INCLUDE_DIR})


target_link_libraries(idyntree-yarp LINK_PUBLIC
                                            ${GSL_LIBRARIES}
                                            ${YARP_LIBRARIES}
                                            idyntree-core
                                            idyntree-model-io
                                            ${Boost_LIBRARIES}
                                            ${TinyXML_LIBRARIES})


install(TARGETS idyntree-yarp
        EXPORT iDynTree
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}/iCub/iDynTree" COMPONENT dev)

set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS idyntree-yarp)


if(IDYNTREE_USE_ICUB_MAIN)
    SET(iDynTree_ICUB_source src/TorqueEstimationTree.cpp
                             src/iDyn2KDL.cpp
                             src/idyn2kdl_icub.cpp)

    SET(iDynTree_ICUB_header include/iCub/iDynTree/iDyn2KDL.h
                             include/iCub/iDynTree/TorqueEstimationTree.h
                             include/iCub/iDynTree/idyn2kdl_icub.h)

    SOURCE_GROUP("Source Files" FILES ${iDynTree_ICUB_source})
    SOURCE_GROUP("Header Files" FILES ${iDynTree_ICUB_header})

    add_library(idyntree-icub ${iDynTree_ICUB_source} ${iDynTree_ICUB_header})

    set_target_properties(idyntree-icub PROPERTIES VERSION ${${VARS_PREFIX}_VERSION}
                                           SOVERSION ${${VARS_PREFIX}_VERSION}
                                           PUBLIC_HEADER "${iDynTree_ICUB_header}")

    target_include_directories(idyntree-icub PUBLIC "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
                                           "$<INSTALL_INTERFACE:${CMAKE_INSTALL_PREFIX}/${${VARS_PREFIX}_INSTALL_INCLUDEDIR}>"
                                            ${skinDynLib_INCLUDE_DIRS}
                                            ${iDyn_INCLUDE_DIRS}
                                            ${GSL_INCLUDE_DIRS}
                                            ${YARP_INCLUDE_DIRS}
                                            ${EIGEN3_INCLUDE_DIR})


     target_link_libraries(idyntree-icub LINK_PUBLIC  skinDynLib iKin iDyn
                                            ${GSL_LIBRARIES}
                                            ${YARP_LIBRARIES}
                                            idyntree-core
                                            idyntree-model-io
                                            ${Boost_LIBRARIES}
                                            ${TinyXML_LIBRARIES})


      install(TARGETS idyntree-icub
        EXPORT iDynTree
        RUNTIME DESTINATION "${${VARS_PREFIX}_INSTALL_BINDIR}" COMPONENT bin
        LIBRARY DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT shlib
        ARCHIVE DESTINATION "${${VARS_PREFIX}_INSTALL_LIBDIR}" COMPONENT lib
        PUBLIC_HEADER DESTINATION "${${VARS_PREFIX}_INSTALL_INCLUDEDIR}/iCub/iDynTree" COMPONENT dev)

      set_property(GLOBAL APPEND PROPERTY ${VARS_PREFIX}_TARGETS idyntree-icub)

endif()


if(IDYNTREE_COMPILE_TESTS)
    add_subdirectory(tests)
endif(IDYNTREE_COMPILE_TESTS)
