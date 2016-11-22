# cmake requirements
cmake_minimum_required(VERSION 2.8 )

# Build options have to be before PROJECT(...)
SET(CMAKE_CONFIGURATION_TYPES "Debug;Release" CACHE PATH "Configuration types")
SET(CMAKE_BUILD_TYPE "Release" CACHE PATH "Current build configuration")

# PUTSLAM Project configuration
PROJECT(Inzynierka)
SET(CMAKE_CXX_FLAGS "-std=c++11 -pthread")

# Folders
SET_PROPERTY(GLOBAL PROPERTY USE_FOLDERS ON)

# Settings
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include")
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/include/Defs")


#additional dependencies
LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake_modules)

# Boost
FIND_PACKAGE(Boost COMPONENTS system filesystem thread)

# OpenCV
FIND_PACKAGE(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )

# PCL
set(Boost_USE_STATIC_LIBS ON)
set(Boost_USE_STATIC ON)
set(PCL_DIR "/home/szymon/Inzyniera v1/pcl/build")
FIND_PACKAGE(PCL 1.8 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

#OpenNI
INCLUDE_DIRECTORIES("${CMAKE_SOURCE_DIR}/3rdParty/OpenNI2")

#Octomap
FIND_PACKAGE(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_directories(${OCTOMAP_LIBRARY_DIRS})
add_definitions(${OCTOMAP_DEFINITIONS})

#Eigen
#set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/3rdParty/Eigen)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
LINK_LIBRARIES(${EIGEN3_LIBRARY_DIRS})
add_definitions(-Wno-deprecated -DEIGEN_DONT_VECTORIZE -DEIGEN_DISABLE_UNALIGNED_ARRAY_ASSERT)

# Executable output directory
SET(RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Executable output directory")
mark_as_advanced(RUNTIME_OUTPUT_DIRECTORY)

# Dynamic library output directory
SET(LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/bin${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Dynamic library output directory")
mark_as_advanced(LIBRARY_OUTPUT_DIRECTORY)

# Static library output directory
SET(ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/build/lib${OUTPUT_DIRECTORY_POSTFIX} CACHE PATH "Static library output directory")
mark_as_advanced(ARCHIVE_OUTPUT_DIRECTORY)

###############################################################################
#
# PUTSLAM tinyXML2 library
#
###############################################################################

add_library(tinyxml2 SHARED ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.h ${CMAKE_CURRENT_SOURCE_DIR}/3rdParty/tinyXML/tinyxml2.cpp)



SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${RUNTIME_OUTPUT_DIRECTORY})
SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${LIBRARY_OUTPUT_DIRECTORY})
SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${ARCHIVE_OUTPUT_DIRECTORY})
foreach(CONFIGURATION_TYPE ${CMAKE_CONFIGURATION_TYPES})
	string(TOUPPER ${CONFIGURATION_TYPE} CONFIGURATION_TYPE)
	SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${RUNTIME_OUTPUT_DIRECTORY})
	SET(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${LIBRARY_OUTPUT_DIRECTORY})
	SET(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CONFIGURATION_TYPE} ${ARCHIVE_OUTPUT_DIRECTORY})
endforeach(CONFIGURATION_TYPE CMAKE_CONFIGURATION_TYPES)

if (MSVC)
  SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /EHsc")
endif()


#if(BUILD_Inzynierka)
        SET(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-rpath ./")
       # SET(DEMO_SOURCES ./simple_example.cpp)
        ADD_EXECUTABLE(importPcl importPcl.cpp)
        TARGET_LINK_LIBRARIES(importPcl octomap octomath octovis boost_system ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
        add_executable(pcl2tree pcl2tree.cpp)
        TARGET_LINK_LIBRARIES(pcl2tree octomap octomath octovis boost_system ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
	add_executable(concatenate concatenate.cpp)
	target_link_libraries(concatenate octomap octomath octovis boost_system ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
        add_executable(visualizer visualizer.cpp)
        target_link_libraries(visualizer octomap octomath octovis boost_system ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES})
        add_executable(ICP ICP.cpp)
        target_link_libraries(ICP octomap octomath octovis boost_system ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES})
        add_executable(umeyama umeyama.cpp)
        target_link_libraries(umeyama octomap octomath octovis boost_system ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES})
        add_executable(graber graber.cpp)
        target_link_libraries(graber octomap tinyxml2 octomath octovis boost_system  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${OpenCV_LIBS} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})
        add_executable(mnozenie mnozenie.cpp)
        target_link_libraries(mnozenie octomap tinyxml2 octomath octovis boost_system  ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${OpenCV_LIBS} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})

        add_executable(depth2pcd depth2pcd.cpp)
        target_link_libraries(depth2pcd  octomap tinyxml2 octomath octovis boost_system ${PCL_LIBRARIES} ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${OpenCV_LIBS} ${Boost_FILESYSTEM_LIBRARY} ${Boost_SYSTEM_LIBRARY})



           #ten target link łączy dopiero bilioteke z plikiem cpp, musi sie zgadzac z inkludami

        #add_dependencies(octomap_axample octomap octomath octovis boost_system ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
        #TARGET_LINK_LIBRARIES(octomap_axample octomap octomath octovis boost_system ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES})
        #INSTALL(TARGETS octomap_axample RUNTIME DESTINATION bin)

#endif(BUILD_Inzynierka)

