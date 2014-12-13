cmake_minimum_required(VERSION 2.8)
set (PROJECT CLSFProcessor) 
ENABLE_TESTING()
#SET(CMAKE_BUILD_TYPE "Release")

project (${PROJECT}) 
add_definitions(-DICLSFPROCESSOR_EXPORTS)
set (HEADERS include/ICLSFProcessor.h)

include_directories (include) 
include_directories ($ENV{LIBS_ROOT}/pugixml/src) 
include_directories ($ENV{LIBS_ROOT}/eigen) 

find_package( Boost )

set (Boost_USE_STATIC_LIBS OFF)
set (Boost_USE_MULTITHREADED TRUE)

link_directories (${Boost_LIBRARY_DIRS})
include_directories (${Boost_INCLUDE_DIRS})

set (SOURCES
	$ENV{LIBS_ROOT}/pugixml/src/pugixml.cpp
	source/dllmain.cpp
	source/ICLSFProcessor.cpp
	source/CLSFParser/ExpressionCompiler.cpp
	source/CLSFParser/CLSFNodes.cpp
	source/CLSFParser/ExpressionParser.cpp) 
 
#set(LIBRARY_OUTPUT_PATH library)
add_library (${PROJECT} SHARED ${HEADERS} ${SOURCES})

set (PROJECT CLSFProcessorTest) 

#SET(CMAKE_BUILD_TYPE "Release")

project (${PROJECT}) 

IF(WIN32)
set(CMAKE_FIND_LIBRARY_PREFIXES "lib")
ENDIF(WIN32)

set (HEADERS include/ICLSFProcessor.h)

include_directories (include) 
include_directories ($ENV{LIBS_ROOT}/pugixml/src) 
include_directories ($ENV{LIBS_ROOT}/eigen) 

set (Boost_USE_STATIC_LIBS OFF)
set (Boost_USE_MULTITHREADED TRUE)

#find_package (Boost COMPONENTS unit_test_framework REQUIRED)

link_directories (${Boost_LIBRARY_DIRS})
include_directories (${Boost_INCLUDE_DIRS})

set (SOURCES
	$ENV{LIBS_ROOT}/pugixml/src/pugixml.cpp
	test/CLSFProcessorTest.cpp) 
	
add_executable (${PROJECT} ${HEADERS} ${SOURCES})
target_link_libraries (${PROJECT} CLSFProcessor ${Boost_LIBRARIES})
add_test(${PROJECT} ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT} --log_level=all --report_format=XML --report_level=detailed)
 