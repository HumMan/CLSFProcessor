cmake_minimum_required(VERSION 2.8)

set (PROJECT CLSFProcessor) 
ENABLE_TESTING()
#SET(CMAKE_BUILD_TYPE "Release")

project (${PROJECT}) 

include_directories (include) 
include_directories (submodules/pugixml/src) 
include_directories (submodules/eigen) 

file(GLOB_RECURSE ${PROJECT_NAME}_SRC
    "Source/*.h"
    "Source/*.cpp"
	"submodules/pugixml/src/pugixml.cpp"
)

add_executable (${PROJECT} ${${PROJECT_NAME}_SRC})