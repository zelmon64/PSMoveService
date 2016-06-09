cmake_minimum_required(VERSION 3.0)

set(ROOT_DIR ${CMAKE_CURRENT_LIST_DIR}/../..)
set(FREEPIE_PLUGIN_INCL_DIRS)
set(FREEPIE_PLUGIN_REQ_LIBS)

# platform independent include paths
list(APPEND FREEPIE_PLUGIN_INCL_DIRS
    ${ROOT_DIR}/src/psmoveclient/
    ${PROTOBUF_INCLUDE_DIRS})

#freePIE
FIND_PACKAGE(FreePIE REQUIRED)
list(APPEND FREEPIE_PLUGIN_INCL_DIRS ${FREEPIE_INCLUDE_DIR})
list(APPEND FREEPIE_PLUGIN_REQ_LIBS ${FREEPIE_LIBRARIES})

# platform independent libraries
list(APPEND FREEPIE_PLUGIN_REQ_LIBS
    PSMoveClient)

# Source files that are needed for the shared library
file(GLOB FREEPIE_PLUGIN_LIBRARY_SRC
    "${CMAKE_CURRENT_LIST_DIR}/*.h"
    "${CMAKE_CURRENT_LIST_DIR}/*.cpp"
)

# Shared library
add_library(fp_driver_psmoveservice SHARED ${FREEPIE_PLUGIN_LIBRARY_SRC})
target_include_directories(fp_driver_psmoveservice PUBLIC ${FREEPIE_PLUGIN_INCL_DIRS})
target_link_libraries(fp_driver_psmoveservice ${FREEPIE_PLUGIN_REQ_LIBS})

# Install
IF(${CMAKE_SYSTEM_NAME} MATCHES "Windows")
install(TARGETS fp_driver_psmoveservice
	RUNTIME DESTINATION ${ROOT_DIR}/win32/bin
	LIBRARY DESTINATION ${ROOT_DIR}/win32/lib
	ARCHIVE DESTINATION ${ROOT_DIR}/win32/lib)
ELSE() #Linux/Darwin
ENDIF()
