SET(LIB_NAME ToolCommon)

SET(SOURCE_FILES
	${KLAYGE_PROJECT_DIR}/Tools/src/Common/ImagePlane.cpp
	${KLAYGE_PROJECT_DIR}/Tools/src/Common/TexConverter.cpp
	${KLAYGE_PROJECT_DIR}/Tools/src/Common/TexMetadata.cpp
	${KLAYGE_PROJECT_DIR}/Tools/src/Common/ToolCommon.cpp
)

SET(HEADER_FILES
	${KLAYGE_PROJECT_DIR}/Tools/Include/KlayGE/TexConverter.hpp
	${KLAYGE_PROJECT_DIR}/Tools/Include/KlayGE/TexMetadata.hpp
	${KLAYGE_PROJECT_DIR}/Tools/Include/KlayGE/ToolCommon.hpp
	${KLAYGE_PROJECT_DIR}/Tools/src/Common/ImagePlane.hpp
)

SOURCE_GROUP("Source Files" FILES ${SOURCE_FILES})
SOURCE_GROUP("Header Files" FILES ${HEADER_FILES})

INCLUDE_DIRECTORIES(${Boost_INCLUDE_DIR})
INCLUDE_DIRECTORIES(${KLAYGE_PROJECT_DIR}/../External/FreeImage/Source)
INCLUDE_DIRECTORIES(${KLAYGE_PROJECT_DIR}/../External/rapidjson/include)
INCLUDE_DIRECTORIES(${KLAYGE_PROJECT_DIR}/../KFL/include)
INCLUDE_DIRECTORIES(${KLAYGE_PROJECT_DIR}/Core/Include)
INCLUDE_DIRECTORIES(${KLAYGE_PROJECT_DIR}/Tools/Include)
LINK_DIRECTORIES(${KLAYGE_PROJECT_DIR}/../External/FreeImage/lib/${KLAYGE_PLATFORM_NAME})
LINK_DIRECTORIES(${KLAYGE_PROJECT_DIR}/../KFL/lib/${KLAYGE_PLATFORM_NAME})
IF(KLAYGE_PLATFORM_DARWIN OR KLAYGE_PLATFORM_LINUX)
	LINK_DIRECTORIES(${KLAYGE_BIN_DIR})
ELSE()
	LINK_DIRECTORIES(${KLAYGE_OUTPUT_DIR})
ENDIF()

ADD_LIBRARY(${LIB_NAME} SHARED
	${SOURCE_FILES} ${HEADER_FILES})

SET(LIB_OUTPUT_NAME ${LIB_NAME}${KLAYGE_OUTPUT_SUFFIX})

SET_TARGET_PROPERTIES(${LIB_NAME} PROPERTIES
	ARCHIVE_OUTPUT_DIRECTORY ${KLAYGE_TOOLS_LIB_OUTPUT_DIR}
	ARCHIVE_OUTPUT_DIRECTORY_DEBUG ${KLAYGE_TOOLS_LIB_OUTPUT_DIR}
	ARCHIVE_OUTPUT_DIRECTORY_RELEASE ${KLAYGE_TOOLS_LIB_OUTPUT_DIR}
	ARCHIVE_OUTPUT_DIRECTORY_RELWITHDEBINFO ${KLAYGE_TOOLS_LIB_OUTPUT_DIR}
	ARCHIVE_OUTPUT_DIRECTORY_MINSIZEREL ${KLAYGE_TOOLS_LIB_OUTPUT_DIR}
	PROJECT_LABEL ${LIB_NAME}
	DEBUG_POSTFIX ${CMAKE_DEBUG_POSTFIX}
	OUTPUT_NAME ${LIB_OUTPUT_NAME}
)
IF(KLAYGE_PLATFORM_DARWIN OR KLAYGE_PLATFORM_LINUX)
	SET_TARGET_PROPERTIES(${LIB_NAME} PROPERTIES
		LIBRARY_OUTPUT_DIRECTORY ${KLAYGE_BIN_DIR}
		LIBRARY_OUTPUT_DIRECTORY_DEBUG ${KLAYGE_BIN_DIR}
		LIBRARY_OUTPUT_DIRECTORY_RELEASE ${KLAYGE_BIN_DIR}
		LIBRARY_OUTPUT_DIRECTORY_RELWITHDEBINFO ${KLAYGE_BIN_DIR}
		LIBRARY_OUTPUT_DIRECTORY_MINSIZEREL ${KLAYGE_BIN_DIR})
ENDIF()

ADD_PRECOMPILED_HEADER(${LIB_NAME} "KlayGE/KlayGE.hpp" "${KLAYGE_PROJECT_DIR}/Core/Include"
	"${KLAYGE_PROJECT_DIR}/Tools/src/Common/ToolCommon.cpp")

SET(EXTRA_LINKED_LIBRARIES ${EXTRA_LINKED_LIBRARIES}
	debug FreeImage${KLAYGE_OUTPUT_SUFFIX}${CMAKE_DEBUG_POSTFIX} optimized FreeImage${KLAYGE_OUTPUT_SUFFIX}
	debug KlayGE_Core${KLAYGE_OUTPUT_SUFFIX}${CMAKE_DEBUG_POSTFIX} optimized KlayGE_Core${KLAYGE_OUTPUT_SUFFIX}
	debug KFL${KLAYGE_OUTPUT_SUFFIX}${CMAKE_DEBUG_POSTFIX} optimized KFL${KLAYGE_OUTPUT_SUFFIX}
	${KLAYGE_FILESYSTEM_LIBRARY})
IF(KLAYGE_PLATFORM_LINUX)
	SET(EXTRA_LINKED_LIBRARIES ${EXTRA_LINKED_LIBRARIES} dl pthread)
ENDIF()
TARGET_LINK_LIBRARIES(${LIB_NAME} ${EXTRA_LINKED_LIBRARIES})
ADD_DEPENDENCIES(${LIB_NAME} AllInEngine)

IF(KLAYGE_PLATFORM_WINDOWS)
	ADD_POST_BUILD(${LIB_NAME} "")

	INSTALL(TARGETS ${LIB_NAME}
		RUNTIME DESTINATION ${KLAYGE_BIN_DIR}
		LIBRARY DESTINATION ${KLAYGE_BIN_DIR}
		ARCHIVE DESTINATION ${KLAYGE_TOOLS_LIB_OUTPUT_DIR}
	)
ENDIF()

SET_TARGET_PROPERTIES(${LIB_NAME} PROPERTIES FOLDER "Tools")
