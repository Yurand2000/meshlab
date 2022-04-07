#filter_curvature_skeleton/branch_extender
set (SOURCES
	"BranchExtender.cpp"
)

set (HEADERS
	"BranchExtender.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
