#filter_curvature_skeleton/simplifySkeleton
set (SOURCES
	"SimplifySkeleton.cpp"
)

set (HEADERS
	"SimplifySkeleton.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
