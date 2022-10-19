#filter_curvature_skeleton/test_filters/findPathFill

set (SOURCES
	"FindPathFillFilter.cpp"
	"FindPathFillFilter_apply.cpp"
	"FindPathFillFilter_params.cpp"
)

set (HEADERS
	"FindPathFillFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
