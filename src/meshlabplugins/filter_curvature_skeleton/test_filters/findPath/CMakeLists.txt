#filter_curvature_skeleton/test_filters/findPath

set (SOURCES
	"FindPathFilter.cpp"
	"FindPathFilter_apply.cpp"
	"FindPathFilter_params.cpp"
)

set (HEADERS
	"FindPathFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
