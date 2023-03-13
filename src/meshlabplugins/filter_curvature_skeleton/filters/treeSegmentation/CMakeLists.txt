#filter_curvature_skeleton/filters/treeSegmentation
set (SOURCES
	"TreeSegmentationFilter.cpp"
	"TreeSegmentationFilter_apply.cpp"
	"TreeSegmentationFilter_params.cpp"
)

set (HEADERS
	"TreeSegmentationFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
