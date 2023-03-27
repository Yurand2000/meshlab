#filter_curvature_skeleton/filters/branchMeasure
set (SOURCES
	"BranchMeasureFilter.cpp"
	"BranchMeasureFilter_apply.cpp"
	"BranchMeasureFilter_params.cpp"
)

set (HEADERS
	"BranchMeasureFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
