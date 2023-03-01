#filter_curvature_skeleton/test_filters/measureBranch

set (SOURCES
	"MeasureBranchFilter.cpp"
	"MeasureBranchFilter_apply.cpp"
	"MeasureBranchFilter_params.cpp"
)

set (HEADERS
	"MeasureBranchFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
