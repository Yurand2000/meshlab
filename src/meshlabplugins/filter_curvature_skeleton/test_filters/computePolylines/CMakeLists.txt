#filter_curvature_skeleton/test_filters/computePolylines

set (SOURCES
	"ComputePolylinesFilter.cpp"
	"ComputePolylinesFilter_apply.cpp"
	"ComputePolylinesFilter_params.cpp"
)

set (HEADERS
	"ComputePolylinesFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
