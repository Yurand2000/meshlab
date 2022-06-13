#filter_curvature_skeleton/filters/measuresCompute
set (SOURCES
	"MeasuresComputeFilter.cpp"
	"MeasuresComputeFilter_apply.cpp"
	"MeasuresComputeFilter_params.cpp"
)

set (HEADERS
	"MeasuresComputeFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
