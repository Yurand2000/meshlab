#filter_curvature_skeleton/filters/orderCompute
set (SOURCES
	"OrderComputeFilter.cpp"
	"OrderComputeFilter_apply.cpp"
	"OrderComputeFilter_params.cpp"
)

set (HEADERS
	"OrderComputeFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
