#filter_curvature_skeleton/filters/polylineCutting
set (SOURCES
	"PolylineCuttingFilter.cpp"
	"PolylineCuttingFilter_apply.cpp"
	"PolylineCuttingFilter_params.cpp"
)

set (HEADERS
	"PolylineCuttingFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
