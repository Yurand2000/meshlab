#filter_curvature_skeleton/test_filters/refinePolyline

set (SOURCES
	"RefinePolylineFilter.cpp"
	"RefinePolylineFilter_apply.cpp"
	"RefinePolylineFilter_params.cpp"
)

set (HEADERS
	"RefinePolylineFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
