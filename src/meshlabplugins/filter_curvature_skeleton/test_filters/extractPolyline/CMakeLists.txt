#filter_curvature_skeleton/test_filters/extractPolyline

set (SOURCES
	"ExtractPolylineFilter.cpp"
	"ExtractPolylineFilter_apply.cpp"
	"ExtractPolylineFilter_params.cpp"
)

set (HEADERS
	"ExtractPolylineFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
