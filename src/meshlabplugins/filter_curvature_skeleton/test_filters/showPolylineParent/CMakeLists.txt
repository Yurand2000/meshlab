#filter_curvature_skeleton/test_filters/showPolylineParent

set (SOURCES
	"ShowPolylineParentFilter.cpp"
	"ShowPolylineParentFilter_apply.cpp"
	"ShowPolylineParentFilter_params.cpp"
)

set (HEADERS
	"ShowPolylineParentFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
