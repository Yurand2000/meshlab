#filter_curvature_skeleton/test_filters/showMinRadius

set (SOURCES
	"ShowMinRadiusFilter.cpp"
	"ShowMinRadiusFilter_apply.cpp"
	"ShowMinRadiusFilter_params.cpp"
)

set (HEADERS
	"ShowMinRadiusFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
