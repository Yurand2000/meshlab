#filter_curvature_skeleton/test_filters/cutOnPolylines

set (SOURCES
	"CutOnPolylinesFilter.cpp"
	"CutOnPolylinesFilter_apply.cpp"
	"CutOnPolylinesFilter_params.cpp"
)

set (HEADERS
	"CutOnPolylinesFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
