#filter_curvature_skeleton/filters/skeletonDistanceToQuality
set (SOURCES
	"DistanceToQualityFilter.cpp"
)

set (HEADERS
	"DistanceToQualityFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
