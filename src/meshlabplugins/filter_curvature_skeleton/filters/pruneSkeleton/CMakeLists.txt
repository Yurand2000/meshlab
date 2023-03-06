#filter_curvature_skeleton/filters/pruneSkeleton
set (SOURCES
	"PruneSkeletonFilter.cpp"
	"PruneSkeletonFilter_apply.cpp"
	"PruneSkeletonFilter_params.cpp"
)

set (HEADERS
	"PruneSkeletonFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
