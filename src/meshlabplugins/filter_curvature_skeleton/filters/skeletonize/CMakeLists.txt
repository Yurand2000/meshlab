#filter_curvature_skeleton/filters/skeletonize
set (SOURCES
	"SkeletonizeFilter.cpp"
	"SkeletonizeFilter_apply.cpp"
	"SkeletonizeFilter_param_list.cpp"
)

set (HEADERS
	"SkeletonizeFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
