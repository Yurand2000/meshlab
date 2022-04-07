#filter_curvature_skeleton/filters/skeletonize
set (SOURCES
	"AlgorithmSkeletonize.cpp"
	"SkeletonizeFilter.cpp"
	"SkeletonizeFilter_apply.cpp"
	"SkeletonizeFilter_param_list.cpp"
)

set (HEADERS
	"AlgorithmSkeletonize.h"
	"SkeletonizeFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
