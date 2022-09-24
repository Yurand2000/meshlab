#filter_curvature_skeleton/filters/pruneExtend
set (SOURCES
	"PruneExtendFilter.cpp"
	"PruneExtendFilter_apply.cpp"
	"PruneExtendFilter_params.cpp"
)

set (HEADERS
	"PruneExtendFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
