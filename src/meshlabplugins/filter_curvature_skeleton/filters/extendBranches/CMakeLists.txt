#filter_curvature_skeleton/filters/extendBranches
set (SOURCES
	"ExtendBranchesFilter.cpp"
	"ExtendBranchesFilter_apply.cpp"
	"ExtendBranchesFilter_params.cpp"
)

set (HEADERS
	"ExtendBranchesFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
