#filter_curvature_skeleton/test_filters/computeBranches

set (SOURCES
	"ComputeBranchesFilter.cpp"
	"ComputeBranchesFilter_apply.cpp"
	"ComputeBranchesFilter_params.cpp"
)

set (HEADERS
	"ComputeBranchesFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
