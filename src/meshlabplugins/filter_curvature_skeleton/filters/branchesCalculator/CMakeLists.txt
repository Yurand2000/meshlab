#filter_curvature_skeleton/filters/branchesCalculator
set (SOURCES
	"BranchesCalculatorFilter.cpp"
	"BranchesCalculatorFilter_apply.cpp"
	"BranchesCalculatorFilter_params.cpp"
)

set (HEADERS
	"BranchesCalculatorFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
