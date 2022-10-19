#filter_curvature_skeleton/test_filters
set (SOURCES
)

set (HEADERS
	"testFilters.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})

add_subdirectory(findPath)
add_subdirectory(findPathFill)
add_subdirectory(computeBranches)
add_subdirectory(computePolylines)
add_subdirectory(extractPolyline)
add_subdirectory(refinePolyline)
