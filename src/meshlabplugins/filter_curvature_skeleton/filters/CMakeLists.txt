#filter_curvature_skeleton/filters
set (SOURCES
)

set (HEADERS
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})

add_subdirectory(skeletonize)
add_subdirectory(pruneSkeleton)
add_subdirectory(treeSegmentation)
add_subdirectory(polylineCutting)
add_subdirectory(branchMeasure)
