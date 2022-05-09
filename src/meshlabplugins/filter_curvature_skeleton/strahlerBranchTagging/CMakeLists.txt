#filter_curvature_skeleton/strahlerBranchTagging
set (SOURCES
	"SkeletonTreeBuilder.cpp"
	"StrahlerBranchTagger.cpp"
)

set (HEADERS
	"SkeletonTreeBuilder.h"
	"StrahlerBranchTagger.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
