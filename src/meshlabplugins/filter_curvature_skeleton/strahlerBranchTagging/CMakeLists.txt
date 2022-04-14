#filter_curvature_skeleton/strahlerBranchTagging
set (SOURCES
	"SkeletonTreeBuilder.cpp"
	"SkeletonTreeBuilder_private.cpp"
	"StrahlerBranchTagger.cpp"
	"StrahlerBranchTagger_private.cpp"
)

set (HEADERS
	"SkeletonTreeBuilder.h"
	"SkeletonTreeBuilder_private.h"
	"StrahlerBranchTagger.h"
	"StrahlerBranchTagger_private.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
