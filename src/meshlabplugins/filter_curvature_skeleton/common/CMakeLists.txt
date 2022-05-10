#filter_curvature_skeleton/common
set (SOURCES
	"TemplateFilter.cpp"

	"BranchExtender.cpp"
	"SimplifySkeleton.cpp"
	"StrahlerBranchTagger.cpp"
)

set (HEADERS
	"additionalAttributeNames.h"
	"TemplateFilter.h"
	"SkeletonMesh.h"

	"BranchExtender.h"
	"SimplifySkeleton.h"
	"StrahlerBranchTagger.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
