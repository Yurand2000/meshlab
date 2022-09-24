#filter_curvature_skeleton/common
set (SOURCES
	"TemplateFilter.cpp"
	
	"AlgorithmSkeletonize.cpp"
	"BranchExtender.cpp"
)

set (HEADERS
	"TemplateFilter.h"
	"SkeletonMesh.h"
	"Utils.h"
	"Utils.imp.h"
	
	"AlgorithmSkeletonize.h"
	"BranchExtender.h"

	"BranchTagger.h"
	"BranchTagger.imp.h"
	
	"SimplifySkeleton.h"
	"SimplifySkeleton.imp.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
