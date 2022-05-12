#filter_curvature_skeleton/cgalAdapter
set (SOURCES
	"CGALMeshSkeletonizer.cpp"
)

set (HEADERS
	"CGALMeshConverter.h"
	"CGALMeshSkeletonizer.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
