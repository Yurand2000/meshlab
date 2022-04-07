#filter_curvature_skeleton/cgalAdapter
set (SOURCES
	"MeshConverter_cmesh_to_cgal.cpp"
	"MeshConverter_meso_skeleton_to_cmesh.cpp"
	"MeshConverter_skeleton_to_cmesh.cpp"
	"MeshSkeletonizer.cpp"
)

set (HEADERS
	"typedefs.h"
	"MeshConverter.h"
	"MeshSkeletonizer.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
