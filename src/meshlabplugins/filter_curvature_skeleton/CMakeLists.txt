
set(FILTER_NAME filter_curvature_skeleton)

set(SOURCES
	"filter_curvature_skeleton.cpp"
	"cgal_adapter/mesh_converter_cmesh_to_cgal.cpp"
	"cgal_adapter/mesh_converter_skeleton_to_cmesh.cpp"
	"cgal_adapter/mesh_converter_meso_skeleton_to_cmesh.cpp"
	"cgal_adapter/mesh_skeletonizer.cpp"
	"filters/filter_template.cpp"
	"filters/skeleton_distance_to_quality.cpp"
	"filters/skeleton_index_to_quality.cpp"
	"filters/skeletonize_manual.cpp"
	"filters/skeletonize_manual_apply.cpp"
	"filters/skeletonize_manual_param_list.cpp"
	"filters/algorithm_skeletonize.cpp"
)

set(HEADERS
	"filter_curvature_skeleton.h"
	"cgal_adapter/typedefs.h"
	"cgal_adapter/mesh_converter.h"
	"cgal_adapter/mesh_skeletonizer.h"
	"filters/filter_template.h"
	"filters/skeletonize_manual.h"
	"filters/skeleton_distance_to_quality.h"
	"filters/skeleton_index_to_quality.h"
	"filters/algorithm_skeletonize.h"
	"filters/additional_attribute_names.h"
)

if(NOT TARGET external-boost)
	message(STATUS "Skipping ${FILTER_NAME} - don't know about boost on this system.")
elseif(NOT TARGET external-cgal)
	message(STATUS "Skipping ${FILTER_NAME} - don't know about cgal on this system.")
else()
	add_meshlab_plugin(${FILTER_NAME} ${SOURCES} ${HEADERS})

	target_link_libraries(${FILTER_NAME} PRIVATE external-boost external-cgal)
endif()

