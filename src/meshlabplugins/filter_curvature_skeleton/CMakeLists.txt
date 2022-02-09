
set(FILTER_NAME filter_curvature_skeleton)

set(SOURCES
	"filter_curvature_skeleton.cpp"
	"cgal_adapter/mesh_converter.cpp"
)

set(HEADERS
	"filter_curvature_skeleton.h"
	"cgal_adapter/mesh_converter.h"
)

if(NOT TARGET external-boost)
	message(STATUS "Skipping ${FILTER_NAME} - don't know about boost on this system.")
elseif(NOT TARGET external-cgal)
	message(STATUS "Skipping ${FILTER_NAME} - don't know about cgal on this system.")
else()
	add_meshlab_plugin(${FILTER_NAME} ${SOURCES} ${HEADERS})

	target_link_libraries(${FILTER_NAME} PRIVATE external-boost external-cgal)
endif()

