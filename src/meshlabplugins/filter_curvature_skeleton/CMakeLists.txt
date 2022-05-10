set(FILTER_NAME filter_curvature_skeleton)

if(NOT TARGET external-boost)
	message(STATUS "Skipping ${FILTER_NAME} - don't know about boost on this system.")
elseif(NOT TARGET external-cgal)
	message(STATUS "Skipping ${FILTER_NAME} - don't know about cgal on this system.")
else()
	add_meshlab_plugin(${FILTER_NAME} ${SOURCES} ${HEADERS})

	target_include_directories(${FILTER_NAME} PRIVATE "${CMAKE_SOURCE_DIR}")
	target_link_libraries(${FILTER_NAME} PRIVATE external-boost external-cgal)
	target_sources(${FILTER_NAME} PRIVATE
		"filter_curvature_skeleton.h"
		"filter_curvature_skeleton.cpp"
	)
	
	add_subdirectory(common)
	add_subdirectory(filters)
	add_subdirectory(cgalAdapter)
endif()



