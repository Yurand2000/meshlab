#filter_curvature_skeleton/test_filters/closeHoles

set (SOURCES
	"CloseHolesFilter.cpp"
	"CloseHolesFilter_apply.cpp"
	"CloseHolesFilter_params.cpp"
)

set (HEADERS
	"CloseHolesFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
