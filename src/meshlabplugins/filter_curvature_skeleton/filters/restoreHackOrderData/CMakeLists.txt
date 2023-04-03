#filter_curvature_skeleton/filters/restoreHackOrderData
set (SOURCES
	"RestoreHackOrderDataFilter.cpp"
	"RestoreHackOrderDataFilter_apply.cpp"
	"RestoreHackOrderDataFilter_params.cpp"
)

set (HEADERS
	"RestoreHackOrderDataFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
