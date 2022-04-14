#filter_curvature_skeleton/filters/strahlerTagging
set (SOURCES
	"StrahlerTaggingFilter.cpp"
	"StrahlerTaggingFilter_apply.cpp"
	"StrahlerTaggingFilter_params.cpp"
)

set (HEADERS
	"StrahlerTaggingFilter.h"
)

list(TRANSFORM SOURCES PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
list(TRANSFORM HEADERS PREPEND "${CMAKE_CURRENT_SOURCE_DIR}/")
target_sources(${FILTER_NAME} PRIVATE ${SOURCES} ${HEADERS})
