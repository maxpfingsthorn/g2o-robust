
include_directories(include)

add_library(g2o_types_max_mixture_original SHARED
	src/types_max_mixture_original.cpp
	src/edge_se2_mixture.cpp
	src/edge_se2_pointxy_mixture.cpp
	src/edge_se3_mixture.cpp
)

target_link_libraries(g2o_types_max_mixture_original ${G2O_CORE_LIBRARY} ${G2O_TYPES_SLAM3D} ${G2O_TYPES_SLAM2D})

install(FILES 
	include/types_max_mixture_original.h
	include/edge_se2_mixture.h
	include/edge_se2_pointxy_mixture.h
	include/edge_se3_mixture.h
	DESTINATION include
)

install(TARGETS
	g2o_types_max_mixture_original
	DESTINATION lib
)
