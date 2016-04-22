
include_directories(include)

add_library(g2o_types_robust_edges SHARED
	src/types_robust_edges.cpp
)

target_link_libraries(g2o_types_robust_edges ${G2O_CORE_LIBRARY} ${G2O_TYPES_SLAM3D} ${G2O_TYPES_SLAM2D})

install(FILES 
	include/edge_robustified.hpp
	DESTINATION include
)

install(TARGETS
	g2o_types_robust_edges
	DESTINATION lib
)
