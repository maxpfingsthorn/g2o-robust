
include_directories(include)

add_library(g2o_types_switchable_constraints SHARED
	src/types_switchable_constraints.cpp
	src/vertex_switchLinear.cpp
	src/edge_switchPrior.cpp
	src/edge_se2Switchable.cpp
	src/edge_se3Switchable.cpp
)

target_link_libraries(g2o_types_switchable_constraints ${G2O_CORE_LIBRARY} ${G2O_STUFF_LIBRARY} ${G2O_TYPES_SLAM3D})


install(FILES 
	include/types_switchable_constraints.h
	include/vertex_switchLinear.h
	include/edge_switchPrior.h
	include/edge_se2Switchable.h
	include/edge_se3Switchable.h
	DESTINATION include
)

install(TARGETS
	g2o_types_switchable_constraints
	DESTINATION lib
)
