
include_directories(include)

add_library(g2o_types_max_mixture SHARED
	src/types_max_mixture_new.cpp
	src/max_mixture_manager.cpp
)

target_link_libraries(g2o_types_max_mixture ${G2O_CORE_LIBRARY} ${G2O_TYPES_SLAM3D} ${G2O_TYPES_SLAM2D})

install(FILES 
	include/edge_max_mixture.hpp
	include/max_mixture_interface.h
	include/max_mixture_manager.h
	DESTINATION include
)

install(TARGETS
	g2o_types_max_mixture
	DESTINATION lib
)
