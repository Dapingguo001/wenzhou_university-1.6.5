############################################################################
#rst add moudle
############################################################################
px4_add_module(
	MODULE modules__rst_mission
	MAIN rst_mission
	STACK_MAIN 1300
	COMPILE_FLAGS
	SRCS
		rst_mission_main.cpp
		mission_type.cpp
		rc_set_waypoint.cpp
		ab_point.cpp

	DEPENDS
		platforms__common
	)
# vim: set noet ft=cmake fenc=utf-8 ff=unix :
