#ifndef RST_MISSION_H
#define RST_MISSION_H


#pragma once

#include <drivers/drv_hrt.h>

#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/home_position.h>

#include "rc_set_waypoint.h"
#include "mission_type.h"
#include "ab_point.h"

#define rc_set_waypoint 0
#define ab_point        1
#define mission_type_sum 4


class RST_mission
{

public:

	RST_mission();
    ~RST_mission();
    
    int	start();

    struct manual_control_setpoint_s sp_man;
    struct vehicle_global_position_s global_pos;
    struct home_position_s			 home_pos; 				/**< home position */

    bool sp_man_updated = false;
    bool global_pos_updated = false;
    bool home_pos_updated = false;

    struct manual_control_setpoint_s *get_manual_control_setpoint(){return &sp_man;}
    struct vehicle_global_position_s *get_vehicle_global_position(){return &global_pos;}
    struct home_position_s *get_home_position(){return &home_pos;}
    bool         get_global_position_valid(){return (hrt_absolute_time() -_global_position_timestamp) < 5000000 ? true:false;};


private:
    
    bool	_task_should_exit{false};	/**< if true, sensor task should exit */
    int		_rst_mission_task{-1};		/**< task handle for sensor task */

    unsigned		_main_loop_delay;
    int             _mission_mode;

    Mission_type *_mission_type_array[mission_type_sum];

    RC_set_waypoint  _rc_set_waypoint;

    bool             _global_position_valid;
    uint64_t         _global_position_timestamp;

    AB_point      _ab_point;
    

    static void	task_main_trampoline();
    
    int		task_main();

};


#endif