#include <systemlib/err.h>
#include <geo/geo.h>
#include <lib/mathlib/mathlib.h>
#include <stdint.h>

#include <px4_defines.h>
#include <dataman/dataman.h>

#include <navigator/navigation.h>

#include <uORB/topics/mission.h>

#include <lib/geo/geo.h>

#include <systemlib/mavlink_log.h>

#include "ab_point.h"
#include "rst_mission.h"
#include "mission_type.h"


AB_point::AB_point(RST_mission *rst_mission):
                    _rst_mission(rst_mission),
                    _Pre_rocker_position(-2),
                    _Cur_rocker_position(-2),
                    _Rocker_enable(false),
                    _offboard_mission_sub(-1),
                    _offboard_mission_pub(nullptr)
{
    _offboard_mission_sub = orb_subscribe(ORB_ID(offboard_mission));    
}

AB_point::~AB_point()
{

}

void
AB_point::run()
{
    struct mission_item_s mission_item;
    
        set_altitude();
        if(ab_point_Recording_waypoint())
        {
            if(_Rocker_enable && _rst_mission->get_global_position_valid())
            {
                dm_item_t dm_item = DM_KEY_WAYPOINTS_OFFBOARD(1);

                for(int i=0 ; i< 4; i++)
                {
                    usleep(1000000);
                    _mission_count ++;
                    if(ab_point_get_waypoint_count() == 0){
        
                        ab_point_takeoff_waypoint(&mission_item);
                        if(dm_write(dm_item, 0, DM_PERSIST_POWER_ON_RESET, &mission_item,
                            sizeof(struct mission_item_s)) != sizeof(struct mission_item_s)){
        
                                mavlink_log_critical(_mavlink_log_pub,"waypoint write error\n"); 
                            }
        
                    }
                    else{
                        ab_point_Navigation_waypoint(&mission_item);
                        if(dm_write(dm_item, ab_point_get_waypoint_count(), DM_PERSIST_POWER_ON_RESET, &mission_item,
                            sizeof(struct mission_item_s)) != sizeof(struct mission_item_s)){
        
                                mavlink_log_critical(_mavlink_log_pub,"waypoint write error\n");
                            }
                    }
        
                    if (ab_point_update_active_mission(1, ab_point_get_waypoint_count()+1, 0) == PX4_OK)
                    {
                        mavlink_log_info(_mavlink_log_pub, "rc set waypoint number is %d",ab_point_get_waypoint_count());
                    }
                    else if(_Rocker_enable)
                    {
                        mavlink_log_critical(_mavlink_log_pub,"gps no valid, can not set waypoint\n"); 
                    }
                }
            }
            _Rocker_enable = true;
        } 
}


bool
AB_point::ab_point_Recording_waypoint()
{
    if(_rst_mission->get_manual_control_setpoint()->aux1 < -0.8f){
        _Cur_rocker_position = -1;
    }
    if(_rst_mission->get_manual_control_setpoint()->aux1 >  0.8f){
        _Cur_rocker_position = 1;
    }

    if(_Pre_rocker_position != _Cur_rocker_position && ((_Cur_rocker_position == -1) || (_Cur_rocker_position == 1)))
    {
        _Pre_rocker_position = _Cur_rocker_position;
        return true;
    }
    return false;
}



void
AB_point::ab_point_takeoff_waypoint(struct mission_item_s *mission_item)
{
    mission_item->lat = _rst_mission->get_vehicle_global_position()->lat;
    mission_item->lon = _rst_mission->get_vehicle_global_position()->lon;
    mission_item->altitude = mission_altitude;//_rst_mission->get_home_position()->alt + mission_altitude;
    
    mission_item->altitude_is_relative = true;
    
    mission_item->nav_cmd = NAV_CMD_TAKEOFF;
    
    mission_item->pitch_min = 0.0f;
    
    mission_item->yaw = NAN;
    
    mission_item->autocontinue = 1;
    
    mission_item->do_jump_current_count = 0;
    
    mission_item->origin = ORIGIN_MAVLINK;

    start_point.lat_rad = _rst_mission->get_vehicle_global_position()->lat/180.0 *M_PI;
    start_point.lon_rad = _rst_mission->get_vehicle_global_position()->lon/180.0 *M_PI;
    start_point.sin_lat = sinf(start_point.lat_rad);
    start_point.cos_lat = cosf(start_point.lat_rad);
    
}


void
AB_point::ab_point_Navigation_waypoint(struct mission_item_s *mission_item)
{
    double next_lat_sp;
    double next_lon_sp; 
    map_projection_reproject(&start_point, virtual_square_width*mission_path[_mission_count][0],  virtual_square_width*mission_path[_mission_count][1], &next_lat_sp, &next_lon_sp);

    mission_item->lat = next_lat_sp;
    mission_item->lon = next_lon_sp;
    mission_item->altitude =  mission_altitude;//_rst_mission->get_vehicle_global_position()->alt + mission_altitude;
    
    mission_item->altitude_is_relative = true;

    mission_item->nav_cmd = NAV_CMD_WAYPOINT;

    mission_item->time_inside = 0.0f;
    mission_item->acceptance_radius = 0.0f;

    mission_item->yaw =  NAN;

    mission_item->autocontinue = 1;
    
    mission_item->do_jump_current_count = 0;
    
    mission_item->origin = ORIGIN_MAVLINK;  

}


int
AB_point::ab_point_get_waypoint_count()
{
    mission_s mission_state;

    int ret = dm_read(DM_KEY_MISSION_STATE, 0, &mission_state, sizeof(mission_s)) == sizeof(mission_s);

    if (ret > 0) {
        return mission_state.count;

    } else if (ret == 0) {
        return 0;

    } else {
        return 0;
        PX4_WARN("offboard mission init failed");
    }
    return 0;
}



int
AB_point::ab_point_update_active_mission(int dataman_id, unsigned count, int seq)
{
	struct mission_s mission;

	mission.dataman_id = dataman_id;
	mission.count = count;
	mission.current_seq = seq;

	/* update mission state in dataman */
	int res = dm_write(DM_KEY_MISSION_STATE, 0, DM_PERSIST_POWER_ON_RESET, &mission, sizeof(mission_s));

	if (res == sizeof(mission_s)) {
		/* update active mission state */
		/*/_dataman_id = dataman_id;
		_count = count;
		_current_seq = seq;
		_my_dataman_id = _dataman_id;*/

		/* mission state saved successfully, publish offboard_mission topic */
		if (_offboard_mission_pub == nullptr) {
			_offboard_mission_pub = orb_advertise(ORB_ID(offboard_mission), &mission);

		} else {
			orb_publish(ORB_ID(offboard_mission), _offboard_mission_pub, &mission);
		}

		return PX4_OK;

	} else {
		warnx("WPM: ERROR: can't save mission state");

		/*if (_filesystem_errcount++ < FILESYSTEM_ERRCOUNT_NOTIFY_LIMIT) {
			_mavlink->send_statustext_critical("Mission storage: Unable to write to microSD");
		}*/

		return PX4_ERROR;
	}
}

void
AB_point::set_altitude()
{
    if(_rst_mission->get_manual_control_setpoint()->aux2 < -0.8f){
        _Cur_aux2_rocker_position = -1;
    }
    if(_rst_mission->get_manual_control_setpoint()->aux2 >  0.8f){
        _Cur_aux2_rocker_position = 1;
    }

    if(_Pre_aux2_rocker_position != _Cur_aux2_rocker_position && ((_Cur_aux2_rocker_position == -1) || (_Cur_aux2_rocker_position == 1)))
    {
        _Pre_aux2_rocker_position = _Cur_aux2_rocker_position;
        mission_altitude = mission_altitude + 5.0f;

        if((int)(mission_altitude) == 35)
        {
            mission_altitude = 5.0f;
        }
        mavlink_log_critical(_mavlink_log_pub,"set waypoint altitude %.2f\n",(double)mission_altitude); 
    } 
}