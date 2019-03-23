
#ifndef AB_POINT_H
#define AB_POINT_H

#pragma once


#include <uORB/topics/position_setpoint_triplet.h>

#include <navigator/navigation.h>

#include "mission_type.h"
#include <systemlib/mavlink_log.h>

#include <lib/geo/geo.h>

class RST_mission;


class AB_point : public Mission_type
{
public:
    AB_point(RST_mission *rst_mission);
    ~AB_point();

    void run() override;

private:

    RST_mission     *_rst_mission;
    orb_advert_t	_pos_sp_triplet_pub{nullptr}; 

    orb_advert_t         *_mavlink_log_pub;

    struct map_projection_reference_s start_point;
 
    uint8_t      virtual_square_width = 20;

    float mission_path[16][2] = {{0,0}, {1,0}, {2,0}, {3,0},
                                  {3,1}, {2,1}, {1,1}, {0,1},
                                  {0,2}, {1,2}, {2,2}, {3,2},
                                  {3,3}, {2,3}, {1,3}, {0,3}};

    //float mission_path[4][2] = {{0,0},{1,0},{1,1},{0,1}};

    float mission_altitude = 5.0f;

    int _mission_count = -1;


    int     _Pre_rocker_position;
    int     _Cur_rocker_position;

    int     _Pre_aux2_rocker_position;
    int     _Cur_aux2_rocker_position;


    bool    _Rocker_enable;

    int                 _offboard_mission_sub;
    orb_advert_t		_offboard_mission_pub;

    bool            ab_point_Recording_waypoint();
    void            ab_point_takeoff_waypoint(struct mission_item_s *mission_item);
    void            ab_point_Navigation_waypoint(struct mission_item_s *mission_item);

    int             ab_point_get_waypoint_count();
    int             ab_point_update_active_mission(int dataman_id, unsigned count, int seq);

    void            set_altitude();

};


#endif