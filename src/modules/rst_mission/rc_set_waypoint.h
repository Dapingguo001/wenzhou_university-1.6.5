
#ifndef RC_SET_WAYPOINT_H
#define RC_SET_WAYPOINT_H

#pragma once

#include <navigator/navigation.h>

#include "mission_type.h"
#include <systemlib/mavlink_log.h>


class RST_mission;


class RC_set_waypoint : public Mission_type
{
public:
    RC_set_waypoint(RST_mission *rst_mission);
    ~RC_set_waypoint();

    void run() override;

private:

    RST_mission     *_rst_mission; 
    
    int     _Pre_rocker_position;
    int     _Cur_rocker_position;

    bool    _Rocker_enable;

    struct mission_item_s _mission_item;

    int     _dataman_id;


    int                 _offboard_mission_sub;
    orb_advert_t		_offboard_mission_pub;

    orb_advert_t         *_mavlink_log_pub;


    bool    Recording_waypoint();

    void    takeoff_waypoint(struct mission_item_s *mission_item);

    void    Navigation_waypoint(struct mission_item_s *mission_item);

    int     get_waypoint_count();


    int     update_active_mission(int dataman_id, unsigned count, int seq);

};


#endif