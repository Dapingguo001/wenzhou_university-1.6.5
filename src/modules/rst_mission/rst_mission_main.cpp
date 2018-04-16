
#include "rst_mission.h"
#include "rc_set_waypoint.h"

#include <cfloat>

#include <dataman/dataman.h>
#include <drivers/drv_hrt.h>
#include <geo/geo.h>
#include <mathlib/mathlib.h>
#include <px4_config.h>
#include <px4_defines.h>
#include <px4_posix.h>
#include <px4_tasks.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <px4_getopt.h>

#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/home_position.h>

#include <systemlib/systemlib.h>


extern "C" __EXPORT int rst_mission_main(int argc, char *argv[]);

static void usage();

namespace rst_mission
{
    RST_mission	*g_rst_mission;
}


RST_mission::RST_mission():
                    _main_loop_delay(200000),
                    _mission_mode(-1),
                    _rc_set_waypoint(this),
                    _global_position_valid(false),
                    _ab_point(this)      
{
    _mission_type_array[rc_set_waypoint] = &_rc_set_waypoint;
    _mission_type_array[ab_point] = &_ab_point;
    _global_position_timestamp = hrt_absolute_time();
}

RST_mission::~RST_mission()
{
    if (_rst_mission_task != -1) {
        
         /* task wakes up every 100ms or so at the longest */
        _task_should_exit = true;
        
        /* wait for a second for the task to quit at our request */
        unsigned i = 0;
        
        do {
            /* wait 20ms */
            usleep(20000);
        
            /* if we have given up, kill it */
            if (++i > 50) {
                px4_task_delete(_rst_mission_task);
                break;
            }
        } while (_rst_mission_task != -1);
     }
        
     rst_mission::g_rst_mission = nullptr;
    
}

int
RST_mission::start()
{
    ASSERT(_rst_mission_task == -1);
    
        /* start the task */
     _rst_mission_task = px4_task_spawn_cmd("rst_mission",
                        SCHED_DEFAULT,
                        SCHED_PRIORITY_DEFAULT,
                        1800,
                        (px4_main_t)&RST_mission::task_main_trampoline,
                        nullptr);
    
    if (_rst_mission_task < 0) {
        warn("task start failed");
        return -errno;
    }
    
    return OK;
}


void
RST_mission::task_main_trampoline()
{
	rst_mission::g_rst_mission->task_main();
}


int
RST_mission::task_main()
{

    _mission_mode = ab_point;

    int sp_man_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
    memset(&sp_man, 0, sizeof(sp_man));
    
    int global_pos_sub = orb_subscribe(ORB_ID(vehicle_global_position));
    memset(&global_pos, 0, sizeof(global_pos));

    int home_pos_sub = orb_subscribe(ORB_ID(home_position));
    memset(&home_pos, 0, sizeof(home_pos));

    //int _pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));

    
    while (!_task_should_exit) {

        usleep(_main_loop_delay);

        orb_check(sp_man_sub, &sp_man_updated);
        if (sp_man_updated) {
			orb_copy(ORB_ID(manual_control_setpoint), sp_man_sub, &sp_man);
        }

        orb_check(global_pos_sub, &global_pos_updated);
        if (global_pos_updated) {
            _global_position_timestamp = hrt_absolute_time();
            orb_copy(ORB_ID(vehicle_global_position), global_pos_sub, &global_pos);
        }

        orb_check(home_pos_sub, &home_pos_updated);
        if (home_pos_updated) {
            orb_copy(ORB_ID(home_position), home_pos_sub, &home_pos);
        }

        _mission_type_array[_mission_mode]->run();
        

    }


    PX4_INFO("exiting");
    
    _rst_mission_task = -1; 
    
    return OK;
}


static void usage()
{
	PX4_INFO("usage: rst mission {start|stop} -m {rc_set_waypoint}");
}

int rst_mission_main(int argc, char *argv[])
{
    if (argc < 2) {
		usage();
		return 1;
    }
    
    if (!strcmp(argv[1], "start")) {
        
        if (rst_mission::g_rst_mission != nullptr) {
            PX4_WARN("already running");
            return 1;
        }
        
        rst_mission::g_rst_mission = new RST_mission;
        
        if (rst_mission::g_rst_mission == nullptr) {
            PX4_ERR("alloc failed");
            return 1;
        }
        
        if (OK != rst_mission::g_rst_mission->start()) {
            delete rst_mission::g_rst_mission;
            rst_mission::g_rst_mission = nullptr;
            PX4_ERR("start failed");
            return 1;
        }
        
        return 0;
    }
    else if (!strcmp(argv[1], "stop")) {
		delete rst_mission::g_rst_mission;
		rst_mission::g_rst_mission = nullptr;

    }
    else {
		usage();
        return 1;
    }
    return 0;
}


