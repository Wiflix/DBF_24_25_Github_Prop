#include "mode.h"
#include "Plane.h"

bool ModeCustom::_enter()
{
#if HAL_QUADPLANE_ENABLED
    // check if we should refuse auto mode due to a missing takeoff in
    // guided_wait_takeoff state
    if (plane.previous_mode == &plane.mode_guided &&
        quadplane.guided_wait_takeoff_on_mode_enter) {
        if (!plane.mission.starts_with_takeoff_cmd()) {
            gcs().send_text(MAV_SEVERITY_ERROR,"Takeoff waypoint required");
            quadplane.guided_wait_takeoff = true;
            return false;
        }
    }
    
    if (plane.quadplane.available() && plane.quadplane.enable == 2) {
        plane.auto_state.vtol_mode = true;
    } else {
        plane.auto_state.vtol_mode = false;
    }
#else
    plane.auto_state.vtol_mode = false;
#endif
    plane.turnFlag = false; //This will change to true once turn is started
    plane.next_WP_loc = plane.prev_WP_loc = plane.current_loc; //start with: waypoint location way far out. Then once turn flag is activated, change these vars to make waypoint to the side
    //define the mission in MP as follows: waypoint 1 far out, waypoint 2 in the bonus box
    // start or resume the mission, based on MIS_AUTORESET
    plane.mission.start_or_resume();

    if (hal.util->was_watchdog_armed()) {
        if (hal.util->persistent_data.waypoint_num != 0) {
            gcs().send_text(MAV_SEVERITY_INFO, "Watchdog: resume WP %u", hal.util->persistent_data.waypoint_num);
            plane.mission.set_current_cmd(hal.util->persistent_data.waypoint_num);
            hal.util->persistent_data.waypoint_num = 0;
        }
    }

#if HAL_SOARING_ENABLED
    plane.g2.soaring_controller.init_cruising();
#endif

    return true;
}

void ModeCustom::_exit()
{
    if (plane.mission.state() == AP_Mission::MISSION_RUNNING) {
        plane.mission.stop();

        bool restart = plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_LAND;
#if HAL_QUADPLANE_ENABLED
        if (plane.quadplane.is_vtol_land(plane.mission.get_current_nav_cmd().id)) {
            restart = false;
        }
#endif
        if (restart) {
            plane.landing.restart_landing_sequence();
        }
    }
    plane.auto_state.started_flying_in_auto_ms = 0;
}

void ModeCustom::update()
{
    if (plane.mission.state() != AP_Mission::MISSION_RUNNING) {
        // this could happen if AP_Landing::restart_landing_sequence() returns false which would only happen if:
        // restart_landing_sequence() is called when not executing a NAV_LAND or there is no previous nav point
        plane.set_mode(plane.mode_rtl, ModeReason::MISSION_END);
        gcs().send_text(MAV_SEVERITY_INFO, "Aircraft in auto without a running mission");
        return;
    }

    uint16_t nav_cmd_id = plane.mission.get_current_nav_cmd().id;

#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.in_vtol_auto()) {
        plane.quadplane.control_auto();
        return;
    }
#endif

#if AP_PLANE_GLIDER_PULLUP_ENABLED
    if (pullup.in_pullup()) {
        return;
    }
#endif

    if (nav_cmd_id == MAV_CMD_NAV_TAKEOFF ||
        (nav_cmd_id == MAV_CMD_NAV_LAND && plane.flight_stage == AP_FixedWing::FlightStage::ABORT_LANDING)) {
        plane.takeoff_calc_roll();
        plane.takeoff_calc_pitch();
        plane.takeoff_calc_throttle();
    } else if (nav_cmd_id == MAV_CMD_NAV_LAND) {
        plane.calc_nav_roll();
        plane.calc_nav_pitch();

        // allow landing to restrict the roll limits
        plane.nav_roll_cd = plane.landing.constrain_roll(plane.nav_roll_cd, plane.g.level_roll_limit*100UL);

        if (plane.landing.is_throttle_suppressed()) {
            // if landing is considered complete throttle is never allowed, regardless of landing type
            SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        } else {
            plane.calc_throttle();
        }
#if AP_SCRIPTING_ENABLED
    } else if (nav_cmd_id == MAV_CMD_NAV_SCRIPT_TIME) {
        // NAV_SCRIPTING has a desired roll and pitch rate and desired throttle
        plane.nav_roll_cd = ahrs.roll_sensor;
        plane.nav_pitch_cd = ahrs.pitch_sensor;
#endif
    } else {
        // we are doing normal AUTO flight, the special cases
        // are for takeoff and landing
        if (nav_cmd_id != MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT) {
            plane.steer_state.hold_course_cd = -1;
        }
        
        plane.calc_nav_roll();
        plane.calc_nav_pitch();
        plane.calc_throttle();

        //NEW CODE:
        //pretty much wayguid.m
        if(plane.turnFlag==false){
            double pN = loc_2_pN_VanNuys(plane.current_loc);
            double pE = loc_2_pE_VanNuys(plane.current_loc);
            double distance_to_box_center = 1.25*pi*50*pN;
            double alt_ft = plane.current_loc.alt/30.48; //convert cm to ft
            double dist_to_ground = alt_ft/tan(-plane.GS_com);
            if(abs(distance_to_box_center-dist_to_ground)<50){
                plane.turnFlag=true;
                //set wp loc
                double wpNew_pE = 150;
                double wpNew_pN = pN-100;
                double new_alt = 50; //placeholder
                Location::AltFrame altFrame = plane.current_loc.get_alt_frame;
                Location locNew = Location(pN_pE_VanNuys_2_lat(pN,pE),pN_pE_VanNuys_2_lng(pN,pE), new_Alt, altFrame);
                plane.set_next_WP(locNew);
                Vector3f velNED;
                ahrs.get_velocity_NED(velNED); //FIND WAY TO GET NED VELOCITY VECTOR. This gets us descent rate and speed, which is what we need for GS control

            }
        }
    }
}

void ModeCustom::navigate()
{
    if (AP::ahrs().home_is_set()) {
        plane.mission.update();
    }
}


bool ModeCustom::does_auto_navigation() const
{
#if AP_SCRIPTING_ENABLED
   return (!plane.nav_scripting_active());
#endif
   return true;
}

bool ModeCustom::does_auto_throttle() const
{
#if AP_SCRIPTING_ENABLED
   return (!plane.nav_scripting_active());
#endif
   return true;
}

// returns true if the vehicle can be armed in this mode
bool ModeCustom::_pre_arm_checks(size_t buflen, char *buffer) const
{
#if HAL_QUADPLANE_ENABLED
    if (plane.quadplane.enabled()) {
        if (plane.quadplane.option_is_set(QuadPlane::OPTION::ONLY_ARM_IN_QMODE_OR_AUTO) &&
                !plane.quadplane.is_vtol_takeoff(plane.mission.get_current_nav_cmd().id)) {
            hal.util->snprintf(buffer, buflen, "not in VTOL takeoff");
            return false;
        }
        if (!plane.mission.starts_with_takeoff_cmd()) {
            hal.util->snprintf(buffer, buflen, "missing takeoff waypoint");
            return false;
        }
    }
#endif
    // Note that this bypasses the base class checks
    return true;
}

bool ModeCustom::is_landing() const
{
    return (plane.flight_stage == AP_FixedWing::FlightStage::LAND);
}

void ModeCustom::run()
{
#if AP_PLANE_GLIDER_PULLUP_ENABLED
    if (pullup.in_pullup()) {
        pullup.stabilize_pullup();
        return;
    }
#endif
    
    if (plane.mission.get_current_nav_cmd().id == MAV_CMD_NAV_ALTITUDE_WAIT) {

        wiggle_servos();

        SRV_Channels::set_output_scaled(SRV_Channel::k_throttle, 0.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleLeft, 0.0);
        SRV_Channels::set_output_scaled(SRV_Channel::k_throttleRight, 0.0);

        SRV_Channels::set_output_to_trim(SRV_Channel::k_throttle);
        SRV_Channels::set_output_to_trim(SRV_Channel::k_throttleLeft);
        SRV_Channels::set_output_to_trim(SRV_Channel::k_throttleRight);

        // Relax attitude control
        reset_controllers();

    } else {
        // Normal flight, run base class
        Mode::run();

    }
}
//NOTE: pN and pE aren't actually N/E positions. They are defined relative to the runway position and orientation. pN is along runway with bonus box to the right, and pE is 90 deg of that
double loc_2_pN_VanNuys(Location loc){
    double heading = -38.09; //in degrees, how many degrees east of north runway is (bonus box to the right of runway if runway at 0 deg)
    double home_lat = 34.17535989 *3.14159265/180; // in rad
    double home_long = -118.4818518*3.14159265/180; //in rad
    double current_lat = loc.lat/1e7 * *3.14159265/180; // in rad
    double current_long = loc.lng/1e7 * *3.14159265/180; // in rad
    double R = 20903520; // earth's radius in feet
    double delta_east = R*cos(home_lat)* (current_long-home_long); //distance east of home in ft.
    double delta_north = R*(current_lat-home_lat); //distance north of home in ft
    double pN = -delta_east*sin(-heading)+delta_north*cos(-heading);
    return pN;

}
double loc_2_pE_VanNuys(Location loc){
    double heading = -38.09; //in degrees, how many degrees east of north runway is (bonus box to the right of runway if runway at 0 deg)
    double home_lat = 34.17535989 *3.14159265/180; // in rad
    double home_long = -118.4818518*3.14159265/180; //in rad
    double current_lat = loc.lat/1e7 *3.14159265/180; // in rad
    double current_long = loc.lng/1e7 *3.14159265/180; // in rad
    double R = 20903520; // earth's radius in feet
    double delta_east = R*cos(home_lat)* (current_long-home_long); //distance east of home in ft.
    double delta_north = R*(current_lat-home_lat); //distance north of home in ft
    double pE = delta_east*cos(-heading)+delta_north*sin(-heading);
    return pE;
}
int32_t pN_pE_VanNuys_2_lng(double pN, double pE){
    double heading = -38.09; //in degrees, how many degrees east of north runway is (bonus box to the right of runway if runway at 0 deg)
    double home_lat = 34.17535989 *3.14159265/180; // in rad
    double home_long = -118.4818518*3.14159265/180; //in rad
    double R = 20903520; // earth's radius in feet
    double delta_east = pE*cos(heading)+pN*sin(heading);
    //delta_north = -pN*sin(heading)+pN*cos(heading);
    double current_long = home_long+delta_east/(R*cos(home_lat));
    return (int32_t)round(current_long*1e7*180/3.14159265);
}
int32_t pN_pE_VanNuys_2_lat(double pN, double pE){
    double heading = -38.09; //in degrees, how many degrees east of north runway is (bonus box to the right of runway if runway at 0 deg)
    double home_lat = 34.17535989 *3.14159265/180; // in rad
    double ome_long = -118.4818518*3.14159265/180; //in rad
    double R = 20903520; // earth's radius in feet
    //delta_east = pE*cos(heading)+pN*sin(heading);
    double delta_north = -pN*sin(heading)+pN*cos(heading);
    double current_lat = home_lat+delta_north/R;
    return (int32_t)round(current_lat*1e7*180/3.1415926);
}
