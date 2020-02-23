#include "Copter.h"

#if MODE_STM23 == ENABLED


int aci=0, yukseklik=200, yaricap=300, k=0, hedef_no=1, donus_rate=4000;
void ModeSTM23::run()
{
    // call the correct auto controller
    
    switch (guided_mode) {

    case Guided_TakeOff:
        // run takeoff controller
        takeoff_run();
        break;

    case Guided_WP: {
    // process pilot's yaw input
    //auto_yaw.set_mode(AUTO_YAW_RATE);
    float target_yaw_rate = 0;
    if (!copter.failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->get_control_in());
        if (!is_zero(target_yaw_rate)) {
            auto_yaw.set_mode(AUTO_YAW_HOLD);
            //auto_yaw.set_mode(AUTO_YAW_RATE);
        }
    }

    // if not armed set throttle to zero and exit immediately
    if (is_disarmed_or_landed()) {
        make_safe_spool_down();
        return;
    }

    // set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    // run waypoint controller
    copter.failsafe_terrain_set_status(wp_nav->update_wpnav());

    Vector3f hedef;
    //if (wp_nav->reached_wp_destination(), wp_nav->get_wp_distance_to_destination() < 15)
    //auto_yaw.set_mode(AUTO_YAW_FIXED);
    //auto_yaw.set_fixed_yaw();
    
    if (wp_nav->reached_wp_destination()) {
        switch(hedef_no){
            case 1: {
                auto_yaw.set_mode(AUTO_YAW_HOLD);
                hedef(0,0,yukseklik);
                wp_nav->set_wp_destination(hedef, false);
                hedef_no=2;
                break;
            }
            case 2: {
                auto_yaw.set_mode(AUTO_YAW_RATE);
                hedef(k,0,yukseklik);
                wp_nav->set_wp_destination(hedef, false);
                k+=10;
                if (k>=500){
                    hedef(500,0,yukseklik);
                    wp_nav->set_wp_destination(hedef, false);
                    auto_yaw.set_mode(AUTO_YAW_LOOK_AT_NEXT_WP);
                    k=0;
                    hedef_no=3;
                }
                break;
            }
            case 3: {
                auto_yaw.set_mode(AUTO_YAW_RATE);
                hedef(500-k,0,yukseklik+k);
                wp_nav->set_wp_destination(hedef, false);
                k+=10;
                if (k>=500){
                    hedef(0,0,yukseklik+500);
                    wp_nav->set_wp_destination(hedef, false);
                    auto_yaw.set_mode(AUTO_YAW_LOOK_AT_NEXT_WP);
                    k=0;
                    hedef_no=4;
                }
                break;
            }
            case 4: {
                auto_yaw.set_mode(AUTO_YAW_RATE);
                hedef(0,0,yukseklik);
                wp_nav->set_wp_destination(hedef, false);
                hedef_no=5;
                break;
            }
            case 5: {
                auto_yaw.set_mode(AUTO_YAW_RATE);
                hedef(yaricap*sinf(radians(aci)) , 0 , yukseklik+yaricap*(1-cosf(radians(aci))));
                wp_nav->set_wp_destination(hedef, false);
                aci+=10;
                if (aci>=360){
                    hedef(0,0,yukseklik);
                    wp_nav->set_wp_destination(hedef, false);
                    aci=0;
                    hedef_no=6;
                }
                break;
            }
            case 6: {
                auto_yaw.set_mode(AUTO_YAW_HOLD);
                hedef(0,0,yukseklik-k);
                k+=20;
                if (k>=260){
                    make_safe_spool_down();
                    copter.arming.disarm();
                }
                wp_nav->set_wp_destination(hedef, false);
                break;
            }
        }
    }
    // call z-axis position controller (wpnav should have already updated it's alt target)
    pos_control->update_z_controller();

    // call attitude controller     
    if (auto_yaw.mode() == AUTO_YAW_HOLD) {
        // roll & pitch from waypoint controller, yaw rate from pilot
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), target_yaw_rate);
    } else if (auto_yaw.mode() == AUTO_YAW_RATE) {
        // roll & pitch from waypoint controller, yaw rate from mavlink command or mission item
        //cds: centi degrees per seconds
        //attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.rate_cds());
        attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), donus_rate);
    } else {
        // roll, pitch from waypoint controller, yaw heading from GCS or auto_heading()
        attitude_control->input_euler_angle_roll_pitch_yaw(wp_nav->get_roll(), wp_nav->get_pitch(), auto_yaw.yaw(), true);
    }
    break;
    }
    case Guided_Velocity:
        // run velocity controller
        vel_control_run();
        break;

    case Guided_PosVel:
        // run position-velocity controller
        posvel_control_run();
        break;

    case Guided_Angle:
        // run angle controller
        angle_control_run();
        break;
    }
 }
#endif