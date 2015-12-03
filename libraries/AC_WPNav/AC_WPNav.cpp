/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <AP_HAL.h>
#include <AC_WPNav.h>

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AC_WPNav::var_info[] PROGMEM = {
    // index 0 was used for the old orientation matrix

    // @Param: SPEED
    // @DisplayName: Waypoint Horizontal Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain horizontally during a WP mission
    // @Units: cm/s
    // @Range: 0 2000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED",       0, AC_WPNav, _wp_speed_cms, WPNAV_WP_SPEED),

    // @Param: RADIUS
    // @DisplayName: Waypoint Radius
    // @Description: Defines the distance from a waypoint, that when crossed indicates the wp has been hit.
    // @Units: cm
    // @Range: 100 1000
    // @Increment: 1
    // @User: Standard
    AP_GROUPINFO("RADIUS",      1, AC_WPNav, _wp_radius_cm, WPNAV_WP_RADIUS),

    // @Param: SPEED_UP
    // @DisplayName: Waypoint Climb Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain while climbing during a WP mission
    // @Units: cm/s
    // @Range: 0 1000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED_UP",    2, AC_WPNav, _wp_speed_up_cms, WPNAV_WP_SPEED_UP),

    // @Param: SPEED_DN
    // @DisplayName: Waypoint Descent Speed Target
    // @Description: Defines the speed in cm/s which the aircraft will attempt to maintain while descending during a WP mission
    // @Units: cm/s
    // @Range: 0 1000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("SPEED_DN",    3, AC_WPNav, _wp_speed_down_cms, WPNAV_WP_SPEED_DOWN),

    // @Param: LOIT_SPEED
    // @DisplayName: Loiter Horizontal Maximum Speed
    // @Description: Defines the maximum speed in cm/s which the aircraft will travel horizontally while in loiter mode
    // @Units: cm/s
    // @Range: 0 2000
    // @Increment: 50
    // @User: Standard
    AP_GROUPINFO("LOIT_SPEED",  4, AC_WPNav, _loiter_speed_cms, WPNAV_LOITER_SPEED),

    // @Param: ACCEL
    // @DisplayName: Waypoint Acceleration 
    // @Description: Defines the horizontal acceleration in cm/s/s used during missions
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ACCEL",       5, AC_WPNav, _wp_accel_cms, WPNAV_ACCELERATION),

    // @Param: ACCEL_Z
    // @DisplayName: Waypoint Vertical Acceleration
    // @Description: Defines the vertical acceleration in cm/s/s used during missions
    // @Units: cm/s/s
    // @Range: 50 500
    // @Increment: 10
    // @User: Standard
    AP_GROUPINFO("ACCEL_Z",     6, AC_WPNav, _wp_accel_z_cms, WPNAV_WP_ACCEL_Z_DEFAULT),

    // @Param: LOIT_JERK
    // @DisplayName: Loiter maximum jerk
    // @Description: Loiter maximum jerk in cm/s/s/s
    // @Units: cm/s/s/s
    // @Range: 500 2000
    // @Increment: 1
    // @User: Advanced
    AP_GROUPINFO("LOIT_JERK",   7, AC_WPNav, _loiter_jerk_max_cmsss, WPNAV_LOITER_JERK_MAX_DEFAULT),

    AP_GROUPEND
};

// Default constructor.
// Note that the Vector/Matrix constructors already implicitly zero
// their values.
//
AC_WPNav::AC_WPNav(const AP_InertialNav& inav, const AP_AHRS& ahrs, AC_PosControl& pos_control) :
    _inav(inav),
    _ahrs(ahrs),
    _pos_control(pos_control),
    _loiter_last_update(0),
    _loiter_step(0),
    _pilot_accel_fwd_cms(0),
    _pilot_accel_rgt_cms(0),
    _loiter_accel_cms(WPNAV_LOITER_ACCEL),
    _wp_last_update(0),
    _wp_step(0),
    _track_length(0.0f),
    _track_desired(0.0f),
    _limited_speed_xy_cms(0.0f),
    _track_accel(0.0f),
    _track_speed(0.0f),
    _track_leash_length(0.0f),
    _slow_down_dist(0.0f),
    _spline_time(0.0f),
    _spline_time_scale(0.0f),
    _spline_vel_scaler(0.0f),
    _yaw(0.0f)
{
    AP_Param::setup_object_defaults(this, var_info);

}

///
/// loiter controller
///

/// init_loiter_target in cm from home
void AC_WPNav::init_loiter_target(const Vector3f& position, bool reset_I)
{
    // if reset_I is false we warn position controller not to reset I terms
    if (!reset_I) {
        _pos_control.keep_xy_I_terms();
    }
    
    // initialise position controller
    _pos_control.init_xy_controller();

    // initialise pos controller speed and acceleration
    _pos_control.set_speed_xy(_loiter_speed_cms);
    _loiter_accel_cms = _loiter_speed_cms/2.0f;
    _pos_control.set_accel_xy(_loiter_accel_cms);

    // set target position
    _pos_control.set_xy_target(position.x, position.y);

    // initialise feed forward velocity to zero
    _pos_control.set_desired_velocity_xy(0,0);

    // initialise desired accel and add fake wind
    _loiter_desired_accel.x = 0;
    _loiter_desired_accel.y = 0;

    // initialise pilot input
    _pilot_accel_fwd_cms = 0;
    _pilot_accel_rgt_cms = 0;
}

/// init_loiter_target - initialize's loiter position and feed-forward velocity from current pos and velocity
void AC_WPNav::init_loiter_target()
{
    const Vector3f& curr_pos = _inav.get_position();
    const Vector3f& curr_vel = _inav.get_velocity();

    // initialise position controller
    _pos_control.init_xy_controller();

    // initialise pos controller speed and acceleration
    _pos_control.set_speed_xy(_loiter_speed_cms);
    _loiter_accel_cms = _loiter_speed_cms/2.0f;
    _pos_control.set_accel_xy(_loiter_accel_cms);

    // set target position
    _pos_control.set_xy_target(curr_pos.x, curr_pos.y);

    // move current vehicle velocity into feed forward velocity
    _pos_control.set_desired_velocity_xy(curr_vel.x, curr_vel.y);

    // initialise desired accel and add fake wind
    _loiter_desired_accel.x = (_loiter_accel_cms)*curr_vel.x/_loiter_speed_cms;
    _loiter_desired_accel.y = (_loiter_accel_cms)*curr_vel.y/_loiter_speed_cms;

    // initialise pilot input
    _pilot_accel_fwd_cms = 0;
    _pilot_accel_rgt_cms = 0;
}

/// loiter_soften_for_landing - reduce response for landing
void AC_WPNav::loiter_soften_for_landing()
{
    const Vector3f& curr_pos = _inav.get_position();

    // set target position to current position
    _pos_control.set_xy_target(curr_pos.x, curr_pos.y);
    _pos_control.freeze_ff_xy();
}

/// set_loiter_velocity - allows main code to pass the maximum velocity for loiter
void AC_WPNav::set_loiter_velocity(float velocity_cms)
{
    // range check velocity and update position controller
    if (velocity_cms >= WPNAV_LOITER_SPEED_MIN) {
        _loiter_speed_cms = velocity_cms;

        // initialise pos controller speed
        _pos_control.set_speed_xy(_loiter_speed_cms);

        // initialise pos controller acceleration
        _loiter_accel_cms = _loiter_speed_cms/2.0f;
        _pos_control.set_accel_xy(_loiter_accel_cms);
    }
}

/// set_pilot_desired_acceleration - sets pilot desired acceleration from roll and pitch stick input
void AC_WPNav::set_pilot_desired_acceleration(float control_roll, float control_pitch)
{
    // convert pilot input to desired acceleration in cm/s/s
    _pilot_accel_fwd_cms = -control_pitch * _loiter_accel_cms / 4500.0f;
    _pilot_accel_rgt_cms = control_roll * _loiter_accel_cms / 4500.0f;
}

/// get_loiter_stopping_point_xy - returns vector to stopping point based on a horizontal position and velocity
void AC_WPNav::get_loiter_stopping_point_xy(Vector3f& stopping_point) const
{
	_pos_control.get_stopping_point_xy(stopping_point);
}

/// calc_loiter_desired_velocity - updates desired velocity (i.e. feed forward) with pilot requested acceleration and fake wind resistance
///		updated velocity sent directly to position controller
void AC_WPNav::calc_loiter_desired_velocity(float nav_dt)
{
    // range check nav_dt
    if( nav_dt < 0 ) {
        return;
    }

    // check loiter speed and avoid divide by zero
    if( _loiter_speed_cms < WPNAV_LOITER_SPEED_MIN) {
        _loiter_speed_cms = WPNAV_LOITER_SPEED_MIN;
        _loiter_accel_cms = _loiter_speed_cms/2.0f;
    }

    // rotate pilot input to lat/lon frame
    Vector2f desired_accel;
    desired_accel.x = (_pilot_accel_fwd_cms*_ahrs.cos_yaw() - _pilot_accel_rgt_cms*_ahrs.sin_yaw());
    desired_accel.y = (_pilot_accel_fwd_cms*_ahrs.sin_yaw() + _pilot_accel_rgt_cms*_ahrs.cos_yaw());

    // calculate the difference
    Vector2f des_accel_diff = (desired_accel - _loiter_desired_accel);

    // constrain and scale the desired acceleration
    float des_accel_change_total = pythagorous2(des_accel_diff.x, des_accel_diff.y);
    float accel_change_max = _loiter_jerk_max_cmsss * nav_dt;
    if (des_accel_change_total > accel_change_max && des_accel_change_total > 0.0f) {
        des_accel_diff.x = accel_change_max * des_accel_diff.x/des_accel_change_total;
        des_accel_diff.y = accel_change_max * des_accel_diff.y/des_accel_change_total;
    }
    // adjust the desired acceleration
    _loiter_desired_accel += des_accel_diff;

    // get pos_control's feed forward velocity
    Vector3f desired_vel = _pos_control.get_desired_velocity();

    // add pilot commanded acceleration
    desired_vel.x += _loiter_desired_accel.x * nav_dt;
    desired_vel.y += _loiter_desired_accel.y * nav_dt;

    // reduce velocity with fake wind resistance
    if (_pilot_accel_fwd_cms != 0.0f || _pilot_accel_rgt_cms != 0.0f) {
        desired_vel.x -= (_loiter_accel_cms)*nav_dt*desired_vel.x/_loiter_speed_cms;
        desired_vel.y -= (_loiter_accel_cms)*nav_dt*desired_vel.y/_loiter_speed_cms;
    } else {
        desired_vel.x -= (_loiter_accel_cms-WPNAV_LOITER_ACCEL_MIN)*nav_dt*desired_vel.x/_loiter_speed_cms;
        if(desired_vel.x > 0 ) {
            desired_vel.x = max(desired_vel.x - WPNAV_LOITER_ACCEL_MIN*nav_dt, 0);
        }else if(desired_vel.x < 0) {
            desired_vel.x = min(desired_vel.x + WPNAV_LOITER_ACCEL_MIN*nav_dt, 0);
        }
        desired_vel.y -= (_loiter_accel_cms-WPNAV_LOITER_ACCEL_MIN)*nav_dt*desired_vel.y/_loiter_speed_cms;
        if(desired_vel.y > 0 ) {
            desired_vel.y = max(desired_vel.y - WPNAV_LOITER_ACCEL_MIN*nav_dt, 0);
        }else if(desired_vel.y < 0) {
            desired_vel.y = min(desired_vel.y + WPNAV_LOITER_ACCEL_MIN*nav_dt, 0);
        }
    }

    // send adjusted feed forward velocity back to position controller
    _pos_control.set_desired_velocity_xy(desired_vel.x,desired_vel.y);
}

/// get_bearing_to_target - get bearing to loiter target in centi-degrees
int32_t AC_WPNav::get_loiter_bearing_to_target() const
{
    return get_bearing_cd(_inav.get_position(), _pos_control.get_pos_target());
}

/// update_loiter - run the loiter controller - should be called at 100hz
void AC_WPNav::update_loiter()
{
    // calculate dt
    uint32_t now = hal.scheduler->millis();
    float dt = (now - _loiter_last_update) / 1000.0f;

    // reset step back to 0 if 0.1 seconds has passed and we completed the last full cycle
    if (dt >= WPNAV_LOITER_UPDATE_TIME) {
        // double check dt is reasonable
        if (dt >= 1.0f) {
            dt = 0.0;
        }
        // capture time since last iteration
        _loiter_last_update = now;
        // translate any adjustments from pilot to loiter target
        calc_loiter_desired_velocity(dt);
        // trigger position controller on next update
        _pos_control.trigger_xy();
    }else{
        // run horizontal position controller
        _pos_control.update_xy_controller(true);
    }
}


///
/// waypoint navigation
///

/// wp_and_spline_init - initialise straight line and spline waypoint controllers
///     updates target roll, pitch targets and I terms based on vehicle lean angles
///     should be called once before the waypoint controller is used but does not need to be called before subsequent updates to destination
void AC_WPNav::wp_and_spline_init()
{
    // check _wp_accel_cms is reasonable
    if (_wp_accel_cms <= 0) {
        _wp_accel_cms.set_and_save(WPNAV_ACCELERATION);
    }

    // initialise position controller
    _pos_control.init_xy_controller();

    // initialise position controller speed and acceleration
    _pos_control.set_speed_xy(_wp_speed_cms);
    _pos_control.set_accel_xy(_wp_accel_cms);
    _pos_control.set_speed_z(-_wp_speed_down_cms, _wp_speed_up_cms);
    _pos_control.set_accel_z(_wp_accel_z_cms);
    _pos_control.calc_leash_length_xy();
    _pos_control.calc_leash_length_z();
}

/// set_speed_xy - allows main code to pass target horizontal velocity for wp navigation
void AC_WPNav::set_speed_xy(float speed_cms)
{
    // range check new target speed and update position controller
    if (speed_cms >= WPNAV_WP_SPEED_MIN) {
        _wp_speed_cms = speed_cms;
        _pos_control.set_speed_xy(_wp_speed_cms);
        // flag that wp leash must be recalculated
        _flags.recalc_wp_leash = true;
    }
}

/// set_destination - set destination using cm from home
void AC_WPNav::set_wp_destination(const Vector3f& destination)
{
	Vector3f origin;

    // if waypoint controller is active use the existing position target as the origin
    if ((hal.scheduler->millis() - _wp_last_update) < 1000) {
        origin = _pos_control.get_pos_target();
    } else {
        // if waypoint controller is not active, set origin to reasonable stopping point (using curr pos and velocity)
        _pos_control.get_stopping_point_xy(origin);
        _pos_control.get_stopping_point_z(origin);
    }

    // set origin and destination
    set_wp_origin_and_destination(origin, destination);
}

/// set_origin_and_destination - set origin and destination using lat/lon coordinates
void AC_WPNav::set_wp_origin_and_destination(const Vector3f& origin, const Vector3f& destination)
{
    // store origin and destination locations
    _origin = origin;
    _destination = destination;
    Vector3f pos_delta = _destination - _origin;

    _track_length = pos_delta.length(); // get track length

    // calculate each axis' percentage of the total distance to the destination
    if (_track_length == 0.0f) {
        // avoid possible divide by zero
        _pos_delta_unit.x = 0;
        _pos_delta_unit.y = 0;
        _pos_delta_unit.z = 0;
    }else{
        _pos_delta_unit = pos_delta/_track_length;
    }

    // calculate leash lengths
    calculate_wp_leash_length();

    // initialise yaw heading
    if (_track_length >= WPNAV_YAW_DIST_MIN) {
        _yaw = get_bearing_cd(_origin, _destination);
    } else {
        // set target yaw to current heading.  Alternatively we could pull this from the attitude controller if we had access to it
        _yaw = _ahrs.yaw_sensor;
    }

    // initialise intermediate point to the origin
    _pos_control.set_pos_target(origin);
    _track_desired = 0;             // target is at beginning of track
    _flags.reached_destination = false;
    _flags.fast_waypoint = false;   // default waypoint back to slow
    _flags.slowing_down = false;    // target is not slowing down yet
    _flags.segment_type = SEGMENT_STRAIGHT;
    _flags.new_wp_destination = true;   // flag new waypoint so we can freeze the pos controller's feed forward and smooth the transition

    // initialise the limited speed to current speed along the track
    const Vector3f &curr_vel = _inav.get_velocity();

    // get speed along track (note: we convert vertical speed into horizontal speed equivalent)
    float speed_along_track = curr_vel.x * _pos_delta_unit.x + curr_vel.y * _pos_delta_unit.y + curr_vel.z * _pos_delta_unit.z;
    _limited_speed_xy_cms = constrain_float(speed_along_track,0,_wp_speed_cms);
}

/// shift_wp_origin_to_current_pos - shifts the origin and destination so the origin starts at the current position
///     used to reset the position just before takeoff
///     relies on set_wp_destination or set_wp_origin_and_destination having been called first
void AC_WPNav::shift_wp_origin_to_current_pos()
{
    // return immediately if vehicle is not at the origin
    if (_track_desired > 0.0f) {
        return;
    }

    // get current and target locations
    const Vector3f curr_pos = _inav.get_position();
    const Vector3f pos_target = _pos_control.get_pos_target();

    // calculate difference between current position and target
    Vector3f pos_diff = curr_pos - pos_target;

    // shift origin and destination
    _origin += pos_diff;
    _destination += pos_diff;

    // move pos controller target and disable feed forward
    _pos_control.set_pos_target(curr_pos);
    _pos_control.freeze_ff_xy();
    _pos_control.freeze_ff_z();
}
/// **********************************************************************************************
///
/// 					get_wp_stopping_point_xy -
///
/// **********************************************************************************************
/// get_wp_stopping_point_xy - returns vector to stopping point based on a horizontal position and velocity
void AC_WPNav::get_wp_stopping_point_xy(Vector3f& stopping_point) const
{
	_pos_control.get_stopping_point_xy(stopping_point);
}
/// **********************************************************************************************
///
/// 					advance_wp_target_along_track -
///
/// **********************************************************************************************
/// move a target point along track from origin to destination
/// This is here the magic happens in the wp code.
/// It determines the new position for the target. the AC_Pos code will then make the vehicle move
/// towards that target.
/// As the vehicle gets closer to the target, the target will move away from the vehicle but it will stay on the track.
///
/// The leash is the maximum distance between the vehicle and target.
/// The target can only move to the extents of the leash an then it has to stop
///
/// Called on a timed schedule by AC_WPNav.update_nav().

void AC_WPNav::advance_wp_target_along_track(float dt)
{
	// distance (cm) along the track from the origin that the vehicle has traveled. This ignores the track error.
	float track_distance_covered;

	// distance error (cm) from the track position and the vehicle ( how far off track the vehicle is sitting)
	Vector3f track_error;

	// the farthest distance (cm) ahead of the vehicles present track position that the next target position can lie.
	// This ignores track error.
    float next_target_position_on_track_max;

    // distance (cm)ahead of the vehicles present track position that the next target position can lie .
    // This takes into account the vehicles present track error.
    float track_leash_slack;

    // true when track error is beyond the leash limit and we need to head straight to the track
     bool reached_leash_limit = false;

    // Get the vehicles current location relative to the home location.
    Vector3f curr_pos = _inav.get_position();

    // Now work out where the vehicle is positioned relative to the track origin.
    // The aircraft could be several meters off track to one side.
    // curr_delta is the shortest distance between the track origin and the vehicles current position.
    Vector3f curr_delta = curr_pos - _origin;

    // calculate how far along the track the vehicle is positioned.
    // It could be several meters to one side of track due to drift.
    // We want to know the distance along the track that the vehicle has moved ignoring the track error.
    track_distance_covered = curr_delta.x * _pos_delta_unit.x + curr_delta.y * _pos_delta_unit.y + curr_delta.z * _pos_delta_unit.z;

    // The point on the track where the vehicle would be sitting if we ignored the track error.
    Vector3f track_covered_pos = _pos_delta_unit * track_distance_covered;


    // Knowing how far along the track the vehicle is, We can calculate how far off track it is.
    // track_error represents x,y,z in cm and is the shortest distance from the track. (90 deg to the track)
    track_error = curr_delta - track_covered_pos;

    // calculate the horizontal track error that is 90 deg to the track.
    // we don't care which of the track it is on. The result will always be positive.
    float track_error_xy = pythagorous2(track_error.x, track_error.y);

    // calculate the vertical track error, as for track_error_xy. It is a vertical and always positive distance from the track.
    float track_error_z = fabsf(track_error.z);


    // A target is a point ahead of the vehicles current position.
    // That point should always be between the current position and destination.
    // Think of a dog on a leash racing ahead of you trying to get onto the path and head home.
    // you only allow him to go so far on his leash.
    // Depending on how long the leash is, we can determine where the next target position should be on the track
    // The aim is to determine the best way for the vehicle to get back on track and move towards the destination.
    // A short leash will force the vehicle to regain track sooner before moving on.

    //The leash length is determined by a number of user defined values (USER_WP_SPEED, USER_WP_ACCELL and USER_POS_XY_KP)

    float leash_xy = _pos_control.get_leash_xy();
    float leash_z;
    // If the vehicle is above or below the track, we have two different leashes lengths.
    if (track_error.z >= 0)
    	{
        leash_z = _pos_control.get_leash_up_z();
    	}
    else
    	{
        leash_z = _pos_control.get_leash_down_z();
    	}

    // calculate how far along the track we could move the new target position before reaching the end of the leash
    track_leash_slack = min(_track_leash_length*(leash_z-track_error_z)/leash_z ,
    							_track_leash_length*(leash_xy-track_error_xy)/leash_xy);

    if (track_leash_slack < 0)
    	{
        next_target_position_on_track_max = track_distance_covered;
    	}
    else{
        next_target_position_on_track_max = track_distance_covered + track_leash_slack;
    	}

    // If the vehicle is off track by a distance greater then the leash, then we have special case.
    // We want to then head directly (at 90 deg towards the track) to regain track.
   // So check if target is beyond the leash
    if (_track_desired > next_target_position_on_track_max) {
        reached_leash_limit = true;
    }

    // The next stage is to work out the desired velocity that we want the vehicle to move at along the track

    // Get current velocity
    // This is the vehicles velocity in x,y,z relative to the earth (Earth frame - ef)
    // x is the latitude (North bound =  pos - South bound = neg)
    // y is the longitude ( East bound = pos - West bound =  neg)
    // z is the climb rate (Up is positive, Down is neg)
    const Vector3f &curr_vel = _inav.get_velocity();

    // We want to know the vehicles velocity along the track, not in the x,y,z - ef
    // _pos_delta is the % of each axis that makes up the track from origin to destination.
    // If the track is North-East, X axis and Y axis would be 50% each (0.5f),
    // If the track is South-East, X axis is -50% and Y axis is 50%

    // if speed_along_track is positive, we are heading towards the destination ( A good thing)
    // if negative we are heading back to the origin ( Something is not going well here!)
    float speed_along_track = curr_vel.x * _pos_delta_unit.x +
    							curr_vel.y * _pos_delta_unit.y +
									curr_vel.z * _pos_delta_unit.z;

   
    // _wp_speed_cms is the user defined maximum track speed, comes from the WPNAV_SPEED setting.
    // it can be overridden at run time by AC_WPNav::set_speed_xy()
    //
    // speed_desired_along_track - this is the desired speed in the horizontal plane (xy), its ignores the vertical (z).
    // and is always positive in value (direction-less)
    float speed_desired_along_track = _wp_speed_cms;

   
    // if the vel_along_track is negative ( below zero ), we may be heading back towards the origin,
    // we may also have just gotten a bad result or the vehicle has slipped back just slightly due to gust of wind.
    // We need a bit of tolerance before we panic and change things.
    // We can use the negative value of vel_desired_along_track to create a lower limit and add some tolerance.
    float speed_along_track_lower_limit = -(speed_desired_along_track);

    if (speed_along_track < speed_along_track_lower_limit)
    	{
        // we are traveling back towards the origin - We don't want that!! Now we can panic!!!!
        //_limited_speed_xy_cms = 0;
    	_limited_speed_xy_cms = _track_speed * 0.1f; //Ah- i think this is creating a stall at lower wp_speeds

    	}
    else
    	{
        // increase intermediate target point's velocity if not yet at the leash limit
        if(dt > 0 && !reached_leash_limit) 
        	{
            _limited_speed_xy_cms += 2.0f * _track_accel * dt;
        	}
        
        // do not allow speed to be below zero or over top speed
        _limited_speed_xy_cms = constrain_float(_limited_speed_xy_cms, 0.0f, _track_speed);

        // check if we should begin slowing down
        if (!_flags.fast_waypoint) 
        	{
            float dist_to_dest = _track_length - _track_desired;
            
            if (!_flags.slowing_down && dist_to_dest <= _slow_down_dist) 
            	{
                _flags.slowing_down = true;
            	}
            // if target is slowing down, limit the speed
            if (_flags.slowing_down) 
            	{
                _limited_speed_xy_cms = min(_limited_speed_xy_cms, get_slow_down_speed(dist_to_dest, _track_accel));
            	}
        	}

        // if our current velocity is within the linear velocity range limit the intermediate point's velocity to be no more than the linear_velocity above or below our current velocity
        if (fabsf(speed_along_track) < speed_desired_along_track)
        	{
            _limited_speed_xy_cms = constrain_float(_limited_speed_xy_cms,
            											speed_along_track-speed_desired_along_track,
															speed_along_track + speed_desired_along_track );
        	}
    	}
    
    // advance the current target
    if (!reached_leash_limit)
    	{
    	_track_desired += _limited_speed_xy_cms * dt;

    	// reduce speed if we reach end of leash
        if (_track_desired > next_target_position_on_track_max)
        	{
        	_track_desired = next_target_position_on_track_max;
        	_limited_speed_xy_cms -= 2.0f * _track_accel * dt;

        	if (_limited_speed_xy_cms < 0.0f) _limited_speed_xy_cms = 0.0f;
        	}
    	}

    // do not let desired point go past the end of the track unless it's a fast waypoint
    if (!_flags.fast_waypoint) {
        _track_desired = constrain_float(_track_desired, 0, _track_length);
    } else {
        _track_desired = constrain_float(_track_desired, 0, _track_length + WPNAV_WP_FAST_OVERSHOOT_MAX);
    }

    // recalculate the desired position
    _pos_control.set_pos_target(_origin + _pos_delta_unit * _track_desired);

    // check if we've reached the waypoint
    if( !_flags.reached_destination ) {
        if( _track_desired >= _track_length ) {
            // "fast" waypoints are complete once the intermediate point reaches the destination
            if (_flags.fast_waypoint) {
                _flags.reached_destination = true;
            }else{
                // regular waypoints also require the copter to be within the waypoint radius
                Vector3f dist_to_dest = curr_pos - _destination;
                if( dist_to_dest.length() <= _wp_radius_cm ) {
                    _flags.reached_destination = true;
                }
            }
        }
    }
}

/// get_wp_distance_to_destination - get horizontal distance to destination in cm
float AC_WPNav::get_wp_distance_to_destination() const
{
    // get current location
    Vector3f curr = _inav.get_position();
    return pythagorous2(_destination.x-curr.x,_destination.y-curr.y);
}

/// get_wp_bearing_to_destination - get bearing to next waypoint in centi-degrees
int32_t AC_WPNav::get_wp_bearing_to_destination() const
{
    return get_bearing_cd(_inav.get_position(), _destination);
}
/// ******************************************************************************
///
/// 								update_wpnav
///
/// ******************************************************************************
//run the wp controller - should be called at 100hz or higher (10 ms or less)
void AC_WPNav::update_wpnav()
{
    // calculate dt - result will be in seconds
	// _wp_last_update is stoed in  milli seconds
	uint32_t now = hal.scheduler->millis();
    float dt = (now - _wp_last_update) / 1000.0f;




    // the APM defines a update rate of 0.095 seconds ( dont know why it is not rounded to 0.1 seconds (10hz)
    // the Pixhawk is using 0.02 seconds or (50hz) - since it has processing power.

    if (dt >= WPNAV_WP_UPDATE_TIME)
    	{

    	// capture time since last iteration
    	_wp_last_update = now;

    	// double check dt is reasonable
    	// If for some reason we have missed the timing slot and dt has gotten too big,
    	// then, we will reset dt to 0.
    	// dt is passed into advance_wp_target_along_track and used to advance the target along the track
    	// and set the desired velocity.
    	// If it is too big we can get errors.
        if (dt >= 1.0f) dt = 0.0;

        // advance the target if necessary
        advance_wp_target_along_track(dt);

        // Indicate to the position controller that there may be a new target for it to track
        _pos_control.trigger_xy();


        if (_flags.new_wp_destination)
        	{
            _flags.new_wp_destination = false;
            _pos_control.freeze_ff_xy();
		    _pos_control.freeze_ff_z(); //Moved here by AH, See below
        	}
        // AH - this in wrong spot - should be above
        //_pos_control.freeze_ff_z();
    	}
    else
    	{ // Not time to update the target so do other stuff
        // run horizontal position controller
        _pos_control.update_xy_controller(false);

        // check if leash lengths need updating
        check_wp_leash_length();
    	}
}

// check_wp_leash_length - check if waypoint leash lengths need to be recalculated
//  should be called after _pos_control.update_xy_controller which may have changed the position controller leash lengths
void AC_WPNav::check_wp_leash_length()
{
    // exit immediately if recalc is not required
    if (_flags.recalc_wp_leash) {
        calculate_wp_leash_length();
    }
}

/// calculate_wp_leash_length - calculates horizontal and vertical leash lengths for waypoint controller
void AC_WPNav::calculate_wp_leash_length()
{
    // length of the unit direction vector in the horizontal
    float pos_delta_unit_xy = pythagorous2(_pos_delta_unit.x, _pos_delta_unit.y);

    float pos_delta_unit_z = fabsf(_pos_delta_unit.z);

    float speed_z;
    float leash_z;
    if (_pos_delta_unit.z >= 0) {
        speed_z = _wp_speed_up_cms;
        leash_z = _pos_control.get_leash_up_z();
    }else{
        speed_z = _wp_speed_down_cms;
        leash_z = _pos_control.get_leash_down_z();
    }

    // calculate the maximum acceleration, maximum velocity, and leash length in the direction of travel
    if(pos_delta_unit_z == 0 && pos_delta_unit_xy == 0){
        _track_accel = 0;
        _track_speed = 0;
        _track_leash_length = WPNAV_LEASH_LENGTH_MIN;
    }else if(_pos_delta_unit.z <= 0.2f){
        _track_accel = _wp_accel_cms/pos_delta_unit_xy;
        _track_speed = _wp_speed_cms/pos_delta_unit_xy;
        _track_leash_length = _pos_control.get_leash_xy()/pos_delta_unit_xy;
    }else if(pos_delta_unit_xy == 0){
        _track_accel = _wp_accel_z_cms/pos_delta_unit_z;
        _track_speed = speed_z/pos_delta_unit_z;
        _track_leash_length = leash_z/pos_delta_unit_z;
    }else{
        _track_accel = min(_wp_accel_z_cms/pos_delta_unit_z, _wp_accel_cms/pos_delta_unit_xy);
        _track_speed = min(speed_z/pos_delta_unit_z, _wp_speed_cms/pos_delta_unit_xy);
        _track_leash_length = min(leash_z/pos_delta_unit_z, _pos_control.get_leash_xy()/pos_delta_unit_xy);
    }

    // calculate slow down distance (the distance from the destination when the target point should begin to slow down)
    calc_slow_down_distance(_track_speed, _track_accel);

    // set recalc leash flag to false
    _flags.recalc_wp_leash = false;
}

///
/// spline methods
///

/// set_spline_destination waypoint using position vector (distance from home in cm)
///     stopped_at_start should be set to true if vehicle is stopped at the origin
///     seg_end_type should be set to stopped, straight or spline depending upon the next segment's type
///     next_destination should be set to the next segment's destination if the seg_end_type is SEGMENT_END_STRAIGHT or SEGMENT_END_SPLINE
void AC_WPNav::set_spline_destination(const Vector3f& destination, bool stopped_at_start, spline_segment_end_type seg_end_type, const Vector3f& next_destination)
{
    Vector3f origin;

    // if waypoint controller is active and copter has reached the previous waypoint use current pos target as the origin
    if ((hal.scheduler->millis() - _wp_last_update) < 1000) {
        origin = _pos_control.get_pos_target();
    }else{
        // otherwise calculate origin from the current position and velocity
        _pos_control.get_stopping_point_xy(origin);
        _pos_control.get_stopping_point_z(origin);
    }

    // set origin and destination
    set_spline_origin_and_destination(origin, destination, stopped_at_start, seg_end_type, next_destination);
}

/// set_spline_origin_and_destination - set origin and destination waypoints using position vectors (distance from home in cm)
///     seg_type should be calculated by calling function based on the mission
void AC_WPNav::set_spline_origin_and_destination(const Vector3f& origin, const Vector3f& destination, bool stopped_at_start, spline_segment_end_type seg_end_type, const Vector3f& next_destination)
{
    // mission is "active" if wpnav has been called recently and vehicle reached the previous waypoint
    bool prev_segment_exists = (_flags.reached_destination && ((hal.scheduler->millis() - _wp_last_update) < 1000));

    // check _wp_accel_cms is reasonable to avoid divide by zero
    if (_wp_accel_cms <= 0) {
        _wp_accel_cms.set_and_save(WPNAV_ACCELERATION);
    }

    // segment start types
    // stop - vehicle is not moving at origin
    // straight-fast - vehicle is moving, previous segment is straight.  vehicle will fly straight through the waypoint before beginning it's spline path to the next wp
    //     _flag.segment_type holds whether prev segment is straight vs spline but we don't know if it has a delay
    // spline-fast - vehicle is moving, previous segment is splined, vehicle will fly through waypoint but previous segment should have it flying in the correct direction (i.e. exactly parallel to position difference vector from previous segment's origin to this segment's destination)

    // calculate spline velocity at origin
    if (stopped_at_start || !prev_segment_exists) {
    	// if vehicle is stopped at the origin, set origin velocity to 0.1 * distance vector from origin to destination
    	_spline_origin_vel = (destination - origin) * 0.1f;
    	_spline_time = 0.0f;
    	_spline_vel_scaler = 0.0f;
    }else{
    	// look at previous segment to determine velocity at origin
        if (_flags.segment_type == SEGMENT_STRAIGHT) {
            // previous segment is straight, vehicle is moving so vehicle should fly straight through the origin
            // before beginning it's spline path to the next waypoint. Note: we are using the previous segment's origin and destination
            _spline_origin_vel = (_destination - _origin);
            _spline_time = 0.0f;	// To-Do: this should be set based on how much overrun there was from straight segment?
            _spline_vel_scaler = _pos_control.get_vel_target().length();    // start velocity target from current target velocity
        }else{
            // previous segment is splined, vehicle will fly through origin
            // we can use the previous segment's destination velocity as this segment's origin velocity
            // Note: previous segment will leave destination velocity parallel to position difference vector
            //       from previous segment's origin to this segment's destination)
            _spline_origin_vel = _spline_destination_vel;
            if (_spline_time > 1.0f && _spline_time < 1.1f) {    // To-Do: remove hard coded 1.1f
                _spline_time -= 1.0f;
            }else{
                _spline_time = 0.0f;
            }
            // Note: we leave _spline_vel_scaler as it was from end of previous segment
        }
    }

    // calculate spline velocity at destination
    switch (seg_end_type) {

    case SEGMENT_END_STOP:
        // if vehicle stops at the destination set destination velocity to 0.1 * distance vector from origin to destination
        _spline_destination_vel = (destination - origin) * 0.1f;
        _flags.fast_waypoint = false;
        break;

    case SEGMENT_END_STRAIGHT:
        // if next segment is straight, vehicle's final velocity should face along the next segment's position
        _spline_destination_vel = (next_destination - destination);
        _flags.fast_waypoint = true;
        break;

    case SEGMENT_END_SPLINE:
        // if next segment is splined, vehicle's final velocity should face parallel to the line from the origin to the next destination
        _spline_destination_vel = (next_destination - origin);
        _flags.fast_waypoint = true;
        break;
    }

    // code below ensures we don't get too much overshoot when the next segment is short
    float vel_len = (_spline_origin_vel + _spline_destination_vel).length();
    float pos_len = (destination - origin).length() * 4.0f;
    if (vel_len > pos_len) {
        // if total start+stop velocity is more than twice position difference
        // use a scaled down start and stop velocityscale the  start and stop velocities down
        float vel_scaling = pos_len / vel_len;
        // update spline calculator
        update_spline_solution(origin, destination, _spline_origin_vel * vel_scaling, _spline_destination_vel * vel_scaling);
    }else{
        // update spline calculator
        update_spline_solution(origin, destination, _spline_origin_vel, _spline_destination_vel);
    }

    // initialise yaw heading to current heading
    _yaw = _ahrs.yaw_sensor;

    // store origin and destination locations
    _origin = origin;
    _destination = destination;

    // calculate slow down distance
    calc_slow_down_distance(_wp_speed_cms, _wp_accel_cms);

    // initialise intermediate point to the origin
    _pos_control.set_pos_target(origin);
    _flags.reached_destination = false;
    _flags.segment_type = SEGMENT_SPLINE;
    _flags.new_wp_destination = true;   // flag new waypoint so we can freeze the pos controller's feed forward and smooth the transition
}

/// update_spline - update spline controller
void AC_WPNav::update_spline()
{
    // exit immediately if this is not a spline segment
    if (_flags.segment_type != SEGMENT_SPLINE) {
        return;
    }

    // calculate dt
    uint32_t now = hal.scheduler->millis();
    float dt = (now - _wp_last_update) / 1000.0f;

    // reset step back to 0 if 0.1 seconds has passed and we completed the last full cycle
    if (dt >= WPNAV_WP_UPDATE_TIME) {
        // double check dt is reasonable
        if (dt >= 1.0f) {
            dt = 0.0;
        }
        // capture time since last iteration
        _wp_last_update = now;

        // advance the target if necessary
        advance_spline_target_along_track(dt);
        _pos_control.trigger_xy();
        if (_flags.new_wp_destination) {
            _flags.new_wp_destination = false;
            _pos_control.freeze_ff_xy();
        }
        _pos_control.freeze_ff_z();
    }else{
        // run horizontal position controller
        _pos_control.update_xy_controller(false);
    }
}

/// update_spline_solution - recalculates hermite_spline_solution grid
///		relies on _spline_origin_vel, _spline_destination_vel and _origin and _destination
void AC_WPNav::update_spline_solution(const Vector3f& origin, const Vector3f& dest, const Vector3f& origin_vel, const Vector3f& dest_vel)
{
    _hermite_spline_solution[0] = origin;
    _hermite_spline_solution[1] = origin_vel;
    _hermite_spline_solution[2] = -origin*3.0f -origin_vel*2.0f + dest*3.0f - dest_vel;
    _hermite_spline_solution[3] = origin*2.0f + origin_vel -dest*2.0f + dest_vel;
 }

/// advance_spline_target_along_track - move target location along track from origin to destination
void AC_WPNav::advance_spline_target_along_track(float dt)
{
    if (!_flags.reached_destination) {
        Vector3f target_pos, target_vel;

        // update target position and velocity from spline calculator
        calc_spline_pos_vel(_spline_time, target_pos, target_vel);

        // update velocity
        float spline_dist_to_wp = (_destination - target_pos).length();

        // if within the stopping distance from destination, set target velocity to sqrt of distance * 2 * acceleration
        if (!_flags.fast_waypoint && spline_dist_to_wp < _slow_down_dist) {
            _spline_vel_scaler = safe_sqrt(spline_dist_to_wp * 2.0f * _wp_accel_cms);
        }else if(_spline_vel_scaler < _wp_speed_cms) {
            // increase velocity using acceleration
        	// To-Do: replace 0.1f below with update frequency passed in from main program
            _spline_vel_scaler += _wp_accel_cms* 0.1f;
        }

        // constrain target velocity
        if (_spline_vel_scaler > _wp_speed_cms) {
            _spline_vel_scaler = _wp_speed_cms;
        }

        // scale the spline_time by the velocity we've calculated vs the velocity that came out of the spline calculator
        float target_vel_length = target_vel.length();
        if (target_vel_length != 0.0f) {
            _spline_time_scale = _spline_vel_scaler/target_vel_length;
        }

        // update target position
        _pos_control.set_pos_target(target_pos);

        // update the yaw
        _yaw = RadiansToCentiDegrees(fast_atan2(target_vel.y,target_vel.x));

        // advance spline time to next step
        _spline_time += _spline_time_scale*dt;

        // we will reach the next waypoint in the next step so set reached_destination flag
        // To-Do: is this one step too early?
        if (_spline_time >= 1.0f) {
            _flags.reached_destination = true;
        }
    }
}

// calc_spline_pos_vel_accel - calculates target position, velocity and acceleration for the given "spline_time"
/// 	relies on update_spline_solution being called when the segment's origin and destination were set
void AC_WPNav::calc_spline_pos_vel(float spline_time, Vector3f& position, Vector3f& velocity)
{
    float spline_time_sqrd = spline_time * spline_time;
    float spline_time_cubed = spline_time_sqrd * spline_time;

    position = _hermite_spline_solution[0] + \
               _hermite_spline_solution[1] * spline_time + \
               _hermite_spline_solution[2] * spline_time_sqrd + \
               _hermite_spline_solution[3] * spline_time_cubed;

    velocity = _hermite_spline_solution[1] + \
               _hermite_spline_solution[2] * 2.0f * spline_time + \
               _hermite_spline_solution[3] * 3.0f * spline_time_sqrd;
}


///
/// shared methods
///

// get_bearing_cd - return bearing in centi-degrees between two positions
// To-Do: move this to math library
float AC_WPNav::get_bearing_cd(const Vector3f &origin, const Vector3f &destination) const
{
    float bearing = 9000 + fast_atan2(-(destination.x-origin.x), destination.y-origin.y) * 5729.57795f;
    if (bearing < 0) {
        bearing += 36000;
    }
    return bearing;
}

/// calc_slow_down_distance - calculates distance before waypoint that target point should begin to slow-down assuming it is travelling at full speed
void AC_WPNav::calc_slow_down_distance(float speed_cms, float accel_cmss)
{
	// protect against divide by zero
	if (accel_cmss <= 0.0f) {
		_slow_down_dist = 0.0f;
		return;
	}
    // To-Do: should we use a combination of horizontal and vertical speeds?
    // To-Do: update this automatically when speed or acceleration is changed
    _slow_down_dist = speed_cms * speed_cms / (4.0f*accel_cmss);
}

/// get_slow_down_speed - returns target speed of target point based on distance from the destination (in cm)
float AC_WPNav::get_slow_down_speed(float dist_from_dest_cm, float accel_cmss)
{
    // return immediately if distance is zero (or less)
    if (dist_from_dest_cm <= 0) {
        return WPNAV_WP_TRACK_SPEED_MIN;
    }

    // calculate desired speed near destination
    float target_speed = safe_sqrt(dist_from_dest_cm * 4.0f * accel_cmss);

    // ensure desired speed never becomes too low
    if (target_speed < WPNAV_WP_TRACK_SPEED_MIN) {
        return WPNAV_WP_TRACK_SPEED_MIN;
    } else {
        return target_speed;
    }
}
