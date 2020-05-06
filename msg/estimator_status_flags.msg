uint64 timestamp		# time since system start (microseconds)


# GPS check fail flags to indicate status of GPS checks - see definition below
bool gps_check_fail_gps_fix          # 0 : insufficient fix type (no 3D solution)
bool gps_check_fail_min_sat_count    # 1 : minimum required sat count fail
bool gps_check_fail_min_pdop         # 2 : minimum required PDOP fail
bool gps_check_fail_max_horz_err     # 3 : maximum allowed horizontal position error fail
bool gps_check_fail_max_vert_err     # 4 : maximum allowed vertical position error fail
bool gps_check_fail_max_spd_err      # 5 : maximum allowed speed error fail
bool gps_check_fail_max_horz_drift   # 6 : maximum allowed horizontal position drift fail - requires stationary vehicle
bool gps_check_fail_max_vert_drift   # 7 : maximum allowed vertical position drift fail - requires stationary vehicle
bool gps_check_fail_max_horz_spd_err # 8 : maximum allowed horizontal speed fail - requires stationary vehicle
bool gps_check_fail_max_vert_spd_err # 9 : maximum allowed vertical velocity discrepancy fail


# control mode flags to indicate EKF logic state
bool control_mode_tilt_align  # 0 - true if the filter tilt alignment is complete
bool control_mode_yaw_align   # 1 - true if the filter yaw alignment is complete
bool control_mode_gps         # 2 - true if GPS measurements are being fused
bool control_mode_opt_flow    # 3 - true if optical flow measurements are being fused
bool control_mode_mag_hdg     # 4 - true if a simple magnetic yaw heading is being fused
bool control_mode_mag_3d      # 5 - true if 3-axis magnetometer measurement are being fused
bool control_mode_mag_dec     # 6 - true if synthetic magnetic declination measurements are being fused
bool control_mode_in_air      # 7 - true when thought to be airborne
bool control_mode_wind        # 8 - true when wind velocity is being estimated
bool control_mode_baro_hgt    # 9 - true when baro height is being fused as a primary height reference
bool control_mode_rng_hgt     # 10 - true when range finder height is being fused as a primary height reference
bool control_mode_gps_hgt     # 11 - true when GPS height is being fused as a primary height reference
bool control_mode_ev_pos      # 12 - true when local position data from external vision is being fused
bool control_mode_ev_yaw      # 13 - true when yaw data from external vision measurements is being fused
bool control_mode_ev_hgt      # 14 - true when height data from external vision measurements is being fused
bool control_mode_beta        # 15 - true when synthetic sideslip measurements are being fused
bool control_mode_mag_field   # 16 - true when only the magnetic field states are updated by the magnetometer
bool control_mode_fixed_wing  # 17 - true when thought to be operating as a fixed wing vehicle with constrained sideslip
bool control_mode_mag_fault   # 18 - true when the magnetomer has been declared faulty and is no longer being used
bool control_mode_aspd        # 19 - true when airspeed measurements are being fused
bool control_mode_gnd_effect  # 20 - true when when protection from ground effect induced static pressure rise is active
bool control_mode_rng_stuck   # 21 - true when a stuck range finder sensor has been detected
bool control_mode_gps_yaw     # 22 - true when yaw (not ground course) data from a GPS receiver is being fused
bool control_mode_mag_aligned # 23 - true when the in-flight mag field alignment has been completed


# filter fault flags to indicate EKF internal faults
bool filter_fault_magnetometer_x       # 0 - true if the fusion of the magnetometer X-axis has encountered a numerical error
bool filter_fault_magnetometer_y       # 1 - true if the fusion of the magnetometer Y-axis has encountered a numerical error
bool filter_fault_magnetometer_z       # 2 - true if the fusion of the magnetometer Z-axis has encountered a numerical error
bool filter_fault_magnetic_heading     # 3 - true if the fusion of the magnetic heading has encountered a numerical error
bool filter_fault_magnetic_declination # 4 - true if the fusion of the magnetic declination has encountered a numerical error
bool filter_fault_airspeed             # 5 - true if fusion of the airspeed has encountered a numerical error
bool filter_fault_synthetic_sideslip   # 6 - true if fusion of the synthetic sideslip constraint has encountered a numerical error
bool filter_fault_optical_flow_x       # 7 - true if fusion of the optical flow X axis has encountered a numerical error
bool filter_fault_optical_flow_y       # 8 - true if fusion of the optical flow Y axis has encountered a numerical error
bool filter_fault_velocity_north       # 9 - true if fusion of the North velocity has encountered a numerical error
bool filter_fault_velocity_east        # 10 - true if fusion of the East velocity has encountered a numerical error
bool filter_fault_velocity_down        # 11 - true if fusion of the Down velocity has encountered a numerical error
bool filter_fault_position_north       # 12 - true if fusion of the North position has encountered a numerical error
bool filter_fault_position_east        # 13 - true if fusion of the East position has encountered a numerical error
bool filter_fault_position_down        # 14 - true if fusion of the Down position has encountered a numerical error
bool filter_fault_delta_velocity_bias  # 15 - true if bad delta velocity bias estimates have been detected


# Innovation check flags to indicate pass/fail status of innovation consistency checks
bool innovation_check_velocity_rejected            # 0 - true if velocity observations have been rejected
bool innovation_check_horizontal_position_rejected # 1 - true if horizontal position observations have been rejected
bool innovation_check_vertical_position_rejected   # 2 - true if vertical position observations have been rejected
bool innovation_check_magnetometer_x_rejected      # 3 - true if the X magnetometer observation has been rejected
bool innovation_check_magnetometer_y_rejected      # 4 - true if the Y magnetometer observation has been rejected
bool innovation_check_magnetometer_z_rejected      # 5 - true if the Z magnetometer observation has been rejected
bool innovation_check_yaw_rejected                 # 6 - true if the yaw observation has been rejected
bool innovation_check_airspeed_rejected            # 7 - true if the airspeed observation has been rejected
bool innovation_check_synthetic_sideslip_rejected  # 8 - true if the synthetic sideslip observation has been rejected
bool innovation_check_height_agl_rejected          # 9 - true if the height above ground observation has been rejected
bool innovation_check_optical_flow_x_rejected      # 10 - true if the X optical flow observation has been rejected
bool innovation_check_optical_flow_y_rejected      # 11 - true if the Y optical flow observation has been rejected


# Solution status flags indicating which filter kinematic state outputs are valid for flight control use.
bool solution_status_attitude                     # 0 - True if the attitude estimate is good
bool solution_status_horizontal_velocity          # 1 - True if the horizontal velocity estimate is good
bool solution_status_vertical_velocity            # 2 - True if the vertical velocity estimate is good
bool solution_status_horizontal_position_rel      # 3 - True if the horizontal position (relative) estimate is good
bool solution_status_horizontal_position_abs      # 4 - True if the horizontal position (absolute) estimate is good
bool solution_status_vertical_position_abs        # 5 - True if the vertical position (absolute) estimate is good
bool solution_status_vertical_position_agl        # 6 - True if the vertical position (above ground) estimate is good
bool solution_status_constant_position            # 7 - True if the EKF is in a constant position mode and is not using external measurements (eg GPS or optical flow)
bool solution_status_position_data_sufficient_rel # 8 - True if the EKF has sufficient data to enter a mode that will provide a (relative) position estimate
bool solution_status_position_data_sufficient_abs # 9 - True if the EKF has sufficient data to enter a mode that will provide a (absolute) position estimate
bool solution_status_gps_glitch                   # 10 - True if the EKF has detected a GPS glitch
bool solution_status_accelerometer_data_bad       # 11 - True if the EKF has detected bad accelerometer data


# preflight failure flags
bool preflight_fail_innov_heading
bool preflight_fail_innov_vel_horiz
bool preflight_fail_innov_vel_vert
bool preflight_fail_innov_height
bool preflight_fail_mag_field_disturbed

bool preflight_fail_accel_bias
bool preflight_fail_gyro_bias

# TODO:
# COM_ARM_MAG_STR - optionally ignore pre_flt_fail_mag_field_disturbed
# COM_ARM_EKF_HGT - status.hgt_test_ratio
# COM_ARM_EKF_VEL - status.vel_test_ratio
# COM_ARM_EKF_POS - status.pos_test_ratio
# COM_ARM_EKF_YAW - status.mag_test_ratio
# COM_ARM_EKF_AB  - states 13, 14, 15 (accel bias) # preflight_fail_accel_bias # TODO: fail is bias > half the limit (EKF2_ABL_LIM)
# COM_ARM_EKF_GB  - states 10, 11, 12 (gyro bias) # preflight_fail_gyro_bias

# GPS checks todo: do we care about GPS or not?
