/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: edit_struct.c,v 2.22 1998/02/03 04:20:01 bamoran Exp $
 *----------------------------------------------------------------------*/
/*----------------------------------------------------------------------*
  sensor:             m_present_time(s) 0
  sensor:                       m_dt(s) 
  sensor:                   m_roll(rad) 0
  sensor:                  m_pitch(rad) 0
  sensor:                    m_yaw(rad) 0
  sensor:            m_roll_rate(rad/s) 0
  sensor:           m_pitch_rate(rad/s) 0
  sensor:             m_yaw_rate(rad/s) 0
  sensor:           m_inclino_roll(rad)
  sensor:          m_inclino_pitch(rad)
  sensor:            m_compass_yaw(rad)
  sensor:                 m_dr_north(m) 0
  sensor:                  m_dr_east(m) 0
  sensor:                    m_dr_dn(m)
  sensor:                    m_dr_de(m)
  sensor:                    m_pos_n(m)
  sensor:                    m_pos_e(m)
  sensor:    uo_magnetic_variation(rad) -0.268781   (W15.4 deg)
  sensor:        m_paro_pres_period(us)
  sensor:        m_paro_temp_period(us) 
  sensor:              m_init_pres(bar) -1
  sensor:               m_pressure(bar) 
  sensor:                    m_depth(m) 0
  sensor:               e_latitude(rad) 0.7393
  sensor:             m_depth_rate(m/s) 0
  sensor:               u_prop_pitch(m) 0.324
  sensor:            m_water_speed(m/s) 0
  sensor:                 m_altitude(m) -1     (negative means no valid altimeter return)
  sensor:              m_st500_range(m) -1
  sensor:              m_st200_range(m) -1
  sensor:               water_cond(S/m) 3
  sensor:              water_temp(degC) 10
  sensor:               m_salinity(psu) 35
  sensor:         u_speed_of_sound(m/s) 1485
  sensor:                 c_dr_north(m) 0
  sensor:                  c_dr_east(m) 0
  sensor:         c_elevator_angle(rad) 0
  sensor:         m_elevator_angle(rad) 
  sensor:           c_rudder_angle(rad) 0
  sensor:           m_rudder_angle(rad) 
  sensor:        effective_drag_coeff() 0.21
  sensor:       c_thruster_current(Amp) 0
  sensor:       m_thruster_current(Amp) 0
  sensor:          m_thruster_per(16us) 
  sensor:      m_thruster_rawbatt(8bit) 
  sensor:        m_propeller_speed(rpm) 0
  sensor:           m_copley_voltage(V) 
  sensor:           m_copley_current(A) 
  sensor:           m_battery_voltage(V) 0
  sensor:          m_battery_current(A) 0
  sensor:             m_battery_temp(K) 0
  sensor:              m_batt_soc_est() 1
  sensor:           u_cutoff_voltage(V) 0
  sensor:          c_weight1_drop(bool) 0
  sensor:          c_weight2_drop(bool) 0
  sensor:        cc_heading_mode(index) 0
  sensor:          cc_depth_mode(index) 0
  sensor:          cc_speed_mode(index) 0
  sensor:          c_abort_mission(int) 0
  sensor:                    c_depth(m) 0
  sensor:                  c_pitch(rad) 0
  sensor:                c_heading(rad) 0
  sensor:                  c_speed(m/s) 0
  sensor:       c_depth_behavior(index) -1
  sensor:       c_depth_priority(index) -1
  sensor:     c_heading_behavior(index) -1
  sensor:     c_heading_priority(index) -1
  sensor:       c_speed_behavior(index) -1
  sensor:       c_speed_priority(index) -1
  sensor:           m_he_elev_raw(8bit)
  sensor:            m_he_rud_raw(8bit)
  sensor:            m_he_elev_deg(deg)
  sensor:             m_he_rud_deg(deg)
  sensor:                uo_control_kpp 1
  sensor:                uo_control_kdp 0.9
  sensor:                uo_control_kip 0
  sensor:     uo_control_pitch_lim(deg) 30
  sensor:     uo_control_depth_lim(deg) 0.3
  sensor:                uo_control_kph 0.75
  sensor:                uo_control_kdh 1.5
  sensor:                uo_control_kih 0
  sensor:               h_integral(rad) 0
  sensor:               d_integral(rad) 0
  sensor:               p_integral(rad) 0
  sensor:                uo_depth_map_k 0.5
  sensor:              uo_depth_int_k() 0
  sensor:                   uo_tau(sec) 5
  sensor:              uo_max_hr(rad/s) 0.1
  sensor:                uo_max_dr(m/s) 0.5
  sensor:                    t_hdg(rad) 
  sensor:             t_hdg_rate(rad/s) 
  sensor:                    t_depth(m) 
  sensor:             u_wp_x_track_gain 0.0
  sensor:                     wp_xte(m)
  sensor:                     wp_dtw(m)

config: sim
  sensor:                     real_x(m) 0
  sensor:                     real_y(m) 0
  sensor:                     real_z(m) 0
  sensor:                   real_u(m/s) 0
  sensor:                   real_v(m/s) 0
  sensor:                   real_w(m/s) 0
  sensor:                real_ud(m/s^2) 0
  sensor:                real_vd(m/s^2) 0
  sensor:                real_wd(m/s^2) 0
  sensor:                 real_phi(rad) 0
  sensor:               real_theta(rad) 0
  sensor:                 real_psi(rad) 0
  sensor:                 real_p(rad/s) 0
  sensor:                 real_q(rad/s) 0
  sensor:                 real_r(rad/s) 0
  sensor:              real_pd(rad/s^2) 0
  sensor:              real_qd(rad/s^2) 0
  sensor:              real_rd(rad/s^2) 0
  sensor:                    real_fx(N) 0
  sensor:                    real_fy(N) 0
  sensor:                    real_fz(N) 0
  sensor:                   real_fk(Nm) 0
  sensor:                   real_fm(Nm) 0
  sensor:                   real_fn(Nm) 0
  sensor:           real_depth_noise(m) 0
  sensor:         real_acc_noise(m/s^2) 0
  sensor:       real_excess_buoyancy(N) 0
  sensor:                 real_veh_w(N) 1173
  sensor:                 real_veh_b(N) 1173
  sensor:                  real_wdw1(N) 0
  sensor:                  real_wdw2(N) 0
  sensor:       real_current_north(m/s) 0
  sensor:        real_current_east(m/s) 0
  sensor:       real_thruster_current(Amp) 0
  sensor:       real_thruster_voltage(V) 0
  sensor:         real_thruster_power(W) 0
  sensor:        real_thruster_torque(Nm) 0
  sensor:        real_thruster_thrust(N) 0
  sensor:        real_thr_prop_speed(rad/s) 0
  sensor:       real_thr_motor_speed(rad/s) 0
  sensor:        real_thr_inflow_speed(m/s) 0
  sensor:       real_thr_drive_torque(Nm) 0
  sensor:        real_battery_current(Amp) 0
  sensor:        real_battery_voltage(V) 0
  sensor:          real_battery_power(W) 0
  sensor:            real_battery_soc() 1
  sensor:            real_battery_ocv(V) 0
  sensor:     real_batt_resistance(ohm) 0
  sensor:             real_batt_v_rc(V) 0
  sensor:             real_batt_temp(K) 0
  sensor:       real_batt_fault_heat(W) 0
  sensor:     real_batt_capacity(C) 216000
  sensor:   real_batt_energy_cap(J) 1.04e7
  sensor:     real_batt_charge_rem(C) 216000
  sensor:     real_batt_energy_rem(J) 1.04e7
  sensor:    real_battery_energy_used(J) 0
  sensor:     real_batt_charge_used(C) 0
  sensor:  read_current_from_file(bool) 0
  sensor:  real_sonar_frac_dropout(0-1) 0
  sensor:            u_bottom_type(int) 1
  sensor:             u_bottom_noise(m) 0
  sensor:                   real_alt(m) 0
  sensor:           real_triangle_x1(m) 0
  sensor:           real_triangle_y1(m) 0
  sensor:           real_triangle_z1(m) 0
  sensor:           real_triangle_x2(m) 0
  sensor:           real_triangle_y2(m) 0
  sensor:           real_triangle_z2(m) 0
  sensor:           real_triangle_x3(m) 0
  sensor:           real_triangle_y3(m) 0
  sensor:           real_triangle_z3(m) 0
  sensor:            sim_triangle_x1(m) 0
  sensor:            sim_triangle_y1(m) 0
  sensor:            sim_triangle_z1(m) 0
  sensor:            sim_triangle_x2(m) 0
  sensor:            sim_triangle_y2(m) 0
  sensor:            sim_triangle_z2(m) 0
  sensor:            sim_triangle_x3(m) 0
  sensor:            sim_triangle_y3(m) 0
  sensor:            sim_triangle_z3(m) 0
  sensor:          sim_sonar_phi_t(rad) 0
  sensor:          sim_sonar_phi_b(rad) 0
  sensor:       sim_sonar_intensity(dB) 0
  sensor:          sim_triangle_id(int) 0
  sensor:         real_triangle_id(int) 0
  sensor:              sim_contact_x(m) 0
  sensor:              sim_contact_y(m) 0
  sensor:              sim_contact_z(m) 0
  sensor:             real_contact_x(m) 0
  sensor:             real_contact_y(m) 0
  sensor:             real_contact_z(m) 0
  sensor:           m_fault_active(int) 0
endconfig

config: vs
  sensor:                    vs_mass(kg) 339.0
  sensor:                  vs_ixx(kg.m^2) 10.59
  sensor:                  vs_iyy(kg.m^2) 71.95
  sensor:                  vs_izz(kg.m^2) 71.95
  sensor:                  vs_ixy(kg.m^2) 0.0
  sensor:                  vs_iyz(kg.m^2) 0.0
  sensor:                  vs_izx(kg.m^2) 0.0
  sensor:                      vs_xg(m) 0.0
  sensor:                      vs_yg(m) -0.0002
  sensor:                      vs_zg(m) 0.082
  sensor:                      vs_xb(m) 0.0
  sensor:                      vs_yb(m) 0.0
  sensor:                      vs_zb(m) 0.0
  sensor:                    vs_xdw1(m) 0.0
  sensor:                    vs_ydw1(m) 0.0
  sensor:                    vs_zdw1(m) 0.0
  sensor:                    vs_bdw1(N) 0.0
  sensor:                    vs_xdw2(m) 0.0
  sensor:                    vs_ydw2(m) 0.0
  sensor:                    vs_zdw2(m) 0.0
  sensor:                    vs_bdw2(N) 0.0
  sensor:                  vs_xudot() -32.0
  sensor:                  vs_yvdot() -313.6
  sensor:                  vs_zwdot() -313.6
  sensor:                  vs_kpdot() 0.0
  sensor:                  vs_mqdot() -50.7
  sensor:                  vs_nrdot() -50.7
  sensor:                  vs_kvdot() 0.0
  sensor:                  vs_mwdot() 0.0
  sensor:                  vs_nvdot() 0.0
  sensor:                  vs_ypdot() 0.0
  sensor:                  vs_yrdot() 0.0
  sensor:                  vs_zqdot() 0.0
  sensor:                  vs_xuabu() -10.1
  sensor:                  vs_yvabv() -4930.0
  sensor:                  vs_zwabw() -4930.0
  sensor:                  vs_kpabp() 0.0
  sensor:                  vs_mqabq() -150.0
  sensor:                  vs_nrabr() -150.0
  sensor:                  vs_mwabw() 0.0
  sensor:                  vs_nvabv() 0.0
  sensor:                  vs_yrabr() 0.0
  sensor:                  vs_zqabq() 0.0
  sensor:                    vs_muq() -74.9
  sensor:                    vs_muw() 167.3
  sensor:                    vs_mpr() 50.7
  sensor:                    vs_nur() -74.9
  sensor:                    vs_nuv() -167.3
  sensor:                    vs_npq() -50.7
  sensor:                    vs_xvv() 48.7
  sensor:                    vs_xww() 48.7
  sensor:                    vs_xvr() 411.2
  sensor:                    vs_xwq() -411.2
  sensor:                    vs_xrr() -43.4
  sensor:                    vs_xqq() -43.4
  sensor:                    vs_yur() 112.6
  sensor:                    vs_yuv() -117.5
  sensor:                    vs_ywp() 313.6
  sensor:                    vs_zuq() -112.6
  sensor:                    vs_zuw() -117.5
  sensor:                    vs_zvp() -313.6
  sensor:                   vs_kvt2() 7.737
  sensor:                 vs_fin1_x(m) -1.069
  sensor:                 vs_fin1_y(m) 0.012
  sensor:                 vs_fin1_z(m) -0.1524
  sensor:                 vs_fin2_x(m) -1.069
  sensor:                 vs_fin2_y(m) -0.1524
  sensor:                 vs_fin2_z(m) 0.0
  sensor:                 vs_fin3_x(m) -1.069
  sensor:                 vs_fin3_y(m) 0.012
  sensor:                 vs_fin3_z(m) 0.1524
  sensor:                 vs_fin4_x(m) -1.069
  sensor:                 vs_fin4_y(m) 0.1524
  sensor:                 vs_fin4_z(m) 0.0
  sensor:              vs_fin_area(m^2) 0.0299
  sensor:         vs_fin_aspect_ratio() 4.26
  sensor:               vs_fin_cdo() 0.0065
  sensor:          vs_fin_cd_cross() 0.723
  sensor:               vs_fin_dcl() 3.392
  sensor:            vs_fin_oswald() 0.90
  sensor:        vs_fin_stall_angle(rad) 0.5236
  sensor:           vs_fin_speed(rad/s) 0.6109
  sensor:             vs_act_delay(int) 20
  sensor:        vs_rud_hyst_width(rad) 0.0349
  sensor:       vs_rud_hyst_center(rad) 0.0
  sensor:       vs_elev_hyst_width(rad) 0.0349
  sensor:      vs_elev_hyst_center(rad) 0.0
  sensor:           vs_thruster_kt(Nm/A) 0.456
  sensor:            vs_thruster_eta() 0.80
  sensor:          vs_thruster_pitch(m/rad) 0.124
  sensor:       vs_thruster_duct_area(m^2) 0.28
  sensor:    vs_thruster_prop_diameter(m) 0.28
  sensor:    vs_thruster_wake_fraction() 1.0
  sensor:          vs_thruster_kt_scale() 1.0
  sensor:          vs_thruster_kq_scale() 1.0
  sensor:         vs_thruster_eta_scale() 1.0
  sensor:        vs_thruster_area_scale() 1.0
  sensor:       vs_thruster_motor_kt(Nm/A) 0.034
  sensor:    vs_thruster_motor_ke(V/rad/s) 0.034
  sensor:        vs_thruster_motor_r(ohm) 0.800
  sensor:        vs_thruster_gear_ratio() 4.0
  sensor:          vs_thruster_gear_eff() 0.891
  sensor:   vs_thruster_seal_friction(Nm) 0.092
  sensor:    vs_thruster_current_limit(Amp) 999.0
  sensor:    vs_thruster_voltage_limit(V) 999.0
  sensor:              vs_batt_chem() 1
  sensor:       vs_battery_nominal_voltage(V) 48.0
  sensor:         vs_batt_series_cells() 32
  sensor:       vs_batt_parallel_strs() 5
  sensor:           vs_batt_capacity(C) 216000.0
  sensor:         vs_batt_energy_cap(J) 1.04e7
  sensor:          vs_batt_initial_soc() 1.0
  sensor:          vs_batt_r0_cell(ohm) 0.003
  sensor:         vs_batt_r_wiring(ohm) 0.01
  sensor:            vs_batt_r_conn(ohm) 0.01
  sensor:                vs_batt_r1(ohm) 0.02
  sensor:                    vs_batt_c1(F) 5000.0
  sensor:          vs_batt_therm_mass(J/K) 50000.0
  sensor:           vs_batt_therm_res(K/W) 0.3
  sensor:                   vs_batt_age() 0.0
  sensor:             vs_batt_cap_scale() 1.0
  sensor:               vs_batt_r_scale() 1.0
  sensor:           vs_batt_parasitic(A) 0.0
  sensor:         vs_batt_max_current(A) 100.0
  sensor:         vs_batt_min_voltage(V) 35.0
  sensor:             vs_batt_bms_enable() 0
  sensor:            vs_batt_bms_vtrip(V) 35.0
  sensor:            vs_batt_bms_itrip(A) 100.0
  sensor:             vs_batt_bms_ttrip(K) 333.0
  sensor:             vs_batt_imbalance() 0.0
  sensor:          vs_batt_weak_cell_v(V) 0.0
  sensor:            vs_batt_curr_bias(A) 0.0
  sensor:            vs_batt_volt_bias(V) 0.0
  sensor:             vs_batt_est_cap(C) 216000.0
  sensor:          vs_batt_est_init_soc() 1.0
  sensor:          vs_batt_runaway_temp(K) 353.0
  sensor:          vs_batt_runaway_heat(W) 0.0
  sensor:                  vs_hotel_load(W) 30.0
endconfig

config: lbl
  sensor:   real_lbl_timing_noise(mSec) 0
  sensor:       real_lbl_frac_bias(0-1) 0 
  sensor:    real_lbl_frac_dropout(0-1) 0
  sensor: real_lbl_bias_amplitude(mSec) 0
  sensor:             m_roll_drift(rad) 
  sensor:            m_pitch_drift(rad) 
  sensor:              m_yaw_drift(rad) 
  sensor:        u_lbl_beacon_number(#) 4
  sensor:               u_lbl_1north(m) 500
  sensor:                u_lbl_1east(m) 500
  sensor:               u_lbl_1depth(m) 10
  sensor:                 u_lbl_1tat(s) 0.003
  sensor:               u_lbl_2north(m) 500
  sensor:                u_lbl_2east(m) -500
  sensor:               u_lbl_2depth(m) 10
  sensor:                 u_lbl_2tat(s) 0.003
  sensor:               u_lbl_3north(m) -500
  sensor:                u_lbl_3east(m) -500
  sensor:               u_lbl_3depth(m) 10
  sensor:                 u_lbl_3tat(s) 0.003
  sensor:               u_lbl_4north(m) -500
  sensor:                u_lbl_4east(m) 500
  sensor:               u_lbl_4depth(m) 10
  sensor:                 u_lbl_4tat(s) 0.003
  sensor:               c_lbl_control() 1
  sensor:                u_lbl_cycle(s) 10
  sensor:              u_lbl_lockout(s) 0
  sensor:              u_lbl_timeout(s) 8
  sensor:                   m_lbl_0t(s) 
  sensor:                   m_lbl_1t(s) 
  sensor:                   m_lbl_2t(s) 
  sensor:                   m_lbl_3t(s) 
  sensor:                   m_lbl_4t(s) 
  sensor:                 lbl_newdata() 
  sensor:             m_lbl_northfix(m) 
  sensor:              m_lbl_eastfix(m) 
  sensor:                 lbl_fixstat() 
  sensor:                 lbl_fixgain() 0.2835
  sensor:              lbl_range_tol(m) 25
  sensor:                lbl_fix_err(m) 30
  sensor:            lbl_min_angle(rad) 0.2618 (15 deg)
  sensor:            m_lbl_est_north(m) 
  sensor:             m_lbl_est_east(m) 
  sensor:            m_lbl_heading(rad) 
  sensor:       u_lbl_mean_threshold(m) 20
  sensor:         u_lbl_dr_threshold(m) 20
  sensor:             m_lbl_dr_north(m) 0
  sensor:              m_lbl_dr_east(m) 0
  sensor:            m_lbl_ping_time(s) -1e6
  sensor:              m_lbl_num_used() 
  sensor:                m_lbl_valid1() 
  sensor:                m_lbl_valid2() 
  sensor:                m_lbl_valid3() 
  sensor:                m_lbl_valid4() 
  sensor:         m_lbl_dist_to_line(m) 
  sensor:         m_lbl_dist_to_last(m) 
  sensor:         m_lbl_trans_thresh(m) 
  sensor:         m_lbl_longi_thresh(m) 
  sensor:             u_trans_thresh(m) 15
  sensor:             u_mahala_thresh() 36
  sensor:                   m_mahala1() 
  sensor:                   m_mahala2() 
  sensor:                   m_mahala3() 
  sensor:                   m_mahala4() 
  sensor:                m_ekf_north(m) 0
  sensor:                 m_ekf_east(m) 0
  sensor:  u_lbl_transponder_mode(bool) 1
endconfig

config: ahrs
  sensor:              m_accel_x(m/s^2) 0
  sensor:              m_accel_y(m/s^2) 0
  sensor:              m_accel_z(m/s^2) 0
  sensor:                m_mag_x(gauss) 0
  sensor:                m_mag_y(gauss) 0
  sensor:                m_mag_z(gauss) 0
endconfig

behavior: mission_timer
  b_arg:                        time(s) 0

behavior: acquire_heading
  b_arg:                   heading(rad) 0.0
  b_arg:          acceptable_error(rad) 0.0
  b_arg:          behavior_active(bool) 0.0

behavior: ascend
  b_arg:                   heading(rad) 0
  b_arg:              rudder_angle(rad) 0
  b_arg:              ascent_angle(rad) 0
  b_arg:                     speed(m/s) 0
  b_arg:                   end_depth(m) 1
  b_arg:      drop_descent_weight(bool) 0
  b_arg:       drop_ascent_weight(bool) 0
  b_arg:     abort_mission_active(bool) 0

behavior: broadcast
  b_arg:                  start_time(s) 120
  b_arg:                    duration(s) 43200 (12 hr)
  b_arg:                    xs_delay(s) 120
  b_arg:                    xs_cycle(s) 120
  b_arg:                   uam_delay(s) 0
  b_arg:                   uam_cycle(s) 1

behavior: chirp_fin
  b_arg:                   key_delay(s) 0.0
  b_arg:                   key_depth(m) 0.0
  b_arg:                 fin_name(bool) 0.0
  b_arg:                 amplitude(rad) 0.0
  b_arg:                  init_freq(hz) 0.0
  b_arg:                 final_freq(hz) 0.0
  b_arg:                    duration(s) 0.0

behavior: depth_envelope
  b_arg:                   max_depth(m) 0.0
  b_arg:                   min_depth(m) 0.0
  b_arg:           alt_env_active(bool) 0.0
  b_arg:                min_altitude(m) 0.0
  b_arg:      depth_cutoff_active(bool) 0.0
  b_arg:                cutoff_depth(m) 0.0

behavior: descend
  b_arg:                   heading(rad) 0
  b_arg:              rudder_angle(rad) 0
  b_arg:             descent_angle(rad) 0
  b_arg:                     speed(m/s) 0
  b_arg:                   end_depth(m) 0
  b_arg:                min_altitude(m) 0
  b_arg:      drop_descent_weight(bool) 0
  b_arg:       drop_ascent_weight(bool) 0
  b_arg:                    max_time(s) 0

config: whoi
  sensor:                m_usbl_count() 
  sensor:           m_usbl_azimuth(rad) 
  sensor:         m_usbl_elevation(rad) 
  sensor:                m_usbl_time(s) 
  sensor:                m_usbl_qual(%) 
  sensor:              m_usbl_rdot(m/s) 
  sensor:              u_usbl_ringbuf() 10
  sensor:            m_inertial_az(rad)
  sensor:            m_inertial_el(rad)
  sensor:         m_usbl_rdot_mean(m/s) 
  sensor:      m_usbl_azimuth_norm(rad) 
  sensor:                  m_usbl_r0(s) 
  sensor:                 m_usbl_rem(s) 
  sensor:               m_usbl_range(m) 
  sensor:                 sim_dock_n(m) 100
  sensor:                 sim_dock_e(m) 200
  sensor:                 sim_dock_d(m)  50
  sensor:                   m_dock_n(m) 
  sensor:                   m_dock_e(m) 
  sensor:                   m_dock_d(m) 
  sensor:                 u_dock_vbl(m) 100
  sensor:                u_dock_zoff(m)   1.6
  sensor:                u_lower_tat(s)   0.010
  sensor:                m_lower_raw(s)
  sensor:                u_bucky_tat(s)   0.003
  sensor:                m_bucky_raw(s)
  sensor:              m_cnav_radial(m)
  sensor:               m_cnav_axial(m)
  sensor:               m_cnav_status()   0
  sensor:                   autoparse()   0
  sensor:            uo_t_straight(sec)  30
  sensor:         uo_search_direction()   0
  sensor:           uo_search_hr(rad/s)   0.05
  sensor:           uo_usbl_update(sec)   0.5
  sensor:                  m_latchio0()
  sensor:                  m_latchio1() 
  sensor:                  m_latchio2()
  sensor:                  m_latchio3()
  sensor:              m_pole_present() 
  sensor:                 latch_start() 0
  sensor:                 latch_check() 0
  sensor:                  latch_stop() 0
  sensor:        m_load_charge(coulomb) 
  sensor:          m_load_voltage(volt) 

behavior: dock
  b_arg:                   idle_time(s) 30
  b_arg:                 thrust_time(s)  0
  b_arg:                     speed(m/s)  0

behavior: undock
  b_arg:                  cycle_time(s) 60
  b_arg:              fwd_time_start(s)  0
  b_arg:                fwd_time_dur(s) 10
  b_arg:              rev_time_start(s) 11
  b_arg:                rev_time_dur(s) 19
  b_arg:                     speed(m/s)  0.6
  b_arg:                    rudder(rad)  0
  b_arg:                  elevator(rad)  0

behavior: whoi_homing
  b_arg:                        time(s) 0
  b_arg:                     speed(m/s) 0
  b_arg:                       depth(m) 0
  b_arg:                   heading(rad) 0
  b_arg:                         gain() 0
  b_arg:                  direction_k() 0
  b_arg:            heading_offset(rad) 0
  b_arg:         pole_transient_time(s) 0
  b_arg:               bailout_range(m) -1e6
  b_arg:           cnav_enable_range(m) 100

endconfig: whoi

behavior: set_elevator
  b_arg:                   heading(rad) 0.0
  b_arg:                  elevator(rad) 0.0
  b_arg:                     speed(m/s) 0.0
  b_arg:                        time(s) 0.0
  b_arg:                   max_depth(m) 10.0
  b_arg:                   min_depth(m) 0.0
  b_arg:                 max_pitch(rad) 0.520000
  b_arg:                 finished(bool) 0.0

behavior: set_rudder
  b_arg:                    rudder(rad) 0.0
  b_arg:                       depth(m) 0.0
  b_arg:                     speed(m/s) 0.0
  b_arg:                        time(s) 0.0
  b_arg:                 finished(bool) 0.0

behavior: setmode
  b_arg:              heading_mode(int) 0.0
  b_arg:           heading_commmand1(?) 0.0
  b_arg:           heading_commmand2(?) 0.0
  b_arg:                depth_mode(int) 0.0
  b_arg:             depth_commmand1(?) 0.0
  b_arg:             depth_commmand2(?) 0.0
  b_arg:                speed_mode(int) 0.0
  b_arg:             speed_commmand1(?) 0.0
  b_arg:             speed_commmand2(?) 0.0
  b_arg:                        time(s) 0.0

behavior: setpoint
  b_arg:                   heading(rad) 0.0
  b_arg:                       depth(m) 0.0
  b_arg:                     speed(m/s) 0.0
  b_arg:                        time(s) 0.0
  b_arg:                    cycles(int) 1.0
  b_arg:                      turn(rad) 0.0
  b_arg:             desired_pitch(rad) 0.0

behavior: setvector
  b_arg:                     heading(m) 0.0
  b_arg:                     pitch(rad) 0.0
  b_arg:                     speed(m/s) 0.0
  b_arg:                        time(s) 0.0

behavior: survey_dead_reckon
  b_arg:             trans_heading(rad) 0.0
  b_arg:                  trans_time(s) 0.0
  b_arg:              heading_leg1(rad) 0.0
  b_arg:                   time_leg1(s) 0.0
  b_arg:              heading_leg2(rad) 0.0
  b_arg:                   time_leg2(s) 0.0
  b_arg:                      cycles(#) 0.0
  b_arg:                       depth(m) 0.0
  b_arg:             transit_speed(m/s) 0.0
  b_arg:              survey_speed(m/s) 0.0

behavior: waypoint_2d
  b_arg:                       wpt_n(m)  0
  b_arg:                       wpt_e(m)  0
  b_arg:                       depth(m)  0
  b_arg:                     speed(m/s)  0
  b_arg:                     capture(m)  5
  b_arg:                  circle_mode()  1
  b_arg:                   heading(rad) -1
  b_arg:                    gain(rad/m)  0.0175
  b_arg:                     timeout(s)  0
  b_arg:             abort_on_timeout()  0

behavior: wiggle
  b_arg:                        time(s) 30
  b_arg:                      period(s)  6
  b_arg:          rudder_amplitude(rad)  0
  b_arg:        elevator_amplitude(rad)  0
  b_arg:         thruster_current(amps)  0

behavior: launch
  b_arg:                     speed(m/s) 0
  b_arg:            elevator_angle(rad) 0
  b_arg:                       depth(m) 0
  b_arg:                     timeout(s) 5

behavior: yo_yo
  b_arg:             desired_pitch(rad) 0.0
  b_arg:                   top_depth(m) 0.0
  b_arg:                bottom_depth(m) 0.0
  b_arg:                min_altitude(m) 0.0
  b_arg:                    cycles(int) 0.0
  b_arg:                    max_time(s) 0.0

behavior: picket_line
  b_arg:                     start_n(m) 0
  b_arg:                     start_e(m) 0
  b_arg:                       end_n(m) 0
  b_arg:                       end_e(m) 0
  b_arg:                       depth(m) 0
  b_arg:                     speed(m/s) 0
  b_arg:                     capture(m) 5

behavior: whale_trailing
  b_arg:                       depth(m) 0
  b_arg:                     speed(m/s) 0
  b_arg:                     capture(m) 5
  b_arg:         following_distance(m) 0
  b_arg:                side_offset(m) 0

END
 *----------------------------------------------------------------------*/

#include <ctype.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "ody_defs.h"
#include "ody_types.h"
#include "odyssey_defs.h"

                                /* typedef from parse */
struct sensor_name {
    char name[NAME_SIZE];
} long_name_array[MAX_SENSOR], short_name_array[MAX_SENSOR], 
	lower_b_array[MAX_BEHAVIOR], b_array[MAX_BEHAVIOR];

typedef enum { FALSE, TRUE } bool_t;

typedef enum {READING_SENSORS = 1, READING_BEHAVIORS} pass_t;


#ifndef MAX_CFG			/* compiler can override this value */
#define MAX_CFG 32
#endif

typedef struct {
    char name[NAME_SIZE];
    bool_t found;
} config_t;

typedef struct {
    char name[NAME_SIZE];	/* vehicle name (group) */
    char active[NAME_SIZE];	/* active config in defs section */
    bool_t output_disabled;
    config_t cfglist[MAX_CFG];
    unsigned count;
} cfglst_t;

				/* useful globals */
static int line_no, verbose;
static pass_t pass = READING_SENSORS;
cfglst_t cfgs;

/* prototypes */
void parse_cmd_line(int argc, char **argv);
void load_configs(const char *name);
FILE *fopen_fatal(const char *name, const char *type);

int add_sensor(
    char *line,
    int *n,
    FILE *lfile,
    FILE *sfile,
    FILE *hfile
);

int add_behavior(
    char *line,
    int *numBeh,
    int n_b_args,
    FILE *lfile,
    FILE *bfile,
    FILE *hfile
);

int add_b_arg(
    char *line,
    int *n,
    FILE *lfile,
    FILE *bfile
);


/*-------------------------------------------------------------------------
  main function starts here
  -----------------------------------------------------------------------*/

int main( int argc, char **argv )
{
    char *rcsrev = "$Revision: 2.22 $";
    char *rcsdate = "$Date: 1998/02/03 04:20:01 $";

				/* prototypes */
    void parse_cmd_line( int argc, char **argv );
    void load_configs( const char *name );
    FILE *fopen_fatal( const char *name, const char *type );
    int add_sensor( char *line, int *n, FILE *lfile, FILE *sfile,
		    FILE *hfile );
    int add_b_arg( char *line, int *n, FILE *lfile, FILE *bfile );

				/* local vars */
    int i, num_sensors=0, num_behaviors=0, num_b_args=0;
    FILE *fin, *lfile, *sfile, *bfile, *hfile, *cfile, *rfile;
    char infile_name[MAX_LINE], line[MAX_LINE], first_word[MAX_LINE],
	*lfile_name = "tools/data_list",
	*sfile_name = "tools/sensors",
	*bfile_name = "tools/behaviors",
	*hfile_name = "src/Odyssey/Vehicle/vehicle.h",
  *cfile_name = "src/Odyssey/Vehicle/vehicle.c",
  *rfile_name = "src/Odyssey/Vehicle/Behaviors/odyssey_behavior_registry.c";
				/* time stamp vars */
    struct tm *tp;
    time_t now;
    char cnow[32];
    char config_file[MAX_LINE];

				/* set default config name */
    strcpy( cfgs.name, "Simulator" );
				/* parse command line */
    parse_cmd_line( argc, argv );
				/* make RCS announcement */
    fputs( "Odyssey II data structure editor\n", stderr );
    fprintf( stderr, "\t%.*s, %.*s\n", (int)strlen(rcsrev+1)-2, rcsrev+1,
	     (int)strlen(rcsdate+1)-2, rcsdate+1 );

    now = time(NULL);		/* get time now */
    tp = gmtime( &now );
    strftime( cnow, sizeof(cnow), "%Y/%m/%d %H:%M:%S GMT", tp );

    if ( verbose )
	fprintf( stderr, "time: %s ...\n", cnow );

				/* announce config */
    if ( verbose )
	fprintf( stderr, "config: %s ...\n", cfgs.name );
				/* read config file for list of configs */
    snprintf(
      config_file,
      sizeof(config_file),
      "tools/%s.cfg",
      cfgs.name
    );

    load_configs(config_file);
                                /* open edit_struct.c,v $ (less
				   prefix and ,v) in order to read the
				   sensor and behavior lists */
    strcpy(infile_name, "tools/edit_struct.c");
    fin = fopen_fatal( infile_name, "r" );
				/* open files for output */
    lfile = fopen_fatal( lfile_name, "w" );
    sfile = fopen_fatal( sfile_name, "w" );
    bfile = fopen_fatal( bfile_name, "w" );
    hfile = fopen_fatal( hfile_name, "w" );
    cfile = fopen_fatal( cfile_name, "w" );
    rfile = fopen_fatal(rfile_name, "w");

				/* put the necessary include statements in
				   the C-file */
    fputs( "#include <float.h>\n"
	   "#include <stdio.h>\n"
     "#include <string.h>\n"
	   "#include \"ody_defs.h\"\n"
	   "#include \"ody_types.h\"\n"
       "#include \"sim.h\"\n", cfile );
                                /* jleonard EXTERN hack */
    fputs( "\n#undef EXTERN\n#define EXTERN \n\n"
	   "#include \"ody_var.h\"\n", cfile );
				/* variable creation_date in C-file */
    fprintf( cfile, "\nchar *creation_date = \"%s\";\n\n", cnow ); 

				/* same for H-file */
    fprintf( hfile, "#include \"ody_defs.h\"\n");
    fprintf( hfile, "#include \"ody_types.h\"\n");
    fprintf( hfile, "#include \"ody_var.h\"\n");
    fprintf( hfile, "#define CREATION_DATE \"%s\"\n\n", cnow );

    fputs(
        "#include <string.h>\n"
        "#include \"ody_types.h\"\n"
        "#include \"odyssey_behaviors.h\"\n\n",
        rfile
    );

				/* open sss in sensor file */
    fputs( "const struct sensor sss[] = {\n", sfile );
				/* open bTemplate in behavior file */
    fputs( "const struct behavior bTemplate[] = {\n", bfile );

                                /* read this file a line at a time, look
                                   for keywords */
    while ( fgets(line, MAX_LINE, fin) != NULL )
    {
        line_no++;
                                /* extract first word from line */
        if ( sscanf(line, " %s", first_word) == 1 )
	{
            if ( strcmp(first_word, "END") == 0 )
	    {
		if ( verbose )
		{
		    if ( pass == READING_SENSORS )
			for ( i = 0; i < cfgs.count; i++ )
			    fprintf( stderr, "cfg: %s ... %s\n",
				     cfgs.cfglist[i].name,
				     cfgs.cfglist[i].found ? "OK" : "??" );
		    fprintf( stderr, "line %03d: END in pass %d ...\n",
			     line_no, pass );
		}
		if ( pass == READING_SENSORS )
		{
		    rewind( fin );
		    line_no = 0;
		    pass = READING_BEHAVIORS;
		    continue;
		}
                break;
            }
	    if ( strcmp(first_word, "config:") == 0 &&
		 sscanf(line, " config: %s", cfgs.active ) == 1 )
	    {
		for ( i = 0; i < cfgs.count; i++ )
		{
		    if ( strcmp(cfgs.active, cfgs.cfglist[i].name) == 0 )
		    {
			cfgs.cfglist[i].found = TRUE;
			break;
		    }
		}
		if ( i == cfgs.count )
		    cfgs.output_disabled = TRUE;
		continue;
	    }
	    if ( strcmp(first_word, "endconfig") == 0 )
	    {
		cfgs.output_disabled = FALSE;
		continue;
	    }
	    if ( cfgs.output_disabled == FALSE )
	    {
		if ( pass == READING_SENSORS &&
		     strcmp(first_word, "sensor:") == 0 )

                    add_sensor( line, &num_sensors, lfile, sfile, hfile );
            
		if ( pass == READING_BEHAVIORS )
		{
		    if ( strcmp(first_word, "behavior:") == 0 )
		    {
			add_behavior( line, &num_behaviors, num_b_args, lfile, 
				      bfile, hfile );
			num_b_args = 0;
		    }
		    else if ( strcmp(first_word, "b_arg:") == 0 )
		
			add_b_arg( line, &num_b_args, lfile, bfile );
		}
	    }
	} /* if sscanf ... */
    } /* while ... */

    fprintf( stderr, "created %d sensor values, %d behaviors\n",
	     num_sensors, num_behaviors );

    fclose( fin );
    fclose( lfile );
    fclose( hfile );

    fputs( "\n};\n\nconst int num_sensors = "
	   "sizeof(sss)/sizeof(struct sensor);\n\n", sfile );
    fclose( sfile );
    sfile = fopen_fatal( sfile_name, "r" );

                                /* need to add (B_ARG_SIZE - n_b_args)
                                   blank lines to bfile */
    for ( i = 0; i < (B_ARG_SIZE - num_b_args); i++ ) 
        fprintf( bfile, ",\n        \"\", 0");

    fputs( "\n};\n\nconst int num_behaviors = "
	   "sizeof(bTemplate)/sizeof(struct behavior);\n", bfile );
    fclose( bfile );
    bfile = fopen_fatal( bfile_name, "r" );

                                /* append sfile and bfile to cfile */
    while ( fgets(line, MAX_LINE, sfile) != NULL ) 
	fputs( line, cfile );

    while ( fgets(line, MAX_LINE, bfile) != NULL )
	fputs( line, cfile );

    fputc( '\n', cfile );

    for ( i = 0; i < num_sensors; i++ )
        fprintf( cfile, "int %s = -1;\n", short_name_array[i].name );

    fprintf( cfile, "\n\nint *sindex[] = {\n" );
    for ( i = 0; i < num_sensors; i++ )
    {
        if (i)
	    fprintf( cfile, ",\n" );
        fprintf( cfile, "    &%s", short_name_array[i].name );
    }
    fputs( "\n};\n\n", cfile );

		/* generate behavior function registry */
    for (i = 0; i < num_behaviors; i++)
        fprintf(
            rfile,
            "extern int %s(struct behavior *, int, int);\n",
            lower_b_array[i].name
        );

    fputs(
        "\n"
        "int assign_behavior_fcn_ptr(const char *name, struct behavior *b)\n"
        "{\n"
        "    if (name == NULL || b == NULL)\n"
        "        return -1;\n\n"
        "    b->fcn = NULL;\n\n",
        rfile
    );

    for (i = 0; i < num_behaviors; i++)
        fprintf(
            rfile,
            "    if (strcmp(\"%s\", name) == 0) b->fcn = %s;\n",
            lower_b_array[i].name,
            lower_b_array[i].name
        );

    fputs(
        "\n"
        "    if (b->fcn == NULL)\n"
        "        return -1;\n\n"
        "    return 0;\n"
        "}\n",
        rfile
    );

    fclose( cfile );
    fclose( rfile );

    return 0;
}


/*-------------------------------------------------------------------------
  parse_cmd_line: process the command line for options, arguments
  -----------------------------------------------------------------------*/

void parse_cmd_line( int argc, char **argv )
{
    bool_t error = FALSE;
    char *arg0;
    int c;

    while ( (c = getopt(argc, argv, "f:v")) != EOF )
    {
	switch (c)
	{
	case 'f':
				/* force config to have this name */
	    strcpy( cfgs.name, optarg );
	    break;
	case 'v':
	    verbose = 1;
	    break;
	case '?':
	default:
	    error = TRUE;
	}
    }
				/* make sure argv[1] exists */
    if ( error )
    {
	arg0 = strrchr( argv[0], '/' );
	if ( !arg0++ ) arg0 = argv[0];
	fprintf( stderr, "usage: %s [-v]"
		 "[-f Amphitrite|Borealis|Xanthos|Wodyssey|Sim]\n", arg0 );
	exit( -1 );
    }
}


/*-------------------------------------------------------------------------
  load_configs: read the config file and load the requested configurations
  -----------------------------------------------------------------------*/

void load_configs( const char *name )
{
				/* prototypes */
    FILE *fopen_fatal( const char *name, const char *type );
				/* variables */
    FILE *fcfg;
    char line[MAX_LINE];

    fcfg = fopen_fatal( name, "r" );

    while ( fgets(line, sizeof(line), fcfg) != NULL )
    {
	if ( cfgs.count < MAX_CFG )
	{
	    strncpy( cfgs.cfglist[cfgs.count].name, line, strlen(line) - 1 );
	    cfgs.count++;
	}
	else
	    fprintf( stderr, "max cfgs exceeded, skipping %.*s ...\n",
		     (int)strlen(line) - 1, line );
    }
    fclose( fcfg );
}


/*-------------------------------------------------------------------------
  fopen_fatal: fatal error if filename can't be opened with the desired
  type of access
  -----------------------------------------------------------------------*/

FILE *fopen_fatal( const char *name, const char *type )
{
    FILE *fp;

    if ( (fp = fopen(name, type)) == NULL )
    {
	perror( name );
	exit( -1 );
    }
    return fp;
}


/*-------------------------------------------------------------------------
  add_sensor: 
  -----------------------------------------------------------------------*/

int add_sensor( char *line, int *n, FILE *lfile, FILE *sfile, FILE *hfile )
{
    int nconv, i;
    char name[NAME_SIZE];
    double value;

    if ( ((*n) + 1) > MAX_SENSOR )

        fprintf( stderr, "add_sensor: max_sensor (%d) exceeded\n",
		 MAX_SENSOR );

    if ((nconv = sscanf(line, " sensor: %63s %lf\n", name, &value)) > 0 )
        (*n)++;
    else
    {
        fprintf( stderr, "add_sensor:, sscanf returns %d\n", nconv );
        return -1;
    }

    if ( strcpy(long_name_array[(*n)-1].name, name) == NULL ) 
        fprintf( stderr, "add_sensor: strcpy fails on index %d\n", *n );

    for ( i = 0; (name[i] != '(') && (i < strlen(name)); i++ ) 
        short_name_array[(*n)-1].name[i] = toupper( name[i] );
    short_name_array[(*n)-1].name[i] = '\0';

    if (*n != 1)
	fputs( ",\n", sfile );
    fprintf( sfile, "    \"%s\", ", name );
    if ( nconv > 1 )
    {
	fprintf( sfile, "%lg, 0, 0, 0, 0, DBL_MAX, -DBL_MAX", value );
	fprintf( lfile, "  sensor: %29s %g\n", name, value );
    }
    else
    {
	fputs( "0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX", sfile );
	fprintf( lfile, "  sensor: %29s\n", name );
    }
				/* make external declarations in h-file */
    fputs( "extern int ", hfile );
    for ( i = 0; (name[i] != '(') && (i < strlen(name)); i++ ) 
        fputc( toupper(name[i]), hfile );
    fputs( ";\n", hfile );

    return 0;
}


/*-------------------------------------------------------------------------
  add_behavior: when "behavior: name num" is encountered, conclude previous
  behavior by adding remaining blank b_args and ",\n".  Concatenate
  beginning of new behavior
  -----------------------------------------------------------------------*/

int add_behavior( char *line, int *numBeh, int n_b_args, FILE *lfile,
                  FILE *bfile, FILE *hfile )
{
    int status, i;
    char name[NAME_SIZE];

    if ( ((*numBeh) + 1) > MAX_BEHAVIOR )
        fprintf( stderr, "add_behavior: max_number of behaviors=%d "
                 "exceeeded\n", MAX_BEHAVIOR );

    status = sscanf( line, " behavior: %s\n", name );
    if ( status == 1 )
        (*numBeh)++;
    else
    {
        fprintf( stderr, 
                 "error in add_behavior(), sscanf returns %d\n", status );
        return -1;
    }

    fprintf( lfile, " behavior: %s %2d\n", name, *numBeh );
    
    strcpy( lower_b_array[(*numBeh)-1].name, name );

    for ( i = 0; i < strlen(name); i++ )
        b_array[(*numBeh)-1].name[i] = toupper( name[i] );

				/* all but 1st require proper conclusion of
				   previous behavor in bfile to allow
				   concatenation of current behavior */
    if ( *numBeh != 1 )
    {
				/* concatenation starts with ",\n" */
	fputs( ",\n", bfile );
				/* also need to add (B_ARG_SIZE - n_b_args)
                                   lines with blank arguments */
        for ( i = 0; i < (B_ARG_SIZE - n_b_args); i++ ) 
            fputs( "        \"\", 0,\n", bfile );
    } else
				/* first behavior gets comment in h-file */
	fputs( "\n/*\n"
	       " * behavior names and corresponding id numbers\n"
	       " */\n", hfile );

				/* zeros for state, start & end times, and
				   the function pointer (*fcn()) */
    fprintf( bfile, "    \"%s\", 0, -1., 0., 0", name );

				/* define the uppercase name to be the
				   value of its index in the H file */
    fputs( "#define ", hfile );
    for ( i = 0; i < strlen(name); i++ ) 
        fputc( toupper(name[i]), hfile );
    fprintf( hfile, " %d\n", (*numBeh) - 1 ); 

    return 0;
}

int add_b_arg( char *line, int *n, FILE *lfile, FILE *bfile )
{
    int status;
    char name[NAME_SIZE];
    double value;

    if ( ((*n) + 1) > B_ARG_SIZE )

        fprintf( stderr, "add_b_arg: n (%d) exceeds B_ARG_SIZE (%d)\n", *n,
		 B_ARG_SIZE );

    status = sscanf( line, " b_arg: %s %lf\n", name, &value );
    if (status == 2)
        (*n)++;
    else
    {
        fprintf( stderr, "add_b_arg (line %d): sscanf = %d\n", line_no,
		 status );
        return -1;
    }
    fprintf( lfile, "  b_arg: %30s %g\n", name, value );
    fprintf( bfile, ",\n        \"%s\", %g", name, value );

    return 0;
}
