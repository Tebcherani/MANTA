#include <float.h>
#include <stdio.h>
#include <string.h>
#include "ody_defs.h"
#include "ody_types.h"

#undef EXTERN
#define EXTERN 

#include "ody_var.h"
#include "ody_func.h"

char *creation_date = "2026/08/08 01:00:40 GMT";

const struct sensor sss[] = {
    "m_present_time(s)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_dt(s)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_roll(rad)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_pitch(rad)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_yaw(rad)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_roll_rate(rad/s)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_pitch_rate(rad/s)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_yaw_rate(rad/s)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_inclino_roll(rad)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_inclino_pitch(rad)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_compass_yaw(rad)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_dr_north(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_dr_east(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_dr_dn(m)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_dr_de(m)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_pos_n(m)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_pos_e(m)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "uo_magnetic_variation(rad)", -0.268781, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_paro_pres_period(us)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_paro_temp_period(us)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_init_pres(bar)", -1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_pressure(bar)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_depth(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "e_latitude(rad)", 0.7393, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_depth_rate(m/s)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_prop_pitch(m)", 0.324, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_water_speed(m/s)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_altitude(m)", -1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_st500_range(m)", -1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_st200_range(m)", -1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "water_cond(S/m)", 3, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "water_temp(degC)", 10, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_salinity(psu)", 35, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_speed_of_sound(m/s)", 1485, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "c_dr_north(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "c_dr_east(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "c_elevator_angle(rad)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_elevator_angle(rad)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "c_rudder_angle(rad)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_rudder_angle(rad)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "effective_drag_coeff()", 0.21, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "c_thruster_current(Amp)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_thruster_current(Amp)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_thruster_per(16us)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_thruster_rawbatt(8bit)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_propeller_speed(rpm)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_copley_voltage(V)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_copley_current(A)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_battery_voltage(V)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_battery_current(A)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_battery_temp(K)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_batt_soc_est()", 1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_cutoff_voltage(V)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "c_weight1_drop(bool)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "c_weight2_drop(bool)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "cc_heading_mode(index)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "cc_depth_mode(index)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "cc_speed_mode(index)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "c_abort_mission(int)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "c_depth(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "c_pitch(rad)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "c_heading(rad)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "c_speed(m/s)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "c_depth_behavior(index)", -1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "c_depth_priority(index)", -1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "c_heading_behavior(index)", -1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "c_heading_priority(index)", -1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "c_speed_behavior(index)", -1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "c_speed_priority(index)", -1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_he_elev_raw(8bit)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_he_rud_raw(8bit)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_he_elev_deg(deg)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_he_rud_deg(deg)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "uo_control_kpp", 1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "uo_control_kdp", 0.9, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "uo_control_kip", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "uo_control_pitch_lim(deg)", 30, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "uo_control_depth_lim(deg)", 0.3, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "uo_control_kph", 0.75, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "uo_control_kdh", 1.5, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "uo_control_kih", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "h_integral(rad)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "d_integral(rad)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "p_integral(rad)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "uo_depth_map_k", 0.5, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "uo_depth_int_k()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "uo_tau(sec)", 5, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "uo_max_hr(rad/s)", 0.1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "uo_max_dr(m/s)", 0.5, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "t_hdg(rad)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "t_hdg_rate(rad/s)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "t_depth(m)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_wp_x_track_gain", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "wp_xte(m)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "wp_dtw(m)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_x(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_y(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_z(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_u(m/s)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_v(m/s)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_w(m/s)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_ud(m/s^2)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_vd(m/s^2)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_wd(m/s^2)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_phi(rad)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_theta(rad)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_psi(rad)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_p(rad/s)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_q(rad/s)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_r(rad/s)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_pd(rad/s^2)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_qd(rad/s^2)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_rd(rad/s^2)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_fx(N)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_fy(N)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_fz(N)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_fk(Nm)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_fm(Nm)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_fn(Nm)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_depth_noise(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_acc_noise(m/s^2)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_excess_buoyancy(N)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_veh_w(N)", 1173, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_veh_b(N)", 1173, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_wdw1(N)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_wdw2(N)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_current_north(m/s)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_current_east(m/s)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_thruster_current(Amp)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_thruster_voltage(V)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_thruster_power(W)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_thruster_torque(Nm)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_thruster_thrust(N)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_thr_prop_speed(rad/s)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_thr_motor_speed(rad/s)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_thr_inflow_speed(m/s)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_thr_drive_torque(Nm)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_battery_current(Amp)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_battery_voltage(V)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_battery_power(W)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_battery_soc()", 1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_battery_ocv(V)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_batt_resistance(ohm)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_batt_v_rc(V)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_batt_temp(K)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_batt_fault_heat(W)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_batt_capacity(C)", 216000, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_batt_energy_cap(J)", 1.04e+07, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_batt_charge_rem(C)", 216000, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_batt_energy_rem(J)", 1.04e+07, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_battery_energy_used(J)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_batt_charge_used(C)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "read_current_from_file(bool)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_sonar_frac_dropout(0-1)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_bottom_type(int)", 1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_bottom_noise(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_alt(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_triangle_x1(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_triangle_y1(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_triangle_z1(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_triangle_x2(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_triangle_y2(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_triangle_z2(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_triangle_x3(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_triangle_y3(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_triangle_z3(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "sim_triangle_x1(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "sim_triangle_y1(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "sim_triangle_z1(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "sim_triangle_x2(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "sim_triangle_y2(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "sim_triangle_z2(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "sim_triangle_x3(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "sim_triangle_y3(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "sim_triangle_z3(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "sim_sonar_phi_t(rad)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "sim_sonar_phi_b(rad)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "sim_sonar_intensity(dB)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "sim_triangle_id(int)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_triangle_id(int)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "sim_contact_x(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "sim_contact_y(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "sim_contact_z(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_contact_x(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_contact_y(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_contact_z(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_fault_active(int)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_mass(kg)", 339, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_ixx(kg.m^2)", 10.59, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_iyy(kg.m^2)", 71.95, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_izz(kg.m^2)", 71.95, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_ixy(kg.m^2)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_iyz(kg.m^2)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_izx(kg.m^2)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_xg(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_yg(m)", -0.0002, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_zg(m)", 0.082, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_xb(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_yb(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_zb(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_xdw1(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_ydw1(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_zdw1(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_bdw1(N)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_xdw2(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_ydw2(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_zdw2(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_bdw2(N)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_xudot()", -32, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_yvdot()", -313.6, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_zwdot()", -313.6, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_kpdot()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_mqdot()", -50.7, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_nrdot()", -50.7, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_kvdot()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_mwdot()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_nvdot()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_ypdot()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_yrdot()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_zqdot()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_xuabu()", -10.1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_yvabv()", -4930, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_zwabw()", -4930, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_kpabp()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_mqabq()", -150, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_nrabr()", -150, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_mwabw()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_nvabv()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_yrabr()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_zqabq()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_muq()", -74.9, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_muw()", 167.3, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_mpr()", 50.7, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_nur()", -74.9, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_nuv()", -167.3, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_npq()", -50.7, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_xvv()", 48.7, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_xww()", 48.7, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_xvr()", 411.2, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_xwq()", -411.2, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_xrr()", -43.4, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_xqq()", -43.4, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_yur()", 112.6, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_yuv()", -117.5, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_ywp()", 313.6, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_zuq()", -112.6, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_zuw()", -117.5, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_zvp()", -313.6, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_kvt2()", 7.737, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_fin1_x(m)", -1.069, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_fin1_y(m)", 0.012, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_fin1_z(m)", -0.1524, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_fin2_x(m)", -1.069, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_fin2_y(m)", -0.1524, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_fin2_z(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_fin3_x(m)", -1.069, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_fin3_y(m)", 0.012, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_fin3_z(m)", 0.1524, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_fin4_x(m)", -1.069, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_fin4_y(m)", 0.1524, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_fin4_z(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_fin_area(m^2)", 0.0299, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_fin_aspect_ratio()", 4.26, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_fin_cdo()", 0.0065, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_fin_cd_cross()", 0.723, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_fin_dcl()", 3.392, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_fin_oswald()", 0.9, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_fin_stall_angle(rad)", 0.5236, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_fin_speed(rad/s)", 0.6109, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_act_delay(int)", 20, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_rud_hyst_width(rad)", 0.0349, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_rud_hyst_center(rad)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_elev_hyst_width(rad)", 0.0349, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_elev_hyst_center(rad)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_thruster_kt(Nm/A)", 0.456, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_thruster_eta()", 0.8, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_thruster_pitch(m/rad)", 0.124, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_thruster_duct_area(m^2)", 0.28, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_thruster_prop_diameter(m)", 0.28, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_thruster_wake_fraction()", 1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_thruster_kt_scale()", 1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_thruster_kq_scale()", 1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_thruster_eta_scale()", 1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_thruster_area_scale()", 1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_thruster_motor_kt(Nm/A)", 0.034, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_thruster_motor_ke(V/rad/s)", 0.034, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_thruster_motor_r(ohm)", 0.8, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_thruster_gear_ratio()", 4, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_thruster_gear_eff()", 0.891, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_thruster_seal_friction(Nm)", 0.092, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_thruster_current_limit(Amp)", 999, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_thruster_voltage_limit(V)", 999, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_chem()", 1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_battery_nominal_voltage(V)", 48, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_series_cells()", 32, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_parallel_strs()", 5, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_capacity(C)", 216000, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_energy_cap(J)", 1.04e+07, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_initial_soc()", 1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_r0_cell(ohm)", 0.003, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_r_wiring(ohm)", 0.01, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_r_conn(ohm)", 0.01, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_r1(ohm)", 0.02, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_c1(F)", 5000, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_therm_mass(J/K)", 50000, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_therm_res(K/W)", 0.3, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_age()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_cap_scale()", 1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_r_scale()", 1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_parasitic(A)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_max_current(A)", 100, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_min_voltage(V)", 35, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_bms_enable()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_bms_vtrip(V)", 35, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_bms_itrip(A)", 100, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_bms_ttrip(K)", 333, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_imbalance()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_weak_cell_v(V)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_curr_bias(A)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_volt_bias(V)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_est_cap(C)", 216000, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_est_init_soc()", 1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_runaway_temp(K)", 353, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_batt_runaway_heat(W)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "vs_hotel_load(W)", 30, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "edge_tr_xs_time(s)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "edge_am200_bacs()", 617505, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "edge_xs_databyte()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_lbl_timing_noise(mSec)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_lbl_frac_bias(0-1)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_lbl_frac_dropout(0-1)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "real_lbl_bias_amplitude(mSec)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_roll_drift(rad)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_pitch_drift(rad)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_yaw_drift(rad)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lbl_beacon_number(#)", 4, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lbl_1north(m)", 500, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lbl_1east(m)", 500, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lbl_1depth(m)", 10, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lbl_1tat(s)", 0.003, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lbl_2north(m)", 500, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lbl_2east(m)", -500, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lbl_2depth(m)", 10, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lbl_2tat(s)", 0.003, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lbl_3north(m)", -500, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lbl_3east(m)", -500, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lbl_3depth(m)", 10, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lbl_3tat(s)", 0.003, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lbl_4north(m)", -500, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lbl_4east(m)", 500, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lbl_4depth(m)", 10, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lbl_4tat(s)", 0.003, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "c_lbl_control()", 1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lbl_cycle(s)", 10, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lbl_lockout(s)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lbl_timeout(s)", 8, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_lbl_0t(s)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_lbl_1t(s)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_lbl_2t(s)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_lbl_3t(s)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_lbl_4t(s)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "lbl_newdata()", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_lbl_northfix(m)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_lbl_eastfix(m)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "lbl_fixstat()", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "lbl_fixgain()", 0.2835, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "lbl_range_tol(m)", 25, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "lbl_fix_err(m)", 30, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "lbl_min_angle(rad)", 0.2618, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_lbl_est_north(m)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_lbl_est_east(m)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_lbl_heading(rad)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lbl_mean_threshold(m)", 20, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lbl_dr_threshold(m)", 20, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_lbl_dr_north(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_lbl_dr_east(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_lbl_ping_time(s)", -1e+06, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_lbl_num_used()", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_lbl_valid1()", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_lbl_valid2()", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_lbl_valid3()", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_lbl_valid4()", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_lbl_dist_to_line(m)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_lbl_dist_to_last(m)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_lbl_trans_thresh(m)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_lbl_longi_thresh(m)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_trans_thresh(m)", 15, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_mahala_thresh()", 36, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_mahala1()", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_mahala2()", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_mahala3()", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_mahala4()", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_ekf_north(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_ekf_east(m)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lbl_transponder_mode(bool)", 1, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_usbl_count()", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_usbl_azimuth(rad)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_usbl_elevation(rad)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_usbl_time(s)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_usbl_qual(%)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_usbl_rdot(m/s)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_usbl_ringbuf()", 10, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_inertial_az(rad)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_inertial_el(rad)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_usbl_rdot_mean(m/s)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_usbl_azimuth_norm(rad)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_usbl_r0(s)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_usbl_rem(s)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_usbl_range(m)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "sim_dock_n(m)", 100, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "sim_dock_e(m)", 200, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "sim_dock_d(m)", 50, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_dock_n(m)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_dock_e(m)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_dock_d(m)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_dock_vbl(m)", 100, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_dock_zoff(m)", 1.6, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_lower_tat(s)", 0.01, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_lower_raw(s)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_bucky_tat(s)", 0.003, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_bucky_raw(s)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_cnav_radial(m)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_cnav_axial(m)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_cnav_status()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "autoparse()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "uo_t_straight(sec)", 30, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "uo_search_direction()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "uo_search_hr(rad/s)", 0.05, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "uo_usbl_update(sec)", 0.5, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_latchio0()", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_latchio1()", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_latchio2()", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_latchio3()", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_pole_present()", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "latch_start()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "latch_check()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "latch_stop()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_load_charge(coulomb)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_load_voltage(volt)", 0, -1, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_edc_b_x()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_edc_b_y()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_edc_b_z()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "u_edc_z_gain()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "m_edc_b_intensity()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "e_edc_b_north()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "e_edc_b_east()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "e_edc_b_down()", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "e_edc_b_bearing(rad)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "e_edc_b_inclination(rad)", 0, 0, 0, 0, 0, DBL_MAX, -DBL_MAX,
    "e_edc_axial_range(m)", 15, 0, 0, 0, 0, DBL_MAX, -DBL_MAX
};

const int num_sensors = sizeof(sss)/sizeof(struct sensor);

const struct behavior bTemplate[] = {
    "mission_timer", 0, -1., 0., 0,
        "time(s)", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
    "acquire_heading", 0, -1., 0., 0,
        "heading(rad)", 0,
        "acceptable_error(rad)", 0,
        "behavior_active(bool)", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
    "ascend", 0, -1., 0., 0,
        "heading(rad)", 0,
        "rudder_angle(rad)", 0,
        "ascent_angle(rad)", 0,
        "speed(m/s)", 0,
        "end_depth(m)", 1,
        "drop_descent_weight(bool)", 0,
        "drop_ascent_weight(bool)", 0,
        "abort_mission_active(bool)", 0,
        "", 0,
        "", 0,
    "broadcast", 0, -1., 0., 0,
        "start_time(s)", 120,
        "duration(s)", 43200,
        "xs_delay(s)", 120,
        "xs_cycle(s)", 120,
        "uam_delay(s)", 0,
        "uam_cycle(s)", 1,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
    "chirp_fin", 0, -1., 0., 0,
        "key_delay(s)", 0,
        "key_depth(m)", 0,
        "fin_name(bool)", 0,
        "amplitude(rad)", 0,
        "init_freq(hz)", 0,
        "final_freq(hz)", 0,
        "duration(s)", 0,
        "", 0,
        "", 0,
        "", 0,
    "depth_envelope", 0, -1., 0., 0,
        "max_depth(m)", 0,
        "min_depth(m)", 0,
        "alt_env_active(bool)", 0,
        "min_altitude(m)", 0,
        "depth_cutoff_active(bool)", 0,
        "cutoff_depth(m)", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
    "descend", 0, -1., 0., 0,
        "heading(rad)", 0,
        "rudder_angle(rad)", 0,
        "descent_angle(rad)", 0,
        "speed(m/s)", 0,
        "end_depth(m)", 0,
        "min_altitude(m)", 0,
        "drop_descent_weight(bool)", 0,
        "drop_ascent_weight(bool)", 0,
        "max_time(s)", 0,
        "", 0,
    "dock", 0, -1., 0., 0,
        "idle_time(s)", 30,
        "thrust_time(s)", 0,
        "speed(m/s)", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
    "undock", 0, -1., 0., 0,
        "cycle_time(s)", 60,
        "fwd_time_start(s)", 0,
        "fwd_time_dur(s)", 10,
        "rev_time_start(s)", 11,
        "rev_time_dur(s)", 19,
        "speed(m/s)", 0.6,
        "rudder(rad)", 0,
        "elevator(rad)", 0,
        "", 0,
        "", 0,
    "whoi_homing", 0, -1., 0., 0,
        "time(s)", 0,
        "speed(m/s)", 0,
        "depth(m)", 0,
        "heading(rad)", 0,
        "gain()", 0,
        "direction_k()", 0,
        "heading_offset(rad)", 0,
        "pole_transient_time(s)", 0,
        "bailout_range(m)", -1e+06,
        "cnav_enable_range(m)", 100,
    "edc_homing", 0, -1., 0., 0,
        "elev_gain(num)", 0,
        "azim_gain(num)", 0,
        "range_cutout(m)", 0,
        "speed(m/s)", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
    "set_elevator", 0, -1., 0., 0,
        "heading(rad)", 0,
        "elevator(rad)", 0,
        "speed(m/s)", 0,
        "time(s)", 0,
        "max_depth(m)", 10,
        "min_depth(m)", 0,
        "max_pitch(rad)", 0.52,
        "finished(bool)", 0,
        "", 0,
        "", 0,
    "set_rudder", 0, -1., 0., 0,
        "rudder(rad)", 0,
        "depth(m)", 0,
        "speed(m/s)", 0,
        "time(s)", 0,
        "finished(bool)", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
    "setmode", 0, -1., 0., 0,
        "heading_mode(int)", 0,
        "heading_commmand1(?)", 0,
        "heading_commmand2(?)", 0,
        "depth_mode(int)", 0,
        "depth_commmand1(?)", 0,
        "depth_commmand2(?)", 0,
        "speed_mode(int)", 0,
        "speed_commmand1(?)", 0,
        "speed_commmand2(?)", 0,
        "time(s)", 0,
    "setpoint", 0, -1., 0., 0,
        "heading(rad)", 0,
        "depth(m)", 0,
        "speed(m/s)", 0,
        "time(s)", 0,
        "cycles(int)", 1,
        "turn(rad)", 0,
        "desired_pitch(rad)", 0,
        "", 0,
        "", 0,
        "", 0,
    "setvector", 0, -1., 0., 0,
        "heading(m)", 0,
        "pitch(rad)", 0,
        "speed(m/s)", 0,
        "time(s)", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
    "survey_dead_reckon", 0, -1., 0., 0,
        "trans_heading(rad)", 0,
        "trans_time(s)", 0,
        "heading_leg1(rad)", 0,
        "time_leg1(s)", 0,
        "heading_leg2(rad)", 0,
        "time_leg2(s)", 0,
        "cycles(#)", 0,
        "depth(m)", 0,
        "transit_speed(m/s)", 0,
        "survey_speed(m/s)", 0,
    "remote_delay", 0, -1., 0., 0,
        "time(s)", 0,
        "heading(rad)", 0,
        "min_time(sec)", 1,
        "max_time(sec)", 4.6,
        "mission_abort_enable(bool)", 0,
        "finished(bool)", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
    "waypoint_2d", 0, -1., 0., 0,
        "wpt_n(m)", 0,
        "wpt_e(m)", 0,
        "depth(m)", 0,
        "speed(m/s)", 0,
        "capture(m)", 5,
        "circle_mode()", 1,
        "heading(rad)", -1,
        "gain(rad/m)", 0.0175,
        "timeout(s)", 0,
        "abort_on_timeout()", 0,
    "wiggle", 0, -1., 0., 0,
        "time(s)", 30,
        "period(s)", 6,
        "rudder_amplitude(rad)", 0,
        "elevator_amplitude(rad)", 0,
        "thruster_current(amps)", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
    "launch", 0, -1., 0., 0,
        "speed(m/s)", 0,
        "elevator_angle(rad)", 0,
        "depth(m)", 0,
        "timeout(s)", 5,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0,
    "yo_yo", 0, -1., 0., 0,
        "desired_pitch(rad)", 0,
        "top_depth(m)", 0,
        "bottom_depth(m)", 0,
        "min_altitude(m)", 0,
        "cycles(int)", 0,
        "max_time(s)", 0,
        "", 0,
        "", 0,
        "", 0,
        "", 0
};

const int num_behaviors = sizeof(bTemplate)/sizeof(struct behavior);

int M_PRESENT_TIME = -1;
int M_DT = -1;
int M_ROLL = -1;
int M_PITCH = -1;
int M_YAW = -1;
int M_ROLL_RATE = -1;
int M_PITCH_RATE = -1;
int M_YAW_RATE = -1;
int M_INCLINO_ROLL = -1;
int M_INCLINO_PITCH = -1;
int M_COMPASS_YAW = -1;
int M_DR_NORTH = -1;
int M_DR_EAST = -1;
int M_DR_DN = -1;
int M_DR_DE = -1;
int M_POS_N = -1;
int M_POS_E = -1;
int UO_MAGNETIC_VARIATION = -1;
int M_PARO_PRES_PERIOD = -1;
int M_PARO_TEMP_PERIOD = -1;
int M_INIT_PRES = -1;
int M_PRESSURE = -1;
int M_DEPTH = -1;
int E_LATITUDE = -1;
int M_DEPTH_RATE = -1;
int U_PROP_PITCH = -1;
int M_WATER_SPEED = -1;
int M_ALTITUDE = -1;
int M_ST500_RANGE = -1;
int M_ST200_RANGE = -1;
int WATER_COND = -1;
int WATER_TEMP = -1;
int M_SALINITY = -1;
int U_SPEED_OF_SOUND = -1;
int C_DR_NORTH = -1;
int C_DR_EAST = -1;
int C_ELEVATOR_ANGLE = -1;
int M_ELEVATOR_ANGLE = -1;
int C_RUDDER_ANGLE = -1;
int M_RUDDER_ANGLE = -1;
int EFFECTIVE_DRAG_COEFF = -1;
int C_THRUSTER_CURRENT = -1;
int M_THRUSTER_CURRENT = -1;
int M_THRUSTER_PER = -1;
int M_THRUSTER_RAWBATT = -1;
int M_PROPELLER_SPEED = -1;
int M_COPLEY_VOLTAGE = -1;
int M_COPLEY_CURRENT = -1;
int M_BATTERY_VOLTAGE = -1;
int M_BATTERY_CURRENT = -1;
int M_BATTERY_TEMP = -1;
int M_BATT_SOC_EST = -1;
int U_CUTOFF_VOLTAGE = -1;
int C_WEIGHT1_DROP = -1;
int C_WEIGHT2_DROP = -1;
int CC_HEADING_MODE = -1;
int CC_DEPTH_MODE = -1;
int CC_SPEED_MODE = -1;
int C_ABORT_MISSION = -1;
int C_DEPTH = -1;
int C_PITCH = -1;
int C_HEADING = -1;
int C_SPEED = -1;
int C_DEPTH_BEHAVIOR = -1;
int C_DEPTH_PRIORITY = -1;
int C_HEADING_BEHAVIOR = -1;
int C_HEADING_PRIORITY = -1;
int C_SPEED_BEHAVIOR = -1;
int C_SPEED_PRIORITY = -1;
int M_HE_ELEV_RAW = -1;
int M_HE_RUD_RAW = -1;
int M_HE_ELEV_DEG = -1;
int M_HE_RUD_DEG = -1;
int UO_CONTROL_KPP = -1;
int UO_CONTROL_KDP = -1;
int UO_CONTROL_KIP = -1;
int UO_CONTROL_PITCH_LIM = -1;
int UO_CONTROL_DEPTH_LIM = -1;
int UO_CONTROL_KPH = -1;
int UO_CONTROL_KDH = -1;
int UO_CONTROL_KIH = -1;
int H_INTEGRAL = -1;
int D_INTEGRAL = -1;
int P_INTEGRAL = -1;
int UO_DEPTH_MAP_K = -1;
int UO_DEPTH_INT_K = -1;
int UO_TAU = -1;
int UO_MAX_HR = -1;
int UO_MAX_DR = -1;
int T_HDG = -1;
int T_HDG_RATE = -1;
int T_DEPTH = -1;
int U_WP_X_TRACK_GAIN = -1;
int WP_XTE = -1;
int WP_DTW = -1;
int REAL_X = -1;
int REAL_Y = -1;
int REAL_Z = -1;
int REAL_U = -1;
int REAL_V = -1;
int REAL_W = -1;
int REAL_UD = -1;
int REAL_VD = -1;
int REAL_WD = -1;
int REAL_PHI = -1;
int REAL_THETA = -1;
int REAL_PSI = -1;
int REAL_P = -1;
int REAL_Q = -1;
int REAL_R = -1;
int REAL_PD = -1;
int REAL_QD = -1;
int REAL_RD = -1;
int REAL_FX = -1;
int REAL_FY = -1;
int REAL_FZ = -1;
int REAL_FK = -1;
int REAL_FM = -1;
int REAL_FN = -1;
int REAL_DEPTH_NOISE = -1;
int REAL_ACC_NOISE = -1;
int REAL_EXCESS_BUOYANCY = -1;
int REAL_VEH_W = -1;
int REAL_VEH_B = -1;
int REAL_WDW1 = -1;
int REAL_WDW2 = -1;
int REAL_CURRENT_NORTH = -1;
int REAL_CURRENT_EAST = -1;
int REAL_THRUSTER_CURRENT = -1;
int REAL_THRUSTER_VOLTAGE = -1;
int REAL_THRUSTER_POWER = -1;
int REAL_THRUSTER_TORQUE = -1;
int REAL_THRUSTER_THRUST = -1;
int REAL_THR_PROP_SPEED = -1;
int REAL_THR_MOTOR_SPEED = -1;
int REAL_THR_INFLOW_SPEED = -1;
int REAL_THR_DRIVE_TORQUE = -1;
int REAL_BATTERY_CURRENT = -1;
int REAL_BATTERY_VOLTAGE = -1;
int REAL_BATTERY_POWER = -1;
int REAL_BATTERY_SOC = -1;
int REAL_BATTERY_OCV = -1;
int REAL_BATT_RESISTANCE = -1;
int REAL_BATT_V_RC = -1;
int REAL_BATT_TEMP = -1;
int REAL_BATT_FAULT_HEAT = -1;
int REAL_BATT_CAPACITY = -1;
int REAL_BATT_ENERGY_CAP = -1;
int REAL_BATT_CHARGE_REM = -1;
int REAL_BATT_ENERGY_REM = -1;
int REAL_BATTERY_ENERGY_USED = -1;
int REAL_BATT_CHARGE_USED = -1;
int READ_CURRENT_FROM_FILE = -1;
int REAL_SONAR_FRAC_DROPOUT = -1;
int U_BOTTOM_TYPE = -1;
int U_BOTTOM_NOISE = -1;
int REAL_ALT = -1;
int REAL_TRIANGLE_X1 = -1;
int REAL_TRIANGLE_Y1 = -1;
int REAL_TRIANGLE_Z1 = -1;
int REAL_TRIANGLE_X2 = -1;
int REAL_TRIANGLE_Y2 = -1;
int REAL_TRIANGLE_Z2 = -1;
int REAL_TRIANGLE_X3 = -1;
int REAL_TRIANGLE_Y3 = -1;
int REAL_TRIANGLE_Z3 = -1;
int SIM_TRIANGLE_X1 = -1;
int SIM_TRIANGLE_Y1 = -1;
int SIM_TRIANGLE_Z1 = -1;
int SIM_TRIANGLE_X2 = -1;
int SIM_TRIANGLE_Y2 = -1;
int SIM_TRIANGLE_Z2 = -1;
int SIM_TRIANGLE_X3 = -1;
int SIM_TRIANGLE_Y3 = -1;
int SIM_TRIANGLE_Z3 = -1;
int SIM_SONAR_PHI_T = -1;
int SIM_SONAR_PHI_B = -1;
int SIM_SONAR_INTENSITY = -1;
int SIM_TRIANGLE_ID = -1;
int REAL_TRIANGLE_ID = -1;
int SIM_CONTACT_X = -1;
int SIM_CONTACT_Y = -1;
int SIM_CONTACT_Z = -1;
int REAL_CONTACT_X = -1;
int REAL_CONTACT_Y = -1;
int REAL_CONTACT_Z = -1;
int M_FAULT_ACTIVE = -1;
int VS_MASS = -1;
int VS_IXX = -1;
int VS_IYY = -1;
int VS_IZZ = -1;
int VS_IXY = -1;
int VS_IYZ = -1;
int VS_IZX = -1;
int VS_XG = -1;
int VS_YG = -1;
int VS_ZG = -1;
int VS_XB = -1;
int VS_YB = -1;
int VS_ZB = -1;
int VS_XDW1 = -1;
int VS_YDW1 = -1;
int VS_ZDW1 = -1;
int VS_BDW1 = -1;
int VS_XDW2 = -1;
int VS_YDW2 = -1;
int VS_ZDW2 = -1;
int VS_BDW2 = -1;
int VS_XUDOT = -1;
int VS_YVDOT = -1;
int VS_ZWDOT = -1;
int VS_KPDOT = -1;
int VS_MQDOT = -1;
int VS_NRDOT = -1;
int VS_KVDOT = -1;
int VS_MWDOT = -1;
int VS_NVDOT = -1;
int VS_YPDOT = -1;
int VS_YRDOT = -1;
int VS_ZQDOT = -1;
int VS_XUABU = -1;
int VS_YVABV = -1;
int VS_ZWABW = -1;
int VS_KPABP = -1;
int VS_MQABQ = -1;
int VS_NRABR = -1;
int VS_MWABW = -1;
int VS_NVABV = -1;
int VS_YRABR = -1;
int VS_ZQABQ = -1;
int VS_MUQ = -1;
int VS_MUW = -1;
int VS_MPR = -1;
int VS_NUR = -1;
int VS_NUV = -1;
int VS_NPQ = -1;
int VS_XVV = -1;
int VS_XWW = -1;
int VS_XVR = -1;
int VS_XWQ = -1;
int VS_XRR = -1;
int VS_XQQ = -1;
int VS_YUR = -1;
int VS_YUV = -1;
int VS_YWP = -1;
int VS_ZUQ = -1;
int VS_ZUW = -1;
int VS_ZVP = -1;
int VS_KVT2 = -1;
int VS_FIN1_X = -1;
int VS_FIN1_Y = -1;
int VS_FIN1_Z = -1;
int VS_FIN2_X = -1;
int VS_FIN2_Y = -1;
int VS_FIN2_Z = -1;
int VS_FIN3_X = -1;
int VS_FIN3_Y = -1;
int VS_FIN3_Z = -1;
int VS_FIN4_X = -1;
int VS_FIN4_Y = -1;
int VS_FIN4_Z = -1;
int VS_FIN_AREA = -1;
int VS_FIN_ASPECT_RATIO = -1;
int VS_FIN_CDO = -1;
int VS_FIN_CD_CROSS = -1;
int VS_FIN_DCL = -1;
int VS_FIN_OSWALD = -1;
int VS_FIN_STALL_ANGLE = -1;
int VS_FIN_SPEED = -1;
int VS_ACT_DELAY = -1;
int VS_RUD_HYST_WIDTH = -1;
int VS_RUD_HYST_CENTER = -1;
int VS_ELEV_HYST_WIDTH = -1;
int VS_ELEV_HYST_CENTER = -1;
int VS_THRUSTER_KT = -1;
int VS_THRUSTER_ETA = -1;
int VS_THRUSTER_PITCH = -1;
int VS_THRUSTER_DUCT_AREA = -1;
int VS_THRUSTER_PROP_DIAMETER = -1;
int VS_THRUSTER_WAKE_FRACTION = -1;
int VS_THRUSTER_KT_SCALE = -1;
int VS_THRUSTER_KQ_SCALE = -1;
int VS_THRUSTER_ETA_SCALE = -1;
int VS_THRUSTER_AREA_SCALE = -1;
int VS_THRUSTER_MOTOR_KT = -1;
int VS_THRUSTER_MOTOR_KE = -1;
int VS_THRUSTER_MOTOR_R = -1;
int VS_THRUSTER_GEAR_RATIO = -1;
int VS_THRUSTER_GEAR_EFF = -1;
int VS_THRUSTER_SEAL_FRICTION = -1;
int VS_THRUSTER_CURRENT_LIMIT = -1;
int VS_THRUSTER_VOLTAGE_LIMIT = -1;
int VS_BATT_CHEM = -1;
int VS_BATTERY_NOMINAL_VOLTAGE = -1;
int VS_BATT_SERIES_CELLS = -1;
int VS_BATT_PARALLEL_STRS = -1;
int VS_BATT_CAPACITY = -1;
int VS_BATT_ENERGY_CAP = -1;
int VS_BATT_INITIAL_SOC = -1;
int VS_BATT_R0_CELL = -1;
int VS_BATT_R_WIRING = -1;
int VS_BATT_R_CONN = -1;
int VS_BATT_R1 = -1;
int VS_BATT_C1 = -1;
int VS_BATT_THERM_MASS = -1;
int VS_BATT_THERM_RES = -1;
int VS_BATT_AGE = -1;
int VS_BATT_CAP_SCALE = -1;
int VS_BATT_R_SCALE = -1;
int VS_BATT_PARASITIC = -1;
int VS_BATT_MAX_CURRENT = -1;
int VS_BATT_MIN_VOLTAGE = -1;
int VS_BATT_BMS_ENABLE = -1;
int VS_BATT_BMS_VTRIP = -1;
int VS_BATT_BMS_ITRIP = -1;
int VS_BATT_BMS_TTRIP = -1;
int VS_BATT_IMBALANCE = -1;
int VS_BATT_WEAK_CELL_V = -1;
int VS_BATT_CURR_BIAS = -1;
int VS_BATT_VOLT_BIAS = -1;
int VS_BATT_EST_CAP = -1;
int VS_BATT_EST_INIT_SOC = -1;
int VS_BATT_RUNAWAY_TEMP = -1;
int VS_BATT_RUNAWAY_HEAT = -1;
int VS_HOTEL_LOAD = -1;
int EDGE_TR_XS_TIME = -1;
int EDGE_AM200_BACS = -1;
int EDGE_XS_DATABYTE = -1;
int REAL_LBL_TIMING_NOISE = -1;
int REAL_LBL_FRAC_BIAS = -1;
int REAL_LBL_FRAC_DROPOUT = -1;
int REAL_LBL_BIAS_AMPLITUDE = -1;
int M_ROLL_DRIFT = -1;
int M_PITCH_DRIFT = -1;
int M_YAW_DRIFT = -1;
int U_LBL_BEACON_NUMBER = -1;
int U_LBL_1NORTH = -1;
int U_LBL_1EAST = -1;
int U_LBL_1DEPTH = -1;
int U_LBL_1TAT = -1;
int U_LBL_2NORTH = -1;
int U_LBL_2EAST = -1;
int U_LBL_2DEPTH = -1;
int U_LBL_2TAT = -1;
int U_LBL_3NORTH = -1;
int U_LBL_3EAST = -1;
int U_LBL_3DEPTH = -1;
int U_LBL_3TAT = -1;
int U_LBL_4NORTH = -1;
int U_LBL_4EAST = -1;
int U_LBL_4DEPTH = -1;
int U_LBL_4TAT = -1;
int C_LBL_CONTROL = -1;
int U_LBL_CYCLE = -1;
int U_LBL_LOCKOUT = -1;
int U_LBL_TIMEOUT = -1;
int M_LBL_0T = -1;
int M_LBL_1T = -1;
int M_LBL_2T = -1;
int M_LBL_3T = -1;
int M_LBL_4T = -1;
int LBL_NEWDATA = -1;
int M_LBL_NORTHFIX = -1;
int M_LBL_EASTFIX = -1;
int LBL_FIXSTAT = -1;
int LBL_FIXGAIN = -1;
int LBL_RANGE_TOL = -1;
int LBL_FIX_ERR = -1;
int LBL_MIN_ANGLE = -1;
int M_LBL_EST_NORTH = -1;
int M_LBL_EST_EAST = -1;
int M_LBL_HEADING = -1;
int U_LBL_MEAN_THRESHOLD = -1;
int U_LBL_DR_THRESHOLD = -1;
int M_LBL_DR_NORTH = -1;
int M_LBL_DR_EAST = -1;
int M_LBL_PING_TIME = -1;
int M_LBL_NUM_USED = -1;
int M_LBL_VALID1 = -1;
int M_LBL_VALID2 = -1;
int M_LBL_VALID3 = -1;
int M_LBL_VALID4 = -1;
int M_LBL_DIST_TO_LINE = -1;
int M_LBL_DIST_TO_LAST = -1;
int M_LBL_TRANS_THRESH = -1;
int M_LBL_LONGI_THRESH = -1;
int U_TRANS_THRESH = -1;
int U_MAHALA_THRESH = -1;
int M_MAHALA1 = -1;
int M_MAHALA2 = -1;
int M_MAHALA3 = -1;
int M_MAHALA4 = -1;
int M_EKF_NORTH = -1;
int M_EKF_EAST = -1;
int U_LBL_TRANSPONDER_MODE = -1;
int M_USBL_COUNT = -1;
int M_USBL_AZIMUTH = -1;
int M_USBL_ELEVATION = -1;
int M_USBL_TIME = -1;
int M_USBL_QUAL = -1;
int M_USBL_RDOT = -1;
int U_USBL_RINGBUF = -1;
int M_INERTIAL_AZ = -1;
int M_INERTIAL_EL = -1;
int M_USBL_RDOT_MEAN = -1;
int M_USBL_AZIMUTH_NORM = -1;
int M_USBL_R0 = -1;
int M_USBL_REM = -1;
int M_USBL_RANGE = -1;
int SIM_DOCK_N = -1;
int SIM_DOCK_E = -1;
int SIM_DOCK_D = -1;
int M_DOCK_N = -1;
int M_DOCK_E = -1;
int M_DOCK_D = -1;
int U_DOCK_VBL = -1;
int U_DOCK_ZOFF = -1;
int U_LOWER_TAT = -1;
int M_LOWER_RAW = -1;
int U_BUCKY_TAT = -1;
int M_BUCKY_RAW = -1;
int M_CNAV_RADIAL = -1;
int M_CNAV_AXIAL = -1;
int M_CNAV_STATUS = -1;
int AUTOPARSE = -1;
int UO_T_STRAIGHT = -1;
int UO_SEARCH_DIRECTION = -1;
int UO_SEARCH_HR = -1;
int UO_USBL_UPDATE = -1;
int M_LATCHIO0 = -1;
int M_LATCHIO1 = -1;
int M_LATCHIO2 = -1;
int M_LATCHIO3 = -1;
int M_POLE_PRESENT = -1;
int LATCH_START = -1;
int LATCH_CHECK = -1;
int LATCH_STOP = -1;
int M_LOAD_CHARGE = -1;
int M_LOAD_VOLTAGE = -1;
int M_EDC_B_X = -1;
int M_EDC_B_Y = -1;
int M_EDC_B_Z = -1;
int U_EDC_Z_GAIN = -1;
int M_EDC_B_INTENSITY = -1;
int E_EDC_B_NORTH = -1;
int E_EDC_B_EAST = -1;
int E_EDC_B_DOWN = -1;
int E_EDC_B_BEARING = -1;
int E_EDC_B_INCLINATION = -1;
int E_EDC_AXIAL_RANGE = -1;


int *sindex[] = {
    &M_PRESENT_TIME,
    &M_DT,
    &M_ROLL,
    &M_PITCH,
    &M_YAW,
    &M_ROLL_RATE,
    &M_PITCH_RATE,
    &M_YAW_RATE,
    &M_INCLINO_ROLL,
    &M_INCLINO_PITCH,
    &M_COMPASS_YAW,
    &M_DR_NORTH,
    &M_DR_EAST,
    &M_DR_DN,
    &M_DR_DE,
    &M_POS_N,
    &M_POS_E,
    &UO_MAGNETIC_VARIATION,
    &M_PARO_PRES_PERIOD,
    &M_PARO_TEMP_PERIOD,
    &M_INIT_PRES,
    &M_PRESSURE,
    &M_DEPTH,
    &E_LATITUDE,
    &M_DEPTH_RATE,
    &U_PROP_PITCH,
    &M_WATER_SPEED,
    &M_ALTITUDE,
    &M_ST500_RANGE,
    &M_ST200_RANGE,
    &WATER_COND,
    &WATER_TEMP,
    &M_SALINITY,
    &U_SPEED_OF_SOUND,
    &C_DR_NORTH,
    &C_DR_EAST,
    &C_ELEVATOR_ANGLE,
    &M_ELEVATOR_ANGLE,
    &C_RUDDER_ANGLE,
    &M_RUDDER_ANGLE,
    &EFFECTIVE_DRAG_COEFF,
    &C_THRUSTER_CURRENT,
    &M_THRUSTER_CURRENT,
    &M_THRUSTER_PER,
    &M_THRUSTER_RAWBATT,
    &M_PROPELLER_SPEED,
    &M_COPLEY_VOLTAGE,
    &M_COPLEY_CURRENT,
    &M_BATTERY_VOLTAGE,
    &M_BATTERY_CURRENT,
    &M_BATTERY_TEMP,
    &M_BATT_SOC_EST,
    &U_CUTOFF_VOLTAGE,
    &C_WEIGHT1_DROP,
    &C_WEIGHT2_DROP,
    &CC_HEADING_MODE,
    &CC_DEPTH_MODE,
    &CC_SPEED_MODE,
    &C_ABORT_MISSION,
    &C_DEPTH,
    &C_PITCH,
    &C_HEADING,
    &C_SPEED,
    &C_DEPTH_BEHAVIOR,
    &C_DEPTH_PRIORITY,
    &C_HEADING_BEHAVIOR,
    &C_HEADING_PRIORITY,
    &C_SPEED_BEHAVIOR,
    &C_SPEED_PRIORITY,
    &M_HE_ELEV_RAW,
    &M_HE_RUD_RAW,
    &M_HE_ELEV_DEG,
    &M_HE_RUD_DEG,
    &UO_CONTROL_KPP,
    &UO_CONTROL_KDP,
    &UO_CONTROL_KIP,
    &UO_CONTROL_PITCH_LIM,
    &UO_CONTROL_DEPTH_LIM,
    &UO_CONTROL_KPH,
    &UO_CONTROL_KDH,
    &UO_CONTROL_KIH,
    &H_INTEGRAL,
    &D_INTEGRAL,
    &P_INTEGRAL,
    &UO_DEPTH_MAP_K,
    &UO_DEPTH_INT_K,
    &UO_TAU,
    &UO_MAX_HR,
    &UO_MAX_DR,
    &T_HDG,
    &T_HDG_RATE,
    &T_DEPTH,
    &U_WP_X_TRACK_GAIN,
    &WP_XTE,
    &WP_DTW,
    &REAL_X,
    &REAL_Y,
    &REAL_Z,
    &REAL_U,
    &REAL_V,
    &REAL_W,
    &REAL_UD,
    &REAL_VD,
    &REAL_WD,
    &REAL_PHI,
    &REAL_THETA,
    &REAL_PSI,
    &REAL_P,
    &REAL_Q,
    &REAL_R,
    &REAL_PD,
    &REAL_QD,
    &REAL_RD,
    &REAL_FX,
    &REAL_FY,
    &REAL_FZ,
    &REAL_FK,
    &REAL_FM,
    &REAL_FN,
    &REAL_DEPTH_NOISE,
    &REAL_ACC_NOISE,
    &REAL_EXCESS_BUOYANCY,
    &REAL_VEH_W,
    &REAL_VEH_B,
    &REAL_WDW1,
    &REAL_WDW2,
    &REAL_CURRENT_NORTH,
    &REAL_CURRENT_EAST,
    &REAL_THRUSTER_CURRENT,
    &REAL_THRUSTER_VOLTAGE,
    &REAL_THRUSTER_POWER,
    &REAL_THRUSTER_TORQUE,
    &REAL_THRUSTER_THRUST,
    &REAL_THR_PROP_SPEED,
    &REAL_THR_MOTOR_SPEED,
    &REAL_THR_INFLOW_SPEED,
    &REAL_THR_DRIVE_TORQUE,
    &REAL_BATTERY_CURRENT,
    &REAL_BATTERY_VOLTAGE,
    &REAL_BATTERY_POWER,
    &REAL_BATTERY_SOC,
    &REAL_BATTERY_OCV,
    &REAL_BATT_RESISTANCE,
    &REAL_BATT_V_RC,
    &REAL_BATT_TEMP,
    &REAL_BATT_FAULT_HEAT,
    &REAL_BATT_CAPACITY,
    &REAL_BATT_ENERGY_CAP,
    &REAL_BATT_CHARGE_REM,
    &REAL_BATT_ENERGY_REM,
    &REAL_BATTERY_ENERGY_USED,
    &REAL_BATT_CHARGE_USED,
    &READ_CURRENT_FROM_FILE,
    &REAL_SONAR_FRAC_DROPOUT,
    &U_BOTTOM_TYPE,
    &U_BOTTOM_NOISE,
    &REAL_ALT,
    &REAL_TRIANGLE_X1,
    &REAL_TRIANGLE_Y1,
    &REAL_TRIANGLE_Z1,
    &REAL_TRIANGLE_X2,
    &REAL_TRIANGLE_Y2,
    &REAL_TRIANGLE_Z2,
    &REAL_TRIANGLE_X3,
    &REAL_TRIANGLE_Y3,
    &REAL_TRIANGLE_Z3,
    &SIM_TRIANGLE_X1,
    &SIM_TRIANGLE_Y1,
    &SIM_TRIANGLE_Z1,
    &SIM_TRIANGLE_X2,
    &SIM_TRIANGLE_Y2,
    &SIM_TRIANGLE_Z2,
    &SIM_TRIANGLE_X3,
    &SIM_TRIANGLE_Y3,
    &SIM_TRIANGLE_Z3,
    &SIM_SONAR_PHI_T,
    &SIM_SONAR_PHI_B,
    &SIM_SONAR_INTENSITY,
    &SIM_TRIANGLE_ID,
    &REAL_TRIANGLE_ID,
    &SIM_CONTACT_X,
    &SIM_CONTACT_Y,
    &SIM_CONTACT_Z,
    &REAL_CONTACT_X,
    &REAL_CONTACT_Y,
    &REAL_CONTACT_Z,
    &M_FAULT_ACTIVE,
    &VS_MASS,
    &VS_IXX,
    &VS_IYY,
    &VS_IZZ,
    &VS_IXY,
    &VS_IYZ,
    &VS_IZX,
    &VS_XG,
    &VS_YG,
    &VS_ZG,
    &VS_XB,
    &VS_YB,
    &VS_ZB,
    &VS_XDW1,
    &VS_YDW1,
    &VS_ZDW1,
    &VS_BDW1,
    &VS_XDW2,
    &VS_YDW2,
    &VS_ZDW2,
    &VS_BDW2,
    &VS_XUDOT,
    &VS_YVDOT,
    &VS_ZWDOT,
    &VS_KPDOT,
    &VS_MQDOT,
    &VS_NRDOT,
    &VS_KVDOT,
    &VS_MWDOT,
    &VS_NVDOT,
    &VS_YPDOT,
    &VS_YRDOT,
    &VS_ZQDOT,
    &VS_XUABU,
    &VS_YVABV,
    &VS_ZWABW,
    &VS_KPABP,
    &VS_MQABQ,
    &VS_NRABR,
    &VS_MWABW,
    &VS_NVABV,
    &VS_YRABR,
    &VS_ZQABQ,
    &VS_MUQ,
    &VS_MUW,
    &VS_MPR,
    &VS_NUR,
    &VS_NUV,
    &VS_NPQ,
    &VS_XVV,
    &VS_XWW,
    &VS_XVR,
    &VS_XWQ,
    &VS_XRR,
    &VS_XQQ,
    &VS_YUR,
    &VS_YUV,
    &VS_YWP,
    &VS_ZUQ,
    &VS_ZUW,
    &VS_ZVP,
    &VS_KVT2,
    &VS_FIN1_X,
    &VS_FIN1_Y,
    &VS_FIN1_Z,
    &VS_FIN2_X,
    &VS_FIN2_Y,
    &VS_FIN2_Z,
    &VS_FIN3_X,
    &VS_FIN3_Y,
    &VS_FIN3_Z,
    &VS_FIN4_X,
    &VS_FIN4_Y,
    &VS_FIN4_Z,
    &VS_FIN_AREA,
    &VS_FIN_ASPECT_RATIO,
    &VS_FIN_CDO,
    &VS_FIN_CD_CROSS,
    &VS_FIN_DCL,
    &VS_FIN_OSWALD,
    &VS_FIN_STALL_ANGLE,
    &VS_FIN_SPEED,
    &VS_ACT_DELAY,
    &VS_RUD_HYST_WIDTH,
    &VS_RUD_HYST_CENTER,
    &VS_ELEV_HYST_WIDTH,
    &VS_ELEV_HYST_CENTER,
    &VS_THRUSTER_KT,
    &VS_THRUSTER_ETA,
    &VS_THRUSTER_PITCH,
    &VS_THRUSTER_DUCT_AREA,
    &VS_THRUSTER_PROP_DIAMETER,
    &VS_THRUSTER_WAKE_FRACTION,
    &VS_THRUSTER_KT_SCALE,
    &VS_THRUSTER_KQ_SCALE,
    &VS_THRUSTER_ETA_SCALE,
    &VS_THRUSTER_AREA_SCALE,
    &VS_THRUSTER_MOTOR_KT,
    &VS_THRUSTER_MOTOR_KE,
    &VS_THRUSTER_MOTOR_R,
    &VS_THRUSTER_GEAR_RATIO,
    &VS_THRUSTER_GEAR_EFF,
    &VS_THRUSTER_SEAL_FRICTION,
    &VS_THRUSTER_CURRENT_LIMIT,
    &VS_THRUSTER_VOLTAGE_LIMIT,
    &VS_BATT_CHEM,
    &VS_BATTERY_NOMINAL_VOLTAGE,
    &VS_BATT_SERIES_CELLS,
    &VS_BATT_PARALLEL_STRS,
    &VS_BATT_CAPACITY,
    &VS_BATT_ENERGY_CAP,
    &VS_BATT_INITIAL_SOC,
    &VS_BATT_R0_CELL,
    &VS_BATT_R_WIRING,
    &VS_BATT_R_CONN,
    &VS_BATT_R1,
    &VS_BATT_C1,
    &VS_BATT_THERM_MASS,
    &VS_BATT_THERM_RES,
    &VS_BATT_AGE,
    &VS_BATT_CAP_SCALE,
    &VS_BATT_R_SCALE,
    &VS_BATT_PARASITIC,
    &VS_BATT_MAX_CURRENT,
    &VS_BATT_MIN_VOLTAGE,
    &VS_BATT_BMS_ENABLE,
    &VS_BATT_BMS_VTRIP,
    &VS_BATT_BMS_ITRIP,
    &VS_BATT_BMS_TTRIP,
    &VS_BATT_IMBALANCE,
    &VS_BATT_WEAK_CELL_V,
    &VS_BATT_CURR_BIAS,
    &VS_BATT_VOLT_BIAS,
    &VS_BATT_EST_CAP,
    &VS_BATT_EST_INIT_SOC,
    &VS_BATT_RUNAWAY_TEMP,
    &VS_BATT_RUNAWAY_HEAT,
    &VS_HOTEL_LOAD,
    &EDGE_TR_XS_TIME,
    &EDGE_AM200_BACS,
    &EDGE_XS_DATABYTE,
    &REAL_LBL_TIMING_NOISE,
    &REAL_LBL_FRAC_BIAS,
    &REAL_LBL_FRAC_DROPOUT,
    &REAL_LBL_BIAS_AMPLITUDE,
    &M_ROLL_DRIFT,
    &M_PITCH_DRIFT,
    &M_YAW_DRIFT,
    &U_LBL_BEACON_NUMBER,
    &U_LBL_1NORTH,
    &U_LBL_1EAST,
    &U_LBL_1DEPTH,
    &U_LBL_1TAT,
    &U_LBL_2NORTH,
    &U_LBL_2EAST,
    &U_LBL_2DEPTH,
    &U_LBL_2TAT,
    &U_LBL_3NORTH,
    &U_LBL_3EAST,
    &U_LBL_3DEPTH,
    &U_LBL_3TAT,
    &U_LBL_4NORTH,
    &U_LBL_4EAST,
    &U_LBL_4DEPTH,
    &U_LBL_4TAT,
    &C_LBL_CONTROL,
    &U_LBL_CYCLE,
    &U_LBL_LOCKOUT,
    &U_LBL_TIMEOUT,
    &M_LBL_0T,
    &M_LBL_1T,
    &M_LBL_2T,
    &M_LBL_3T,
    &M_LBL_4T,
    &LBL_NEWDATA,
    &M_LBL_NORTHFIX,
    &M_LBL_EASTFIX,
    &LBL_FIXSTAT,
    &LBL_FIXGAIN,
    &LBL_RANGE_TOL,
    &LBL_FIX_ERR,
    &LBL_MIN_ANGLE,
    &M_LBL_EST_NORTH,
    &M_LBL_EST_EAST,
    &M_LBL_HEADING,
    &U_LBL_MEAN_THRESHOLD,
    &U_LBL_DR_THRESHOLD,
    &M_LBL_DR_NORTH,
    &M_LBL_DR_EAST,
    &M_LBL_PING_TIME,
    &M_LBL_NUM_USED,
    &M_LBL_VALID1,
    &M_LBL_VALID2,
    &M_LBL_VALID3,
    &M_LBL_VALID4,
    &M_LBL_DIST_TO_LINE,
    &M_LBL_DIST_TO_LAST,
    &M_LBL_TRANS_THRESH,
    &M_LBL_LONGI_THRESH,
    &U_TRANS_THRESH,
    &U_MAHALA_THRESH,
    &M_MAHALA1,
    &M_MAHALA2,
    &M_MAHALA3,
    &M_MAHALA4,
    &M_EKF_NORTH,
    &M_EKF_EAST,
    &U_LBL_TRANSPONDER_MODE,
    &M_USBL_COUNT,
    &M_USBL_AZIMUTH,
    &M_USBL_ELEVATION,
    &M_USBL_TIME,
    &M_USBL_QUAL,
    &M_USBL_RDOT,
    &U_USBL_RINGBUF,
    &M_INERTIAL_AZ,
    &M_INERTIAL_EL,
    &M_USBL_RDOT_MEAN,
    &M_USBL_AZIMUTH_NORM,
    &M_USBL_R0,
    &M_USBL_REM,
    &M_USBL_RANGE,
    &SIM_DOCK_N,
    &SIM_DOCK_E,
    &SIM_DOCK_D,
    &M_DOCK_N,
    &M_DOCK_E,
    &M_DOCK_D,
    &U_DOCK_VBL,
    &U_DOCK_ZOFF,
    &U_LOWER_TAT,
    &M_LOWER_RAW,
    &U_BUCKY_TAT,
    &M_BUCKY_RAW,
    &M_CNAV_RADIAL,
    &M_CNAV_AXIAL,
    &M_CNAV_STATUS,
    &AUTOPARSE,
    &UO_T_STRAIGHT,
    &UO_SEARCH_DIRECTION,
    &UO_SEARCH_HR,
    &UO_USBL_UPDATE,
    &M_LATCHIO0,
    &M_LATCHIO1,
    &M_LATCHIO2,
    &M_LATCHIO3,
    &M_POLE_PRESENT,
    &LATCH_START,
    &LATCH_CHECK,
    &LATCH_STOP,
    &M_LOAD_CHARGE,
    &M_LOAD_VOLTAGE,
    &M_EDC_B_X,
    &M_EDC_B_Y,
    &M_EDC_B_Z,
    &U_EDC_Z_GAIN,
    &M_EDC_B_INTENSITY,
    &E_EDC_B_NORTH,
    &E_EDC_B_EAST,
    &E_EDC_B_DOWN,
    &E_EDC_B_BEARING,
    &E_EDC_B_INCLINATION,
    &E_EDC_AXIAL_RANGE
};

extern int mission_timer( struct behavior *, int, int );
extern int acquire_heading( struct behavior *, int, int );
extern int ascend( struct behavior *, int, int );
extern int broadcast( struct behavior *, int, int );
extern int chirp_fin( struct behavior *, int, int );
extern int depth_envelope( struct behavior *, int, int );
extern int descend( struct behavior *, int, int );
extern int dock( struct behavior *, int, int );
extern int undock( struct behavior *, int, int );
extern int whoi_homing( struct behavior *, int, int );
extern int edc_homing( struct behavior *, int, int );
extern int set_elevator( struct behavior *, int, int );
extern int set_rudder( struct behavior *, int, int );
extern int setmode( struct behavior *, int, int );
extern int setpoint( struct behavior *, int, int );
extern int setvector( struct behavior *, int, int );
extern int survey_dead_reckon( struct behavior *, int, int );
extern int remote_delay( struct behavior *, int, int );
extern int waypoint_2d( struct behavior *, int, int );
extern int wiggle( struct behavior *, int, int );
extern int launch( struct behavior *, int, int );
extern int yo_yo( struct behavior *, int, int );


int assign_behavior_fcn_ptr( char *name, struct behavior *b)
{
#ifndef COMM_VEHICLE_C
    if ( b == NULL ) {
        return -1;
    }

    if ( strcmp("mission_timer", name) == 0 ) b->fcn = mission_timer;
    if ( strcmp("acquire_heading", name) == 0 ) b->fcn = acquire_heading;
    if ( strcmp("ascend", name) == 0 ) b->fcn = ascend;
    if ( strcmp("broadcast", name) == 0 ) b->fcn = broadcast;
    if ( strcmp("chirp_fin", name) == 0 ) b->fcn = chirp_fin;
    if ( strcmp("depth_envelope", name) == 0 ) b->fcn = depth_envelope;
    if ( strcmp("descend", name) == 0 ) b->fcn = descend;
    if ( strcmp("dock", name) == 0 ) b->fcn = dock;
    if ( strcmp("undock", name) == 0 ) b->fcn = undock;
    if ( strcmp("whoi_homing", name) == 0 ) b->fcn = whoi_homing;
    if ( strcmp("edc_homing", name) == 0 ) b->fcn = edc_homing;
    if ( strcmp("set_elevator", name) == 0 ) b->fcn = set_elevator;
    if ( strcmp("set_rudder", name) == 0 ) b->fcn = set_rudder;
    if ( strcmp("setmode", name) == 0 ) b->fcn = setmode;
    if ( strcmp("setpoint", name) == 0 ) b->fcn = setpoint;
    if ( strcmp("setvector", name) == 0 ) b->fcn = setvector;
    if ( strcmp("survey_dead_reckon", name) == 0 ) b->fcn = survey_dead_reckon;
    if ( strcmp("remote_delay", name) == 0 ) b->fcn = remote_delay;
    if ( strcmp("waypoint_2d", name) == 0 ) b->fcn = waypoint_2d;
    if ( strcmp("wiggle", name) == 0 ) b->fcn = wiggle;
    if ( strcmp("launch", name) == 0 ) b->fcn = launch;
    if ( strcmp("yo_yo", name) == 0 ) b->fcn = yo_yo;
    if ( b->fcn == NULL )
    {
        return -1;
    }
#endif /* COMM_VEHICLE_C */
}
