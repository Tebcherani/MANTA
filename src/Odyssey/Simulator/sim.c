/*----------------------------------------------------------------------*
   Copyright (C) 1994-1997, Massachusetts Institute of Technology.
	 Sea Grant Autonomous Underwater Vehicles Laboratory.
			 All rights reserved.
  $Id: sim.c,v 2.21 1998/01/20 00:31:51 bamoran Exp $
 *----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------*
 | sim.c    Odyssey II hydrodynamic simulator with Franz Hover's updates |
 |                                                                       |
 | Last mod:  6/9/95 Franz Hover                                         |
 | History: copied to SGI from Mac by June 1, 1994 by jleonard           |
 |          compilation of vehicle simulation routines written by        |
 |          Jim Bellingham and other Sea Grant staff (198?-1994)         |
 |          90% of the code came from Draper, a student of Abkowitz.     |
 |                                                                       |
 |          From March to June, 1995 this code was significantly         |
 |          improved by Franz Hover.  Errors were removed and the        |
 |          model was updated based on theoretical considerations,       |
 |          Draper's modeling program, and analysis of mission data      |
 |                                                                       |
 | NOTE: all vehicle physical parameters (mass, inertia, CG/CB,         |
 | hydrodynamic and fin coefficients) are read from vs_* variables in   |
 | the data structure, populated by load_vehicle_def() before           |
 | init_simulation() is called.  There are no external coefficient      |
 | files.                                                                |
 *-----------------------------------------------------------------------*/

#include <stdlib.h>
#include <stdio.h>
#include <math.h>

#include "vehicle.h"
#include "odyssey_data_struct.h"
#include "world_sim.h"

/*------------------- generic constants -------------------------------*/

#define FTOM   0.3048               /* from feet to meters */
#define FTOM2  (0.3048 * 0.3048)
#define FTOM3  (0.3048 * 0.3048 * 0.3048)


/*=====================================================================
  Modified this to reflect nature of Odyssey II's current control
  on the thruster controller...  Calculate torque (Kt*amps) then
  calculate propellor RPM for that torque and water speed.  This in
  turn allows one to compute thrust....
  J. G. Bellingham        1/94
  fsh update     5/95    
 *====================================================================*/
        
struct thruster_state {
    double amps;
    double inflow;
    double prop_omega;
    double motor_omega;
    double thrust;
    double vehicle_torque;
    double motor_voltage;
    double elec_power;
    double drivetrain_torque;
};

static struct thruster_state last_thruster_state;
static int last_thruster_state_valid = 0;
static double battery_energy_used = 0.0;
static double battery_charge_used = 0.0;
static double battery_energy_capacity = 0.0;
static double battery_charge_capacity = 0.0;
static double battery_initial_soc = 1.0;
static double battery_terminal_voltage = 48.0;
static double battery_v_rc = 0.0;
static double battery_temp = 296.15;
static int battery_bms_tripped = 0;
static int battery_initialized = 0;
static double last_battery_time = -1.0;

static double clip_abs( double x, double limit )
{
    if (limit <= 0.0) return x;
    if (fabs(x) > limit) {
        if (x > 0.0) return limit;
        return -limit;
    }
    return x;
}

static void compute_thruster_state(const world_state_t *world_state, double real_amps, double w_speed,
                                    struct thruster_state *ts )
{
    double current_limit, voltage_limit;
    double amps, inflow, area, eta, pitch, kt_fit;
    double numerator, denominator;
    double gear_ratio, motor_ke, motor_r, motor_kt, gear_eff, seal_friction;
    double kt_scale, kq_scale, eta_scale, area_scale, wake_fraction;

    current_limit = read_sensor(VS_THRUSTER_CURRENT_LIMIT);
    voltage_limit = read_sensor(VS_THRUSTER_VOLTAGE_LIMIT);

    amps = clip_abs(real_amps, current_limit);

    wake_fraction = read_sensor(VS_THRUSTER_WAKE_FRACTION);
    if (wake_fraction <= 0.0) wake_fraction = 1.0;
    inflow = wake_fraction * w_speed;

    kt_scale = read_sensor(VS_THRUSTER_KT_SCALE);
    kq_scale = read_sensor(VS_THRUSTER_KQ_SCALE);
    eta_scale = read_sensor(VS_THRUSTER_ETA_SCALE);
    area_scale = read_sensor(VS_THRUSTER_AREA_SCALE);

    if (kt_scale < 0.0) kt_scale = 0.0;
    if (kq_scale < 0.0) kq_scale = 0.0;
    if (eta_scale <= 0.0) eta_scale = 1.0;
    if (area_scale <= 0.0) area_scale = 1.0;

    /*
       Legacy effective calibration model. The effective area is left as
       vs_thruster_duct_area for backwards compatibility. The physical
       propeller diameter is stored separately as vs_thruster_prop_diameter
       for later Kt(J)/Kq(J) mapping and documentation.
    */
    kt_fit = read_sensor(VS_THRUSTER_KT) * kt_scale;
    eta = read_sensor(VS_THRUSTER_ETA) * eta_scale;
    pitch = read_sensor(VS_THRUSTER_PITCH);
    area = read_sensor(VS_THRUSTER_DUCT_AREA) * area_scale;

    numerator = kt_fit * amps + .5 * area * eta * pitch * world_state->seawater_density * inflow*inflow;
    denominator = .5 * area * eta * eta * eta * pitch * pitch * pitch * world_state->seawater_density;

    if (denominator > 0.0 && numerator > 0.0)
        ts->prop_omega = sqrt(numerator/denominator);
    else
        ts->prop_omega = 0.0;

    ts->thrust = world_state->seawater_density * area *
        (pow(eta*pitch*ts->prop_omega,2) - inflow*eta*pitch*ts->prop_omega);

    ts->vehicle_torque = read_sensor(VS_THRUSTER_KT) * amps * kq_scale;

    gear_ratio = read_sensor(VS_THRUSTER_GEAR_RATIO);
    motor_ke = read_sensor(VS_THRUSTER_MOTOR_KE);
    motor_r = read_sensor(VS_THRUSTER_MOTOR_R);
    motor_kt = read_sensor(VS_THRUSTER_MOTOR_KT);
    gear_eff = read_sensor(VS_THRUSTER_GEAR_EFF);
    seal_friction = read_sensor(VS_THRUSTER_SEAL_FRICTION);

    if (gear_ratio <= 0.0) gear_ratio = 1.0;
    if (gear_eff <= 0.0) gear_eff = 1.0;

    ts->motor_omega = gear_ratio * ts->prop_omega;
    ts->motor_voltage = motor_r*amps + motor_ke*ts->motor_omega;

    /* Voltage saturation is a fault/limit hook. With the default high limit,
       this has no effect and the legacy calibration is preserved. */
    if (voltage_limit > 0.0 && fabs(ts->motor_voltage) > voltage_limit) {
        amps = amps * voltage_limit / fabs(ts->motor_voltage);
        numerator = kt_fit*amps + .5*area*eta*pitch*world_state->seawater_density*inflow*inflow;
        if (denominator > 0.0 && numerator > 0.0)
            ts->prop_omega = sqrt(numerator/denominator);
        else
            ts->prop_omega = 0.0;
        ts->thrust = world_state->seawater_density * area *
            (pow(eta*pitch*ts->prop_omega,2) - inflow*eta*pitch*ts->prop_omega);
        ts->vehicle_torque = read_sensor(VS_THRUSTER_KT) * amps * kq_scale;
        ts->motor_omega = gear_ratio * ts->prop_omega;
        ts->motor_voltage = motor_r*amps + motor_ke*ts->motor_omega;
    }

    ts->elec_power = ts->motor_voltage * amps;
    ts->drivetrain_torque = gear_eff*gear_ratio*motor_kt*amps;
    if (ts->prop_omega > 0.0)
        ts->drivetrain_torque -= seal_friction;
    else if (ts->prop_omega < 0.0)
        ts->drivetrain_torque += seal_friction;

    ts->amps = amps;
    ts->inflow = inflow;
}

static double interp_table( double x, const double *xs, const double *ys, int n )
{
    int i;

    if (n <= 0) return 0.0;
    if (x <= xs[0]) return ys[0];
    if (x >= xs[n-1]) return ys[n-1];

    for (i = 0; i < n-1; i++) {
        if (x >= xs[i] && x <= xs[i+1]) {
            if (xs[i+1] == xs[i]) return ys[i];
            return ys[i] + (ys[i+1] - ys[i]) *
                (x - xs[i]) / (xs[i+1] - xs[i]);
        }
    }
    return ys[n-1];
}

static double battery_capacity_temp_scale( int chem, double temp_k )
{
    double temp_c;

    temp_c = temp_k - 273.15;
    if (chem == 2) {
        if (temp_c <= -20.0) return 0.55;
        if (temp_c < 25.0) return 0.55 + 0.45*(temp_c + 20.0)/45.0;
        return 1.0;
    }

    if (temp_c <= -2.0) return 0.70;
    if (temp_c < 23.0) return 0.70 + 0.30*(temp_c + 2.0)/25.0;
    return 1.0;
}

static double battery_cell_ocv( int chem, double soc, double temp_k,
                                double age )
{
    static const double ag_soc[] = {
        0.00, 0.05, 0.10, 0.15, 0.25, 0.40, 0.60, 0.80, 0.90, 0.95, 1.00
    };
    static const double ag_ocv[] = {
        0.80, 1.10, 1.35, 1.44, 1.48, 1.50, 1.51, 1.52, 1.53, 1.56, 1.62
    };
    static const double li_soc[] = {
        0.00, 0.05, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70, 0.80, 0.90, 0.95, 1.00
    };
    static const double li_ocv[] = {
        2.80, 3.20, 3.45, 3.62, 3.70, 3.74, 3.78, 3.82, 3.88, 3.96, 4.05, 4.12, 4.20
    };
    double ocv, temp_c, temp_ref_c, temp_coeff, age_coeff;

    if (soc < 0.0) soc = 0.0;
    if (soc > 1.0) soc = 1.0;
    if (age < 0.0) age = 0.0;

    temp_c = temp_k - 273.15;
    if (chem == 2) {
        ocv = interp_table(soc, li_soc, li_ocv,
                           sizeof(li_soc)/sizeof(li_soc[0]));
        temp_ref_c = 25.0;
        temp_coeff = -0.0004;
        age_coeff = 0.04;
    }
    else {
        ocv = interp_table(soc, ag_soc, ag_ocv,
                           sizeof(ag_soc)/sizeof(ag_soc[0]));
        temp_ref_c = 23.0;
        temp_coeff = -0.0008;
        age_coeff = 0.08;
    }

    ocv += temp_coeff * (temp_ref_c - temp_c);
    ocv -= age_coeff * age;
    if (ocv < 0.0) ocv = 0.0;
    return ocv;
}

static double battery_soc_res_scale( int chem, double soc )
{
    static const double ag_soc[] = {
        0.05, 0.10, 0.20, 0.40, 0.60, 0.80, 1.00
    };
    static const double ag_scale[] = {
        6.0, 3.0, 1.5, 1.1, 1.0, 1.0, 1.1
    };
    static const double li_soc[] = {
        0.05, 0.10, 0.20, 0.40, 0.60, 0.80, 1.00
    };
    static const double li_scale[] = {
        4.0, 2.0, 1.4, 1.05, 1.0, 1.0, 1.2
    };

    if (chem == 2)
        return interp_table(soc, li_soc, li_scale,
                            sizeof(li_soc)/sizeof(li_soc[0]));
    return interp_table(soc, ag_soc, ag_scale,
                        sizeof(ag_soc)/sizeof(ag_soc[0]));
}

static double battery_temp_res_scale( int chem, double temp_k )
{
    static const double ag_temp[] = {271.15, 296.15, 308.15};
    static const double ag_scale[] = {2.5, 1.0, 0.8};
    static const double li_temp[] = {253.15, 273.15, 298.15, 318.15};
    static const double li_scale[] = {6.0, 2.0, 1.0, 0.8};

    if (chem == 2)
        return interp_table(temp_k, li_temp, li_scale,
                            sizeof(li_temp)/sizeof(li_temp[0]));
    return interp_table(temp_k, ag_temp, ag_scale,
                        sizeof(ag_temp)/sizeof(ag_temp[0]));
}

static void update_battery_load( struct thruster_state *ts )
{
    double hotel_load, battery_voltage, battery_power, battery_current;
    double present_time, dt, charge_remaining, energy_remaining, soc;
    double ocv, resistance, r0_cell, r1, c1, tau, target_v_rc;
    double series_cells, parallel_strings, age, cap_scale, r_scale;
    double therm_mass, therm_res, heat, seawater_k, weak_v, temp_scale;
    double est_cap, est_soc, measured_current, measured_voltage;
    double bms_vtrip, bms_itrip, bms_ttrip, max_current, min_voltage;
    int chem;

    hotel_load = read_sensor(VS_HOTEL_LOAD);
    if (hotel_load < 0.0) hotel_load = 0.0;

    battery_voltage = read_sensor(VS_BATTERY_NOMINAL_VOLTAGE);
    if (battery_voltage <= 0.0) battery_voltage = 48.0;

    if (!battery_initialized) {
        chem = (int) (read_sensor(VS_BATT_CHEM) + 0.5);
        if (chem != 2) chem = 1;

        cap_scale = read_sensor(VS_BATT_CAP_SCALE);
        if (cap_scale < 0.0) cap_scale = 0.0;

        battery_temp = read_sensor(WATER_TEMP) + 273.15;
        temp_scale = battery_capacity_temp_scale(chem, battery_temp);
        battery_charge_capacity = read_sensor(VS_BATT_CAPACITY) *
            cap_scale * temp_scale;
        battery_energy_capacity = read_sensor(VS_BATT_ENERGY_CAP) *
            cap_scale * temp_scale;
        battery_initial_soc = read_sensor(VS_BATT_INITIAL_SOC);

        if (battery_initial_soc < 0.0) battery_initial_soc = 0.0;
        if (battery_initial_soc > 1.0) battery_initial_soc = 1.0;

        if (battery_charge_capacity <= 0.0 && battery_energy_capacity > 0.0)
            battery_charge_capacity = battery_energy_capacity / battery_voltage;
        if (battery_energy_capacity <= 0.0 && battery_charge_capacity > 0.0)
            battery_energy_capacity = battery_charge_capacity * battery_voltage;

        battery_charge_used = (1.0 - battery_initial_soc) * battery_charge_capacity;
        battery_energy_used = (1.0 - battery_initial_soc) * battery_energy_capacity;
        battery_terminal_voltage = battery_voltage;
        battery_v_rc = 0.0;
        battery_bms_tripped = 0;
        battery_initialized = 1;
    }

    battery_power = fabs(ts->elec_power) + hotel_load;
    if (battery_terminal_voltage > 1.0)
        battery_current = battery_power / battery_terminal_voltage;
    else
        battery_current = battery_power / battery_voltage;
    battery_current += read_sensor(VS_BATT_PARASITIC);
    if (battery_current < 0.0) battery_current = 0.0;

    present_time = read_sensor(M_PRESENT_TIME);
    if (last_battery_time >= 0.0 && present_time >= last_battery_time)
        dt = present_time - last_battery_time;
    else
        dt = 0.0;

    charge_remaining = battery_charge_capacity - battery_charge_used;
    energy_remaining = battery_energy_capacity - battery_energy_used;
    if (charge_remaining < 0.0) charge_remaining = 0.0;
    if (energy_remaining < 0.0) energy_remaining = 0.0;

    if (battery_charge_capacity > 0.0)
        soc = charge_remaining / battery_charge_capacity;
    else if (battery_energy_capacity > 0.0)
        soc = energy_remaining / battery_energy_capacity;
    else
        soc = 0.0;

    chem = (int) (read_sensor(VS_BATT_CHEM) + 0.5);
    if (chem != 2) chem = 1;

    series_cells = read_sensor(VS_BATT_SERIES_CELLS);
    parallel_strings = read_sensor(VS_BATT_PARALLEL_STRS);
    if (series_cells <= 0.0) series_cells = 1.0;
    if (parallel_strings <= 0.0) parallel_strings = 1.0;

    age = read_sensor(VS_BATT_AGE);
    if (age < 0.0) age = 0.0;

    r0_cell = read_sensor(VS_BATT_R0_CELL);
    r_scale = read_sensor(VS_BATT_R_SCALE);
    if (r_scale <= 0.0) r_scale = 1.0;

    resistance = r0_cell * series_cells / parallel_strings *
        battery_soc_res_scale(chem, soc) *
        battery_temp_res_scale(chem, battery_temp) *
        (1.0 + age) * r_scale;
    resistance += read_sensor(VS_BATT_R_WIRING) + read_sensor(VS_BATT_R_CONN);
    resistance += fabs(read_sensor(VS_BATT_IMBALANCE)) * r0_cell * series_cells;
    if (resistance < 0.0) resistance = 0.0;

    r1 = read_sensor(VS_BATT_R1) * r_scale;
    c1 = read_sensor(VS_BATT_C1);
    if (r1 < 0.0) r1 = 0.0;
    if (c1 < 0.0) c1 = 0.0;
    if (r1 > 0.0 && c1 > 0.0 && dt > 0.0) {
        tau = r1 * c1;
        target_v_rc = battery_current * r1;
        if (dt >= tau)
            battery_v_rc = target_v_rc;
        else
            battery_v_rc += (target_v_rc - battery_v_rc) * dt / tau;
    }
    else if (r1 <= 0.0)
        battery_v_rc = 0.0;

    weak_v = read_sensor(VS_BATT_WEAK_CELL_V);
    weak_v += fabs(read_sensor(VS_BATT_IMBALANCE)) *
        battery_cell_ocv(chem, soc, battery_temp, age);

    ocv = series_cells * battery_cell_ocv(chem, soc, battery_temp, age)
        - weak_v;
    if (ocv < 0.0) ocv = 0.0;

    battery_terminal_voltage = ocv - battery_current*resistance - battery_v_rc;
    min_voltage = read_sensor(VS_BATT_MIN_VOLTAGE);
    if (soc <= 0.0 || battery_terminal_voltage < 0.0)
        battery_terminal_voltage = 0.0;

    max_current = read_sensor(VS_BATT_MAX_CURRENT);
    if (max_current > 0.0 && battery_current > max_current)
        battery_current = max_current;

    bms_vtrip = read_sensor(VS_BATT_BMS_VTRIP);
    bms_itrip = read_sensor(VS_BATT_BMS_ITRIP);
    bms_ttrip = read_sensor(VS_BATT_BMS_TTRIP);
    if (read_sensor(VS_BATT_BMS_ENABLE) > 0.5) {
        if ((bms_vtrip > 0.0 && battery_terminal_voltage < bms_vtrip) ||
            (bms_itrip > 0.0 && battery_current > bms_itrip) ||
            (bms_ttrip > 0.0 && battery_temp > bms_ttrip))
            battery_bms_tripped = 1;
    }
    if (min_voltage > 0.0 && battery_terminal_voltage < min_voltage &&
        soc <= 0.0)
        battery_terminal_voltage = 0.0;
    if (battery_bms_tripped) {
        battery_terminal_voltage = 0.0;
        battery_current = 0.0;
        battery_power = 0.0;
    }

    heat = battery_current*battery_current*(resistance + r1);
    if (battery_temp > read_sensor(VS_BATT_RUNAWAY_TEMP))
        heat += read_sensor(VS_BATT_RUNAWAY_HEAT);

    therm_mass = read_sensor(VS_BATT_THERM_MASS);
    therm_res = read_sensor(VS_BATT_THERM_RES);
    seawater_k = read_sensor(WATER_TEMP) + 273.15;
    if (therm_mass > 0.0 && therm_res > 0.0 && dt > 0.0)
        battery_temp += dt * (heat - (battery_temp - seawater_k)/therm_res)
            / therm_mass;

    battery_power = battery_terminal_voltage * battery_current;

    battery_energy_used += battery_power * dt;
    battery_charge_used += battery_current * dt;
    last_battery_time = present_time;

    charge_remaining = battery_charge_capacity - battery_charge_used;
    energy_remaining = battery_energy_capacity - battery_energy_used;
    if (charge_remaining < 0.0) charge_remaining = 0.0;
    if (energy_remaining < 0.0) energy_remaining = 0.0;
    if (battery_charge_capacity > 0.0)
        soc = charge_remaining / battery_charge_capacity;
    else if (battery_energy_capacity > 0.0)
        soc = energy_remaining / battery_energy_capacity;
    else
        soc = 0.0;

    measured_current = battery_current + read_sensor(VS_BATT_CURR_BIAS);
    measured_voltage = battery_terminal_voltage + read_sensor(VS_BATT_VOLT_BIAS);

    est_cap = read_sensor(VS_BATT_EST_CAP);
    if (est_cap <= 0.0) est_cap = battery_charge_capacity;
    est_soc = read_sensor(VS_BATT_EST_INIT_SOC) - battery_charge_used / est_cap;
    if (est_soc < 0.0) est_soc = 0.0;
    if (est_soc > 1.0) est_soc = 1.0;

    update_sensor( REAL_BATTERY_POWER, battery_power );
    update_sensor( REAL_BATTERY_CURRENT, battery_current );
    update_sensor( REAL_BATTERY_VOLTAGE, battery_terminal_voltage );
    update_sensor( REAL_BATTERY_SOC, soc );
    update_sensor( REAL_BATTERY_OCV, ocv );
    update_sensor( REAL_BATT_RESISTANCE, resistance );
    update_sensor( REAL_BATT_V_RC, battery_v_rc );
    update_sensor( REAL_BATT_TEMP, battery_temp );
    update_sensor( REAL_BATT_FAULT_HEAT, heat );
    update_sensor( REAL_BATT_CAPACITY, battery_charge_capacity );
    update_sensor( REAL_BATT_ENERGY_CAP, battery_energy_capacity );
    update_sensor( REAL_BATT_CHARGE_REM, charge_remaining );
    update_sensor( REAL_BATT_ENERGY_REM, energy_remaining );
    update_sensor( REAL_BATTERY_ENERGY_USED, battery_energy_used );
    update_sensor( REAL_BATT_CHARGE_USED, battery_charge_used );

    update_sensor( M_LOAD_CHARGE, battery_charge_used );
    update_sensor( M_LOAD_VOLTAGE, measured_voltage );
    update_sensor( M_BATTERY_VOLTAGE, measured_voltage );
    update_sensor( M_BATTERY_CURRENT, measured_current );
    update_sensor( M_BATTERY_TEMP, battery_temp );
    update_sensor( M_BATT_SOC_EST, est_soc );
}

static void update_thruster_diagnostics( struct thruster_state *ts )
{
    /* m_propeller_speed is existing telemetry; keep it for compatibility
       while also logging simulator truth in real_thruster_* channels. */
    update_sensor( M_PROPELLER_SPEED, 2*M_PI*60/5.54*ts->prop_omega );
    update_sensor( REAL_THR_PROP_SPEED, ts->prop_omega );
    update_sensor( REAL_THR_MOTOR_SPEED, ts->motor_omega );
    update_sensor( REAL_THRUSTER_VOLTAGE, ts->motor_voltage );
    update_sensor( REAL_THRUSTER_POWER, ts->elec_power );
    update_sensor( REAL_THRUSTER_TORQUE, ts->vehicle_torque );
    update_sensor( REAL_THRUSTER_THRUST, ts->thrust );
    update_sensor( REAL_THR_INFLOW_SPEED, ts->inflow );
    update_sensor( REAL_THR_DRIVE_TORQUE, ts->drivetrain_torque );
    update_sensor( REAL_THRUSTER_CURRENT, ts->amps );

    /* In simulator mode these measured channels are ideal telemetry from the
       truth model. Sensor faults should corrupt m_* rather than real_* values. */
    update_sensor( M_COPLEY_VOLTAGE, ts->motor_voltage );
    update_sensor( M_COPLEY_CURRENT, ts->amps );

    update_battery_load(ts);
}

double thruster_force(const world_state_t *world_state, double real_amps, double w_speed )
{
    compute_thruster_state(world_state, real_amps, w_speed, &last_thruster_state);
    last_thruster_state_valid = 1;
    update_thruster_diagnostics(&last_thruster_state);
    return last_thruster_state.thrust;
}

/*======================================================================
  Modified this to reflect nature of Odyssey II's current control
  on the thruster controller...torque is proportional to current...
  J. G. Bellingham        1/94
 *======================================================================*/

double thruster_torque(const world_state_t *world_state, double real_amps, double w_speed )
{
    if (!last_thruster_state_valid ||
        last_thruster_state.amps != real_amps ||
        last_thruster_state.inflow != read_sensor(VS_THRUSTER_WAKE_FRACTION)*w_speed)
        compute_thruster_state(world_state, real_amps, w_speed, &last_thruster_state);

    return last_thruster_state.vehicle_torque;
}