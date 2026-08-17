#include <math.h>
#include <stddef.h>
#include <stdlib.h>

#include "sensor_water_conductivity.h"
#include "sim_log.h"

static double ref_conductivity_ratio(double);
static double pressure_coeff(double);
static double salinity_conductivity_conversion(double, double, double, int);
static double salinity_derivative(double, double);
static double compute_salinity(double, double);
static double temp_coeff_a(double);
static double temp_coeff_b(double);

static double ref_conductivity_ratio(double temp) {
    return (((1.0031e-9 * temp - 6.9698e-7) * temp + 1.104259e-4) * temp + 2.00564e-2) * temp + 0.6766097;
}

static double pressure_coeff(double pressure) {
    return ((3.989e-15 * pressure - 6.370e-10) * pressure + 2.070e-5) * pressure;
}

static double salinity_conductivity_conversion(double input, double temp, double pressure, int convert_salinity_to_conductivity) {
    double conductivity_ratio, temp_corrected_conductivity_ratio;
    double salinity, diff;
    double temp_correction, B_temp_corrected;
    int i;
    double temp_diff = temp - 15.0;

    double A = temp_coeff_a(temp);
    double B = temp_coeff_b(temp);
    double P = pressure_coeff(pressure);

    if (convert_salinity_to_conductivity) {
        temp_corrected_conductivity_ratio = sqrt(input / 35.0);
        salinity = compute_salinity(temp_corrected_conductivity_ratio, temp_diff);
        diff = fabs(salinity - input);

        for(i = 0; i <= 10; i++) {
            while(diff >= 1.0e-4) {
                temp_corrected_conductivity_ratio += (input - salinity)/salinity_derivative(temp_corrected_conductivity_ratio, temp_diff);
                salinity = compute_salinity(temp_corrected_conductivity_ratio, temp_diff);
                diff = fabs(salinity - input);
            }
        }

        temp_correction = ref_conductivity_ratio(temp) * temp_corrected_conductivity_ratio * temp_corrected_conductivity_ratio;
        B_temp_corrected = B - temp_correction * A;

        conductivity_ratio = sqrt(fabs(B_temp_corrected * B_temp_corrected + 4.0 * A * temp_correction * (P + B))) - B_temp_corrected;
        salinity = 0.5 * conductivity_ratio / A;

        if(salinity <= 0.02) return(0.0);
        else return salinity;
    }
    else {  // Convert conductivity to salinity
        temp_corrected_conductivity_ratio = sqrt(fabs(input / (ref_conductivity_ratio(temp) * (1.0 + P / B + A * input))));
        salinity = compute_salinity(temp_corrected_conductivity_ratio, temp_diff);
        
        if (salinity <= 0.0005) return(0.0);
        else return(salinity);
    }
}

static double salinity_derivative(double conductivity_ratio, double temp) {
    return ((((13.5405 * conductivity_ratio - 28.1044) * conductivity_ratio + 42.2823) * conductivity_ratio + 50.7702) * conductivity_ratio - 0.1692) 
                + (temp/(1.0 + 0.0162 * temp)) * ((((-0.0720 * conductivity_ratio + 0.2544) * conductivity_ratio - 0.1125) * conductivity_ratio - 0.0132) * conductivity_ratio - 0.0056);
}

static double compute_salinity(double conductivity_ratio, double temp) {
    return ((((2.7081 * conductivity_ratio - 7.0261) * conductivity_ratio + 14.0941) * conductivity_ratio + 25.3851) * conductivity_ratio - 0.1692) * conductivity_ratio + 0.0080 + (temp/(1.0 + 0.0162 * temp)) 
                * (((((-0.0144 * conductivity_ratio + 0.0636) * conductivity_ratio - 0.0375) * conductivity_ratio - 0.0066) * conductivity_ratio - 0.0056) * conductivity_ratio + 0.0005);        
}

static double temp_coeff_a(double temp) {
    return -3.107e-3 * temp + 0.4215;
}

static double temp_coeff_b(double temp) {
    return (4.464e-4 * temp + 3.426e-2) * temp + 1.0;
}

double update_water_conductivity(const water_conductivity_input_t *input) {
    const double transition_depth_1 = 6.0;
    const double transition_depth_2 = 18.0;
    const double transition_depth_3 = 32.0;

    const double salinity = 23.1759;
    const double temp = 40.0;

    double pressure;
    double conductivity = 0.0;

    if (input == NULL) {
        write_to_sim_log("[sensor_water_conductivity] Failed to update water conductivity: input pointer is NULL.\n");
        return 0.0;
    }

    if (input->true_depth <= transition_depth_1) pressure = 5.0 * input->true_depth;
    else if (input->true_depth <= transition_depth_2) pressure = 4.0 * input->true_depth;
    else if (input->true_depth <= transition_depth_3) pressure = 2.0 * input->true_depth;

    conductivity = salinity_conductivity_conversion(salinity, temp, pressure, 1);
}