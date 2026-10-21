"""Run with Python; requires a C compiler, without ROS or a running simulation."""
from pathlib import Path
import subprocess
import tempfile
import unittest


class WhaleVelocityReturnTest(unittest.TestCase):
    def test_estimator_prediction_and_updates(self):
        source = (Path(__file__).resolve().parents[1] /
                  'src/Odyssey/Vehicle/Behaviors/odyssey_behaviors_whale_trailing.c').read_text()
        # Compile the actual estimator functions independently of vehicle controls.
        estimator = source[source.index('/* Starting settings;'):source.index('/*\nbehavior: whale_trailing')]
        prediction = source[source.index('int get_whale_trailing_prediction('):source.index('int whale_trailing(')]
        checks = r'''
#include <assert.h>
int main(void) {
    const double tau = WHALE_VELOCITY_RETURN_TIME_S;
    whale_axis_filter_t axis = {100.0, 3.0, 1.0};
    assert(fabs(predict_whale_axis_position(&axis, 0.0, 1.5) - 100.0) < 1e-12);
    assert(fabs(predict_whale_axis_velocity(&axis, tau, 1.5) -
                (1.5 + 1.5 / exp(1.0))) < 1e-12);
    assert(fabs(predict_whale_axis_velocity(&axis, 30*tau, 1.5) - 1.5) < 1e-10);
    assert(fabs(predict_whale_axis_position(&axis, tau, 1.5) -
                (100.0 + 1.5*tau + 1.5*tau*(1-exp(-1.0)))) < 1e-9);
    whale_axis_filter_t east = {0.0, -2.0, 1.0};
    assert(fabs(predict_whale_axis_velocity(&east, 30*tau, 0.0)) < 1e-10);
    assert(fabs(predict_whale_axis_position(&east, 30*tau, 0.0) + 2*tau) < 1e-8);

    /* Exact nominal-motion fixes must preserve nominal velocity. */
    const double gaps[] = {1.1e-6, 0.001, 1.0, 17.999, 18.0, 60.0, 1800.0, 86400.0};
    for (unsigned i=0; i<sizeof(gaps)/sizeof(gaps[0]); ++i) {
        const double dt = gaps[i];
        whale_axis_filter_t a = {0.0, 1.5, 1.0};
        assert(update_whale_axis(&a, 1.5*dt, dt, 1.5));
        assert(a.position == 1.5*dt);
        assert(fabs(a.velocity - 1.5) < 1e-9);
        assert(isfinite(a.velocity_variance) && a.velocity_variance >= 0);
    }
    /* Measurements still correct an imperfect prior. */
    whale_axis_filter_t corrected = {0.0, 1.5, 1.0};
    assert(update_whale_axis(&corrected, 60.0, 60.0, 1.5));
    assert(corrected.position == 60.0 && corrected.velocity < 1.5);

    set_whale_trailing_target(0.0, 0.0, 10.0);
    whale_filter.north.velocity = 3.0;
    whale_filter.east.velocity = -2.0;
    double t, n1, e1, n2, e2;
    assert(get_whale_trailing_prediction(1810.0, &t, &n1, &e1));
    assert(get_whale_trailing_prediction(1810.0, &t, &n2, &e2));
    assert(n1 == n2 && e1 == e2 && t == 10.0);
    assert(whale_filter.north.velocity == 3.0);
    set_whale_trailing_target(99.0, 99.0, 10.0);
    set_whale_trailing_target(99.0, 99.0, 9.0);
    assert(whale_filter.north.position == 0.0);
    assert(!get_whale_trailing_prediction(9.0, &t, &n1, &e1));
    /* Irregularly spaced fixes recover the same constant velocity. */
    whale_filter = (whale_filter_t){0};
    const double times[] = {0.0, 10.0, 25.0, 40.0, 70.0, 100.0, 130.0};
    for (unsigned i = 0; i < sizeof(times)/sizeof(times[0]); ++i) {
        set_whale_trailing_target(2.0*times[i], -0.5*times[i], times[i]);
        if (i >= 2) {
            assert(fabs(whale_filter.north.velocity - 2.0) < 1e-10);
            assert(fabs(whale_filter.east.velocity + 0.5) < 1e-10);
        }
    }
    assert(whale_filter.recent_fix_count == 5);
    assert(whale_filter.recent_fixes[0].time == 25.0);
    /* A short-lived endpoint deviation has less influence than two-fix velocity. */
    set_whale_trailing_target(350.0, -80.0, 160.0);
    assert(fabs(whale_filter.north.velocity - 2.2) < 1e-10);
    assert(whale_filter.north.position == 350.0);
    assert(whale_filter.north.velocity_variance > 0.0);
    set_whale_trailing_target(999.0, 999.0, 160.0);
    assert(whale_filter.recent_fix_count == 5);
    assert(whale_filter.north.position == 350.0);
    /* Reacquisition drops stale fixes instead of fitting across a long outage. */
    set_whale_trailing_target(8000.0, -2000.0, 4000.0);
    assert(whale_filter.recent_fix_count == 1);
    set_whale_trailing_target(8020.0, -2005.0, 4010.0);
    set_whale_trailing_target(8040.0, -2010.0, 4020.0);
    assert(whale_filter.recent_fix_count == 3);
    assert(fabs(whale_filter.north.velocity - 2.0) < 1e-10);
    assert(fabs(whale_filter.east.velocity + 0.5) < 1e-10);
    return 0;
}
'''
        with tempfile.TemporaryDirectory() as directory:
            cfile = Path(directory) / 'test.c'
            binary = Path(directory) / 'test'
            cfile.write_text('#include <math.h>\n#include <stddef.h>\n' + estimator + prediction + checks)
            subprocess.run(['cc', '-std=c99', '-Wall', '-Wextra', '-Werror', str(cfile), '-lm', '-o', str(binary)], check=True)
            subprocess.run([str(binary)], check=True)


if __name__ == '__main__':
    unittest.main()
