#ifndef MANTA_WHALE_ESTIMATOR_H
#define MANTA_WHALE_ESTIMATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include "whale_estimator_state.h"

/* Default tuning; initialization copies these into state.config. */
#define WHALE_VELOCITY_WINDOW_MAX_AGE_S 1800.0
#define WHALE_ACCELERATION_PSD 0.001
#define WHALE_VELOCITY_RETURN_TIME_S 1800.0
#define WHALE_NOMINAL_NORTH_VELOCITY 1.5
#define WHALE_NOMINAL_EAST_VELOCITY 0.0
#define WHALE_INITIAL_VELOCITY_VARIANCE 1.0

/* Initialize defaults and clear history, input, and output. */
void whale_estimator_init(whale_estimator_state_t *state);

/* Read state.received_fix; return and store whether the fix was accepted. */
int whale_estimator_update(whale_estimator_state_t *state);

/* Read state.prediction_time_s and write state.prediction. The filter stays at
 * its last accepted fix so a delayed fix can still be processed in order.
 */
int whale_estimator_predict(whale_estimator_state_t *state);

#ifdef __cplusplus
}
#endif

#endif /* MANTA_WHALE_ESTIMATOR_H */
