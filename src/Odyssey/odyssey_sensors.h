#ifndef ODYSSEY_SENSORS_H
#define ODYSSEY_SENSORS_H

int update_odyssey_sensors(void);
void update_acceleration(void);
void update_angular_rate(void);
void update_compass_yaw(void);
void update_heading(void);

#endif /* ODYSSEY_SENSORS_H */