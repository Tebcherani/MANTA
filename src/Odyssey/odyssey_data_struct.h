#ifndef ODYSSEY_DATA_STRUCT_H
#define ODYSSEY_DATA_STRUCT_H

int init_odyssey_data_structure(void);

int match_sensor(const char *);
double read_sensor(int);
void update_sensor(int, double);
void add_to_sensor(int, double);

#endif /* ODYSSEY_DATA_STRUCT_H */