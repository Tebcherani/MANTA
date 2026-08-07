#ifndef ODYSSEY_DATA_STRUCT_H
#define ODYSSEY_DATA_STRUCT_H

int init_data_structure(void);

double read_sensor(int index);

void update_sensor(int index, double value);
void update_add(int index, double value);

#endif /* ODYSSEY_DATA_STRUCT_H */