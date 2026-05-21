#ifndef DS5_BRIDGE_GYRO_AIM_H
#define DS5_BRIDGE_GYRO_AIM_H

#include <cstdint>

void gyro_aim_process_report(uint8_t *report, uint16_t len);

#endif //DS5_BRIDGE_GYRO_AIM_H
