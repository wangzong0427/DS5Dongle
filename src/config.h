//
// Created by awalol on 2026/5/4.
//

#ifndef DS5_BRIDGE_CONFIG_H
#define DS5_BRIDGE_CONFIG_H

#include <cstdint>

struct __attribute__((packed)) Config_body {
    uint8_t config_version; // Config Version
    float haptics_gain; // [1.0,2.0]
    float speaker_volume; // [-100,0]
    uint8_t inactive_time; // [5,60] min
    uint8_t disable_inactive_disconnect; // bool: 0 disable,1 enable
    uint8_t disable_pico_led; // bool
    uint8_t polling_rate_mode; // 0: 250Hz, 1: 500Hz, 2: real-time
    uint8_t audio_buffer_length; // [16,128]
    uint8_t controller_mode; // 0: DS5, 1: DSE, 2: Auto
    uint8_t gyro_aim_enabled; // bool
    float gyro_aim_sens_x; // yaw -> right stick X
    float gyro_aim_sens_y; // pitch -> right stick Y
    uint8_t gyro_aim_deadzone; // gyro raw value deadzone
    uint8_t gyro_aim_smoothing; // [0,100]
    uint8_t gyro_aim_max_output; // [1,127]
    uint8_t gyro_aim_l2_threshold; // [0,255]
    uint8_t gyro_aim_invert_x; // bool
    uint8_t gyro_aim_invert_y; // bool
};

struct __attribute__((packed)) Config {
    uint32_t magic;
    uint16_t version;
    uint32_t crc32; // Config_body crc32, only calc and verify when save
    uint16_t size;  // Config_body size
    Config_body body;
};

void config_default();
void config_load();
bool config_save();
const Config_body& get_config();
void set_config(const uint8_t *new_config, const uint16_t len);
void config_valid();
void set_config(const Config_body &new_config);
extern bool is_dse;

#endif //DS5_BRIDGE_CONFIG_H
