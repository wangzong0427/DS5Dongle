#include <cassert>
#include <cstdint>
#include <cstring>

#include "config.h"
#include "gyro_aim.h"

namespace {

Config_body test_config{};

void reset_config() {
    std::memset(&test_config, 0, sizeof(test_config));
    test_config.gyro_aim_enabled = 1;
    test_config.gyro_aim_sens_x = 0.01f;
    test_config.gyro_aim_sens_y = 0.01f;
    test_config.gyro_aim_deadzone = 0;
    test_config.gyro_aim_smoothing = 0;
    test_config.gyro_aim_max_output = 127;
    test_config.gyro_aim_l2_threshold = 30;
    test_config.gyro_aim_invert_x = 0;
    test_config.gyro_aim_invert_y = 0;
}

void write_i16_le(uint8_t *report, const int offset, const int16_t value) {
    report[offset] = static_cast<uint8_t>(value & 0xff);
    report[offset + 1] = static_cast<uint8_t>((value >> 8) & 0xff);
}

void reset_report(uint8_t *report) {
    std::memset(report, 0, 63);
    report[2] = 128;
    report[3] = 128;
    report[4] = 255;
}

void reset_processor_state() {
    uint8_t report[63]{};
    reset_report(report);
    report[4] = 0;
    gyro_aim_process_report(report, sizeof(report));
}

void disabled_mode_leaves_report_unchanged() {
    reset_config();
    reset_processor_state();
    test_config.gyro_aim_enabled = 0;

    uint8_t report[63]{};
    reset_report(report);
    write_i16_le(report, 15, 1000);
    write_i16_le(report, 19, 1000);

    gyro_aim_process_report(report, sizeof(report));

    assert(report[2] == 128);
    assert(report[3] == 128);
}

void l2_gate_blocks_motion() {
    reset_config();
    reset_processor_state();

    uint8_t report[63]{};
    reset_report(report);
    report[4] = 29;
    write_i16_le(report, 15, 1000);
    write_i16_le(report, 19, 1000);

    gyro_aim_process_report(report, sizeof(report));

    assert(report[2] == 128);
    assert(report[3] == 128);
}

void gyro_motion_adds_to_right_stick() {
    reset_config();
    reset_processor_state();

    uint8_t report[63]{};
    reset_report(report);
    write_i16_le(report, 15, -500);
    write_i16_le(report, 19, 1000);

    gyro_aim_process_report(report, sizeof(report));

    assert(report[2] == 138);
    assert(report[3] == 123);
}

void deadzone_zeros_small_motion() {
    reset_config();
    reset_processor_state();
    test_config.gyro_aim_deadzone = 20;

    uint8_t report[63]{};
    reset_report(report);
    write_i16_le(report, 15, 19);
    write_i16_le(report, 19, -19);

    gyro_aim_process_report(report, sizeof(report));

    assert(report[2] == 128);
    assert(report[3] == 128);
}

void inversion_flips_output() {
    reset_config();
    reset_processor_state();
    test_config.gyro_aim_invert_x = 1;
    test_config.gyro_aim_invert_y = 1;

    uint8_t report[63]{};
    reset_report(report);
    write_i16_le(report, 15, -500);
    write_i16_le(report, 19, 1000);

    gyro_aim_process_report(report, sizeof(report));

    assert(report[2] == 118);
    assert(report[3] == 133);
}

void max_output_and_stick_range_are_clamped() {
    reset_config();
    reset_processor_state();
    test_config.gyro_aim_max_output = 20;

    uint8_t report[63]{};
    reset_report(report);
    report[2] = 250;
    report[3] = 5;
    write_i16_le(report, 15, -10000);
    write_i16_le(report, 19, 10000);

    gyro_aim_process_report(report, sizeof(report));

    assert(report[2] == 255);
    assert(report[3] == 0);
}

void smoothing_resets_when_l2_released() {
    reset_config();
    reset_processor_state();
    test_config.gyro_aim_smoothing = 50;

    uint8_t report[63]{};
    reset_report(report);
    write_i16_le(report, 19, 1000);

    gyro_aim_process_report(report, sizeof(report));
    assert(report[2] == 133);

    reset_report(report);
    report[4] = 0;
    gyro_aim_process_report(report, sizeof(report));
    assert(report[2] == 128);

    reset_report(report);
    write_i16_le(report, 19, 1000);
    gyro_aim_process_report(report, sizeof(report));
    assert(report[2] == 133);
}

}  // namespace

const Config_body& get_config() {
    return test_config;
}

int main() {
    disabled_mode_leaves_report_unchanged();
    l2_gate_blocks_motion();
    gyro_motion_adds_to_right_stick();
    deadzone_zeros_small_motion();
    inversion_flips_output();
    max_output_and_stick_range_are_clamped();
    smoothing_resets_when_l2_released();
    return 0;
}
