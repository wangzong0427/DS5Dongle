#include "gyro_aim.h"

#include <algorithm>
#include <cmath>
#include <cstdint>

#include "config.h"

namespace {

constexpr uint8_t RIGHT_STICK_X_OFFSET = 2;
constexpr uint8_t RIGHT_STICK_Y_OFFSET = 3;
constexpr uint8_t L2_OFFSET = 4;
constexpr uint8_t GYRO_PITCH_OFFSET = 15;
constexpr uint8_t GYRO_YAW_OFFSET = 19;
constexpr uint16_t MIN_REPORT_LEN = GYRO_YAW_OFFSET + sizeof(int16_t);

float filtered_x = 0.0f;
float filtered_y = 0.0f;

int16_t read_i16_le(const uint8_t *data) {
    return static_cast<int16_t>(static_cast<uint16_t>(data[0]) | (static_cast<uint16_t>(data[1]) << 8));
}

int apply_deadzone(const int value, const uint8_t deadzone) {
    return std::abs(value) <= deadzone ? 0 : value;
}

float smooth_axis(const float previous, const float current, const uint8_t smoothing) {
    if (smoothing == 0) {
        return current;
    }
    const float keep = static_cast<float>(smoothing) / 100.0f;
    return previous * keep + current * (1.0f - keep);
}

void reset_filter() {
    filtered_x = 0.0f;
    filtered_y = 0.0f;
}

} // namespace

void gyro_aim_process_report(uint8_t *report, const uint16_t len) {
    if (report == nullptr || len < MIN_REPORT_LEN) {
        reset_filter();
        return;
    }

    const auto &config = get_config();
    if (!config.gyro_aim_enabled || report[L2_OFFSET] < config.gyro_aim_l2_threshold) {
        reset_filter();
        return;
    }

    const int yaw = apply_deadzone(read_i16_le(report + GYRO_YAW_OFFSET), config.gyro_aim_deadzone);
    const int pitch = apply_deadzone(read_i16_le(report + GYRO_PITCH_OFFSET), config.gyro_aim_deadzone);

    float offset_x = static_cast<float>(yaw) * config.gyro_aim_sens_x;
    float offset_y = static_cast<float>(pitch) * config.gyro_aim_sens_y;

    if (config.gyro_aim_invert_x) {
        offset_x = -offset_x;
    }
    if (config.gyro_aim_invert_y) {
        offset_y = -offset_y;
    }

    filtered_x = smooth_axis(filtered_x, offset_x, config.gyro_aim_smoothing);
    filtered_y = smooth_axis(filtered_y, offset_y, config.gyro_aim_smoothing);

    const int max_output = static_cast<int>(config.gyro_aim_max_output);
    const int stick_offset_x = std::clamp(static_cast<int>(std::lround(filtered_x)), -max_output, max_output);
    const int stick_offset_y = std::clamp(static_cast<int>(std::lround(filtered_y)), -max_output, max_output);

    report[RIGHT_STICK_X_OFFSET] = static_cast<uint8_t>(std::clamp(report[RIGHT_STICK_X_OFFSET] + stick_offset_x, 0, 255));
    report[RIGHT_STICK_Y_OFFSET] = static_cast<uint8_t>(std::clamp(report[RIGHT_STICK_Y_OFFSET] + stick_offset_y, 0, 255));
}
