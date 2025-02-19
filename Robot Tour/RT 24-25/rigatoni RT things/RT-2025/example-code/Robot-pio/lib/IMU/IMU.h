#ifndef IMU_H
#define IMU_H

#include "JY901.h"

class IMU
{
private:
    // Constants for filtering and conversion
    static constexpr double ANGLE_THRESHOLD = 40.0;
    static constexpr double JUMP_THRESHOLD = 1.5;
    static constexpr int MAX_FILTER_ATTEMPTS = 6;

    double prev_z_angle = 0;
    double z_angle = 0;
    double curr_z_angle = 0;

    // Add error state
    bool imu_error = false;

    // Add timeout handling
    unsigned long last_update = 0;
    static constexpr unsigned long TIMEOUT_MS = 10;

    unsigned long count = 0;

public:
    void Start()
    {
        JY901.StartIIC();
        // Add initialization check
        JY901.GetAngle();
    }

    void ResetAngle()
    {
        JY901.WriteWord(0x76, 0x00);
        prev_z_angle = 0;
        z_angle = 0;
        curr_z_angle = 0;
        count = 0;
    }

    void UpdateAngle()
    {
        int cnt = 0;
        double tmp_min = 99999;
        double tmp_max = 0;

        JY901.GetAngle();
        curr_z_angle = abs((double)JY901.stcAngle.Angle[2] / 32768.0 * 180);

        tmp_min = _min(curr_z_angle, tmp_min);
        tmp_max = _max(curr_z_angle, tmp_max);
        // Filter out sudden large jumps in angle readings
        unsigned long start_time = millis();

        while (prev_z_angle > ANGLE_THRESHOLD &&
               curr_z_angle - prev_z_angle > JUMP_THRESHOLD &&
               cnt < MAX_FILTER_ATTEMPTS &&
               (millis() - start_time) < TIMEOUT_MS)
        {
            JY901.GetAngle();
            curr_z_angle = abs((double)JY901.stcAngle.Angle[2] / 32768.0 * 180);
            tmp_min = _min(curr_z_angle, tmp_min);
            tmp_max = _max(curr_z_angle, tmp_max);
            cnt++;
        }
        // Check for timeout condition
        if ((millis() - start_time) >= TIMEOUT_MS)
        {
            imu_error = true;
            logger.error("IMU filter timeout for first filter");
        }

        if (cnt > 0)
        {
            Serial.println("Fix " + String(prev_z_angle, 3) + " " +
                           String(curr_z_angle, 3) + " " + String(tmp_min, 3) + " " +
                           String(tmp_max, 3) + " " + String(cnt));
            curr_z_angle = tmp_min;
        }

        // Reset start time for second filter
        start_time = millis();

        while (curr_z_angle > 0 &&
               prev_z_angle > 0 &&
               curr_z_angle < prev_z_angle &&
               (millis() - start_time) < TIMEOUT_MS)
        {
            JY901.GetAngle();
            curr_z_angle = abs((double)JY901.stcAngle.Angle[2] / 32768.0 * 180);
        }

        // Check for timeout condition
        if ((millis() - start_time) >= TIMEOUT_MS)
        {
            logger.error("IMU filter timeout for second filter, prev_z_angle: %D, curr_z_angle: %D", prev_z_angle, curr_z_angle);
            curr_z_angle = _max(curr_z_angle, prev_z_angle);
            imu_error = true;
        }

        count++;

        prev_z_angle = z_angle;
        z_angle = curr_z_angle;
    }

    double GetAngle(bool updateAngle = true)
    {
        if (updateAngle)
            UpdateAngle();
        return z_angle;
    }

    bool HasError() const { return imu_error; }

    unsigned long GetCount() const { return count; }
};

#endif
