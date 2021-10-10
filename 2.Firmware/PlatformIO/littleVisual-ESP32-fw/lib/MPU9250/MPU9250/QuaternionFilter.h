#pragma once
#ifndef QUATERNIONFILTER_H
#define QUATERNIONFILTER_H

enum class QuatFilterSel {
    NONE,
    MADGWICK,
    MAHONY,
};

class QuaternionFilter {
    // for madgwick
    float GyroMeasError = PI * (40.0f / 180.0f);     // gyroscope measurement error in rads/s (start at 40 deg/s)
    float GyroMeasDrift = PI * (0.0f / 180.0f);      // gyroscope measurement drift in rad/s/s (start at 0.0 deg/s/s)
    float beta = sqrt(3.0f / 4.0f) * GyroMeasError;  // compute beta
    float zeta = sqrt(3.0f / 4.0f) * GyroMeasDrift;  // compute zeta, the other free parameter in the Madgwick scheme usually set to a small or zero value

    // for mahony
    float Kp = 30.0;
    float Ki = 0.0;

    QuatFilterSel filter_sel{QuatFilterSel::MADGWICK};
    double deltaT{0.};
    uint32_t newTime{0}, oldTime{0};

public:
    void select_filter(QuatFilterSel sel) {
        filter_sel = sel;
    }

    void update(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q) {
        newTime = micros();
        deltaT = newTime - oldTime;
        oldTime = newTime;
        deltaT = fabs(deltaT * 0.001 * 0.001);

        switch (filter_sel) {
            case QuatFilterSel::MADGWICK:
                madgwick(ax, ay, az, gx, gy, gz, mx, my, mz, q);
                break;
            case QuatFilterSel::MAHONY:
                mahony(ax, ay, az, gx, gy, gz, mx, my, mz, q);
                break;
            default:
                no_filter(ax, ay, az, gx, gy, gz, mx, my, mz, q);
                break;
        }
    }

    void no_filter(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q) {
        float q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];  // variable for readability
        q[0] += 0.5f * (-q1 * gx - q2 * gy - q3 * gz) * deltaT;
        q[1] += 0.5f * (q0 * gx + q2 * gz - q3 * gy) * deltaT;
        q[2] += 0.5f * (q0 * gy - q1 * gz + q3 * gx) * deltaT;
        q[3] += 0.5f * (q0 * gz + q1 * gy - q2 * gx) * deltaT;
        float recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        q[0] *= recipNorm;
        q[1] *= recipNorm;
        q[2] *= recipNorm;
        q[3] *= recipNorm;
    }

    // Madgwick Quaternion Update
    void madgwick(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q) {
        double q0 = q[0], q1 = q[1], q2 = q[2], q3 = q[3];  // short name local variable for readability
        double recipNorm;
        double s0, s1, s2, s3;
        double qDot1, qDot2, qDot3, qDot4;
        double hx, hy;
        double _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

        // Rate of change of quaternion from gyroscope
        qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
        qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
        qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
        qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

        // Normalise accelerometer measurement
        double a_norm = ax * ax + ay * ay + az * az;
        if (a_norm == 0.) return;  // handle NaN
        recipNorm = 1.0 / sqrt(a_norm);
        ax *= recipNorm;
        ay *= recipNorm;
        az *= recipNorm;

        // Normalise magnetometer measurement
        double m_norm = mx * mx + my * my + mz * mz;
        if (m_norm == 0.) return;  // handle NaN
        recipNorm = 1.0 / sqrt(m_norm);
        mx *= recipNorm;
        my *= recipNorm;
        mz *= recipNorm;

        // Auxiliary variables to avoid repeated arithmetic
        _2q0mx = 2.0f * q0 * mx;
        _2q0my = 2.0f * q0 * my;
        _2q0mz = 2.0f * q0 * mz;
        _2q1mx = 2.0f * q1 * mx;
        _2q0 = 2.0f * q0;
        _2q1 = 2.0f * q1;
        _2q2 = 2.0f * q2;
        _2q3 = 2.0f * q3;
        _2q0q2 = 2.0f * q0 * q2;
        _2q2q3 = 2.0f * q2 * q3;
        q0q0 = q0 * q0;
        q0q1 = q0 * q1;
        q0q2 = q0 * q2;
        q0q3 = q0 * q3;
        q1q1 = q1 * q1;
        q1q2 = q1 * q2;
        q1q3 = q1 * q3;
        q2q2 = q2 * q2;
        q2q3 = q2 * q3;
        q3q3 = q3 * q3;

        // Reference direction of Earth's magnetic field
        hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
        hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
        _2bx = sqrt(hx * hx + hy * hy);
        _2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
        _4bx = 2.0f * _2bx;
        _4bz = 2.0f * _2bz;

        // Gradient decent algorithm corrective step
        s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
        recipNorm = 1.0 / sqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3);  // normalise step magnitude
        s0 *= recipNorm;
        s1 *= recipNorm;
        s2 *= recipNorm;
        s3 *= recipNorm;

        // Apply feedback step
        qDot1 -= beta * s0;
        qDot2 -= beta * s1;
        qDot3 -= beta * s2;
        qDot4 -= beta * s3;

        // Integrate rate of change of quaternion to yield quaternion
        q0 += qDot1 * deltaT;
        q1 += qDot2 * deltaT;
        q2 += qDot3 * deltaT;
        q3 += qDot4 * deltaT;

        // Normalise quaternion
        recipNorm = 1.0 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
        q0 *= recipNorm;
        q1 *= recipNorm;
        q2 *= recipNorm;
        q3 *= recipNorm;

        q[0] = q0;
        q[1] = q1;
        q[2] = q2;
        q[3] = q3;
    }

    // Mahony accelleration filter
    // Mahony scheme uses proportional and integral filtering on
    // the error between estimated reference vector (gravity) and measured one.
    // Madgwick's implementation of Mayhony's AHRS algorithm.
    // See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
    // Free parameters in the Mahony filter and fusion scheme,
    // Kp for proportional feedback, Ki for integral
    // float Kp = 30.0;
    // float Ki = 0.0;
    // with MPU-9250, angles start oscillating at Kp=40. Ki does not seem to help and is not required.
    // with MPU-6050, some instability observed at Kp=100 Now set to 30.
    void mahony(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz, float* q) {
        float recipNorm;
        float vx, vy, vz;
        float ex, ey, ez;  //error terms
        float qa, qb, qc;
        static float ix = 0.0, iy = 0.0, iz = 0.0;  //integral feedback terms
        float tmp;

        // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
        tmp = ax * ax + ay * ay + az * az;
        if (tmp > 0.0) {
            // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
            recipNorm = 1.0 / sqrt(tmp);
            ax *= recipNorm;
            ay *= recipNorm;
            az *= recipNorm;

            // Estimated direction of gravity in the body frame (factor of two divided out)
            vx = q[1] * q[3] - q[0] * q[2];
            vy = q[0] * q[1] + q[2] * q[3];
            vz = q[0] * q[0] - 0.5f + q[3] * q[3];

            // Error is cross product between estimated and measured direction of gravity in body frame
            // (half the actual magnitude)
            ex = (ay * vz - az * vy);
            ey = (az * vx - ax * vz);
            ez = (ax * vy - ay * vx);

            // Compute and apply to gyro term the integral feedback, if enabled
            if (Ki > 0.0f) {
                ix += Ki * ex * deltaT;  // integral error scaled by Ki
                iy += Ki * ey * deltaT;
                iz += Ki * ez * deltaT;
                gx += ix;  // apply integral feedback
                gy += iy;
                gz += iz;
            }

            // Apply proportional feedback to gyro term
            gx += Kp * ex;
            gy += Kp * ey;
            gz += Kp * ez;
        }

        // Integrate rate of change of quaternion, q cross gyro term
        deltaT = 0.5 * deltaT;
        gx *= deltaT;  // pre-multiply common factors
        gy *= deltaT;
        gz *= deltaT;
        qa = q[0];
        qb = q[1];
        qc = q[2];
        q[0] += (-qb * gx - qc * gy - q[3] * gz);
        q[1] += (qa * gx + qc * gz - q[3] * gy);
        q[2] += (qa * gy - qb * gz + q[3] * gx);
        q[3] += (qa * gz + qb * gy - qc * gx);

        // renormalise quaternion
        recipNorm = 1.0 / sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
        q[0] = q[0] * recipNorm;
        q[1] = q[1] * recipNorm;
        q[2] = q[2] * recipNorm;
        q[3] = q[3] * recipNorm;
    }
};

#endif  // QUATERNIONFILTER_H
