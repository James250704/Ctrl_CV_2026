// ============================================================================
// test_kinematics.cpp — Mecanum 運動學 + PID + RampedTarget 單元測試
//
// 測試邏輯與 move/main.cpp 中的 Kinematics/PidController/RampedTarget 一致，
// 但直接在 test 中重新實作（避免 include 帶入硬體依賴）。
// ============================================================================

#include <unity.h>
#include <cmath>
#include <cstring>
#include <cstdint>

// ---- Copy constants from signal_shared.h ----
namespace RobotConfig {
constexpr float MAX_RPS       = 7.0f;
constexpr float PID_OUT_LIMIT = 30.0f;
constexpr float MOTOR_MAX_DUTY = 40000U;
}

// ---- Copy Kinematics from move/main.cpp ----
namespace Kinematics {
constexpr float HALF_LENGTH  = 0.25f;
constexpr float HALF_WIDTH   = 0.25f;
constexpr float WHEEL_RADIUS = 0.05f;
constexpr float L_PLUS_W     = HALF_LENGTH + HALF_WIDTH;
constexpr float INV_R        = 1.0f / WHEEL_RADIUS;
constexpr float kFF          = 7.0f;
constexpr float ACCEL_LIMIT  = 20.0f;

inline void computeWheelTargets(float vx, float vy, float omega,
                                float& fl, float& fr, float& rl, float& rr) {
    float lwOmega = L_PLUS_W * omega;
    fl = (vx - vy - lwOmega) * INV_R;
    fr = (vx + vy + lwOmega) * INV_R;
    rl = (vx + vy - lwOmega) * INV_R;
    rr = (vx - vy + lwOmega) * INV_R;
}

inline float feedforward(float targetRps) { return targetRps / kFF; }
} // namespace Kinematics

// ---- Copy PidController from move/main.cpp ----
class PidController {
public:
    float kp = 1.5f, ki = 0.0f, kd = 0.0f;
    float outLimit = RobotConfig::PID_OUT_LIMIT;
    float integralLimit = 5.0f;

    void reset() {
        integral_ = 0.0f;
        prevError_ = 0.0f;
        firstRun_ = true;
    }

    float update(float setpoint, float measurement, float dt) {
        float error = setpoint - measurement;
        if (firstRun_) { prevError_ = error; firstRun_ = false; }

        float pTerm = kp * error;
        integral_ += ki * error * dt;
        if (integral_ > integralLimit) integral_ = integralLimit;
        if (integral_ < -integralLimit) integral_ = -integralLimit;
        float dTerm = kd * (error - prevError_) / dt;
        prevError_ = error;

        float pidOut = pTerm + integral_ + dTerm;
        float ffOut = Kinematics::feedforward(setpoint);
        float output = pidOut + ffOut;

        if (output > outLimit) output = outLimit;
        if (output < -outLimit) output = -outLimit;
        return output;
    }

private:
    float integral_ = 0.0f;
    float prevError_ = 0.0f;
    bool firstRun_ = true;
};

// ---- Copy RampedTarget from move/main.cpp ----
class RampedTarget {
public:
    explicit RampedTarget(float maxDeltaPerSec) : maxDelta_(maxDeltaPerSec) {}
    void reset(float value = 0.0f) { previous_ = value; }
    float update(float target, float dt) {
        float maxStep = maxDelta_ * dt;
        float diff = target - previous_;
        if (diff > maxStep) diff = maxStep;
        if (diff < -maxStep) diff = -maxStep;
        previous_ += diff;
        return previous_;
    }
private:
    float maxDelta_;
    float previous_ = 0.0f;
};

// ---- Helper ----
#define ASSERT_FLOAT_NEAR(a, b, eps) \
    TEST_ASSERT_TRUE_MESSAGE(std::fabs((a) - (b)) < (eps), \
        "Float values not within tolerance")

// ============================================================================
// Mecanum Kinematics Tests
// ============================================================================

void test_mecanum_forward(void) {
    // Pure forward: vx=1, vy=0, omega=0 → all wheels same speed
    float fl, fr, rl, rr;
    Kinematics::computeWheelTargets(1.0f, 0.0f, 0.0f, fl, fr, rl, rr);

    // All should be vx / WHEEL_RADIUS = 1.0 / 0.05 = 20.0
    ASSERT_FLOAT_NEAR(fl, 20.0f, 0.01f);
    ASSERT_FLOAT_NEAR(fr, 20.0f, 0.01f);
    ASSERT_FLOAT_NEAR(rl, 20.0f, 0.01f);
    ASSERT_FLOAT_NEAR(rr, 20.0f, 0.01f);
}

void test_mecanum_strafe_right(void) {
    // Pure strafe right: vx=0, vy=1, omega=0
    float fl, fr, rl, rr;
    Kinematics::computeWheelTargets(0.0f, 1.0f, 0.0f, fl, fr, rl, rr);

    // fl = -vy/R = -20, fr = vy/R = 20, rl = vy/R = 20, rr = -vy/R = -20
    ASSERT_FLOAT_NEAR(fl, -20.0f, 0.01f);
    ASSERT_FLOAT_NEAR(fr,  20.0f, 0.01f);
    ASSERT_FLOAT_NEAR(rl,  20.0f, 0.01f);
    ASSERT_FLOAT_NEAR(rr, -20.0f, 0.01f);
}

void test_mecanum_rotate(void) {
    // Pure rotation: vx=0, vy=0, omega=1
    float fl, fr, rl, rr;
    Kinematics::computeWheelTargets(0.0f, 0.0f, 1.0f, fl, fr, rl, rr);

    // fl = -(L+W)*omega/R = -10, fr = (L+W)*omega/R = 10
    // rl = -(L+W)*omega/R = -10, rr = (L+W)*omega/R = 10
    ASSERT_FLOAT_NEAR(fl, -10.0f, 0.01f);
    ASSERT_FLOAT_NEAR(fr,  10.0f, 0.01f);
    ASSERT_FLOAT_NEAR(rl, -10.0f, 0.01f);
    ASSERT_FLOAT_NEAR(rr,  10.0f, 0.01f);
}

void test_mecanum_zero_input(void) {
    float fl, fr, rl, rr;
    Kinematics::computeWheelTargets(0.0f, 0.0f, 0.0f, fl, fr, rl, rr);
    ASSERT_FLOAT_NEAR(fl, 0.0f, 0.001f);
    ASSERT_FLOAT_NEAR(fr, 0.0f, 0.001f);
    ASSERT_FLOAT_NEAR(rl, 0.0f, 0.001f);
    ASSERT_FLOAT_NEAR(rr, 0.0f, 0.001f);
}

// ============================================================================
// PID Controller Tests
// ============================================================================

void test_pid_proportional_only(void) {
    PidController pid;
    pid.kp = 2.0f;
    pid.ki = 0.0f;
    pid.kd = 0.0f;
    pid.outLimit = 100.0f;

    float dt = 0.01f;

    // setpoint=10, measurement=0 → error=10, pTerm=20, ffOut=10/7≈1.43
    float output = pid.update(10.0f, 0.0f, dt);
    float expected = 2.0f * 10.0f + Kinematics::feedforward(10.0f);
    ASSERT_FLOAT_NEAR(output, expected, 0.01f);
}

void test_pid_output_limit(void) {
    PidController pid;
    pid.kp = 100.0f;  // Very high gain to trigger limit
    pid.ki = 0.0f;
    pid.kd = 0.0f;
    pid.outLimit = 30.0f;

    float output = pid.update(100.0f, 0.0f, 0.01f);
    TEST_ASSERT_TRUE(output <= 30.0f);
}

void test_pid_integral_windup(void) {
    PidController pid;
    pid.kp = 0.0f;
    pid.ki = 10.0f;
    pid.kd = 0.0f;
    pid.integralLimit = 5.0f;
    pid.outLimit = 100.0f;

    float dt = 0.01f;
    // Run many iterations to build up integral
    for (int i = 0; i < 1000; i++) {
        pid.update(10.0f, 0.0f, dt);
    }
    // Integral should be clamped at 5.0
    float output = pid.update(10.0f, 0.0f, dt);
    // The output includes ffOut = 10/7 ≈ 1.43, plus integral (clamped at 5.0)
    // But integral was already at 5.0 after first 500 iterations
    // So output = 0 + 5.0 + 0 + ffOut = 6.43
    TEST_ASSERT_TRUE(output <= 5.0f + Kinematics::feedforward(10.0f) + 0.1f);
}

void test_pid_reset(void) {
    PidController pid;
    pid.ki = 1.0f;
    pid.outLimit = 100.0f;

    // Build up integral
    for (int i = 0; i < 100; i++) {
        pid.update(10.0f, 0.0f, 0.01f);
    }

    // Reset
    pid.reset();

    // After reset, integral should be 0
    float output = pid.update(0.0f, 0.0f, 0.01f);
    // output = kp*0 + 0 + kd*0 + ffOut(0) = 0
    ASSERT_FLOAT_NEAR(output, 0.0f, 0.001f);
}

// ============================================================================
// RampedTarget Tests
// ============================================================================

void test_ramped_target_step_response(void) {
    RampedTarget ramp(10.0f);  // max 10 units/sec
    float dt = 0.1f;           // 100ms

    // First update: target=100, ramp can move at most 10*0.1 = 1.0
    float result = ramp.update(100.0f, dt);
    ASSERT_FLOAT_NEAR(result, 1.0f, 0.001f);

    // Second update: still ramping
    result = ramp.update(100.0f, dt);
    ASSERT_FLOAT_NEAR(result, 2.0f, 0.001f);
}

void test_ramped_target_no_overshoot(void) {
    RampedTarget ramp(10.0f);
    float dt = 0.1f;

    // Target = 5.0, ramp rate = 10/s → takes 0.5s to reach
    for (int i = 0; i < 100; i++) {
        float result = ramp.update(5.0f, dt);
        TEST_ASSERT_TRUE(result <= 5.0f + 0.001f);
    }
    // After 100 iterations (10s), should be at 5.0
    ASSERT_FLOAT_NEAR(ramp.update(5.0f, dt), 5.0f, 0.001f);
}

void test_ramped_target_symmetry(void) {
    RampedTarget ramp(10.0f);
    float dt = 0.1f;

    // Positive ramp
    ramp.reset(0.0f);
    float pos = ramp.update(100.0f, dt);

    // Negative ramp
    ramp.reset(0.0f);
    float neg = ramp.update(-100.0f, dt);

    ASSERT_FLOAT_NEAR(pos, -neg, 0.001f);
}

// ============================================================================
// Main
// ============================================================================

int main(void) {
    UNITY_BEGIN();

    // Kinematics
    RUN_TEST(test_mecanum_forward);
    RUN_TEST(test_mecanum_strafe_right);
    RUN_TEST(test_mecanum_rotate);
    RUN_TEST(test_mecanum_zero_input);

    // PID
    RUN_TEST(test_pid_proportional_only);
    RUN_TEST(test_pid_output_limit);
    RUN_TEST(test_pid_integral_windup);
    RUN_TEST(test_pid_reset);

    // RampedTarget
    RUN_TEST(test_ramped_target_step_response);
    RUN_TEST(test_ramped_target_no_overshoot);
    RUN_TEST(test_ramped_target_symmetry);

    return UNITY_END();
}
