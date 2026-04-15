// #include <Arduino.h>
#include "ml_pid.hpp"
#include "unity.h"

void setUp(void) {
    // set stuff up here
}

void tearDown(void) {
    // clean stuff up here
}

void test_pid_no_error(void)
{
    ML_PID pid;

    pid.Set_Params(1.0f, 0.5f, 0.2f, 100.0f);

    // Scenario 1: Current value equals target value.
    int currentVal = 100;
    int targetVal = 100;
    unsigned long deltaTimeUs = 1000;
    float signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0, signal);

    // Scenario 2: Small delay time, still zero error.
    deltaTimeUs = 2000;
    signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0, signal);

    // Scenario 3: Non-zero kp and kd, but zero error.
    pid.Set_Params(2.0f, 1.0f, 0.5f, 100.0f);
    signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0, signal);

    // Scenario 4: Floating-point precision error.
    currentVal = 100.0001;
    targetVal = 100.0001;
    signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0, signal);

    // Scenario 5: Zero error, but large umax value.
    pid.Set_Params(1.0f, 0.5f, 0.2f, 1000.0f);
    signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0, signal);
}

void test_pid_positive_error(void) {
    ML_PID pid;
    pid.Set_Params(1.0f, 0.5f, 0.2f, 100.0f);

    // Scenario 1: Small positive error.
    int currentVal = 50;
    int targetVal = 100;
    unsigned long deltaTimeUs = 1000;
    float signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    TEST_ASSERT_GREATER_THAN(0, signal);

    // Scenario 2: Large positive error.
    currentVal = 0;
    targetVal = 100;
    signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    TEST_ASSERT_GREATER_THAN(0, signal);

    // Scenario 3: Small error with large `kp`.
    pid.Set_Params(10.0f, 0.5f, 0.2f, 100.0f);
    currentVal = 95;
    targetVal = 100;
    signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    TEST_ASSERT_GREATER_THAN(0, signal);

    // Scenario 4: Positive error with non-zero `kd`.
    pid.Set_Params(1.0f, 1.0f, 0.2f, 100.0f);
    currentVal = 50;
    targetVal = 100;
    signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    TEST_ASSERT_GREATER_THAN(0, signal);

    // Scenario 5: Positive error with small `ki`.
    pid.Set_Params(1.0f, 0.5f, 0.01f, 100.0f);
    signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    TEST_ASSERT_GREATER_THAN(0, signal);
}

void test_pid_negative_error(void) {
    ML_PID pid;
    pid.Set_Params(1.0f, 0.5f, 0.2f, 100.0f);

    // Scenario 1: Small negative error.
    int currentVal = 150;
    int targetVal = 100;
    unsigned long deltaTimeUs = 1000;
    float signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    TEST_ASSERT_LESS_THAN(0, signal);

    // Scenario 2: Large negative error.
    currentVal = 200;
    targetVal = 100;
    signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    TEST_ASSERT_LESS_THAN(0, signal);

    // Scenario 3: Small error with high `kp`.
    pid.Set_Params(10.0f, 0.5f, 0.2f, 100.0f);
    currentVal = 110;
    targetVal = 100;
    signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    TEST_ASSERT_LESS_THAN(0, signal);

    // Scenario 4: Negative error with non-zero `kd`.
    pid.Set_Params(1.0f, 1.0f, 0.2f, 100.0f);
    currentVal = 150;
    targetVal = 100;
    signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    TEST_ASSERT_LESS_THAN(0, signal);

    // Scenario 5: Negative error with small `ki`.
    pid.Set_Params(1.0f, 0.5f, 0.01f, 100.0f);
    signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    TEST_ASSERT_LESS_THAN(0, signal);
}

void test_pid_computesignal(void) {
    ML_PID pid;

    // Scenario 1: Small error, no derivative or integral contribution.
    pid.Set_Params(1.0f, 0.0f, 0.0f, 100.0f); // kp=1, kd=0, ki=0, umax=100
    int currentVal = 50;
    int targetVal = 60;
    unsigned long deltaTimeUs = 1000;  // 1000 microseconds (1 ms)

    float signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    float expected_signal = 1.0f * (60 - 50);  // Signal = Kp * Error = 1 * 10 = 10
    TEST_ASSERT_FLOAT_WITHIN(0.01, expected_signal, signal);

    // Scenario 2: Large error, no derivative or integral contribution.
    pid.Set_Params(1.0f, 0.0f, 0.0f, 100.0f);
    currentVal = 0;
    targetVal = 100;
    signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    expected_signal = 1.0f * (100 - 0);  // Signal = 1 * 100 = 100
    TEST_ASSERT_FLOAT_WITHIN(0.01, expected_signal, signal);

    // Scenario 3: Derivative term influence.
    pid.Set_Params(1.0f, 0.5f, 0.0f, 6000.0f);  // kp=1, kd=0.5, ki=0, umax=100
    currentVal = 50;
    targetVal = 70;  // Target increases by 20
    unsigned long deltaTimeUs_2 = 2000;  // 2 ms passed
    signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs_2);
    float derivative = 0.5f * (70 - 50) / (2000.0f / 1000000.0f); // Derivative part
    expected_signal = 1.0f * (70 - 50) + derivative;  // Signal = Kp * Error + Kd * Derivative = 1 * 20 + 0.5 * 20 / (2 ms) = 20 + 0.5 * 10000 = 20 + 5000 = 5020
    TEST_ASSERT_FLOAT_WITHIN(0.01, expected_signal, signal);

    // Scenario 4: Integral term influence.
    pid.Set_Params(1.0f, 0.0f, 0.1f, 500.0f);  // kp=1, kd=0, ki=0.1, umax=100
    currentVal = 50;
    targetVal = 100;  // Error persists
    unsigned long deltaTimeUs_3 = 1000;  // 1 ms passed
    pid.Compute_Signal(currentVal, targetVal, deltaTimeUs_3);  // First computation
    signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs_3);  // Second computation (integral accumulates)
    expected_signal = 1.0f * (100 - 50) + 0.1f * (100 * deltaTimeUs_3 / 1000000.0f);  // Integral accumulates over time
    TEST_ASSERT_FLOAT_WITHIN(0.01, expected_signal, signal);

    // Scenario 5: Signal saturation (umax limit).
    pid.Set_Params(1.0f, 0.5f, 0.0f, 50.0f);  // kp=1, kd=0.5, ki=0, umax=50
    currentVal = 0;
    targetVal = 100;  // Large error
    deltaTimeUs = 1000;  // 1 ms
    signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    expected_signal = 1.0f * (100 - 0) + 0.5f * (100 - 0) / (1000.0f / 1000000.0f); // Proportional + Derivative
    if (expected_signal > 50) {
        expected_signal = 50;  // Signal should be capped at umax
    }
    TEST_ASSERT_FLOAT_WITHIN(0.01, expected_signal, signal);
}

void test_pid_p(void) {
    ML_PID pid;

    // Set parameters for the Proportional (P) test
    pid.Set_Params(2.0f, 0.0f, 0.0f, 500.0f);  // kp=2, kd=0, ki=0, umax=100

    // Test Case 1: Proportional term with a small error.
    int currentVal = 50;
    int targetVal = 60;  // Error = 60 - 50 = 10
    unsigned long deltaTimeUs = 1000;  // 1 ms passed
    int16_t signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    int16_t expected_signal = 2.0f * (targetVal - currentVal);  // Expected: 2 * 10 = 20
    TEST_ASSERT_FLOAT_WITHIN(0.01, expected_signal, signal);

    // Test Case 2: Proportional term with a larger error.
    currentVal = 0;
    targetVal = 100;  // Error = 100 - 0 = 100
    signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    expected_signal = 2.0f * (targetVal - currentVal);  // Expected: 2 * 100 = 200
    TEST_ASSERT_FLOAT_WITHIN(0.01, expected_signal, signal);
}

void test_pid_i(void) {
    ML_PID pid;

    // Set parameters for the Integral (I) test
    pid.Set_Params(0.0f, 0.0f, 0.1f, 100.0f);  // kp=0, kd=0, ki=0.1, umax=100

    // Test Case 1: Integral term with a constant error over multiple calls.
    int currentVal = 50;
    int targetVal = 100;  // Error = 100 - 50 = 50
    unsigned long deltaTimeUs = 1000;  // 1 ms passed

    // First computation
    float signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    float expected_signal_1 = 0.1f * 50 * (0.001);  // Expected: 0.1 * 50 = 5
    TEST_ASSERT_FLOAT_WITHIN(0.1, expected_signal_1, signal);

    // Second computation (integral accumulates)
    signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    float expected_signal_2 = (0.1f * 100) * (0.001);  // Expected: 5 + 5 = 10
    TEST_ASSERT_FLOAT_WITHIN(0.01, expected_signal_2, signal);

    // Third computation (integral continues accumulating)
    signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    float expected_signal_3 = (0.1f * 150) * (0.001);  // Expected: 10 + 5 = 15
    TEST_ASSERT_FLOAT_WITHIN(0.01, expected_signal_3, signal);
}

void test_pid_d(void) {
    ML_PID pid;

    // Set parameters for the Derivative (D) test
    pid.Set_Params(0.0f, 0.5f, 0.0f, 10000.0f);  // kp=0, kd=0.5, ki=0, umax=100

    // Test Case 1: Derivative term with a change in error.
    int currentVal = 50;
    int targetVal = 60;  // Error = 60 - 50 = 10
    unsigned long deltaTimeUs = 1000;  // 1 ms passed

    // First computation
    float signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    // There is no previous error, so the derivative term is 0
    float expected_signal = 10 * 1000 * 0.5;
    TEST_ASSERT_FLOAT_WITHIN(0.01, expected_signal, signal);

    // Test Case 2: Derivative term with a change in error (next computation).
    currentVal = 60;
    targetVal = 80;  // Error = 80 - 60 = 20

    signal = pid.Compute_Signal(currentVal, targetVal, deltaTimeUs);
    // The derivative term should respond to the change in error: (20 - 10) / (1 ms)
    float derivative = 0.5f * (20 - 10) / (1000.0f / 1000000.0f);  // 0.5 * 10 / (1 ms) = 5000
    expected_signal = derivative;  // Since Kp and Ki are both 0, the expected signal is just the derivative

    TEST_ASSERT_EQUAL_INT16(expected_signal, signal);
}



int main(void) {
    UNITY_BEGIN();
    RUN_TEST(test_pid_no_error);
    RUN_TEST(test_pid_positive_error);
    RUN_TEST(test_pid_negative_error);
    RUN_TEST(test_pid_computesignal);
    RUN_TEST(test_pid_p);
    RUN_TEST(test_pid_i);
    RUN_TEST(test_pid_d);
    return UNITY_END();
}