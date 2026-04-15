/**
 * Author Mason Lopez
 *
 *
 * simple pid from curio res
 */

#include <stdint.h>
#include <math.h>

class ML_PID
{
public:
    // simple constructor
    ML_PID() : m_kp(1), m_kd(0), m_ki(0), m_umax(255), m_error_prev(0), m_error_integral(0) {}

    // set paramaets
    void Set_Params(float kp, float ki, float kd, float umaxIn)
    {
        m_kp = kp;
        m_kd = kd;
        m_ki = ki;

        m_umax = umaxIn;
        m_error_prev = 0;
        m_error_integral = 0;
    }

    float Compute_Signal(int currentVal, int targetVal, unsigned long deltaTimeUs)
    {
        float dtSec = deltaTimeUs / 1.0e6;
        
        // calculate the error
        float error = targetVal - currentVal;

        // derivative
        float dedt = (error-m_error_prev) / dtSec;

        // integral
        m_error_integral += error * dtSec;
        m_error_prev = error;

        // calc control signal
        float sig = m_kp*error + m_kd*dedt + m_ki*m_error_integral;

        if (sig > m_umax)
            sig = m_umax;
        if (sig < -1 * m_umax)
            sig = -1 * m_umax;

        return sig;
    }

    void Get_Params(float &kp, float &ki, float &kd)
    {
        kp = m_kp;
        ki = m_ki;
        kd = m_kd;
    }

private:
    float m_kp, m_kd, m_ki, m_umax;

    float m_error_prev, m_error_integral;
};
