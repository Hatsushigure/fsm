#ifndef PID_HPP
#define PID_HPP

class PID
{
private:
    double m_proportionalGain;
    double m_integralGain;
    double m_derivativeGain;

    double lastError = 0.0;
    double integral = 0.0;

public:
    PID() = default;

    void setParameters(double proportionalGain, double integralGain, double derivativeGain)
    {
        this->m_proportionalGain = proportionalGain;
        this->m_integralGain = integralGain;
        this->m_derivativeGain = derivativeGain;
    }

    double calculate(double error)
    {
        auto derivative = error - lastError;
        integral += error;

        if (integral >= 3)   // magic number $\pm 3$, why
            integral = 3;
        else if (integral <= -3)
            integral = -3;

        auto vel =
            m_proportionalGain * error +
            m_integralGain * integral +
            m_derivativeGain * derivative;

        lastError = error;

        return vel;
    }

    void reset()
    {
        lastError = 0.0;
        integral = 0.0;
    }
};

#endif // PID_HPP