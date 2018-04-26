#if !defined(PID_H)
#define PID_H

/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include <stdint.h>
#include <vector>

/**************************************************************************************************
 *  CLASS DEFINITION
 *************************************************************************************************/
class PID
{
public:

    enum StateMachine
    {
        STATE_INVALID,
        STATE_FIRST,
        STATE_SECOND,
        STATE_THIRD,
        TOTAL_NUM_STATES
    };

    enum PidControl
    {
        PID_CONTROL_PROPORTIONAL,
        PID_CONTROL_DIFFERENTIAL,
        PID_CONTROL_INTEGRAL,
        TOTAL_PID_CONTROL_TYPES
    };

    // Constructor
    PID();

    // Destructor.
    ~PID();

    double steer(double cte);

private:

    // Errors
    std::vector<double> m_error;

    // Coefficients
    std::vector<double> m_tau;

    //  Control the hyper parameters
    std::vector<double> m_dp;

    StateMachine m_state;

    double m_bestError;

    uint32_t m_timeStep;

    double m_totalError;

    PidControl m_pidControlType;

    bool m_doStateThree;

    const double m_zoomOutFactor = 1.1;

    const double m_zoomInFactor = 0.9;

    const uint32_t m_iter = 100;

    // Update the PID error variables given cross track error.
    void twiddle();

    // Calculate the total PID error.
    void TotalError(double cte);

    void stateTransition();

    void pidControlTypeTransition();

    void firstState();

    void secondState();

    void thirdState();
};

#endif // PID_H
