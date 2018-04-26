/**************************************************************************************************
 *  INCLUDES
 *************************************************************************************************/
#include "PID.h"

#include <algorithm>
#include <iostream>
#include <iterator>
#include <stdint.h>

/**************************************************************************************************
 *  NAMESPACES
 *************************************************************************************************/
using namespace std;

/**************************************************************************************************
 *  LOCAL HELPER FUNCTIONS
 *************************************************************************************************/
static double operator*(const vector<double> &v1, const vector<double> &multFactor);

/**************************************************************************************************
 *  CLASS METHODS
 *************************************************************************************************/

/**************************************************************************************************
 *  @brief Constructor
 *************************************************************************************************/
PID::PID() :
                m_error { 0, 0, 0 }, m_tau { 0.3, 10, 0 }, m_dp { 0.2, 1.2, 0.001 },
                m_state(STATE_INVALID), m_bestError(numeric_limits<double>::max()), m_timeStep(1),
                m_totalError(0), m_pidControlType(PID_CONTROL_PROPORTIONAL), m_doStateThree(false)
{
}

/**************************************************************************************************
 *  @brief Destructor
 *************************************************************************************************/
PID::~PID()
{
}

/**************************************************************************************************
 *  @brief Calculate the steering input.
 *************************************************************************************************/
double PID::steer(double cte)
{
    static uint32_t initTMinus1PropotionalError = false;

    // Initialize the error of the proportional controller.
    if (initTMinus1PropotionalError == false)
    {
        m_error[PID_CONTROL_PROPORTIONAL] = cte;
        initTMinus1PropotionalError = true;
    }

    m_error[PID_CONTROL_DIFFERENTIAL] = (cte - m_error[PID_CONTROL_PROPORTIONAL]);
    m_error[PID_CONTROL_INTEGRAL] += cte;
    m_error[PID_CONTROL_PROPORTIONAL] = cte;

    TotalError(cte);

    // Calculate the steering input
    double steer = -(m_tau * m_error);

    cout<<"m_tau[0] :: "<<m_tau[0]<<"  m_tau[1] :: "<<m_tau[1]<<"  m_tau[2] :: "<<m_tau[2]<<endl;
    // Limit the steering input between +/- 1
    if (steer > 1)
    {
        steer = 1;
    }
    else if (steer < -1)
    {
        steer = -1;
    }

    m_timeStep += 1;

    return steer;
}

/**************************************************************************************************
 *  @brief Calculate the total PID error.
 *************************************************************************************************/
void PID::TotalError(double cte)
{
    if (m_timeStep >= m_iter)
    {
        m_totalError += (cte * cte);
    }

    if (m_timeStep % (2 * m_iter) == 0)
    {
        m_totalError = m_totalError / m_iter;
        twiddle();
        m_timeStep = 1;
        m_totalError = 0;
    }
}

/**************************************************************************************************
 *  @brief Update the PID error variables given cross track error.
 *************************************************************************************************/
void PID::twiddle()
{
    if ((STATE_INVALID == m_state))
    {
        m_bestError = m_totalError;
    }

    // Change the state of the state machine for the Twiddle algo.
    stateTransition();

    if (STATE_FIRST == m_state)
    {
        firstState();
    }
    else if (STATE_SECOND == m_state)
    {
        secondState();
    }
    else if (STATE_THIRD == m_state)
    {
        thirdState();
    }
}

/**************************************************************************************************
 *  @brief State transition function
 *************************************************************************************************/
void PID::stateTransition()
{
    if ((STATE_INVALID == m_state))
    {
        m_state = STATE_FIRST;
    }
    else if ((STATE_FIRST == m_state))
    {
        m_state = STATE_SECOND;
    }
    else if ((STATE_SECOND == m_state))
    {
        if (m_doStateThree)
        {
            m_state = STATE_THIRD;
        }
        else
        {
            m_state = STATE_FIRST;
        }
    }
    else if ((STATE_THIRD == m_state)) // and (m_sumDp < m_tolerance))
    {
        m_state = STATE_FIRST;
    }
}

/**************************************************************************************************
 *  @brief This function changes the control type of the pid controller.
 *************************************************************************************************/
void PID::pidControlTypeTransition()
{
    if (m_pidControlType == PID_CONTROL_PROPORTIONAL)
    {
        m_pidControlType = PID_CONTROL_DIFFERENTIAL;
    }
    else if (m_pidControlType == PID_CONTROL_DIFFERENTIAL)
    {
        m_pidControlType = PID_CONTROL_INTEGRAL;
    }
    else if (m_pidControlType == PID_CONTROL_INTEGRAL)
    {
        m_pidControlType = PID_CONTROL_PROPORTIONAL;
    }
}
/**************************************************************************************************
 *  @brief Function for init state processing
 *************************************************************************************************/
void PID::firstState()
{
    cout<<"First state before ::"<<m_tau[m_pidControlType]<< "  control type :: "<<m_pidControlType<<endl;
    m_tau[m_pidControlType] += m_dp[m_pidControlType];
    cout<<"First state after ::"<<m_tau[m_pidControlType]<< "  control type :: "<<m_pidControlType<<endl;

}

/**************************************************************************************************
 *  @brief Function for under shoot state processing
 *************************************************************************************************/
void PID::secondState()
{
    cout<<"second state before ::"<<m_tau[m_pidControlType]<< "  control type :: "<<m_pidControlType<<endl;

    if (m_totalError < m_bestError)
    {
        m_bestError = m_totalError;
        m_dp[m_pidControlType] *= m_zoomOutFactor;
        m_doStateThree = false;
    }
    else
    {
        m_tau[m_pidControlType] -= (m_dp[m_pidControlType] * 2);
        m_doStateThree = true;
    }
    cout<<"second state before ::"<<m_tau[m_pidControlType]<< "  control type :: "<<m_pidControlType<<endl;
}

/**************************************************************************************************
 *  @brief Function for over shoot state processing
 *************************************************************************************************/
void PID::thirdState()
{
    cout<<"third state before ::"<<m_tau[m_pidControlType]<< "  control type :: "<<m_pidControlType<<endl;

    if (m_totalError < m_bestError)
    {
        m_bestError = m_totalError;
        m_dp[m_pidControlType] *= m_zoomOutFactor;
    }
    else
    {
        m_tau[m_pidControlType] += m_dp[m_pidControlType];
        m_dp[m_pidControlType] *= m_zoomInFactor;
    }

    cout<<"third state before ::"<<m_tau[m_pidControlType]<< "  control type :: "<<m_pidControlType<<endl;

    pidControlTypeTransition();
}

/**************************************************************************************************
 *  @brief Overload the *= operator for calculating the element wise sum product
 *************************************************************************************************/
double operator*(const vector<double> &v1, const vector<double> &v2)
{
    double result = 0;

    for (uint32_t i = 0; i < v1.size(); ++i)
    {
        result += (v1[i] * v2[i]);
    }

    return result;
}
