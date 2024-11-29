/** ===========================================================================
    @ingroup    PIDController
    @addtogroup PIDController_Module PID_Controller

    @package    PID_Controller
    @brief      This module provides functionalities for configuring, resetting,
                and computing the output of a Proportional-Integral-Derivative 
                (PID) controller used in various control systems.

    @file       PIDController.c
    @headerfile PIDController.h

    @author     Rafael V. Volkmer (rafael.v.volkmer@gmail.com)
    @date       28.11.2024

    @details    The PID Controller module allows users to configure the gains for 
                the proportional, integral, and derivative terms, set limits for 
                the controller's integrator and output, and compute the controller's 
                output based on setpoint and measurement inputs. It includes 
                functionalities for initializing, resetting, and applying a moving 
                average filter to measurements to reduce noise and enhance precision.

                Key Features:
                - Initialization and resetting of PID controller parameters.
                - Calculation of PID output using proportional, integral, and 
                  derivative terms.
                - Anti-windup mechanisms to prevent integrator saturation.
                - Implementation of a moving average filter to smooth input 
                  measurements.

    @note       - Ensure that the PID controller parameters are correctly 
                  initialized before use to avoid unexpected behavior.
                - The module relies on standard C library functions like `memset` 
                  for memory operations.
                - The moving average filter buffer size (`FILTER_BUFFER_SIZE`) 
                  should be appropriately defined to balance noise reduction and 
                  responsiveness.
 =========================================================================== **/

/* ==================================== *\
 *            INCLUDED FILES            *
\* ==================================== */

/*< Dependencies >*/
#include <string.h>
#include <errno.h>

/*< Implemented >*/
#include "PIDContoller.h"

/* ==================================== *\
 *            PRIVATE DEFINES           *
\* ==================================== */

/** ====================================
  @def      HALF_CONST
  @package  PID_Controller
  @brief    Represents half (0.5) as 
            a float constant.
 ==================================== **/
#define HALF_CONST      (float)(0.5F)

/** ====================================
  @def      DOUBLE_CONST
  @package  PID_Controller
  @brief    Represents double (2.0) as 
            a float constant.
 ==================================== **/
#define DOUBLE_CONST    (float)(2.0F)

/** ====================================
  @def      TRIPLE_CONST
  @package  PID_Controller
  @brief    Represents triple (3.0) as 
            a float constant.
 ==================================== **/
#define TRIPLE_CONST    (float)(3.0F)

/* ==================================== *\
 *     PRIVATE FUNCTIONS DEFINITION     *
\* ==================================== */

/** ============================================================================
  @fn       PIDController_initController
  @package  PID_Controller

  @brief    Resets the PID controller parameters to their default values.

  @details  This function initializes or resets all parameters and internal states
            of the PID controller by setting the entire `pid_controller_t` structure
            to zero. It uses the `memset` function for this purpose. If the provided
            PID controller pointer is `NULL`, the function returns an appropriate
            error code.

  @param    pid    [in/out]:   Pointer to the PID controller instance to be reset.

  @return   0 on success.
            -EINVAL if the `pid` pointer is `NULL`.
            -ENOMEM if the `memset` function fails to clear the memory.
 =========================================================================== **/
static int PIDController_initController(pid_controller_t *pid)
{
    /*< Variable Declarations >*/
    int ret         = 0u; /*< Return Control >*/

    void *mem_block = NULL;

    /*< Security Checks >*/
    if(pid == NULL)
    {
        ret = -(EINVAL);
        goto end_of_function;
    }

    /*< Start Function Algorithm >*/
    mem_block = memset(pid, 0, sizeof(pid_controller_t));
    if(mem_block == NULL)
    {
        ret = -(ENOMEM);
        goto end_of_function;
    }

    /*< Function Output >*/
end_of_function:
    return ret;
}

/** ============================================================================
  @fn       PIDController_getOut
  @package  PID_Controller

  @brief    Calculates the PID controller output based on the setpoint and current
             measurement.

  @details  This function computes the output of the PID controller using the 
            current setpoint and measurement values. It calculates the proportional, 
            integral, and derivative terms, applies anti-windup by clamping the 
            integrator within specified limits, and ensures the final output is 
            within the defined output bounds. The derivative term is implemented 
            using a band-limited differentiator to reduce noise sensitivity.

  @param    pid        [in/out]:    Pointer to the PID controller instance.
  @param    setpoint   [in]:        Desired value that the process variable should 
                                    reach.
  @param    measure    [in]:        Current value of the process variable.

  @return   PID controller output on success.
            -ENOMEM if the `pid` pointer is `NULL`.
 =========================================================================== **/
static float PIDController_getOut(pid_controller_t *pid, float setpoint, float measure)
{
     /*< Variable Declarations >*/
    float ret           = 0.0f; /*< Return Control >*/

    float kp            = 0.0f;
    float kd            = 0.0f;
    float ki            = 0.0f;
    float t             = 0.0f;
    float tau           = 0.0f;

    float error         = 0.0f;
    float proportional  = 0.0f;
    float numerator     = 0.0f;
    float denominator   = 0.0f;

    /*< Security Checks >*/
    if(pid == NULL)
    {
        ret = -(ENOMEM);
        goto end_of_function;
    }

    /*< Assign Initial Values >*/
    kp = pid->gains.kp;
    kd = pid->gains.kd;
    ki = pid->gains.ki;

    t   = pid->sample_time;
    tau = pid->filter_coefficient;
    
    /*< Calculate Error Term >*/
    error           = (float)(setpoint - measure);

    /*< Calculate Proportional Term >*/
    proportional    = (float)(kp * error);

    /*< Calculate Integral Term - With Trapezoidal Rule >*/
    pid->integrator += (float)(HALF_CONST * ki * t * (error * pid->prev_error));

    /*< Anti-Windup: Clamp Integrator >*/
    pid->integrator = (pid->integrator > pid->limits.int_max) ? pid->limits.int_max :
                      (pid->integrator < pid->limits.int_min) ? pid->limits.int_min : pid->integrator;

    /*< Derivate Term Using Band-Limited Diferrentiator >*/
    numerator   = (float)(
                            (DOUBLE_CONST * tau - t) * pid->differentiator - 
                            DOUBLE_CONST * kd * (measure - pid->prev_measure)
                        );           
    denominator = (float)(DOUBLE_CONST * tau + t);
    
    pid->differentiator = (float)(numerator / denominator);

    /*< Compute Total Output >*/
    pid->output = (float)(proportional + pid->integrator + pid->differentiator);

    /*< Clamp Total Output >*/
    pid->output = (pid->output > pid->limits.out_max) ? pid->limits.out_max :
                  (pid->output < pid->limits.out_min) ? pid->limits.out_min : pid->output;

    /*< Update Previous States >*/
    pid->prev_error     = (float)(error);
    pid->prev_measure   = (float)(measure);

    ret = pid->output;

    /*< Function Output >*/
end_of_function:
    return ret;
}

/** ============================================================================
  @fn       PIDController_initFilter
  @package  PID_Controller

  @brief    Initializes the moving average filter parameters.

  @details  This function resets all parameters and states of the moving average 
            filter by setting the entire `filter_t` structure to zero. It uses 
            the `memset` function for this purpose. If the provided filter pointer 
            is `NULL`, the function returns an appropriate error code.

  @param    filter    [in/out]:   Pointer to the filter instance to be initialized.

  @return   0 on success.
            -EINVAL if the `filter` pointer is `NULL`.
            -ENOMEM if the `memset` function fails to clear the memory.
 =========================================================================== **/
static int PIDController_initFilter(filter_t *filter)
{
    /*< Variable Declarations >*/
    int ret         = 0u; /*< Return Control >*/

    void *mem_block = NULL;

    /*< Security Checks >*/
    if(filter == NULL)
    {
        ret = -(EINVAL);
        goto end_of_function;
    }

    /*< Start Function Algorithm >*/
    mem_block = memset(filter, 0, sizeof(filter_t));
    if(mem_block == NULL)
    {
        ret = -(ENOMEM);
        goto end_of_function;
    }

    /*< Function Output >*/
end_of_function:
    return ret;
}

/** ============================================================================
  @fn       PIDController_filterMovingAverage
  @package  PID_Controller

  @brief    Applies a moving average filter to the input value.

  @details  This function updates the moving average filter with a new input value. 
            It subtracts the oldest value in the buffer from the sum, adds the 
            new value, updates the buffer index, and calculates the average. 
            This helps in smoothing out the measurements by reducing the impact 
            of transient noise.

  @param    filter     [in/out]:   Pointer to the moving average filter instance.
  @param    value      [in]:        New input value to be added to the filter.

  @return   The filtered average value on success.
            -EINVAL if the `filter` pointer is `NULL`.
 =========================================================================== **/
float PIDController_filterMovingAverage(filter_t *filter, float new_value)
{
    /*< Variable Declarations >*/
    float ret = 0.0f; /*< Return Control >*/

    /*< Security Checks >*/
    if(filter == NULL)
    {
        ret = -(EINVAL);
        goto end_of_function;
    }

    /*< Start Function Algorithm >*/
    filter->sum                    -= filter->buffer[filter->index];
    filter->buffer[filter->index]   = new_value;
    filter->sum                    += new_value;
    filter->index                   = ((filter->index + 1u) % FILTER_BUFFER_SIZE);

    ret = (float)(filter->sum / FILTER_BUFFER_SIZE);

    /*< Function Output >*/
end_of_function:
    return ret;
}

/*< end of file >*/
