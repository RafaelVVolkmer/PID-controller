/** ===========================================================================
    @addtogroup PIDController
    @addtogroup PIDController_Module PID_Controller

    @package    PID_Controller
    @brief      This module provides functionalities for configuring, resetting,
                and computing the output of a Proportional-Integral-Derivative 
                (PID) controller used in various control systems.

    @file       PIDController.h

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

#ifndef PIDCONTROLLER_H_
#define PIDCONTROLLER_H_

/* ==================================== *\
 *           PUBLIC DEFINES             *
\* ==================================== */

/** ====================================
  @def      FILTER_BUFFER_SIZE
  @package  PID_Controller
  @brief    Moving average buuffer
            max size
 ==================================== **/
#define FILTER_BUFFER_SIZE 32U

/* ==================================== *\
 *       PUBLIC TYPES DEFINITION        *
\* ==================================== */

/** ============================================================================
  @struct   pid_gains_t
  @package  PID_Controller

  @typedef  pid_gains_t

  @brief    Represents the PID controller's gain coefficients.

  @details  Contains the proportional (kp), integral (ki), and derivative (kd) 
            gain values used to compute the PID controller's output. These 
            coefficients determine the responsiveness and stability of the PID 
            controller in various control systems.
============================================================================ **/
typedef struct __attribute__((packed)) pidGains
{
    float kp;  /**< Proportional gain coefficient. */
    float ki;  /**< Integral gain coefficient. */
    float kd;  /**< Derivative gain coefficient. */
} pid_gains_t;

/** ============================================================================
  @struct   pid_limits_t
  @package  PID_Controller

  @typedef  pid_limits_t

  @brief    Represents the PID controller's operational limits.

  @details  Defines the minimum and maximum limits for both the integrator and 
            the controller's output. These limits are essential to prevent 
            integrator windup and output saturation, ensuring the PID controller
            operates within safe and desired bounds.
============================================================================ **/
typedef struct __attribute__((packed)) pidLimits
{
    float out_min; /**< Minimum output limit of the PID controller. */
    float out_max; /**< Maximum output limit of the PID controller. */

    float int_min; /**< Minimum integrator limit to prevent windup. */
    float int_max; /**< Maximum integrator limit to prevent windup. */
} pid_limits_t;


/** ============================================================================
  @struct   pid_controller_t
  @package  PID_Controller

  @typedef  pid_controller_t

  @brief    Represents the PID controller's state and configuration.

  @details  Contains all necessary parameters and internal states required for the
            operation of a PID controller. This includes the PID gains, operational
            limits, filter coefficient, sample time, integrator and differentiator
            states, previous measurement and error values, and the current output.
            Proper initialization and maintenance of these parameters are crucial
            for the accurate and stable functioning of the PID controller.
============================================================================ **/
typedef struct __attribute__((packed)) pidController
{
    pid_gains_t gains;                /**< PID gain coefficients (kp, ki, kd). */
    pid_limits_t limits;              /**< Operational limits for integrator and output. */

    float filter_coefficient;         /**< Coefficient for the derivative filter (tau). */
    float sample_time;                /**< Time interval between PID calculations (seconds). */

    float integrator;                 /**< Accumulated integral value. */
    float differentiator;             /**< Current derivative value. */

    float prev_measure;               /**< Previous measurement value. */
    float prev_error;                 /**< Previous error value. */
    float output;                     /**< Current PID controller output. */
} pid_controller_t;


/** ============================================================================
  @struct   filter_t
  @package  PID_Controller

  @typedef  filter_t

  @brief    Represents a moving average filter structure.

  @details  Implements a moving average filter to smooth input measurements by
            reducing transient noise. The structure contains a buffer array to 
            store recent input values, an index to track the current position in 
            the buffer,and a sum to facilitate efficient average calculation. 
            The buffer size is defined by `FILTER_BUFFER_SIZE`.
============================================================================ **/
typedef struct __attribute__((packed)) movingAverageFilter
{
    float buffer[FILTER_BUFFER_SIZE]; /**< Circular buffer to store recent input values. */
    unsigned int index;               /**< Current index in the buffer for the next input value. */
    float sum;                        /**< Sum of the current values in the buffer for average calculation. */
} filter_t;


/* ==================================== *\
 *     PUBLIC FUNCTIONS PROTOTYPES      *
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
static int PIDController_initController(pid_controller_t *pid);

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
static float PIDController_getOut(pid_controller_t *pid, float setpoint, float measure);

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
static int PIDController_initFilter(filter_t *filter);

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
float PIDController_filterMovingAverage(filter_t *filter, float new_value);

#endif /* PIDCONTROLLER_H_ */

/*< end of header file >*/
