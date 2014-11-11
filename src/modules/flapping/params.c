#include <systemlib/param/param.h>

// 16 is max name length

/**
 * Pitch to Voltage
 *
 * Pitch to voltage proportional gain
 *
 * @min
 * @max
 * @group Flapping
 */
PARAM_DEFINE_FLOAT(FL_TH2V_P, 10.0f); // pitch to voltage
/**
 * Pitch Integral to Voltage
 *
 * Pitch to voltage integral gain
 *
 * @min
 * @max
 * @group Flapping
 */
PARAM_DEFINE_FLOAT(FL_TH2V_I, 0.0f); // pitch integral to voltage
/**
 * Integral Limiter
 *
 * Upper limit on the  Pitch Integral to Voltage contribution
 *
 * @min
 * @max
 * @group Flapping
 */
PARAM_DEFINE_FLOAT(FL_TH2V_I_MAX, 0.0f); // integral limiter
/**
 * Pitch Rate to Voltage
 *
 * Pitch rate to voltage, i.e. derivative, gain
 *
 * @min
 * @max
 * @group Flapping
 */
PARAM_DEFINE_FLOAT(FL_Q2V, 1.0f); // pitch rate to voltage

/**
 * Servo Travel
 *
 * Angular difference in degrees of servo position from top to 
 * bottom of a flapping cycle
 *
 * @min
 * @max
 * @group Flapping
 */
PARAM_DEFINE_FLOAT(FL_SRV_TRV, 45.0f); // servo travel, deg
/**
 * Wing Up Position
 *
 * Angular position in degrees of the top of the flapping motion
 *
 * @min
 * @max
 * @group Flapping
 */
PARAM_DEFINE_FLOAT(FL_WNG_UP, -30.0f); // wing up pos, deg
/**
 * Wing Down Position
 *
 * Angular position in degrees of the bottom of the flapping motion
 *
 * @min
 * @max
 * @group Flapping
 */
PARAM_DEFINE_FLOAT(FL_WNG_DWN, 25.0f); // wing down pos, deg
/**
 * Wing Glide Position
 *
 * Angular position in degrees of servos during glide
 *
 * @min
 * @max
 * @group Flapping
 */
PARAM_DEFINE_FLOAT(FL_WNG_GLD, -8.0f); // wing glide pos, deg
/**
 * Down-to-Up Time
 *
 * Time in seconds it takes to travel from down position to up position
 *
 * @min
 * @max
 * @group Flapping
 */
PARAM_DEFINE_FLOAT(FL_T_DWN2UP, 0.06f); // down to up time, s
/**
 * Up-to-Glide Time
 *
 * Time in seconds it takes to travel from up position to glide position
 *
 * @min
 * @max
 * @group Flapping
 */
PARAM_DEFINE_FLOAT(FL_T_UP2GLD, 0.16f); // up to glide time, s
/**
 * Throttle Glide Limit
 *
 * Below this throttle value, the vehicle will glide (no flapping)
 *
 * @min 0.0f
 * @max 1.0f
 * @group Flapping
 */
PARAM_DEFINE_FLOAT(FL_THR_GLD, 0.2f); // throttle to glide below, 0-1
/**
 * Throttle to Frequency
 *
 * Throttle to flapping frequency gain
 *
 * @min
 * @max
 * @group Flapping
 */
PARAM_DEFINE_FLOAT(FL_THR2FREQ, 3.3f); // norm. throttle to freq gain
/**
 * Minimum Flapping Frequency
 *
 * Flapping frequency (Hz) of minimum non-glide throttle value
 *
 * @min
 * @max
 * @group Flapping
 */
PARAM_DEFINE_FLOAT(FL_MIN_FREQ, 1.7f); // min flapping freq

/**
 *
 * Learning Index 1
 *
 * Index of the first of three parameters to learn to minimize power consumption
 *
 * @min 0.0f
 * @max 13.0f
 * @group Flapping
 */
PARAM_DEFINE_FLOAT(FL_LRN1, 0.0f);
/**
 *
 * Learning Index 2
 *
 * Index of the second of three parameters to learn to minimize power consumption
 *
 * @min 0.0f
 * @max 13.0f
 * @group Flapping
 */
PARAM_DEFINE_FLOAT(FL_LRN2, 1.0f);
/**
 *
 * Learning Index 3
 *
 * Index of the third of three parameters to learn to minimize power consumption
 *
 * @min 0.0f
 * @max 13.0f
 * @group Flapping
 */
PARAM_DEFINE_FLOAT(FL_LRN3, 2.0f);
