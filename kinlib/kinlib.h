/*
 * kinlib library header
 */

/* Author: Dasharadhan Mahalingam */

#pragma once

namespace kinlib
{
typedef enum{ OPERATION_SUCCESS, 
              OPERATION_FAILURE,
              JOINT_LIMIT_ERROR,
              PARAMETER_ERROR }ErrorCodes;

typedef enum{ PLAN_SUCCES,
              JOINT_LIMITS_VIOLATED,
              JACOBIAN_PINV_NOT_FINITE,
              PLANNER_NOT_CONVERGING,
              UNKNOWN }MotionPlanReturnCodes;
}