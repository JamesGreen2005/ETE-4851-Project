/*
 * Vehicle_Drive.c
 *
 * Academic License - for use in teaching, academic research, and meeting
 * course requirements at degree granting institutions only.  Not for
 * government, commercial, or other organizational use.
 *
 * Code generation for model "Vehicle_Drive".
 *
 * Model version              : 18.12
 * Simulink Coder version : 26.1 (R2026a) 20-Nov-2025
 * C source code generated on : Tue Nov  3 19:03:45 2026
 *
 * Target selection: ert.tlc
 * Embedded hardware selection: ARM Compatible->ARM Cortex-M
 * Code generation objectives: Unspecified
 * Validation result: Not run
 */

#include "Vehicle_Drive.h"
#include "vehicle_io.h"
#include "current_ina219.h"
#include "main.h"
#include "mw_stm32_utils.h"
#include "Vehicle_Drive_types.h"
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include <math.h>
#include "Vehicle_Drive_private.h"
#include "stm_timer_ll.h"
#include <string.h>

/* Block states (default storage) */
DW_Vehicle_Drive_T Vehicle_Drive_DW;

/* Real-time model */
static RT_MODEL_Vehicle_Drive_T Vehicle_Drive_M_;
RT_MODEL_Vehicle_Drive_T *const Vehicle_Drive_M = &Vehicle_Drive_M_;

/* Forward declaration for local functions */
static uint32_T Vehicle_Drive_littleU32(const uint8_T bytes[4]);
static void Vehicle_Drive_failed(seBeCUOhjLsbh84shXJyK7B_Vehic_T *s, uint8_T
  errorCode);
static real_T Vehicle_Drive_mod(real_T x);
static void Vehicle_Dri_UARTWrite_setupImpl(stm32cube_blocks_UARTWrite_Ve_T *obj);
static void Vehicle_D_UARTWrite_setupImpl_f(stm32cube_blocks_UARTWrite_f_T *obj);
static void Vehicle_Drive_SystemCore_setup(stm32cube_blocks_UARTRead_Veh_T *obj);
static void Vehicle_Driv_SystemCore_setup_f(stm32cube_blocks_UARTRead_Veh_T *obj);
static void Vehicle_Dri_PWMOutput_setupImpl(stm32cube_blocks_PWMOutput_Ve_T *obj);

/* Function for MATLAB Function: '<Root>/Pi Motor Command' */
static uint32_T Vehicle_Drive_littleU32(const uint8_T bytes[4])
{
  return ((((uint32_T)bytes[1] << 8) + bytes[0]) + ((uint32_T)bytes[2] << 16)) +
    ((uint32_T)bytes[3] << 24);
}

/* Function for MATLAB Function: '<Root>/BNO055 Controller' */
static void Vehicle_Drive_failed(seBeCUOhjLsbh84shXJyK7B_Vehic_T *s, uint8_T
  errorCode)
{
  s->err = errorCode;
  s->waiting = false;
  s->parse = 0.0;
  s->b_index = 0.0;
  s->attempts++;
  s->delay = 20.0;
  if (s->attempts >= 3.0) {
    s->stage = 1.0;
    s->delay = 1000.0;
    s->attempts = 0.0;
    s->configured = false;
    s->age = 1.0E+6;
    s->cal = 0U;
  }
}

real_T rt_roundd_snf(real_T u)
{
  real_T y;
  if (fabs(u) < 4.503599627370496E+15) {
    if (u >= 0.5) {
      y = floor(u + 0.5);
    } else if (u > -0.5) {
      y = u * 0.0;
    } else {
      y = ceil(u - 0.5);
    }
  } else {
    y = u;
  }

  return y;
}

/* Function for MATLAB Function: '<Root>/Readable Angles' */
static real_T Vehicle_Drive_mod(real_T x)
{
  real_T r;
  if (rtIsNaN(x)) {
    r = (rtNaN);
  } else if (rtIsInf(x)) {
    r = (rtNaN);
  } else {
    r = fmod(x, 10.0);
    if (r == 0.0) {
      r = 0.0;
    } else if (r < 0.0) {
      r += 10.0;
    }
  }

  return r;
}

static void Vehicle_Dri_UARTWrite_setupImpl(stm32cube_blocks_UARTWrite_Ve_T *obj)
{
  uint8_T * txBufferPtr;
  UART_Type_T b;
  txBufferPtr = (void*)(&obj->UARTInternalBuffer[0]);

  /* Start for MATLABSystem: '<S5>/UART Write' */
  b.enableRxOrTx = 2U;
  b.mode = UART_TX_INTERRUPT_MODE;
  b.peripheralPtr = USART6;

  /* Start for MATLABSystem: '<S5>/UART Write' */
  b.txdmaPeripheralPtr = NULL;
  b.txdmastream = 0;
  b.buffer.txBufferSize = 128U;
  b.buffer.txBufferPtr = txBufferPtr;
  obj->UARTHandle = MW_UART_Initialize(&b);
}

static void Vehicle_D_UARTWrite_setupImpl_f(stm32cube_blocks_UARTWrite_f_T *obj)
{
  uint8_T * txBufferPtr;
  UART_Type_T b;
  txBufferPtr = (void*)(&obj->UARTInternalBuffer[0]);

  /* Start for MATLABSystem: '<S11>/UART Write' */
  b.enableRxOrTx = 2U;
  b.mode = UART_TX_INTERRUPT_MODE;
  b.peripheralPtr = USART3;

  /* Start for MATLABSystem: '<S11>/UART Write' */
  b.txdmaPeripheralPtr = NULL;
  b.txdmastream = 0;
  b.buffer.txBufferSize = 256U;
  b.buffer.txBufferPtr = txBufferPtr;
  obj->UARTHandle = MW_UART_Initialize(&b);
}

static void Vehicle_Drive_SystemCore_setup(stm32cube_blocks_UARTRead_Veh_T *obj)
{
  uint8_T * rxBufferPtr;
  UART_Type_T b;

  /* Start for MATLABSystem: '<Root>/Pi Commands USB' */
  obj->isInitialized = 1;
  rxBufferPtr = (void*)(&obj->UARTInternalBuffer[0]);
  b.enableRxOrTx = 1.0;

  /* Start for MATLABSystem: '<Root>/Pi Commands USB' */
  b.mode = UART_RX_INTERRUPT_MODE;
  b.peripheralPtr = USART3;

  /* Start for MATLABSystem: '<Root>/Pi Commands USB' */
  b.rxdmaPeripheralPtr = NULL;
  b.rxdmastream = 0;
  b.disableDMAIntOnError = false;

  /* Start for MATLABSystem: '<Root>/Pi Commands USB' */
  b.buffer.rxBufferSize = 128U;
  b.buffer.rxBufferPtr = rxBufferPtr;
  obj->UARTHandle = MW_UART_Initialize(&b);
  obj->isSetupComplete = true;
}

static void Vehicle_Driv_SystemCore_setup_f(stm32cube_blocks_UARTRead_Veh_T *obj)
{
  uint8_T * rxBufferPtr;
  UART_Type_T b;

  /* Start for MATLABSystem: '<Root>/UART//USART Read' */
  obj->isInitialized = 1;
  rxBufferPtr = (void*)(&obj->UARTInternalBuffer[0]);
  b.enableRxOrTx = 1.0;

  /* Start for MATLABSystem: '<Root>/UART//USART Read' */
  b.mode = UART_RX_INTERRUPT_MODE;
  b.peripheralPtr = USART6;

  /* Start for MATLABSystem: '<Root>/UART//USART Read' */
  b.rxdmaPeripheralPtr = NULL;
  b.rxdmastream = 0;
  b.disableDMAIntOnError = false;

  /* Start for MATLABSystem: '<Root>/UART//USART Read' */
  b.buffer.rxBufferSize = 128U;
  b.buffer.rxBufferPtr = rxBufferPtr;
  obj->UARTHandle = MW_UART_Initialize(&b);
  obj->isSetupComplete = true;
}

static void Vehicle_Dri_PWMOutput_setupImpl(stm32cube_blocks_PWMOutput_Ve_T *obj)
{
  TIM_Type_T b;
  boolean_T isSlaveModeTriggerEnabled;

  /* Start for MATLABSystem: '<S19>/PWM Output' */
  b.PeripheralPtr = TIM1;
  b.isCenterAlignedMode = false;

  /* Start for MATLABSystem: '<S19>/PWM Output' */
  b.repetitionCounter = 0U;
  obj->TimerHandle = Timer_Handle_Init(&b);
  enableTimerInterrupts(obj->TimerHandle, 0);
  enableTimerChannel1(obj->TimerHandle, ENABLE_CH);
  enableTimerChannel2(obj->TimerHandle, ENABLE_CH);
  isSlaveModeTriggerEnabled = isSlaveTriggerModeEnabled(obj->TimerHandle);
  if (!isSlaveModeTriggerEnabled) {
    /* Start for MATLABSystem: '<S19>/PWM Output' */
    enableCounter(obj->TimerHandle, false);
  }
}

/* Model step function */
void Vehicle_Drive_step(void)
{
  GPIO_TypeDef * portNameLoc;
  uint32_T shiftVal;
  real_T desired;
  real_T tmp;
  real_T tmp_0;
  real_T tmp_1;
  real_T tmp_2;
  real_T tmp_3;
  real_T tmp_4;
  real_T tmp_5;
  real_T tmp_6;
  real_T tmp_7;
  int32_T requested[2];
  int32_T bit;
  int32_T ttl;
  uint32_T dataBytesToRead;
  uint32_T session;
  int16_T j_v;
  int16_T l_v;
  uint16_T pulses[2];
  uint16_T v[2];
  uint16_T c;
  uint16_T rtb_e1;
  uint16_T rtb_e2;
  uint16_T rtb_period;
  uint16_T rtb_s1;
  uint16_T rtb_s2;
  uint16_T status;
  uint8_T uartWriteData[256];
  uint8_T encoderDiagnostic[48];
  uint16_T encoderDiagnosticLength;
  uint8_T base[56];
  uint8_T tail[55];
  uint8_T text[6];
  uint8_T e_out[5];
  uint8_T f_out[5];
  uint8_T g_out[5];
  uint8_T h_out[5];
  uint8_T rtb_tx[5];
  uint8_T b_sign;
  uint8_T c_sign;
  uint8_T rtb_count;
  uint8_T st1;
  boolean_T x[2];
  boolean_T rtb_valid;
  boolean_T servoValid;
  static const uint8_T b[56] = { 72U, 61U, 43U, 48U, 48U, 48U, 46U, 48U, 32U,
    82U, 61U, 43U, 48U, 48U, 48U, 46U, 48U, 32U, 80U, 61U, 43U, 48U, 48U, 48U,
    46U, 48U, 32U, 67U, 65U, 76U, 61U, 48U, 47U, 48U, 47U, 48U, 47U, 48U, 32U,
    86U, 65U, 76U, 73U, 68U, 61U, 48U, 32U, 69U, 82U, 82U, 61U, 48U, 48U, 48U,
    13U, 10U };

  static const uint8_T c_0[6] = { 43U, 48U, 48U, 48U, 46U, 48U };

  static const uint8_T d[55] = { 32U, 69U, 49U, 61U, 48U, 48U, 48U, 48U, 48U,
    32U, 69U, 50U, 61U, 48U, 48U, 48U, 48U, 48U, 32U, 65U, 82U, 77U, 61U, 48U,
    32U, 68U, 49U, 61U, 43U, 48U, 48U, 48U, 46U, 48U, 32U, 68U, 50U, 61U, 43U,
    48U, 48U, 48U, 46U, 48U, 32U, 67U, 77U, 68U, 69U, 82U, 82U, 61U, 48U, 48U,
    48U };

  static const uint8_T e[17] = { 32U, 68U, 82U, 73U, 86U, 69U, 61U, 49U, 32U,
    70U, 87U, 61U, 55U, 32U, 83U, 49U, 61U };

  static const uint8_T h[11] = { 32U, 73U, 78U, 65U, 61U, 49U, 32U, 83U, 72U,
    49U, 61U };

  static const uint8_T i[5] = { 32U, 66U, 86U, 49U, 61U };

  static const uint8_T j[5] = { 32U, 83U, 84U, 49U, 61U };

  static const uint8_T l[5] = { 32U, 83U, 72U, 50U, 61U };

  static const uint8_T m[5] = { 32U, 66U, 86U, 50U, 61U };

  static const uint8_T n[5] = { 32U, 83U, 84U, 50U, 61U };

  static const uint8_T c_1[5] = { 170U, 0U, 61U, 1U, 0U };

  static const uint8_T d_0[5] = { 170U, 0U, 7U, 1U, 0U };

  static const uint8_T e_0[5] = { 170U, 0U, 62U, 1U, 0U };

  static const uint8_T f[5] = { 170U, 0U, 59U, 1U, 0U };

  static const uint8_T g[5] = { 170U, 0U, 61U, 1U, 12U };

  boolean_T exitg1;
  boolean_T guard1;
  boolean_T guard2;
  boolean_T guard3;

  /* MATLABSystem: '<Root>/Pi Commands USB' */
  dataBytesToRead = getBytesToBeReadFromBuffer(Vehicle_Drive_DW.obj_h.UARTHandle,
    1U, 0U);
  if (dataBytesToRead > 0U) {
    status = MW_UART_ReceiveUsingBuffer(Vehicle_Drive_DW.obj_h.UARTHandle, &st1,
      dataBytesToRead, &session);
  } else {
    status = getReadErrorStatus(Vehicle_Drive_DW.obj_h.UARTHandle);
    session = 0U;
  }

  if (dataBytesToRead < 1U) {
    status |= 1;
  }

  /* MATLAB Function: '<Root>/Pi Motor Command' incorporates:
   *  MATLABSystem: '<Root>/Pi Commands USB'
   * */
  guard1 = false;
  if (Vehicle_Drive_DW.s.armed) {
    guard1 = true;
  } else {
    x[0] = (Vehicle_Drive_DW.s.servo[0] != 0.0);
    x[1] = (Vehicle_Drive_DW.s.servo[1] != 0.0);
    servoValid = false;
    ttl = 0;
    exitg1 = false;
    while (!exitg1 && (ttl < 2)) {
      if (x[ttl]) {
        servoValid = true;
        exitg1 = true;
      } else {
        ttl++;
      }
    }

    if (servoValid) {
      guard1 = true;
    }
  }

  if (guard1) {
    Vehicle_Drive_DW.s.remaining--;
    if (Vehicle_Drive_DW.s.remaining <= 0.0) {
      Vehicle_Drive_DW.s.err = 2U;
      Vehicle_Drive_DW.s.armed = false;
      Vehicle_Drive_DW.s.permit = false;
      Vehicle_Drive_DW.s.remaining = 0.0;
      Vehicle_Drive_DW.s.target[0] = 0.0;
      Vehicle_Drive_DW.s.actual[0] = 0.0;
      Vehicle_Drive_DW.s.servo[0] = 0.0;
      Vehicle_Drive_DW.s.target[1] = 0.0;
      Vehicle_Drive_DW.s.actual[1] = 0.0;
      Vehicle_Drive_DW.s.servo[1] = 0.0;
    }
  }

  if ((status & 65534U) != 0U) {
    Vehicle_Drive_DW.s.err = 8U;
    Vehicle_Drive_DW.s.armed = false;
    Vehicle_Drive_DW.s.permit = false;
    Vehicle_Drive_DW.s.remaining = 0.0;
    Vehicle_Drive_DW.s.target[0] = 0.0;
    Vehicle_Drive_DW.s.actual[0] = 0.0;
    Vehicle_Drive_DW.s.servo[0] = 0.0;
    Vehicle_Drive_DW.s.target[1] = 0.0;
    Vehicle_Drive_DW.s.actual[1] = 0.0;
    Vehicle_Drive_DW.s.servo[1] = 0.0;
  } else if ((session == 1U) && (status == 0)) {
    for (ttl = 0; ttl < 23; ttl++) {
      Vehicle_Drive_DW.s.bytes[ttl] = Vehicle_Drive_DW.s.bytes[ttl + 1];
    }

    Vehicle_Drive_DW.s.bytes[23] = st1;
    if ((Vehicle_Drive_DW.s.bytes[0] == 195) && (Vehicle_Drive_DW.s.bytes[1] ==
         60)) {
      status = MAX_uint16_T;
      for (ttl = 0; ttl < 22; ttl++) {
        status = (uint16_T)((uint32_T)(Vehicle_Drive_DW.s.bytes[ttl] << 8) ^
                            status);
        for (bit = 0; bit < 8; bit++) {
          c = (uint16_T)(status & 32768);
          status <<= 1;
          if (c != 0) {
            status = (uint16_T)(status ^ 4129);
          }
        }
      }

      if ((Vehicle_Drive_DW.s.bytes[23] << 8) + Vehicle_Drive_DW.s.bytes[22] !=
          status) {
        Vehicle_Drive_DW.s.err = 3U;
        Vehicle_Drive_DW.s.armed = false;
        Vehicle_Drive_DW.s.permit = false;
        Vehicle_Drive_DW.s.remaining = 0.0;
        Vehicle_Drive_DW.s.target[0] = 0.0;
        Vehicle_Drive_DW.s.actual[0] = 0.0;
        Vehicle_Drive_DW.s.servo[0] = 0.0;
        Vehicle_Drive_DW.s.target[1] = 0.0;
        Vehicle_Drive_DW.s.actual[1] = 0.0;
        Vehicle_Drive_DW.s.servo[1] = 0.0;
      } else {
        session = Vehicle_Drive_littleU32(&Vehicle_Drive_DW.s.bytes[4]);
        dataBytesToRead = Vehicle_Drive_littleU32(&Vehicle_Drive_DW.s.bytes[8]);
        requested[0] = (Vehicle_Drive_DW.s.bytes[13] << 8) +
          Vehicle_Drive_DW.s.bytes[12];
        requested[1] = (Vehicle_Drive_DW.s.bytes[15] << 8) +
          Vehicle_Drive_DW.s.bytes[14];
        if (requested[0] >= 32768) {
          requested[0] -= 65536;
        }

        if (requested[1] >= 32768) {
          requested[1] -= 65536;
        }

        pulses[0] = (uint16_T)((Vehicle_Drive_DW.s.bytes[17] << 8) +
          Vehicle_Drive_DW.s.bytes[16]);
        pulses[1] = (uint16_T)((Vehicle_Drive_DW.s.bytes[19] << 8) +
          Vehicle_Drive_DW.s.bytes[18]);
        ttl = (Vehicle_Drive_DW.s.bytes[21] << 8) + Vehicle_Drive_DW.s.bytes[20];
        servoValid = true;
        bit = 0;
        exitg1 = false;
        while (!exitg1 && (bit < 2)) {
          if ((pulses[bit] < 1000) || (pulses[bit] > 2000)) {
            servoValid = false;
            exitg1 = true;
          } else {
            bit++;
          }
        }

        guard1 = false;
        guard2 = false;
        guard3 = false;
        if ((Vehicle_Drive_DW.s.bytes[2] != 5) || (session == 0U) ||
            (Vehicle_Drive_DW.s.bytes[3] > 2)) {
          guard1 = true;
        } else {
          v[0] = (uint16_T)fabs((real_T)requested[0]);
          v[1] = (uint16_T)fabs((real_T)requested[1]);
          rtb_valid = false;
          bit = 0;
          exitg1 = false;
          while (!exitg1 && (bit < 2)) {
            if (v[bit] > 1000) {
              rtb_valid = true;
              exitg1 = true;
            } else {
              bit++;
            }
          }

          if (rtb_valid || (ttl < 50) || (ttl > 250)) {
            guard1 = true;
          } else if (Vehicle_Drive_DW.s.bytes[3] < 2) {
            bit = 0;
            exitg1 = false;
            while (!exitg1 && (bit < 2)) {
              if (requested[bit] != 0) {
                rtb_valid = true;
                exitg1 = true;
              } else {
                bit++;
              }
            }

            if (rtb_valid) {
              guard1 = true;
            } else {
              guard3 = true;
            }
          } else {
            guard3 = true;
          }
        }

        if (guard3) {
          if (Vehicle_Drive_DW.s.bytes[3] == 0) {
            bit = 0;
            exitg1 = false;
            while (!exitg1 && (bit < 2)) {
              if (pulses[bit] != 0) {
                rtb_valid = true;
                exitg1 = true;
              } else {
                bit++;
              }
            }

            if (rtb_valid) {
              guard1 = true;
            } else {
              guard2 = true;
            }
          } else {
            guard2 = true;
          }
        }

        if (guard2) {
          if ((Vehicle_Drive_DW.s.bytes[3] != 0) && !servoValid) {
            guard1 = true;
          } else if ((Vehicle_Drive_DW.s.bytes[3] == 0) && (session !=
                      Vehicle_Drive_DW.s.session)) {
            Vehicle_Drive_DW.s.err = 0U;
            Vehicle_Drive_DW.s.armed = false;
            Vehicle_Drive_DW.s.remaining = 0.0;
            Vehicle_Drive_DW.s.target[0] = 0.0;
            Vehicle_Drive_DW.s.actual[0] = 0.0;
            Vehicle_Drive_DW.s.servo[0] = 0.0;
            Vehicle_Drive_DW.s.target[1] = 0.0;
            Vehicle_Drive_DW.s.actual[1] = 0.0;
            Vehicle_Drive_DW.s.servo[1] = 0.0;
            Vehicle_Drive_DW.s.session = session;
            Vehicle_Drive_DW.s.seq = dataBytesToRead;
            Vehicle_Drive_DW.s.permit = true;
          } else if (session != Vehicle_Drive_DW.s.session) {
            Vehicle_Drive_DW.s.err = 6U;
            Vehicle_Drive_DW.s.armed = false;
            Vehicle_Drive_DW.s.permit = false;
            Vehicle_Drive_DW.s.remaining = 0.0;
            Vehicle_Drive_DW.s.target[0] = 0.0;
            Vehicle_Drive_DW.s.actual[0] = 0.0;
            Vehicle_Drive_DW.s.servo[0] = 0.0;
            Vehicle_Drive_DW.s.target[1] = 0.0;
            Vehicle_Drive_DW.s.actual[1] = 0.0;
            Vehicle_Drive_DW.s.servo[1] = 0.0;
          } else if (dataBytesToRead <= Vehicle_Drive_DW.s.seq) {
            Vehicle_Drive_DW.s.err = 5U;
            Vehicle_Drive_DW.s.armed = false;
            Vehicle_Drive_DW.s.permit = false;
            Vehicle_Drive_DW.s.remaining = 0.0;
            Vehicle_Drive_DW.s.target[0] = 0.0;
            Vehicle_Drive_DW.s.actual[0] = 0.0;
            Vehicle_Drive_DW.s.servo[0] = 0.0;
            Vehicle_Drive_DW.s.target[1] = 0.0;
            Vehicle_Drive_DW.s.actual[1] = 0.0;
            Vehicle_Drive_DW.s.servo[1] = 0.0;
          } else {
            Vehicle_Drive_DW.s.seq = dataBytesToRead;
            switch (Vehicle_Drive_DW.s.bytes[3]) {
             case 0U:
              Vehicle_Drive_DW.s.err = 0U;
              Vehicle_Drive_DW.s.armed = false;
              Vehicle_Drive_DW.s.remaining = 0.0;
              Vehicle_Drive_DW.s.target[0] = 0.0;
              Vehicle_Drive_DW.s.actual[0] = 0.0;
              Vehicle_Drive_DW.s.servo[0] = 0.0;
              Vehicle_Drive_DW.s.target[1] = 0.0;
              Vehicle_Drive_DW.s.actual[1] = 0.0;
              Vehicle_Drive_DW.s.servo[1] = 0.0;
              Vehicle_Drive_DW.s.permit = true;
              break;

             case 1U:
              if (Vehicle_Drive_DW.s.permit || Vehicle_Drive_DW.s.armed) {
                Vehicle_Drive_DW.s.armed = true;
                Vehicle_Drive_DW.s.permit = false;
                Vehicle_Drive_DW.s.servo[0] = pulses[0];
                Vehicle_Drive_DW.s.target[0] = 0.0;
                Vehicle_Drive_DW.s.actual[0] = 0.0;
                Vehicle_Drive_DW.s.servo[1] = pulses[1];
                Vehicle_Drive_DW.s.target[1] = 0.0;
                Vehicle_Drive_DW.s.actual[1] = 0.0;
                Vehicle_Drive_DW.s.remaining = ttl;
                Vehicle_Drive_DW.s.err = 0U;
              } else {
                Vehicle_Drive_DW.s.err = 7U;
                Vehicle_Drive_DW.s.armed = false;
                Vehicle_Drive_DW.s.permit = false;
                Vehicle_Drive_DW.s.remaining = 0.0;
                Vehicle_Drive_DW.s.target[0] = 0.0;
                Vehicle_Drive_DW.s.actual[0] = 0.0;
                Vehicle_Drive_DW.s.servo[0] = 0.0;
                Vehicle_Drive_DW.s.target[1] = 0.0;
                Vehicle_Drive_DW.s.actual[1] = 0.0;
                Vehicle_Drive_DW.s.servo[1] = 0.0;
              }
              break;

             default:
              if (Vehicle_Drive_DW.s.armed) {
                Vehicle_Drive_DW.s.servo[0] = pulses[0];
                Vehicle_Drive_DW.s.target[0] = (real_T)requested[0] / 10.0;
                Vehicle_Drive_DW.s.servo[1] = pulses[1];
                Vehicle_Drive_DW.s.target[1] = (real_T)requested[1] / 10.0;
                Vehicle_Drive_DW.s.remaining = ttl;
                Vehicle_Drive_DW.s.err = 0U;
              } else {
                Vehicle_Drive_DW.s.err = 7U;
                Vehicle_Drive_DW.s.armed = false;
                Vehicle_Drive_DW.s.permit = false;
                Vehicle_Drive_DW.s.remaining = 0.0;
                Vehicle_Drive_DW.s.target[0] = 0.0;
                Vehicle_Drive_DW.s.actual[0] = 0.0;
                Vehicle_Drive_DW.s.servo[0] = 0.0;
                Vehicle_Drive_DW.s.target[1] = 0.0;
                Vehicle_Drive_DW.s.actual[1] = 0.0;
                Vehicle_Drive_DW.s.servo[1] = 0.0;
              }
              break;
            }
          }
        }

        if (guard1) {
          Vehicle_Drive_DW.s.err = 4U;
          Vehicle_Drive_DW.s.armed = false;
          Vehicle_Drive_DW.s.permit = false;
          Vehicle_Drive_DW.s.remaining = 0.0;
          Vehicle_Drive_DW.s.target[0] = 0.0;
          Vehicle_Drive_DW.s.actual[0] = 0.0;
          Vehicle_Drive_DW.s.servo[0] = 0.0;
          Vehicle_Drive_DW.s.target[1] = 0.0;
          Vehicle_Drive_DW.s.actual[1] = 0.0;
          Vehicle_Drive_DW.s.servo[1] = 0.0;
        }
      }

      for (ttl = 0; ttl < 24; ttl++) {
        Vehicle_Drive_DW.s.bytes[ttl] = 0U;
      }
    }
  }

  desired = 0.0;
  servoValid = (Vehicle_Drive_DW.s.armed && (Vehicle_Drive_DW.s.remaining > 0.0));
  if (servoValid) {
    desired = Vehicle_Drive_DW.s.target[0];
  }

  if ((desired != 0.0) && (Vehicle_Drive_DW.s.lastSign[0] != 0.0)) {
    if (rtIsNaN(desired)) {
      tmp_7 = (rtNaN);
    } else if (desired < 0.0) {
      tmp_7 = -1.0;
    } else {
      tmp_7 = (desired > 0.0);
    }

    if (tmp_7 != Vehicle_Drive_DW.s.lastSign[0]) {
      if (Vehicle_Drive_DW.s.actual[0] != 0.0) {
        desired = 0.0;
      } else if (Vehicle_Drive_DW.s.zeroTicks[0] < 500.0) {
        desired = 0.0;
      }
    }
  }

  if (servoValid) {
    Vehicle_Drive_DW.s.actual[0] = desired;
    if (fabs(Vehicle_Drive_DW.s.actual[0]) < 1.0E-9) {
      Vehicle_Drive_DW.s.actual[0] = 0.0;
    }
  } else {
    Vehicle_Drive_DW.s.actual[0] = 0.0;
  }

  if (Vehicle_Drive_DW.s.actual[0] == 0.0) {
    Vehicle_Drive_DW.s.zeroTicks[0] = fmin(500.0, Vehicle_Drive_DW.s.zeroTicks[0]
      + 1.0);
  } else {
    Vehicle_Drive_DW.s.zeroTicks[0] = 0.0;
    if (rtIsNaN(Vehicle_Drive_DW.s.actual[0])) {
      Vehicle_Drive_DW.s.lastSign[0] = (rtNaN);
    } else if (Vehicle_Drive_DW.s.actual[0] < 0.0) {
      Vehicle_Drive_DW.s.lastSign[0] = -1.0;
    } else {
      Vehicle_Drive_DW.s.lastSign[0] = (Vehicle_Drive_DW.s.actual[0] > 0.0);
    }
  }

  desired = 0.0;
  servoValid = (Vehicle_Drive_DW.s.armed && (Vehicle_Drive_DW.s.remaining > 0.0));
  if (servoValid) {
    desired = Vehicle_Drive_DW.s.target[1];
  }

  if ((desired != 0.0) && (Vehicle_Drive_DW.s.lastSign[1] != 0.0)) {
    if (rtIsNaN(desired)) {
      tmp_7 = (rtNaN);
    } else if (desired < 0.0) {
      tmp_7 = -1.0;
    } else {
      tmp_7 = (desired > 0.0);
    }

    if (tmp_7 != Vehicle_Drive_DW.s.lastSign[1]) {
      if (Vehicle_Drive_DW.s.actual[1] != 0.0) {
        desired = 0.0;
      } else if (Vehicle_Drive_DW.s.zeroTicks[1] < 500.0) {
        desired = 0.0;
      }
    }
  }

  if (servoValid) {
    Vehicle_Drive_DW.s.actual[1] = desired;
    if (fabs(Vehicle_Drive_DW.s.actual[1]) < 1.0E-9) {
      Vehicle_Drive_DW.s.actual[1] = 0.0;
    }
  } else {
    Vehicle_Drive_DW.s.actual[1] = 0.0;
  }

  if (Vehicle_Drive_DW.s.actual[1] == 0.0) {
    Vehicle_Drive_DW.s.zeroTicks[1] = fmin(500.0, Vehicle_Drive_DW.s.zeroTicks[1]
      + 1.0);
  } else {
    Vehicle_Drive_DW.s.zeroTicks[1] = 0.0;
    if (rtIsNaN(Vehicle_Drive_DW.s.actual[1])) {
      Vehicle_Drive_DW.s.lastSign[1] = (rtNaN);
    } else if (Vehicle_Drive_DW.s.actual[1] < 0.0) {
      Vehicle_Drive_DW.s.lastSign[1] = -1.0;
    } else {
      Vehicle_Drive_DW.s.lastSign[1] = (Vehicle_Drive_DW.s.actual[1] > 0.0);
    }
  }

  servoValid = (Vehicle_Drive_DW.s.armed && (Vehicle_Drive_DW.s.remaining > 0.0));

  /* MATLABSystem: '<S13>/Digital Port Write' incorporates:
   *  MATLAB Function: '<Root>/Pi Motor Command'
   */
  portNameLoc = GPIOF;
  shiftVal = MW_GPIO_BIT_SHIFT;
  if (servoValid) {
    session = 16384U;
  } else {
    session = 0U;
  }

  session = mw_shift(session, shiftVal);
  dataBytesToRead = mw_shift(16384U, shiftVal);
  LL_GPIO_SetOutputPin(portNameLoc, session);
  LL_GPIO_ResetOutputPin(portNameLoc, ~session & dataBytesToRead);

  /* End of MATLABSystem: '<S13>/Digital Port Write' */

  /* MATLABSystem: '<S15>/Digital Port Write' incorporates:
   *  MATLAB Function: '<Root>/Pi Motor Command'
   */
  portNameLoc = GPIOF;
  shiftVal = MW_GPIO_BIT_SHIFT;
  if (servoValid) {
    session = 32768U;
  } else {
    session = 0U;
  }

  session = mw_shift(session, shiftVal);
  dataBytesToRead = mw_shift(32768U, shiftVal);
  LL_GPIO_SetOutputPin(portNameLoc, session);
  LL_GPIO_ResetOutputPin(portNameLoc, ~session & dataBytesToRead);

  /* End of MATLABSystem: '<S15>/Digital Port Write' */

  /* MATLABSystem: '<Root>/UART//USART Read' */
  dataBytesToRead = getBytesToBeReadFromBuffer(Vehicle_Drive_DW.obj_f.UARTHandle,
    1U, 0U);
  if (dataBytesToRead > 0U) {
    status = MW_UART_ReceiveUsingBuffer(Vehicle_Drive_DW.obj_f.UARTHandle, &st1,
      dataBytesToRead, &session);
  } else {
    status = getReadErrorStatus(Vehicle_Drive_DW.obj_f.UARTHandle);
    session = 0U;
  }

  if (dataBytesToRead < 1U) {
    status |= 1;
  }

  /* MATLAB Function: '<Root>/BNO055 Controller' incorporates:
   *  MATLABSystem: '<Root>/UART//USART Read'
   * */
  for (ttl = 0; ttl < 5; ttl++) {
    rtb_tx[ttl] = 0U;
  }

  rtb_count = 0U;
  Vehicle_Drive_DW.s_l.age = fmin(Vehicle_Drive_DW.s_l.age + 1.0, 1.0E+6);
  if (Vehicle_Drive_DW.s_l.waiting && (session == 1U)) {
    if (status != 0) {
      Vehicle_Drive_failed(&Vehicle_Drive_DW.s_l, 254);
    } else if (Vehicle_Drive_DW.s_l.parse == 0.0) {
      switch (st1) {
       case 187U:
        Vehicle_Drive_DW.s_l.parse = 1.0;
        break;

       case 238U:
        Vehicle_Drive_DW.s_l.parse = 3.0;
        break;
      }
    } else if (Vehicle_Drive_DW.s_l.parse == 1.0) {
      ttl = 1;
      if (Vehicle_Drive_DW.s_l.stage == 7.0) {
        ttl = 6;
      }

      if (((Vehicle_Drive_DW.s_l.stage == 1.0) || (Vehicle_Drive_DW.s_l.stage ==
            7.0) || (Vehicle_Drive_DW.s_l.stage == 8.0)) && (st1 == ttl)) {
        Vehicle_Drive_DW.s_l.need = ttl;
        Vehicle_Drive_DW.s_l.b_index = 0.0;
        Vehicle_Drive_DW.s_l.parse = 2.0;
      } else {
        Vehicle_Drive_failed(&Vehicle_Drive_DW.s_l, 253);
      }
    } else if (Vehicle_Drive_DW.s_l.parse == 2.0) {
      Vehicle_Drive_DW.s_l.b_index++;
      Vehicle_Drive_DW.s_l.payload[(int32_T)Vehicle_Drive_DW.s_l.b_index - 1] =
        st1;
      if (Vehicle_Drive_DW.s_l.b_index == Vehicle_Drive_DW.s_l.need) {
        if (Vehicle_Drive_DW.s_l.stage == 1.0) {
          if (Vehicle_Drive_DW.s_l.payload[0] == 160) {
            Vehicle_Drive_DW.s_l.stage = 2.0;
            Vehicle_Drive_DW.s_l.delay = 2.0;
            Vehicle_Drive_DW.s_l.waiting = false;
            Vehicle_Drive_DW.s_l.timeout = 0.0;
            Vehicle_Drive_DW.s_l.parse = 0.0;
            Vehicle_Drive_DW.s_l.b_index = 0.0;
            Vehicle_Drive_DW.s_l.attempts = 0.0;
          } else {
            Vehicle_Drive_failed(&Vehicle_Drive_DW.s_l, 252);
          }
        } else if (Vehicle_Drive_DW.s_l.stage == 7.0) {
          ttl = (Vehicle_Drive_DW.s_l.payload[1] << 8) +
            Vehicle_Drive_DW.s_l.payload[0];
          if (ttl >= 32768) {
            ttl -= 65536;
          }

          Vehicle_Drive_DW.s_l.h = (real_T)ttl / 16.0;
          ttl = (Vehicle_Drive_DW.s_l.payload[3] << 8) +
            Vehicle_Drive_DW.s_l.payload[2];
          if (ttl >= 32768) {
            ttl -= 65536;
          }

          Vehicle_Drive_DW.s_l.r = (real_T)ttl / 16.0;
          ttl = (Vehicle_Drive_DW.s_l.payload[5] << 8) +
            Vehicle_Drive_DW.s_l.payload[4];
          if (ttl >= 32768) {
            ttl -= 65536;
          }

          Vehicle_Drive_DW.s_l.p = (real_T)ttl / 16.0;
          Vehicle_Drive_DW.s_l.age = 0.0;
          Vehicle_Drive_DW.s_l.err = 0U;
          Vehicle_Drive_DW.s_l.stage = 8.0;
          Vehicle_Drive_DW.s_l.delay = 2.0;
          Vehicle_Drive_DW.s_l.waiting = false;
          Vehicle_Drive_DW.s_l.timeout = 0.0;
          Vehicle_Drive_DW.s_l.parse = 0.0;
          Vehicle_Drive_DW.s_l.b_index = 0.0;
          Vehicle_Drive_DW.s_l.attempts = 0.0;
        } else if (Vehicle_Drive_DW.s_l.stage == 8.0) {
          Vehicle_Drive_DW.s_l.cal = Vehicle_Drive_DW.s_l.payload[0];
          Vehicle_Drive_DW.s_l.stage = 7.0;
          Vehicle_Drive_DW.s_l.delay = 40.0;
          Vehicle_Drive_DW.s_l.waiting = false;
          Vehicle_Drive_DW.s_l.timeout = 0.0;
          Vehicle_Drive_DW.s_l.parse = 0.0;
          Vehicle_Drive_DW.s_l.b_index = 0.0;
          Vehicle_Drive_DW.s_l.attempts = 0.0;
        }
      }
    } else if ((st1 == 1) && (Vehicle_Drive_DW.s_l.stage >= 2.0) &&
               (Vehicle_Drive_DW.s_l.stage <= 6.0)) {
      desired = Vehicle_Drive_DW.s_l.stage;
      Vehicle_Drive_DW.s_l.stage++;
      Vehicle_Drive_DW.s_l.delay = 2.0;
      Vehicle_Drive_DW.s_l.waiting = false;
      Vehicle_Drive_DW.s_l.timeout = 0.0;
      Vehicle_Drive_DW.s_l.parse = 0.0;
      Vehicle_Drive_DW.s_l.b_index = 0.0;
      Vehicle_Drive_DW.s_l.attempts = 0.0;
      if (desired == 2.0) {
        Vehicle_Drive_DW.s_l.delay = 30.0;
      } else if (desired == 4.0) {
        Vehicle_Drive_DW.s_l.delay = 10.0;
      } else if (desired == 6.0) {
        Vehicle_Drive_DW.s_l.configured = true;
        Vehicle_Drive_DW.s_l.delay = 500.0;
      }
    } else if (st1 == 1) {
      Vehicle_Drive_failed(&Vehicle_Drive_DW.s_l, 251);
    } else {
      Vehicle_Drive_failed(&Vehicle_Drive_DW.s_l, st1);
    }
  }

  if (Vehicle_Drive_DW.s_l.waiting) {
    Vehicle_Drive_DW.s_l.timeout--;
    if (Vehicle_Drive_DW.s_l.timeout <= 0.0) {
      Vehicle_Drive_failed(&Vehicle_Drive_DW.s_l, 250);
    }
  } else if (Vehicle_Drive_DW.s_l.delay > 0.0) {
    Vehicle_Drive_DW.s_l.delay--;
  } else {
    if (Vehicle_Drive_DW.s_l.stage == 1.0) {
      rtb_tx[0] = 170U;
      rtb_tx[1] = 1U;
      rtb_tx[2] = 0U;
      rtb_tx[3] = 1U;
      rtb_count = 4U;
    } else if (Vehicle_Drive_DW.s_l.stage == 2.0) {
      for (ttl = 0; ttl < 5; ttl++) {
        rtb_tx[ttl] = c_1[ttl];
      }

      rtb_count = 5U;
    } else if (Vehicle_Drive_DW.s_l.stage == 3.0) {
      for (ttl = 0; ttl < 5; ttl++) {
        rtb_tx[ttl] = d_0[ttl];
      }

      rtb_count = 5U;
    } else if (Vehicle_Drive_DW.s_l.stage == 4.0) {
      for (ttl = 0; ttl < 5; ttl++) {
        rtb_tx[ttl] = e_0[ttl];
      }

      rtb_count = 5U;
    } else if (Vehicle_Drive_DW.s_l.stage == 5.0) {
      for (ttl = 0; ttl < 5; ttl++) {
        rtb_tx[ttl] = f[ttl];
      }

      rtb_count = 5U;
    } else if (Vehicle_Drive_DW.s_l.stage == 6.0) {
      for (ttl = 0; ttl < 5; ttl++) {
        rtb_tx[ttl] = g[ttl];
      }

      rtb_count = 5U;
    } else if (Vehicle_Drive_DW.s_l.stage == 7.0) {
      rtb_tx[0] = 170U;
      rtb_tx[1] = 1U;
      rtb_tx[2] = 26U;
      rtb_tx[3] = 6U;
      rtb_count = 4U;
    } else {
      rtb_tx[0] = 170U;
      rtb_tx[1] = 1U;
      rtb_tx[2] = 53U;
      rtb_tx[3] = 1U;
      rtb_count = 4U;
    }

    Vehicle_Drive_DW.s_l.waiting = true;
    Vehicle_Drive_DW.s_l.timeout = 150.0;
    Vehicle_Drive_DW.s_l.parse = 0.0;
  }

  rtb_valid = (Vehicle_Drive_DW.s_l.configured && (Vehicle_Drive_DW.s_l.age <
    300.0));

  /* MATLABSystem: '<S17>/Digital Port Write' */
  portNameLoc = GPIOB;
  shiftVal = MW_GPIO_BIT_SHIFT;
  if (rtb_valid) {
    session = 128U;
  } else {
    session = 0U;
  }

  session = mw_shift(session, shiftVal);
  dataBytesToRead = mw_shift(128U, shiftVal);
  LL_GPIO_SetOutputPin(portNameLoc, session);
  LL_GPIO_ResetOutputPin(portNameLoc, ~session & dataBytesToRead);

  /* End of MATLABSystem: '<S17>/Digital Port Write' */

  /* Outputs for Enabled SubSystem: '<Root>/IMU Send' incorporates:
   *  EnablePort: '<S5>/Enable'
   */
  if (rtb_count > 0) {
    /* MATLABSystem: '<S5>/UART Write' */
    session = 5U;
    if (rtb_count < 5) {
      session = rtb_count;
    }

    MW_UART_TransmitUsingInterrupt(Vehicle_Drive_DW.obj_g.UARTHandle, &rtb_tx[0],
      session, 0U, &dataBytesToRead);

    /* End of MATLABSystem: '<S5>/UART Write' */
  }

  /* End of Outputs for SubSystem: '<Root>/IMU Send' */

  /* MATLABSystem: '<S19>/PWM Output' incorporates:
   *  MATLAB Function: '<Root>/Motor 1 Direction'
   *  MATLAB Function: '<Root>/Pi Motor Command'
   */
  setDutyCycleInPercentageChannel1(Vehicle_Drive_DW.obj_a.TimerHandle, fmax(0.0,
    Vehicle_Drive_DW.s.actual[0]));
  setDutyCycleInPercentageChannel2(Vehicle_Drive_DW.obj_a.TimerHandle, fmax(0.0,
    -Vehicle_Drive_DW.s.actual[0]));

  /* MATLAB Function: '<Root>/Motor 2 and Encoders' incorporates:
   *  MATLAB Function: '<Root>/Pi Motor Command'
   */
  vehicle_io(Vehicle_Drive_DW.s.actual[1], (uint8_T)servoValid, &rtb_e1, &rtb_e2);
  vehicle_servo_io(Vehicle_Drive_DW.s.servo[0], Vehicle_Drive_DW.s.servo[1]);
  vehicle_servo_status(&rtb_s1, &rtb_s2, &rtb_period);

  /* MATLAB Function: '<Root>/Readable Angles' incorporates:
   *  MATLAB Function: '<Root>/BNO055 Controller'
   *  MATLAB Function: '<Root>/Pi Motor Command'
   */
  for (ttl = 0; ttl < 56; ttl++) {
    base[ttl] = b[ttl];
  }

  for (ttl = 0; ttl < 6; ttl++) {
    text[ttl] = c_0[ttl];
  }

  if (Vehicle_Drive_DW.s_l.h < 0.0) {
    text[0] = 45U;
  }

  ttl = (int32_T)fmin(9999.0, rt_roundd_snf(fabs(Vehicle_Drive_DW.s_l.h) * 10.0));
  text[1] = (uint8_T)((int32_T)floor((real_T)ttl / 1000.0) + 48);
  tmp_7 = rt_roundd_snf(Vehicle_Drive_mod(floor((real_T)ttl / 100.0)) + 48.0);
  if (tmp_7 < 256.0) {
    if (tmp_7 >= 0.0) {
      text[2] = (uint8_T)tmp_7;
    } else {
      text[2] = 0U;
    }
  } else {
    text[2] = MAX_uint8_T;
  }

  tmp_7 = rt_roundd_snf(Vehicle_Drive_mod(floor((real_T)ttl / 10.0)) + 48.0);
  if (tmp_7 < 256.0) {
    if (tmp_7 >= 0.0) {
      text[3] = (uint8_T)tmp_7;
    } else {
      text[3] = 0U;
    }
  } else {
    text[3] = MAX_uint8_T;
  }

  tmp_7 = rt_roundd_snf(Vehicle_Drive_mod((real_T)ttl) + 48.0);
  if (tmp_7 < 256.0) {
    if (tmp_7 >= 0.0) {
      text[5] = (uint8_T)tmp_7;
    } else {
      text[5] = 0U;
    }
  } else {
    text[5] = MAX_uint8_T;
  }

  for (ttl = 0; ttl < 6; ttl++) {
    base[ttl + 2] = text[ttl];
    text[ttl] = c_0[ttl];
  }

  if (Vehicle_Drive_DW.s_l.r < 0.0) {
    text[0] = 45U;
  }

  ttl = (int32_T)fmin(9999.0, rt_roundd_snf(fabs(Vehicle_Drive_DW.s_l.r) * 10.0));
  text[1] = (uint8_T)((int32_T)floor((real_T)ttl / 1000.0) + 48);
  tmp_7 = rt_roundd_snf(Vehicle_Drive_mod(floor((real_T)ttl / 100.0)) + 48.0);
  if (tmp_7 < 256.0) {
    if (tmp_7 >= 0.0) {
      text[2] = (uint8_T)tmp_7;
    } else {
      text[2] = 0U;
    }
  } else {
    text[2] = MAX_uint8_T;
  }

  tmp_7 = rt_roundd_snf(Vehicle_Drive_mod(floor((real_T)ttl / 10.0)) + 48.0);
  if (tmp_7 < 256.0) {
    if (tmp_7 >= 0.0) {
      text[3] = (uint8_T)tmp_7;
    } else {
      text[3] = 0U;
    }
  } else {
    text[3] = MAX_uint8_T;
  }

  tmp_7 = rt_roundd_snf(Vehicle_Drive_mod((real_T)ttl) + 48.0);
  if (tmp_7 < 256.0) {
    if (tmp_7 >= 0.0) {
      text[5] = (uint8_T)tmp_7;
    } else {
      text[5] = 0U;
    }
  } else {
    text[5] = MAX_uint8_T;
  }

  for (ttl = 0; ttl < 6; ttl++) {
    base[ttl + 11] = text[ttl];
    text[ttl] = c_0[ttl];
  }

  if (Vehicle_Drive_DW.s_l.p < 0.0) {
    text[0] = 45U;
  }

  ttl = (int32_T)fmin(9999.0, rt_roundd_snf(fabs(Vehicle_Drive_DW.s_l.p) * 10.0));
  text[1] = (uint8_T)((int32_T)floor((real_T)ttl / 1000.0) + 48);
  tmp_7 = rt_roundd_snf(Vehicle_Drive_mod(floor((real_T)ttl / 100.0)) + 48.0);
  if (tmp_7 < 256.0) {
    if (tmp_7 >= 0.0) {
      text[2] = (uint8_T)tmp_7;
    } else {
      text[2] = 0U;
    }
  } else {
    text[2] = MAX_uint8_T;
  }

  tmp_7 = rt_roundd_snf(Vehicle_Drive_mod(floor((real_T)ttl / 10.0)) + 48.0);
  if (tmp_7 < 256.0) {
    if (tmp_7 >= 0.0) {
      text[3] = (uint8_T)tmp_7;
    } else {
      text[3] = 0U;
    }
  } else {
    text[3] = MAX_uint8_T;
  }

  tmp_7 = rt_roundd_snf(Vehicle_Drive_mod((real_T)ttl) + 48.0);
  if (tmp_7 < 256.0) {
    if (tmp_7 >= 0.0) {
      text[5] = (uint8_T)tmp_7;
    } else {
      text[5] = 0U;
    }
  } else {
    text[5] = MAX_uint8_T;
  }

  for (ttl = 0; ttl < 6; ttl++) {
    base[ttl + 20] = text[ttl];
  }

  base[31] = (uint8_T)((Vehicle_Drive_DW.s_l.cal >> 6) + 48);
  base[33] = (uint8_T)((int32_T)((uint32_T)(Vehicle_Drive_DW.s_l.cal >> 4) & 3U)
                       + 48);
  base[35] = (uint8_T)((int32_T)((uint32_T)(Vehicle_Drive_DW.s_l.cal >> 2) & 3U)
                       + 48);
  base[37] = (uint8_T)((int32_T)(Vehicle_Drive_DW.s_l.cal & 3U) + 48);
  base[45] = (uint8_T)(rtb_valid + 48);
  base[51] = (uint8_T)((int32_T)floor((real_T)Vehicle_Drive_DW.s_l.err / 100.0)
                       + 48);
  tmp_7 = rt_roundd_snf(Vehicle_Drive_mod(floor((real_T)Vehicle_Drive_DW.s_l.err
    / 10.0)) + 48.0);
  if (tmp_7 < 256.0) {
    if (tmp_7 >= 0.0) {
      base[52] = (uint8_T)tmp_7;
    } else {
      base[52] = 0U;
    }
  } else {
    base[52] = MAX_uint8_T;
  }

  tmp_7 = rt_roundd_snf(Vehicle_Drive_mod((real_T)Vehicle_Drive_DW.s_l.err) +
                        48.0);
  if (tmp_7 < 256.0) {
    if (tmp_7 >= 0.0) {
      base[53] = (uint8_T)tmp_7;
    } else {
      base[53] = 0U;
    }
  } else {
    base[53] = MAX_uint8_T;
  }

  for (ttl = 0; ttl < 55; ttl++) {
    tail[ttl] = d[ttl];
  }

  desired = rtb_e1;
  for (ttl = 0; ttl < 5; ttl++) {
    tmp_7 = rt_roundd_snf(Vehicle_Drive_mod(desired) + 48.0);
    if (tmp_7 < 256.0) {
      if (tmp_7 >= 0.0) {
        rtb_tx[4 - ttl] = (uint8_T)tmp_7;
      } else {
        rtb_tx[4 - ttl] = 0U;
      }
    } else {
      rtb_tx[4 - ttl] = MAX_uint8_T;
    }

    desired = floor(desired / 10.0);
  }

  for (ttl = 0; ttl < 5; ttl++) {
    tail[ttl + 4] = rtb_tx[ttl];
  }

  desired = rtb_e2;
  for (ttl = 0; ttl < 5; ttl++) {
    tmp_7 = rt_roundd_snf(Vehicle_Drive_mod(desired) + 48.0);
    if (tmp_7 < 256.0) {
      if (tmp_7 >= 0.0) {
        rtb_tx[4 - ttl] = (uint8_T)tmp_7;
      } else {
        rtb_tx[4 - ttl] = 0U;
      }
    } else {
      rtb_tx[4 - ttl] = MAX_uint8_T;
    }

    desired = floor(desired / 10.0);
  }

  for (ttl = 0; ttl < 5; ttl++) {
    tail[ttl + 13] = rtb_tx[ttl];
  }

  tail[23] = (uint8_T)(servoValid + 48);
  v[0] = (uint16_T)fmin(1000.0, rt_roundd_snf(fabs(Vehicle_Drive_DW.s.actual[0])
    * 10.0));
  v[1] = (uint16_T)fmin(1000.0, rt_roundd_snf(fabs(Vehicle_Drive_DW.s.actual[1])
    * 10.0));
  if (Vehicle_Drive_DW.s.actual[0] < 0.0) {
    tail[28] = 45U;
  }

  if (Vehicle_Drive_DW.s.actual[1] < 0.0) {
    tail[38] = 45U;
  }

  desired = floor((real_T)v[0] / 10.0);
  tmp_7 = rt_roundd_snf(Vehicle_Drive_mod(desired) + 48.0);
  desired = floor(desired / 10.0);
  tmp_5 = rt_roundd_snf(Vehicle_Drive_mod(desired) + 48.0);
  tmp_4 = rt_roundd_snf(Vehicle_Drive_mod(floor(desired / 10.0)) + 48.0);
  if (tmp_4 < 256.0) {
    if (tmp_4 >= 0.0) {
      tail[29] = (uint8_T)tmp_4;
    } else {
      tail[29] = 0U;
    }
  } else {
    tail[29] = MAX_uint8_T;
  }

  if (tmp_5 < 256.0) {
    if (tmp_5 >= 0.0) {
      tail[30] = (uint8_T)tmp_5;
    } else {
      tail[30] = 0U;
    }
  } else {
    tail[30] = MAX_uint8_T;
  }

  if (tmp_7 < 256.0) {
    if (tmp_7 >= 0.0) {
      tail[31] = (uint8_T)tmp_7;
    } else {
      tail[31] = 0U;
    }
  } else {
    tail[31] = MAX_uint8_T;
  }

  tmp_7 = rt_roundd_snf(Vehicle_Drive_mod((real_T)v[0]) + 48.0);
  if (tmp_7 < 256.0) {
    if (tmp_7 >= 0.0) {
      tail[33] = (uint8_T)tmp_7;
    } else {
      tail[33] = 0U;
    }
  } else {
    tail[33] = MAX_uint8_T;
  }

  desired = floor((real_T)v[1] / 10.0);
  tmp_7 = rt_roundd_snf(Vehicle_Drive_mod(desired) + 48.0);
  desired = floor(desired / 10.0);
  tmp_5 = rt_roundd_snf(Vehicle_Drive_mod(desired) + 48.0);
  tmp_4 = rt_roundd_snf(Vehicle_Drive_mod(floor(desired / 10.0)) + 48.0);
  if (tmp_4 < 256.0) {
    if (tmp_4 >= 0.0) {
      tail[39] = (uint8_T)tmp_4;
    } else {
      tail[39] = 0U;
    }
  } else {
    tail[39] = MAX_uint8_T;
  }

  if (tmp_5 < 256.0) {
    if (tmp_5 >= 0.0) {
      tail[40] = (uint8_T)tmp_5;
    } else {
      tail[40] = 0U;
    }
  } else {
    tail[40] = MAX_uint8_T;
  }

  if (tmp_7 < 256.0) {
    if (tmp_7 >= 0.0) {
      tail[41] = (uint8_T)tmp_7;
    } else {
      tail[41] = 0U;
    }
  } else {
    tail[41] = MAX_uint8_T;
  }

  tmp_7 = rt_roundd_snf(Vehicle_Drive_mod((real_T)v[1]) + 48.0);
  if (tmp_7 < 256.0) {
    if (tmp_7 >= 0.0) {
      tail[43] = (uint8_T)tmp_7;
    } else {
      tail[43] = 0U;
    }
  } else {
    tail[43] = MAX_uint8_T;
  }

  tmp_7 = rt_roundd_snf(Vehicle_Drive_mod((real_T)Vehicle_Drive_DW.s.err) + 48.0);
  desired = floor((real_T)Vehicle_Drive_DW.s.err / 10.0);
  tmp_5 = rt_roundd_snf(Vehicle_Drive_mod(desired) + 48.0);
  tmp_4 = rt_roundd_snf(Vehicle_Drive_mod(floor(desired / 10.0)) + 48.0);
  if (tmp_4 < 256.0) {
    if (tmp_4 >= 0.0) {
      tail[52] = (uint8_T)tmp_4;
    } else {
      tail[52] = 0U;
    }
  } else {
    tail[52] = MAX_uint8_T;
  }

  if (tmp_5 < 256.0) {
    if (tmp_5 >= 0.0) {
      tail[53] = (uint8_T)tmp_5;
    } else {
      tail[53] = 0U;
    }
  } else {
    tail[53] = MAX_uint8_T;
  }

  if (tmp_7 < 256.0) {
    if (tmp_7 >= 0.0) {
      tail[54] = (uint8_T)tmp_7;
    } else {
      tail[54] = 0U;
    }
  } else {
    tail[54] = MAX_uint8_T;
  }

  tmp_7 = rt_roundd_snf(Vehicle_Drive_mod((real_T)rtb_s1) + 48.0);
  desired = floor((real_T)rtb_s1 / 10.0);
  tmp_5 = rt_roundd_snf(Vehicle_Drive_mod(desired) + 48.0);
  desired = floor(desired / 10.0);
  tmp_4 = rt_roundd_snf(Vehicle_Drive_mod(desired) + 48.0);
  tmp_3 = rt_roundd_snf(Vehicle_Drive_mod(floor(desired / 10.0)) + 48.0);
  tmp_2 = rt_roundd_snf(Vehicle_Drive_mod((real_T)rtb_s2) + 48.0);
  desired = floor((real_T)rtb_s2 / 10.0);
  tmp_1 = rt_roundd_snf(Vehicle_Drive_mod(desired) + 48.0);
  desired = floor(desired / 10.0);
  tmp_0 = rt_roundd_snf(Vehicle_Drive_mod(desired) + 48.0);
  tmp = rt_roundd_snf(Vehicle_Drive_mod(floor(desired / 10.0)) + 48.0);
  desired = rtb_period;
  for (ttl = 0; ttl < 5; ttl++) {
    tmp_6 = rt_roundd_snf(Vehicle_Drive_mod(desired) + 48.0);
    if (tmp_6 < 256.0) {
      if (tmp_6 >= 0.0) {
        rtb_tx[4 - ttl] = (uint8_T)tmp_6;
      } else {
        rtb_tx[4 - ttl] = 0U;
      }
    } else {
      rtb_tx[4 - ttl] = MAX_uint8_T;
    }

    desired = floor(desired / 10.0);
  }

  current_ina219_status(&j_v, &rtb_e1, &st1, &l_v, &rtb_e2, &rtb_count);
  b_sign = 43U;
  if (j_v < 0) {
    b_sign = 45U;
  }

  desired = fabs((real_T)j_v);
  for (ttl = 0; ttl < 5; ttl++) {
    tmp_6 = rt_roundd_snf(Vehicle_Drive_mod(desired) + 48.0);
    if (tmp_6 < 256.0) {
      if (tmp_6 >= 0.0) {
        g_out[4 - ttl] = (uint8_T)tmp_6;
      } else {
        g_out[4 - ttl] = 0U;
      }
    } else {
      g_out[4 - ttl] = MAX_uint8_T;
    }

    desired = floor(desired / 10.0);
  }

  desired = rtb_e1;
  for (ttl = 0; ttl < 5; ttl++) {
    tmp_6 = rt_roundd_snf(Vehicle_Drive_mod(desired) + 48.0);
    if (tmp_6 < 256.0) {
      if (tmp_6 >= 0.0) {
        e_out[4 - ttl] = (uint8_T)tmp_6;
      } else {
        e_out[4 - ttl] = 0U;
      }
    } else {
      e_out[4 - ttl] = MAX_uint8_T;
    }

    desired = floor(desired / 10.0);
  }

  c_sign = 43U;
  if (l_v < 0) {
    c_sign = 45U;
  }

  desired = fabs((real_T)l_v);
  for (ttl = 0; ttl < 5; ttl++) {
    tmp_6 = rt_roundd_snf(Vehicle_Drive_mod(desired) + 48.0);
    if (tmp_6 < 256.0) {
      if (tmp_6 >= 0.0) {
        h_out[4 - ttl] = (uint8_T)tmp_6;
      } else {
        h_out[4 - ttl] = 0U;
      }
    } else {
      h_out[4 - ttl] = MAX_uint8_T;
    }

    desired = floor(desired / 10.0);
  }

  desired = rtb_e2;
  for (ttl = 0; ttl < 5; ttl++) {
    tmp_6 = rt_roundd_snf(Vehicle_Drive_mod(desired) + 48.0);
    if (tmp_6 < 256.0) {
      if (tmp_6 >= 0.0) {
        f_out[4 - ttl] = (uint8_T)tmp_6;
      } else {
        f_out[4 - ttl] = 0U;
      }
    } else {
      f_out[4 - ttl] = MAX_uint8_T;
    }

    desired = floor(desired / 10.0);
  }

  /* Outputs for Enabled SubSystem: '<Root>/USB Telemetry Send' incorporates:
   *  EnablePort: '<S11>/Enable'
   */
  /* DiscretePulseGenerator: '<Root>/Telemetry 10 Hz' */
  if (((Vehicle_Drive_DW.clockTickCounter < Vehicle_Drive_P.Telemetry10Hz_Duty) &&
       (Vehicle_Drive_DW.clockTickCounter >= 0) ?
       Vehicle_Drive_P.Telemetry10Hz_Amp : 0.0) > 0.0) {
    /* MATLABSystem: '<S11>/UART Write' incorporates:
     *  MATLAB Function: '<Root>/Readable Angles'
     */
    for (ttl = 0; ttl < 54; ttl++) {
      uartWriteData[ttl] = base[ttl];
    }

    for (ttl = 0; ttl < 17; ttl++) {
      uartWriteData[ttl + 54] = e[ttl];
    }

    /* MATLAB Function: '<Root>/Readable Angles' */
    if (tmp_3 < 256.0) {
      if (tmp_3 >= 0.0) {
        /* MATLABSystem: '<S11>/UART Write' */
        uartWriteData[71] = (uint8_T)tmp_3;
      } else {
        /* MATLABSystem: '<S11>/UART Write' */
        uartWriteData[71] = 0U;
      }
    } else {
      /* MATLABSystem: '<S11>/UART Write' */
      uartWriteData[71] = MAX_uint8_T;
    }

    /* MATLABSystem: '<S11>/UART Write' incorporates:
     *  MATLAB Function: '<Root>/Readable Angles'
     */
    uartWriteData[75] = 32U;

    /* MATLAB Function: '<Root>/Readable Angles' */
    if (tmp < 256.0) {
      if (tmp >= 0.0) {
        /* MATLABSystem: '<S11>/UART Write' */
        uartWriteData[79] = (uint8_T)tmp;
      } else {
        /* MATLABSystem: '<S11>/UART Write' */
        uartWriteData[79] = 0U;
      }
    } else {
      /* MATLABSystem: '<S11>/UART Write' */
      uartWriteData[79] = MAX_uint8_T;
    }

    /* MATLABSystem: '<S11>/UART Write' incorporates:
     *  MATLAB Function: '<Root>/Readable Angles'
     */
    uartWriteData[83] = 32U;

    /* MATLAB Function: '<Root>/Readable Angles' */
    if (tmp_4 < 256.0) {
      if (tmp_4 >= 0.0) {
        /* MATLABSystem: '<S11>/UART Write' */
        uartWriteData[72] = (uint8_T)tmp_4;
      } else {
        /* MATLABSystem: '<S11>/UART Write' */
        uartWriteData[72] = 0U;
      }
    } else {
      /* MATLABSystem: '<S11>/UART Write' */
      uartWriteData[72] = MAX_uint8_T;
    }

    /* MATLABSystem: '<S11>/UART Write' incorporates:
     *  MATLAB Function: '<Root>/Readable Angles'
     */
    uartWriteData[76] = 83U;

    /* MATLAB Function: '<Root>/Readable Angles' */
    if (tmp_0 < 256.0) {
      if (tmp_0 >= 0.0) {
        /* MATLABSystem: '<S11>/UART Write' */
        uartWriteData[80] = (uint8_T)tmp_0;
      } else {
        /* MATLABSystem: '<S11>/UART Write' */
        uartWriteData[80] = 0U;
      }
    } else {
      /* MATLABSystem: '<S11>/UART Write' */
      uartWriteData[80] = MAX_uint8_T;
    }

    /* MATLABSystem: '<S11>/UART Write' incorporates:
     *  MATLAB Function: '<Root>/Readable Angles'
     */
    uartWriteData[84] = 84U;

    /* MATLAB Function: '<Root>/Readable Angles' */
    if (tmp_5 < 256.0) {
      if (tmp_5 >= 0.0) {
        /* MATLABSystem: '<S11>/UART Write' */
        uartWriteData[73] = (uint8_T)tmp_5;
      } else {
        /* MATLABSystem: '<S11>/UART Write' */
        uartWriteData[73] = 0U;
      }
    } else {
      /* MATLABSystem: '<S11>/UART Write' */
      uartWriteData[73] = MAX_uint8_T;
    }

    /* MATLABSystem: '<S11>/UART Write' incorporates:
     *  MATLAB Function: '<Root>/Readable Angles'
     */
    uartWriteData[77] = 50U;

    /* MATLAB Function: '<Root>/Readable Angles' */
    if (tmp_1 < 256.0) {
      if (tmp_1 >= 0.0) {
        /* MATLABSystem: '<S11>/UART Write' */
        uartWriteData[81] = (uint8_T)tmp_1;
      } else {
        /* MATLABSystem: '<S11>/UART Write' */
        uartWriteData[81] = 0U;
      }
    } else {
      /* MATLABSystem: '<S11>/UART Write' */
      uartWriteData[81] = MAX_uint8_T;
    }

    /* MATLABSystem: '<S11>/UART Write' incorporates:
     *  MATLAB Function: '<Root>/Readable Angles'
     */
    uartWriteData[85] = 50U;

    /* MATLAB Function: '<Root>/Readable Angles' */
    if (tmp_7 < 256.0) {
      if (tmp_7 >= 0.0) {
        /* MATLABSystem: '<S11>/UART Write' */
        uartWriteData[74] = (uint8_T)tmp_7;
      } else {
        /* MATLABSystem: '<S11>/UART Write' */
        uartWriteData[74] = 0U;
      }
    } else {
      /* MATLABSystem: '<S11>/UART Write' */
      uartWriteData[74] = MAX_uint8_T;
    }

    /* MATLABSystem: '<S11>/UART Write' incorporates:
     *  MATLAB Function: '<Root>/Readable Angles'
     */
    uartWriteData[78] = 61U;

    /* MATLAB Function: '<Root>/Readable Angles' */
    if (tmp_2 < 256.0) {
      if (tmp_2 >= 0.0) {
        /* MATLABSystem: '<S11>/UART Write' */
        uartWriteData[82] = (uint8_T)tmp_2;
      } else {
        /* MATLABSystem: '<S11>/UART Write' */
        uartWriteData[82] = 0U;
      }
    } else {
      /* MATLABSystem: '<S11>/UART Write' */
      uartWriteData[82] = MAX_uint8_T;
    }

    /* MATLABSystem: '<S11>/UART Write' incorporates:
     *  MATLAB Function: '<Root>/Readable Angles'
     */
    uartWriteData[86] = 61U;
    for (ttl = 0; ttl < 5; ttl++) {
      uartWriteData[ttl + 87] = rtb_tx[ttl];
    }

    for (ttl = 0; ttl < 11; ttl++) {
      uartWriteData[ttl + 92] = h[ttl];
    }

    uartWriteData[103] = b_sign;

    /* MATLAB Function: '<Root>/Readable Angles' */
    tmp_7 = rt_roundd_snf(Vehicle_Drive_mod((real_T)st1) + 48.0);
    if (tmp_7 < 256.0) {
      if (tmp_7 >= 0.0) {
        /* MATLABSystem: '<S11>/UART Write' */
        uartWriteData[124] = (uint8_T)tmp_7;
      } else {
        /* MATLABSystem: '<S11>/UART Write' */
        uartWriteData[124] = 0U;
      }
    } else {
      /* MATLABSystem: '<S11>/UART Write' */
      uartWriteData[124] = MAX_uint8_T;
    }

    /* MATLABSystem: '<S11>/UART Write' incorporates:
     *  MATLAB Function: '<Root>/Readable Angles'
     */
    uartWriteData[130] = c_sign;
    for (ttl = 0; ttl < 5; ttl++) {
      uartWriteData[ttl + 104] = g_out[ttl];
      uartWriteData[ttl + 109] = i[ttl];
      uartWriteData[ttl + 114] = e_out[ttl];
      uartWriteData[ttl + 119] = j[ttl];
      uartWriteData[ttl + 125] = l[ttl];
      uartWriteData[ttl + 131] = h_out[ttl];
      uartWriteData[ttl + 136] = m[ttl];
      uartWriteData[ttl + 141] = f_out[ttl];
      uartWriteData[ttl + 146] = n[ttl];
    }

    /* MATLAB Function: '<Root>/Readable Angles' */
    tmp_7 = rt_roundd_snf(Vehicle_Drive_mod((real_T)rtb_count) + 48.0);
    if (tmp_7 < 256.0) {
      if (tmp_7 >= 0.0) {
        /* MATLABSystem: '<S11>/UART Write' */
        uartWriteData[151] = (uint8_T)tmp_7;
      } else {
        /* MATLABSystem: '<S11>/UART Write' */
        uartWriteData[151] = 0U;
      }
    } else {
      /* MATLABSystem: '<S11>/UART Write' */
      uartWriteData[151] = MAX_uint8_T;
    }

    /* MATLABSystem: '<S11>/UART Write' incorporates:
     *  MATLAB Function: '<Root>/Readable Angles'
     */
    for (ttl = 0; ttl < 55; ttl++) {
      uartWriteData[ttl + 152] = tail[ttl];
    }

    uartWriteData[207] = 13U;
    uartWriteData[208] = 10U;
    encoderDiagnosticLength = vehicle_encoder_diag(encoderDiagnostic);
    memmove(&uartWriteData[152 + encoderDiagnosticLength], &uartWriteData[152], 57);
    memcpy(&uartWriteData[152], encoderDiagnostic, encoderDiagnosticLength);
    MW_UART_TransmitUsingInterrupt(Vehicle_Drive_DW.obj.UARTHandle,
      &uartWriteData[0], (uint16_T)(209U + encoderDiagnosticLength), 0U, &dataBytesToRead);
  }

  /* End of DiscretePulseGenerator: '<Root>/Telemetry 10 Hz' */
  /* End of Outputs for SubSystem: '<Root>/USB Telemetry Send' */

  /* Update for DiscretePulseGenerator: '<Root>/Telemetry 10 Hz' */
  if (Vehicle_Drive_DW.clockTickCounter >= Vehicle_Drive_P.Telemetry10Hz_Period
      - 1.0) {
    Vehicle_Drive_DW.clockTickCounter = 0;
  } else {
    Vehicle_Drive_DW.clockTickCounter++;
  }

  /* End of Update for DiscretePulseGenerator: '<Root>/Telemetry 10 Hz' */
}

/* Model initialize function */
void Vehicle_Drive_initialize(void)
{
  /* Registration code */

  /* initialize error status */
  rtmSetErrorStatus(Vehicle_Drive_M, (NULL));

  /* states (dwork) */
  (void) memset((void *)&Vehicle_Drive_DW, 0,
                sizeof(DW_Vehicle_Drive_T));

  {
    int32_T i;

    /* InitializeConditions for DiscretePulseGenerator: '<Root>/Telemetry 10 Hz' */
    Vehicle_Drive_DW.clockTickCounter = 0;

    /* SystemInitialize for MATLAB Function: '<Root>/Pi Motor Command' */
    for (i = 0; i < 24; i++) {
      Vehicle_Drive_DW.s.bytes[i] = 0U;
    }

    Vehicle_Drive_DW.s.session = 0U;
    Vehicle_Drive_DW.s.seq = -1.0;
    Vehicle_Drive_DW.s.armed = false;
    Vehicle_Drive_DW.s.permit = false;
    Vehicle_Drive_DW.s.remaining = 0.0;
    Vehicle_Drive_DW.s.err = 0U;
    Vehicle_Drive_DW.s.target[0] = 0.0;
    Vehicle_Drive_DW.s.actual[0] = 0.0;
    Vehicle_Drive_DW.s.lastSign[0] = 0.0;
    Vehicle_Drive_DW.s.zeroTicks[0] = 500.0;
    Vehicle_Drive_DW.s.servo[0] = 0.0;
    Vehicle_Drive_DW.s.target[1] = 0.0;
    Vehicle_Drive_DW.s.actual[1] = 0.0;
    Vehicle_Drive_DW.s.lastSign[1] = 0.0;
    Vehicle_Drive_DW.s.zeroTicks[1] = 500.0;
    Vehicle_Drive_DW.s.servo[1] = 0.0;

    /* End of SystemInitialize for MATLAB Function: '<Root>/Pi Motor Command' */

    /* SystemInitialize for MATLAB Function: '<Root>/BNO055 Controller' */
    Vehicle_Drive_DW.s_l.delay = 1000.0;
    Vehicle_Drive_DW.s_l.stage = 1.0;
    Vehicle_Drive_DW.s_l.waiting = false;
    Vehicle_Drive_DW.s_l.timeout = 0.0;
    Vehicle_Drive_DW.s_l.parse = 0.0;
    Vehicle_Drive_DW.s_l.b_index = 0.0;
    Vehicle_Drive_DW.s_l.need = 0.0;
    for (i = 0; i < 6; i++) {
      Vehicle_Drive_DW.s_l.payload[i] = 0U;
    }

    Vehicle_Drive_DW.s_l.attempts = 0.0;
    Vehicle_Drive_DW.s_l.h = 0.0;
    Vehicle_Drive_DW.s_l.r = 0.0;
    Vehicle_Drive_DW.s_l.p = 0.0;
    Vehicle_Drive_DW.s_l.cal = 0U;
    Vehicle_Drive_DW.s_l.err = 0U;
    Vehicle_Drive_DW.s_l.age = 1.0E+6;
    Vehicle_Drive_DW.s_l.configured = false;

    /* End of SystemInitialize for MATLAB Function: '<Root>/BNO055 Controller' */

    /* SystemInitialize for Enabled SubSystem: '<Root>/IMU Send' */
    /* Start for MATLABSystem: '<S5>/UART Write' */
    Vehicle_Drive_DW.obj_g.matlabCodegenIsDeleted = false;
    Vehicle_Drive_DW.obj_g.isSetupComplete = false;
    Vehicle_Drive_DW.obj_g.isInitialized = 1;
    Vehicle_Dri_UARTWrite_setupImpl(&Vehicle_Drive_DW.obj_g);
    Vehicle_Drive_DW.obj_g.isSetupComplete = true;

    /* End of SystemInitialize for SubSystem: '<Root>/IMU Send' */

    /* SystemInitialize for Enabled SubSystem: '<Root>/USB Telemetry Send' */
    /* Start for MATLABSystem: '<S11>/UART Write' */
    Vehicle_Drive_DW.obj.matlabCodegenIsDeleted = false;
    Vehicle_Drive_DW.obj.isSetupComplete = false;
    Vehicle_Drive_DW.obj.isInitialized = 1;
    Vehicle_D_UARTWrite_setupImpl_f(&Vehicle_Drive_DW.obj);
    Vehicle_Drive_DW.obj.isSetupComplete = true;

    /* End of SystemInitialize for SubSystem: '<Root>/USB Telemetry Send' */

    /* Start for MATLABSystem: '<Root>/Pi Commands USB' */
    Vehicle_Drive_DW.obj_h.isInitialized = 0;
    Vehicle_Drive_DW.obj_h.matlabCodegenIsDeleted = false;
    Vehicle_Drive_SystemCore_setup(&Vehicle_Drive_DW.obj_h);

    /* Start for MATLABSystem: '<S13>/Digital Port Write' */
    Vehicle_Drive_DW.obj_fr.matlabCodegenIsDeleted = false;
    Vehicle_Drive_DW.obj_fr.isInitialized = 1;
    Vehicle_Drive_DW.obj_fr.isSetupComplete = true;

    /* Start for MATLABSystem: '<S15>/Digital Port Write' */
    Vehicle_Drive_DW.obj_j.matlabCodegenIsDeleted = false;
    Vehicle_Drive_DW.obj_j.isInitialized = 1;
    Vehicle_Drive_DW.obj_j.isSetupComplete = true;

    /* Start for MATLABSystem: '<Root>/UART//USART Read' */
    Vehicle_Drive_DW.obj_f.isInitialized = 0;
    Vehicle_Drive_DW.obj_f.matlabCodegenIsDeleted = false;
    Vehicle_Driv_SystemCore_setup_f(&Vehicle_Drive_DW.obj_f);

    /* Start for MATLABSystem: '<S17>/Digital Port Write' */
    Vehicle_Drive_DW.obj_f5.matlabCodegenIsDeleted = false;
    Vehicle_Drive_DW.obj_f5.isInitialized = 1;
    Vehicle_Drive_DW.obj_f5.isSetupComplete = true;

    /* Start for MATLABSystem: '<S19>/PWM Output' */
    Vehicle_Drive_DW.obj_a.matlabCodegenIsDeleted = false;
    Vehicle_Drive_DW.obj_a.isSetupComplete = false;
    Vehicle_Drive_DW.obj_a.isInitialized = 1;
    Vehicle_Dri_PWMOutput_setupImpl(&Vehicle_Drive_DW.obj_a);
    Vehicle_Drive_DW.obj_a.isSetupComplete = true;
  }
}

/* Model terminate function */
void Vehicle_Drive_terminate(void)
{
  /* Terminate for MATLABSystem: '<Root>/Pi Commands USB' */
  if (!Vehicle_Drive_DW.obj_h.matlabCodegenIsDeleted) {
    Vehicle_Drive_DW.obj_h.matlabCodegenIsDeleted = true;
    if ((Vehicle_Drive_DW.obj_h.isInitialized == 1) &&
        Vehicle_Drive_DW.obj_h.isSetupComplete) {
      MW_UART_DeInit(Vehicle_Drive_DW.obj_h.UARTHandle);
    }
  }

  /* End of Terminate for MATLABSystem: '<Root>/Pi Commands USB' */

  /* Terminate for MATLABSystem: '<S13>/Digital Port Write' */
  if (!Vehicle_Drive_DW.obj_fr.matlabCodegenIsDeleted) {
    Vehicle_Drive_DW.obj_fr.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S13>/Digital Port Write' */

  /* Terminate for MATLABSystem: '<S15>/Digital Port Write' */
  if (!Vehicle_Drive_DW.obj_j.matlabCodegenIsDeleted) {
    Vehicle_Drive_DW.obj_j.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S15>/Digital Port Write' */

  /* Terminate for MATLABSystem: '<Root>/UART//USART Read' */
  if (!Vehicle_Drive_DW.obj_f.matlabCodegenIsDeleted) {
    Vehicle_Drive_DW.obj_f.matlabCodegenIsDeleted = true;
    if ((Vehicle_Drive_DW.obj_f.isInitialized == 1) &&
        Vehicle_Drive_DW.obj_f.isSetupComplete) {
      MW_UART_DeInit(Vehicle_Drive_DW.obj_f.UARTHandle);
    }
  }

  /* End of Terminate for MATLABSystem: '<Root>/UART//USART Read' */

  /* Terminate for MATLABSystem: '<S17>/Digital Port Write' */
  if (!Vehicle_Drive_DW.obj_f5.matlabCodegenIsDeleted) {
    Vehicle_Drive_DW.obj_f5.matlabCodegenIsDeleted = true;
  }

  /* End of Terminate for MATLABSystem: '<S17>/Digital Port Write' */

  /* Terminate for Enabled SubSystem: '<Root>/IMU Send' */
  /* Terminate for MATLABSystem: '<S5>/UART Write' */
  if (!Vehicle_Drive_DW.obj_g.matlabCodegenIsDeleted) {
    Vehicle_Drive_DW.obj_g.matlabCodegenIsDeleted = true;
    if ((Vehicle_Drive_DW.obj_g.isInitialized == 1) &&
        Vehicle_Drive_DW.obj_g.isSetupComplete) {
      MW_UART_DeInit(Vehicle_Drive_DW.obj_g.UARTHandle);
    }
  }

  /* End of Terminate for MATLABSystem: '<S5>/UART Write' */
  /* End of Terminate for SubSystem: '<Root>/IMU Send' */

  /* Terminate for MATLABSystem: '<S19>/PWM Output' */
  if (!Vehicle_Drive_DW.obj_a.matlabCodegenIsDeleted) {
    Vehicle_Drive_DW.obj_a.matlabCodegenIsDeleted = true;
    if ((Vehicle_Drive_DW.obj_a.isInitialized == 1) &&
        Vehicle_Drive_DW.obj_a.isSetupComplete) {
      disableCounter(Vehicle_Drive_DW.obj_a.TimerHandle);
      disableTimerInterrupts(Vehicle_Drive_DW.obj_a.TimerHandle, 0);
      disableTimerChannel1(Vehicle_Drive_DW.obj_a.TimerHandle, ENABLE_CH);
      disableTimerChannel2(Vehicle_Drive_DW.obj_a.TimerHandle, ENABLE_CH);
    }
  }

  /* End of Terminate for MATLABSystem: '<S19>/PWM Output' */

  /* Terminate for Enabled SubSystem: '<Root>/USB Telemetry Send' */
  /* Terminate for MATLABSystem: '<S11>/UART Write' */
  if (!Vehicle_Drive_DW.obj.matlabCodegenIsDeleted) {
    Vehicle_Drive_DW.obj.matlabCodegenIsDeleted = true;
    if ((Vehicle_Drive_DW.obj.isInitialized == 1) &&
        Vehicle_Drive_DW.obj.isSetupComplete) {
      MW_UART_DeInit(Vehicle_Drive_DW.obj.UARTHandle);
    }
  }

  /* End of Terminate for MATLABSystem: '<S11>/UART Write' */
  /* End of Terminate for SubSystem: '<Root>/USB Telemetry Send' */
}
