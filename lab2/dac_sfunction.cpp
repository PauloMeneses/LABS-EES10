/**
  ******************************************************************************
  * @file    daq_sfunction.cpp
  * @author  José Roberto Colombo Junior
  * @brief   An implementation of the DAQ driver to the Simulink interface.
  ******************************************************************************
  * @attention This is free software licensed under GPL.
  *
  * @note A good source of information about Simulink execution sequence:
  * https://www.mathworks.com/help/simulink/sfg/how-the-simulink-engine-interacts-with-c-s-functions.html
  *
  ******************************************************************************
  */

#include <iostream>
#include <cstring>
#include <cmath>

// S-Function information
#define S_FUNCTION_NAME  dacSfunction
#define S_FUNCTION_LEVEL 2
#include "simstruc.h"

// Application libraries
#include "dac_driver.hpp"

// Definitions
constexpr bool disableAutoSync = false;

typedef enum {
    inPortGpioOutCh1 = 0,
    inPortGpioOutCh2,
    inPortGpioOutCh3,
    inPortPwmCh1,
    inPortPwmCh2,
    inPortPwmCh3,
    inPortPwmCh4,
    numberOfInputPorts
} inputPort;

typedef enum {
    outPortGpioInCh1 = 0,
    outPortGpioInCh2,
    outPortGpioInCh3,
    outPortAdCh1,
    outPortAdCh2,
    outPortAdCh3,
    outPortAdCh4,
    outPortEnc1,
    outPortEnc2,
    outPortEnc3,
    outPortRealTime,
    numberOfOutputPorts
} outputPort;

typedef enum {
    positionSerialPort = 0,
    positionSampleTime,
    positionGpioOutCh1FinalState,
    positionGpioOutCh2FinalState,
    positionGpioOutCh3FinalState,
    positionPwmFrequency,
    positionPwmCh1FinalDuty,
    positionPwmCh2FinalDuty,
    positionPwmCh3FinalDuty,
    positionPwmCh4FinalDuty,
    positionEnc1Mode,
    positionEnc1ResetCount,
    positionEnc1Direction,
    positionEnc1Quadrature,
    positionEnc1Filter,
    positionEnc2Mode,
    positionEnc2ResetCount,
    positionEnc2Direction,
    positionEnc2Quadrature,
    positionEnc2Filter,
    positionEnc3Mode,
    positionEnc3ResetCount,
    positionEnc3Direction,
    positionEnc3Quadrature,
    positionEnc3Filter,
    numberOfParameters
} parameterPosition;

typedef struct boardConfig {
    // General config
    char serialPort[20];
    
    // Sample time
    uint64_t sampleTimeMs;
    uint64_t firstInstantTimeUs;

    // Gpio out
    uint8_t gpioOutCh1FinalState;
    uint8_t gpioOutCh2FinalState;
    uint8_t gpioOutCh3FinalState;

    // PWM
    double pwmFrequencyHz;
    double pwmCh1FinalDuty;
    double pwmCh2FinalDuty;
    double pwmCh3FinalDuty;
    double pwmCh4FinalDuty;

    // Encoder 1
    EncoderMode enc1Mode;
    bool enc1ResetCount;
    EncoderDirection enc1Direction;
    EncoderResolution enc1Quadrature;
    unsigned int enc1Filter;

    // Encoder 2
    EncoderMode enc2Mode;
    bool enc2ResetCount;
    EncoderDirection enc2Direction;
    EncoderResolution enc2Quadrature;
    unsigned int enc2Filter;

    // Encoder 3
    EncoderMode enc3Mode;
    bool enc3ResetCount;
    EncoderDirection enc3Direction;
    EncoderResolution enc3Quadrature;
    unsigned int enc3Filter;
} boardConfig_t;

volatile static bool waiting;
void callbackReceivedData()
{
    /*
     * This function will be called automatically by the C++ driver. This function will allow the Simulink to run.
     */
    waiting = false;
    (void) waiting;
}

/*
double escLengthToDuty(const boardConfig_t* config, double length_us)
{
    // limit to esc1MinPulseLengthUs <= length_us <= esc1MaxPulseLengthUs
    length_us = fmax(length_us, config->esc1MinPulseLengthUs);
    length_us = fmin(length_us, config->esc1MaxPulseLengthUs);
    return config->pwmFrequencyHz * (length_us / 1000000.0f);
}
*/

/*
 * The sizes information is used by Simulink to determine the S-function
 * block's characteristics (number of inputs, outputs, states, etc.).
 */
static void mdlInitializeSizes(SimStruct *S)
{
    // Auxiliary variables
    uint32_t i;

    // Set the expected number parameters
    ssSetNumSFcnParams(S, parameterPosition::numberOfParameters);

    // Verify parameter count
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return; /* Parameter mismatch will be reported by Simulink */
    }

    // Any parameter is not tunable because they will be used to configure the board
    for (i = 0; i < parameterPosition::numberOfParameters; i++)
        ssSetSFcnParamTunable(S, i, SS_PRM_NOT_TUNABLE);

    // Set the number of continuous and discrete states of the block
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 1);

    // Set the input ports and remove direct feed through
    if (!ssSetNumInputPorts(S, inputPort::numberOfInputPorts)) return;
    for (i = 0; i < inputPort::numberOfInputPorts; i++)
    {
        ssSetInputPortWidth(S, i, 1);
        ssSetInputPortDirectFeedThrough(S, i, 0);
    }

    // All the output ports have one dimension
    if (!ssSetNumOutputPorts(S, outputPort::numberOfOutputPorts)) return;
    for (i = 0; i < outputPort::numberOfOutputPorts; i++)
        ssSetOutputPortWidth(S, i, 1);

    ssSetNumSampleTimes(S, 1);
    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    // Reserve two elements: the struct with parameter and the quark driver object
    ssSetNumPWork(S, 2);
    ssSetNumModes(S, 0);
    ssSetNumNonsampledZCs(S, 0);

    ssSetOptions(
        S, 
        SS_OPTION_EXCEPTION_FREE_CODE       // it doesn't use routines which can throw exceptions
        | SS_OPTION_DISCRETE_VALUED_OUTPUT
        | SS_OPTION_PLACE_ASAP              // typically used by devices connecting to hardware
    );
}

#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct* S)
{
    real_T* x0 = ssGetRealDiscStates(S);
    int i;

    for (i = 0; i < ssGetNumDiscStates(S); i++)
        x0[i] = 0.0;
}

/*
 * This function is used to specify the sample time(s) for your
 * S-function. You must register the same number of sample times as
 * specified in ssSetNumSampleTimes.
 */
static void mdlInitializeSampleTimes(SimStruct *S)
{
    const double sampleTimeSecs = mxGetScalar(ssGetSFcnParam(S, positionSampleTime)) / 1000;
    ssSetSampleTime(S, 0, sampleTimeSecs);
    ssSetOffsetTime(S, 0, 0.0);
    ssSetModelReferenceSampleTimeDefaultInheritance(S);
}

// Change to #undef to remove function
#define MDL_START
static void mdlStart(SimStruct *S)
{
    // Reserve memory to the struct of configuration options
    auto config = new boardConfig_t;
    ssGetPWork(S)[0] = reinterpret_cast<void*>(config);

    // Try to open the board
    strcpy(config->serialPort, mxArrayToString(ssGetSFcnParam(S, positionSerialPort)));
    config->sampleTimeMs = static_cast<uint64_t>(mxGetScalar(ssGetSFcnParam(S, positionSampleTime)));
    auto board = new DacUsb(config->serialPort, disableAutoSync);
    ssGetPWork(S)[1] = reinterpret_cast<void*>(board);

    if (board->isConnected())
    {
        // Copy configuration parameters
        config->gpioOutCh1FinalState = static_cast<uint8_t>(mxGetScalar(ssGetSFcnParam(S, positionGpioOutCh1FinalState)) - 1);
        config->gpioOutCh2FinalState = static_cast<uint8_t>(mxGetScalar(ssGetSFcnParam(S, positionGpioOutCh2FinalState)) - 1);
        config->gpioOutCh3FinalState = static_cast<uint8_t>(mxGetScalar(ssGetSFcnParam(S, positionGpioOutCh3FinalState)) - 1);
        config->pwmFrequencyHz = mxGetScalar(ssGetSFcnParam(S, positionPwmFrequency));
        config->pwmCh1FinalDuty = mxGetScalar(ssGetSFcnParam(S, positionPwmCh1FinalDuty));
        config->pwmCh2FinalDuty = mxGetScalar(ssGetSFcnParam(S, positionPwmCh2FinalDuty));
        config->pwmCh3FinalDuty = mxGetScalar(ssGetSFcnParam(S, positionPwmCh3FinalDuty));
        config->pwmCh4FinalDuty = mxGetScalar(ssGetSFcnParam(S, positionPwmCh4FinalDuty));
        config->enc1Mode = static_cast<EncoderMode>(mxGetScalar(ssGetSFcnParam(S, positionEnc1Mode)) - 1);
        config->enc1ResetCount = mxGetScalar(ssGetSFcnParam(S, positionEnc1ResetCount)) > 0;
        config->enc1Direction = static_cast<EncoderDirection>(mxGetScalar(ssGetSFcnParam(S, positionEnc1Direction)) - 1);
        config->enc1Quadrature = static_cast<EncoderResolution>(mxGetScalar(ssGetSFcnParam(S, positionEnc1Quadrature)) - 1);
        config->enc1Filter = static_cast<unsigned int>(mxGetScalar(ssGetSFcnParam(S, positionEnc1Filter)));
        config->enc2Mode = static_cast<EncoderMode>(mxGetScalar(ssGetSFcnParam(S, positionEnc2Mode)) - 1);
        config->enc2ResetCount = mxGetScalar(ssGetSFcnParam(S, positionEnc2ResetCount)) > 0;
        config->enc2Direction = static_cast<EncoderDirection>(mxGetScalar(ssGetSFcnParam(S, positionEnc2Direction)) - 1);
        config->enc2Quadrature = static_cast<EncoderResolution>(mxGetScalar(ssGetSFcnParam(S, positionEnc2Quadrature)) - 1);
        config->enc2Filter = static_cast<unsigned int>(mxGetScalar(ssGetSFcnParam(S, positionEnc2Filter)));
        config->enc3Mode = static_cast<EncoderMode>(mxGetScalar(ssGetSFcnParam(S, positionEnc3Mode)) - 1);
        config->enc3ResetCount = mxGetScalar(ssGetSFcnParam(S, positionEnc3ResetCount)) > 0;
        config->enc3Direction = static_cast<EncoderDirection>(mxGetScalar(ssGetSFcnParam(S, positionEnc3Direction)) - 1);
        config->enc3Quadrature = static_cast<EncoderResolution>(mxGetScalar(ssGetSFcnParam(S, positionEnc3Quadrature)) - 1);
        config->enc3Filter = static_cast<unsigned int>(mxGetScalar(ssGetSFcnParam(S, positionEnc3Filter)));

        // Set the PWM frequency
        board->pwmSetFrequency(config->pwmFrequencyHz);

        // Apply the encoder 1 configuration
        board->encoderSetMode(1, config->enc1Mode);
        board->encoderSetDirection(1, config->enc1Direction);
        board->encoderSetResolution(1, config->enc1Quadrature);
        board->encoderSetFilter(1, config->enc1Filter);
        if (config->enc1ResetCount) board->encoderReset(1);

        // Apply the encoder 2 configuration
        board->encoderSetMode(2, config->enc2Mode);
        board->encoderSetDirection(2, config->enc2Direction);
        board->encoderSetResolution(2, config->enc2Quadrature);
        board->encoderSetFilter(2, config->enc2Filter);
        if (config->enc2ResetCount) board->encoderReset(2);

        // Apply the encoder 3 configuration
        board->encoderSetMode(3, config->enc3Mode);
        board->encoderSetDirection(3, config->enc3Direction);
        board->encoderSetResolution(3, config->enc3Quadrature);
        board->encoderSetFilter(3, config->enc3Filter);
        if (config->enc3ResetCount) board->encoderReset(3);

        // Enable the auto-read
        board->enableAutoRead(0.001 * static_cast<double>(config->sampleTimeMs), callbackReceivedData);
        board->sync();
    }
}

#define MDL_UPDATE
static void mdlUpdate(SimStruct* S, int_T tid)
{
    // Auxiliary variables
    UNUSED_ARG(tid);
    const InputRealPtrsType GpioOutCh1 = ssGetInputPortRealSignalPtrs(S, inPortGpioOutCh1);
    const InputRealPtrsType GpioOutCh2 = ssGetInputPortRealSignalPtrs(S, inPortGpioOutCh2);
    const InputRealPtrsType GpioOutCh3 = ssGetInputPortRealSignalPtrs(S, inPortGpioOutCh3);
    const InputRealPtrsType PwmCh1 = ssGetInputPortRealSignalPtrs(S, inPortPwmCh1);
    const InputRealPtrsType PwmCh2 = ssGetInputPortRealSignalPtrs(S, inPortPwmCh2);
    const InputRealPtrsType PwmCh3 = ssGetInputPortRealSignalPtrs(S, inPortPwmCh3);
    const InputRealPtrsType PwmCh4 = ssGetInputPortRealSignalPtrs(S, inPortPwmCh4);
    real_T* block_state = ssGetRealDiscStates(S);

    // Load the board configuration and board driver
    auto config = reinterpret_cast<boardConfig_t*>(ssGetPWork(S)[0]);
    auto board = reinterpret_cast<DacUsb*>(ssGetPWork(S)[1]);

    // Handle the gpio output
    board->digitalWrite(CH1, static_cast<PinState>(*GpioOutCh1[0] > 0));
    board->digitalWrite(CH2, static_cast<PinState>(*GpioOutCh2[0] > 0));
    board->digitalWrite(CH3, static_cast<PinState>(*GpioOutCh3[0] > 0));

    // Handle the pwm/esc
    board->pwmSetDuty(CH1, *PwmCh1[0]);
    board->pwmSetDuty(CH2, *PwmCh2[0]);
    board->pwmSetDuty(CH3, *PwmCh3[0]);
    board->pwmSetDuty(CH4, *PwmCh4[0]);

    // Sync the board
    board->sync();

    // Increase mdlUpdate count (used to force simulink to run this function)
    block_state[0] = block_state[0] + 1;

    // Wait until next sampling time
    waiting = true;
    const uint64_t timeoutUs = getTimeUs() + (1100 * config->sampleTimeMs);
    while ((waiting) && (getTimeUs() < timeoutUs)) {}
}

/* Function: mdlOutputs =======================================================
 * Abstract:
 *    In this function, you compute the outputs of your S-function
 *    block.
 */
static void mdlOutputs(SimStruct *S, int_T tid)
{
    // Auxiliary variables
    (void)tid;
    real_T* yGpioInCh1 = ssGetOutputPortRealSignal(S, outPortGpioInCh1);
    real_T* yGpioInCh2 = ssGetOutputPortRealSignal(S, outPortGpioInCh2);
    real_T* yGpioInCh3 = ssGetOutputPortRealSignal(S, outPortGpioInCh3);
    real_T* yAdCh1 = ssGetOutputPortRealSignal(S, outPortAdCh1);
    real_T* yAdCh2 = ssGetOutputPortRealSignal(S, outPortAdCh2);
    real_T* yAdCh3 = ssGetOutputPortRealSignal(S, outPortAdCh3);
    real_T* yAdCh4 = ssGetOutputPortRealSignal(S, outPortAdCh4);
    real_T* yEnc1 = ssGetOutputPortRealSignal(S, outPortEnc1);
    real_T* yEnc2 = ssGetOutputPortRealSignal(S, outPortEnc2);
    real_T* yEnc3 = ssGetOutputPortRealSignal(S, outPortEnc3);
    real_T* yRealTime = ssGetOutputPortRealSignal(S, outPortRealTime);
    time_T simulation_time = ssGetT(S);

    // Load the board configuration and board driver
    auto config = reinterpret_cast<boardConfig_t *>(ssGetPWork(S)[0]);
    auto board = reinterpret_cast<DacUsb*>(ssGetPWork(S)[1]);

    // Handle digital input CH1, CH2, CH3 and CH4
    *yGpioInCh1 = static_cast<real_T>(board->digitalRead(CH1));
    *yGpioInCh2 = static_cast<real_T>(board->digitalRead(CH2));
    *yGpioInCh3 = static_cast<real_T>(board->digitalRead(CH3));

    // Handle encoders 1 and 2
    *yEnc1 = static_cast<real_T>(board->encoderRead(1));
    *yEnc2 = static_cast<real_T>(board->encoderRead(2));
    *yEnc3 = static_cast<real_T>(board->encoderRead(3));

    // Handle A/D CH1, CH2, CH3 and CH4
    *yAdCh1 = static_cast<real_T>(board->analogRead(CH1));
    *yAdCh2 = static_cast<real_T>(board->analogRead(CH2));
    *yAdCh3 = static_cast<real_T>(board->analogRead(CH3));
    *yAdCh4 = static_cast<real_T>(board->analogRead(CH4));

    // Initialize the first instant time
    if (simulation_time == 0)
    {
        config->firstInstantTimeUs = getTimeUs();
    }

    // Put this time instant in output port
    *yRealTime = 1e-6 * static_cast<double>(getTimeUs() - config->firstInstantTimeUs);
}

/* Function: mdlTerminate =====================================================
 * Abstract:
 *    In this function, you should perform any actions that are necessary
 *    at the termination of a simulation.  For example, if memory was
 *    allocated in mdlStart, this is the place to free it.
 */
static void mdlTerminate(SimStruct *S)
{
    auto config = reinterpret_cast<boardConfig_t*>(ssGetPWork(S)[0]);
    auto board = reinterpret_cast<DacUsb*>(ssGetPWork(S)[1]);

    // Apply the final gpio state
    board->digitalWrite(CH1, static_cast<PinState>(config->gpioOutCh1FinalState));
    board->digitalWrite(CH2, static_cast<PinState>(config->gpioOutCh2FinalState));
    board->digitalWrite(CH3, static_cast<PinState>(config->gpioOutCh3FinalState));

    // Apply the final duty cycle
    board->pwmSetDuty(CH1, config->pwmCh1FinalDuty);
    board->pwmSetDuty(CH2, config->pwmCh2FinalDuty);
    board->pwmSetDuty(CH3, config->pwmCh3FinalDuty);
    board->pwmSetDuty(CH4, config->pwmCh4FinalDuty);
    board->sync();

    while (board->isAutoReadEnabled())
    {
        board->disableAutoRead();
        board->sync();
        delayMs(10);
    }
    delete board;
    delete config;
}

/*=============================*
 * Required S-function trailer *
 *=============================*/

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif

