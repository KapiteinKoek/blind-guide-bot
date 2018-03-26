
#define S_FUNCTION_NAME blindguide
#define S_FUNCTION_LEVEL 2

/*
 * Copyright 2018 Anne Kolmans, Dylan ter Veen, Jarno Brils, Ren??e van Hijfte, and Thomas Wiepking (TU/e Project Robots Everywhere 2017/2018 Q3 Group 12)
 *
 *  Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "simstruc.h"
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

/* include h-files */
#include "Simulink/Bus/busses/bus.h"
#include "Global_par/constants.h"
#include "GeneralFunctions/generic_functions.h"
#include "blindguide.h"

Coordinate createCoordinate(double x, double y) {
    Coordinate c;
    c.x = x;
    c.y = y;
    return c;
}

Borderline createBorderline(Coordinate bottom, Coordinate top, enum side goodSide) {
    Borderline bl;
    bl.bottom = bottom;
    bl.top = top;
    bl.length = createVector(top.x - bottom.x, top.y - bottom.y).length;
    bl.goodSide = goodSide;
    return bl;
}

Vector createVector(double x, double y) {
    Vector v;
    populateVector(x, y, &v);
    return v;
}

/***************************
 * Input Ports definitions *
 ***************************/
#define NINPUTS     3                   /* Number of input ports (0...)*/
#define NINPUTS0    POSETYPESIZE                   /* cur x y o */
#define NINPUTS1    3                   /* Jerrel's Forces */
#define NINPUTS2    48                   /* Ball xyzdxdydz */
double NINPUTS_BGuide[NINPUTS] =  {NINPUTS0,NINPUTS1,NINPUTS2};


/****************************
 * Output Ports definitions *
 ****************************/
#define NOUTPUTS    1
#define NOUTPUTS0   1              /* Resistance */

/*************************************************************************/
static void mdlInitializeSizes(SimStruct *S)
{
    int i,n1, n2, Rworksize;
    
    ssSetNumSFcnParams(S, 0);
    if (ssGetNumSFcnParams(S) != ssGetSFcnParamsCount(S)) {
        return;
    }
    
    /***************************
     * Input Ports definitions *
     ***************************/
    if (!ssSetNumInputPorts(S,NINPUTS)) return;
    /* input port i */
    ssSetInputPortWidth(S,0,NINPUTS_BGuide[0]);
    ssSetInputPortDirectFeedThrough(S,0,1);
    ssSetInputPortRequiredContiguous(S,0,1);
    ssSetInputPortDataType(S,0,SS_INT8);
    /* input port i */
    ssSetInputPortWidth(S,1,NINPUTS_BGuide[1]);
    ssSetInputPortDirectFeedThrough(S,1,1);
    ssSetInputPortRequiredContiguous(S,1,1);
    ssSetInputPortDataType(S,1,SS_DOUBLE);
    /* input port i */
    ssSetInputPortWidth(S,2,NINPUTS_BGuide[2]);
    ssSetInputPortDirectFeedThrough(S,2,1);
    ssSetInputPortRequiredContiguous(S,2,1);
    ssSetInputPortDataType(S,2,SS_INT8);

    
    /****************************
     * Output Ports definitions *
     ****************************/
    if (!ssSetNumOutputPorts(S, NOUTPUTS)) return;
    ssSetOutputPortWidth(S,0,NOUTPUTS0);
    ssSetOutputPortDataType(S,0,SS_DOUBLE);
    

    /***********************
     * Definition of States *
     ************************/
    ssSetNumContStates(S, 0);
    ssSetNumDiscStates(S, 0);
    
    /**********************
     * Default Definitions *
     **********************/
    ssSetNumSampleTimes(S, 1);

    ssSetNumRWork(S, 0);
    ssSetNumIWork(S, 0);
    ssSetNumPWork(S, 0);
    ssSetNumModes(S, 0);
}
    
/*********************************************************************************************/
#if defined(MATLAB_MEX_FILE)
#define MDL_SET_INPUT_PORT_DIMENSION_INFO
static void mdlSetInputPortDimensionInfo(SimStruct        *S,
int_T            port,
const DimsInfo_T *dimsInfo)
{
    if(!ssSetInputPortDimensionInfo(S, port, dimsInfo)) return;
}
#endif

#define MDL_SET_OUTPUT_PORT_DIMENSION_INFO
#if defined(MDL_SET_OUTPUT_PORT_DIMENSION_INFO)
static void mdlSetOutputPortDimensionInfo(SimStruct        *S,
int_T            port,
const DimsInfo_T *dimsInfo)
{
    if (!ssSetOutputPortDimensionInfo(S, port, dimsInfo)) return;
}

#endif

/*************************************************************************/
static void mdlInitializeSampleTimes(SimStruct *S)
{
    ssSetSampleTime(S, 0, INHERITED_SAMPLE_TIME);
    ssSetOffsetTime(S, 0, FIXED_IN_MINOR_STEP_OFFSET);
}

/*************************************************************************/
#define MDL_INITIALIZE_CONDITIONS
static void mdlInitializeConditions(SimStruct *S)
{
#ifndef MATLAB_MEX_FILE
#endif
}

/*************************************************************************/
static void mdlOutputs(SimStruct *S, int_T tid)
{
    /* Input Ports */
    pPose_t cur_xyo = (pPose_t)ssGetInputPortRealSignalPtrs(S,0);             /* motionbus */
    double* Fvec    = (double*)ssGetInputPortRealSignalPtrs(S,1);
    pBall_t ball    = (pBall_t)ssGetInputPortRealSignalPtrs(S,2);

    /* Output Ports */
    double* resistance      = (double*)ssGetOutputPortSignal(S,0);
   
    initializeBorders();
    *resistance = getResistance(cur_xyo->x, cur_xyo->y, cur_xyo->o, Fvec[0], Fvec[1], 1, ball->pos.arr);
    cleanup();
    
}
/*************************************************************************/
static void mdlTerminate(SimStruct *S)
{

}

#ifdef  MATLAB_MEX_FILE    /* Is this file being compiled as a MEX-file? */
#include "simulink.c"      /* MEX-file interface mechanism */
#else
#include "cg_sfun.h"       /* Code generation registration function */
#endif
