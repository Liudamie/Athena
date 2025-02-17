/*
 * relative_localization.c
 *
 *  Created on: Feb 17, 2025
 *      Author: Liu
 */

#include "relative_localization.h"
#include "swarm_ranging.h"
#include "arm_math.h"

static uint16_t MY_UWB_ADDRESS;
static bool isInit;

static float Qv = 0.2f;
static float Qr = 0.1f;
static float Ruwb = 0.2f;
static float InitCovPos = 0.01;
static float InitCovYaw = 0.2f;

static relaVariable_t ralaVar[RANGING_TABLE_SIZE_MAX];

static float A[STATE_DIM_rl][STATE_DIM_rl];
static float h[STATE_DIM_rl] = {0};
static arm_matrix_instance_f32 H = {1, STATE_DIM_rl, h};
static arm_matrix_instance_f32 Am = {STATE_DIM_rl, STATE_DIM_rl, (float *)A};

// Temporary matrices for the covariance updates
static float tmpNN1d[STATE_DIM_rl * STATE_DIM_rl];
static arm_matrix_instance_f32 tmpNN1m = {STATE_DIM_rl, STATE_DIM_rl, tmpNN1d};
static float tmpNN2d[STATE_DIM_rl * STATE_DIM_rl];
static arm_matrix_instance_f32 tmpNN2m = {STATE_DIM_rl, STATE_DIM_rl, tmpNN2d};
static float K[STATE_DIM_rl];
static arm_matrix_instance_f32 Km = {STATE_DIM_rl, 1, (float *)K};
static float tmpNN3d[STATE_DIM_rl * STATE_DIM_rl];
static arm_matrix_instance_f32 tmpNN3m = {STATE_DIM_rl, STATE_DIM_rl, tmpNN3d};
static float HTd[STATE_DIM_rl * 1];
static arm_matrix_instance_f32 HTm = {STATE_DIM_rl, 1, HTd};
static float PHTd[STATE_DIM_rl * 1];
static arm_matrix_instance_f32 PHTm = {STATE_DIM_rl, 1, PHTd};

static bool fullConnect = false;
static uint32_t connectCount = 0;

static short vxj_t, vyj_t;
static short vxi_t, vyi_t;
static uint16_t hi_t, hj_t;

static float vxj, vyj, rj;
static float vxi, vyi, ri;
static uint16_t dij;
static float hi, hj;

const float initDist = 1;
const float doubInitDist = 2;
static const float initPositionRela0[25][STATE_DIM_rl] = {
    {0.0f, 0.0f, 0.0f},               // 0
    {0.0f, -initDist, 0.0f},          // 1
    {-initDist, -initDist, 0.0f},     // 2
    {-initDist, 0.0f, 0.0f},          // 3
    {-initDist, initDist, 0.0f},      // 4
    {0.0f, initDist, 0.0f},           // 5
    {initDist, initDist, 0.0f},       // 6
    {initDist, 0.0f, 0.0f},           // 7
    {initDist, -initDist, 0.0f},      // 8
    {doubInitDist, initDist, 0.0f},   // 9
    {doubInitDist, 0.0f, 0.0f},       // 10
    {doubInitDist, -initDist, 0.0f},  // 11
    {initDist, -doubInitDist, 0.0f},  // 12
    {0.0f, -doubInitDist, 0.0f},      // 13
    {-initDist, -doubInitDist, 0.0f}, // 14
    {-doubInitDist, -initDist, 0.0f}, // 15
    {-doubInitDist, 0.0f, 0.0f},      // 16
    {-doubInitDist, initDist, 0.0f},  // 17
    {-initDist, doubInitDist, 0.0f},  // 18
    {0.0f, doubInitDist, 0.0f},       // 19
    {initDist, doubInitDist, 0.0f},   // 20
    {0.0f, 0.0f, 0.0f}};

// 矩阵转置
static inline void mat_trans(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst)
{
    configASSERT(ARM_MATH_SUCCESS == arm_mat_trans_f32(pSrc, pDst));
}
// 矩阵求逆
static inline void mat_inv(const arm_matrix_instance_f32 *pSrc, arm_matrix_instance_f32 *pDst)
{
    configASSERT(ARM_MATH_SUCCESS == arm_mat_inverse_f32(pSrc, pDst));
}
// 矩阵相乘
static inline void mat_mult(const arm_matrix_instance_f32 *pSrcA, const arm_matrix_instance_f32 *pSrcB, arm_matrix_instance_f32 *pDst)
{
    configASSERT(ARM_MATH_SUCCESS == arm_mat_mult_f32(pSrcA, pSrcB, pDst));
}
// 求根号
static inline float arm_sqrt(float32_t in)
{
    float pOut = 0;
    arm_status result = arm_sqrt_f32(in, &pOut);
    configASSERT(ARM_MATH_SUCCESS == result);
    return pOut;
}

void relativeLocoInit(void){
	if(isInit){
		return ;
	}
	MY_UWB_ADDRESS = uwbGetAddress();
	xTaskCreate( relativeLocoTask, "relative_Localization",2*150*sizeof(StackType_t) ,NULL,osPriorityNormal,NULL);
	//TODO 既然线程是新建的栈空间，那么完全可以少创建一点（loco_Fly这个线程）
	isInit = true;
}

void relaVarInit(relaVariable_t *relaVar, uint16_t neighborAddress){
	for(int i=0; i<STATE_DIM_rl;i++){
		for(int j=0;j<STATE_DIM_rl;j++){
			relaVar[neighborAddress].P[i][j] = 0;
		}
	}
	relaVar[neighborAddress].P[STATE_rlX][STATE_rlX]= InitCovPos;
	relaVar[neighborAddress].P[STATE_rlY][STATE_rlY]= InitCovPos;
	relaVar[neighborAddress].P[STATE_rlYaw][STATE_rlYaw]= InitCovYaw;
	relaVar[neighborAddress].S[STATE_rlX]= initPositionRela0[neighborAddress][STATE_rlX]-initPositionRela0[MY_UWB_ADDRESS][STATE_rlX];
	relaVar[neighborAddress].S[STATE_rlY]= initPositionRela0[neighborAddress][STATE_rlY]-initPositionRela0[MY_UWB_ADDRESS][STATE_rlY];
	relaVar[neighborAddress].S[STATE_rlYaw] = 0;
	relaVar[neighborAddress].oldTimetick = xTaskGetTickCount();
	fullConnect = true;


}

void  relativeLocoTask(void *arg){

}

