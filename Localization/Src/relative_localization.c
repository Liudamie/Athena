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
QueueHandle_t droneQueue;

static float Qv = 0.2f;
static float Qr = 0.1f;
static float Ruwb = 0.2f;
static float InitCovPos = 0.01;
static float InitCovYaw = 0.2f;

static relaVariable_t relaVar[RANGING_TABLE_SIZE_MAX];

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

static currentNeighborAddressInfo_t currentNeighborAddressInfo;

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
	while(1){
		vTaskDelay(10);
		getCurrentNeighborAddressInfo_t(&currentNeighborAddressInfo);

		for(int index =0; index < currentNeighborAddressInfo.size;index++){
			bool isNewAdd;
			uint16_t neighborAddress = currentNeighborAddressInfo.address[index];
			if(getNeighborStateInfo(neighborAddress, &dij, &vxj_t,vyj_t, &rj, &hj_t, &isNewAdd)){
				vxj = (vxj_t + 0.0 )/100;
				vyj = (vyj_t + 0.0 )/100;
				hj = (hj_t + 0.0 )/100;
				if(isNewAdd){
					relaVarInit(relaVar, neighborAddress);
					//TODO 设置newadd为false;
				}else{
					//TODO 获取自身无人机的IMU数据 estimatorKalmanGetSwarmInfo();
					vxi = (vxi_t + 0.0)/100;
					vyi = (vyi_t +0.0 )/100;
					hi = (hi_t +0.0 )/100;
					uint32_t osTick = xTaskGetTickCount();
					float dtEKF = (float)(osTick - relaVar[neighborAddress].oldTimetick) / configTICK_RATE_HZ;
					relaVar[neighborAddress].oldTimetick = osTick;
					relaVar[neighborAddress].height = hj;
					relativeEKF(neighborAddress, vxi, vyi, ri, hi, vxj, vyj, rj, hj, dij, dtEKF);
				}
			}
		}
	}

}
void relativeEKF(int n, float vxi, float vyi, float ri, float hi, float vxj, float vyj, float rj, float hj, uint16_t dij, float dt)
{
    // some preprocessing
    arm_matrix_instance_f32 Pm = {STATE_DIM_rl, STATE_DIM_rl, (float *)relaVar[n].P};
    float cyaw = arm_cos_f32(relaVar[n].S[STATE_rlYaw]);
    float syaw = arm_sin_f32(relaVar[n].S[STATE_rlYaw]);
    float xij = relaVar[n].S[STATE_rlX];
    float yij = relaVar[n].S[STATE_rlY];

    // prediction
    relaVar[n].S[STATE_rlX] = xij + (cyaw * vxj - syaw * vyj - vxi + ri * yij) * dt;
    relaVar[n].S[STATE_rlY] = yij + (syaw * vxj + cyaw * vyj - vyi - ri * xij) * dt;
    relaVar[n].S[STATE_rlYaw] = relaVar[n].S[STATE_rlYaw] + (rj - ri) * dt;
    // A状态转移矩阵
    A[0][0] = 1;
    A[0][1] = ri * dt;
    A[0][2] = (-syaw * vxj - cyaw * vyj) * dt;
    A[1][0] = -ri * dt;
    A[1][1] = 1;
    A[1][2] = (cyaw * vxj - syaw * vyj) * dt;
    A[2][0] = 0;
    A[2][1] = 0;
    A[2][2] = 1;

    mat_mult(&Am, &Pm, &tmpNN1m);      // A P
    mat_trans(&Am, &tmpNN2m);          // A'
    mat_mult(&tmpNN1m, &tmpNN2m, &Pm); // A P A'

    // BQB' = [ Qv*c^2 + Qv*s^2 + Qr*y^2 + Qv,                       -Qr*x*y, -Qr*y]
    //        [                       -Qr*x*y, Qv*c^2 + Qv*s^2 + Qr*x^2 + Qv,  Qr*x]
    //        [                         -Qr*y,                          Qr*x,  2*Qr]*dt^2
    float dt2 = dt * dt;
    relaVar[n].P[0][0] += dt2 * (Qv + Qv + Qr * yij * yij);
    relaVar[n].P[0][1] += dt2 * (-Qr * xij * yij);
    relaVar[n].P[0][2] += dt2 * (-Qr * yij);
    relaVar[n].P[1][0] += dt2 * (-Qr * xij * yij);
    relaVar[n].P[1][1] += dt2 * (Qv + Qv + Qr * xij * xij);
    relaVar[n].P[1][2] += dt2 * (Qr * xij);
    relaVar[n].P[2][0] += dt2 * (-Qr * yij);
    relaVar[n].P[2][1] += dt2 * (Qr * xij);
    relaVar[n].P[2][2] += dt2 * (2 * Qr);

    xij = relaVar[n].S[STATE_rlX];
    yij = relaVar[n].S[STATE_rlY];
    float distPred = arm_sqrt(xij * xij + yij * yij + (hi - hj) * (hi - hj)) + 0.0001f;
    float distMeas = (float)(dij / 100.0f);
    // h矩阵
    h[0] = xij / distPred;
    h[1] = yij / distPred;
    h[2] = 0;

    mat_trans(&H, &HTm);        // H'
    mat_mult(&Pm, &HTm, &PHTm); // PH'
    float HPHR = powf(Ruwb, 2); // HPH' + R
    for (int i = 0; i < STATE_DIM_rl; i++)
    {                                 // Add the element of HPH' to the above
        HPHR += H.pData[i] * PHTd[i]; // this obviously only works if the update is scalar (as in this function)
    }
    for (int i = 0; i < STATE_DIM_rl; i++)
    {
        K[i] = PHTd[i] / HPHR; // kalman gain = (PH' (HPH' + R )^-1)
        // DEBUG_PRINT("K[%d]:%f\n", i, K[i]);
        // DEBUG_PRINT("relaVarStart:%.3lf", relaVar[n].S[i]);
        relaVar[n].S[i] = relaVar[n].S[i] + K[i] * (distMeas - distPred); // state update
        // DEBUG_PRINT(",relaVar:%.3f,K:%.3f,distMeas:%.3f,distPred%.3f\n", relaVar[n].S[i], K[i], distMeas, distPred);
    }
    mat_mult(&Km, &H, &tmpNN1m); // KH
    for (int i = 0; i < STATE_DIM_rl; i++)
    {
        tmpNN1d[STATE_DIM_rl * i + i] -= 1;
    } // KH - I
    mat_trans(&tmpNN1m, &tmpNN2m);     // (KH - I)'
    mat_mult(&tmpNN1m, &Pm, &tmpNN3m); // (KH - I)*P
    mat_mult(&tmpNN3m, &tmpNN2m, &Pm); // (KH - I)*P*(KH - I)'
    // DEBUG_PRINT("dis:%d\n", dij);
}
void relativeInfoRead(float *relaVarParam, float *neighbor_height, currentNeighborAddressInfo_t *dest){
	if(fullConnect){
		for(int index = 0; index < currentNeighborAddressInfo.size; index++){
			uint16_t neighborAddress = currentNeighborAddressInfo.address[index];
			*(relaVarParam + neighborAddress * STATE_DIM_rl + 0) = relaVar[neighborAddress].S[STATE_rlX];
			*(relaVarParam + neighborAddress * STATE_DIM_rl + 1) = relaVar[neighborAddress].S[STATE_rlY];
			*(relaVarParam + neighborAddress * STATE_DIM_rl + 2) = relaVar[neighborAddress].S[STATE_rlYaw];
			*(neighbor_height + neighborAddress ) = relaVar[neighborAddress].height;
		}
		memcpy(dest, currentNeighborAddressInfo.address, sizeof(currentNeighborAddressInfo.address));
		dest->size = currentNeighborAddressInfo.size;
		return true;
	}else{
		return false;
	}
}

