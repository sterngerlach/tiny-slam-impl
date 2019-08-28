
/* tiny_slam_sse.h */

#ifndef TINY_SLAM_SSE_H
#define TINY_SLAM_SSE_H

#ifndef __SSE3__
#error "__SSE3__ macro not defined, must be compiled with -msse3 option"
#endif

#include <pmmintrin.h>
#include <xmmintrin.h>

#include "tiny_slam.h"

/*
 * ロボットの計測と地図との相違を計算
 * IntelのSSE命令を使用
 */
int DistanceScanToMapSSE(const ScanData* pScan,        /* スキャンデータ */
                         const GridMap* pMap,          /* 占有格子地図 */
                         const RobotPosition2D* pPos); /* ロボットの姿勢 */

#endif /* TINY_SLAM_SSE_H */

