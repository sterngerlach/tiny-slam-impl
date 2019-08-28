
/* tiny_slam_sse.cpp */

#include "tiny_slam_sse.h"

union M64ToInt {
    struct {
        int mY;
        int mX;
    } mIntValue;

    __m64 mM64Value;
};

/*
 * ロボットの計測と地図との相違を計算
 * IntelプロセッサのSSE命令を使用
 */
int DistanceScanToMapSSE(const ScanData* pScan,        /* スキャンデータ */
                         const GridMap* pMap,          /* 占有格子地図 */
                         const RobotPosition2D* pPos)  /* ロボットの姿勢 */
{
    /* 現在のロボットの姿勢から計算された, スキャンデータの座標と
     * 現在の占有格子地図との距離を計算 */

    /* 計測と地図との相違 */
    std::int64_t sumValue = 0;

    /* 地図に含まれるスキャン点の個数 */
    int pointsNum = 0;

    /* ロボットの座標と対応する格子のインデックス */
    double cellsPerMeter = static_cast<double>(CELLS_PER_METER);
    double robotMapX = pPos->mX * cellsPerMeter;
    double robotMapY = pPos->mY * cellsPerMeter;

    __m128 robotPos = _mm_set_ps(robotMapX, robotMapY,
                                 robotMapX, robotMapY);

    /* ロボットの回転角(rad) */
    /* cellsPerMeterを先に乗算しておく */
    double thetaRad = DEG_TO_RAD(pPos->mTheta);
    double cosTheta = std::cos(thetaRad) * cellsPerMeter;
    double sinTheta = std::sin(thetaRad) * cellsPerMeter;

    __m128 robotRotation = _mm_set_ps(cosTheta, -sinTheta,
                                      sinTheta, cosTheta);

    for (int i = 0; i < pScan->mPointsNum; ++i) {
        if (pScan->mPoints[i].mValue != CELL_NO_OBSTACLE) {
            /* スキャン点の座標 */
            __m128 scanPos = _mm_set_ps(pScan->mPoints[i].mX,
                                        pScan->mPoints[i].mY,
                                        pScan->mPoints[i].mX,
                                        pScan->mPoints[i].mY);

            /* スキャン点の座標を回転 */
            scanPos = _mm_mul_ps(scanPos, robotRotation);
            scanPos = _mm_hadd_ps(scanPos, scanPos);

            /* 地図座標系に変換 */
            scanPos = _mm_add_ps(scanPos, robotPos);
            
            /* 格子のインデックスに変換 */
            M64ToInt scanMapPos;
            scanMapPos.mM64Value = _mm_cvtps_pi32(scanPos);

            int scanMapX = scanMapPos.mIntValue.mX;
            int scanMapY = scanMapPos.mIntValue.mY;

            /* 格子のインデックスが範囲内であることをチェック */
            if (scanMapX < 0 || scanMapX >= MAP_SIZE)
                continue;
            if (scanMapY < 0 || scanMapY >= MAP_SIZE)
                continue;

            /* スキャン点の位置が, 地図上で占有されていなければ,
             * (地図上の対応する格子の値がCELL_NO_OBSTACLEであれば)
             * 大きな値が加算される(計測と地図との異なり具合が大きい) */
            sumValue += pMap->mCells[scanMapY * MAP_SIZE + scanMapX];
            ++pointsNum;
        }
    }
    
    /* 計測と地図とのずれ具合を表す値を正規化
     * 全スキャン点が地図からはみ出ている場合は, 適当な大きな値を返す */
    if (pointsNum > 0)
        sumValue = sumValue * 1024 / pointsNum;
    else
        sumValue = 2e9;

    return static_cast<int>(sumValue);
}

