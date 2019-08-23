
/* tiny_slam.cpp */

#include "tiny_slam.h"

/*
 * 2つのロボット位置間の距離を計算
 */
double DistanceRobotPosition2D(RobotPosition2D* pPos0,
                               RobotPosition2D* pPos1)
{
    return std::sqrt((pPos0->mX - pPos1->mX) * (pPos0->mX - pPos1->mX) +
                     (pPos0->mY - pPos1->mY) * (pPos0->mY - pPos1->mY));
}

/*
 * 占有格子地図の初期化
 */
void InitializeGridMap(GridMap* pMap)
{
    /* 全ての格子をCELL_UNKNOWNで初期化 */
    for (int i = 0; i < MAP_SIZE; ++i)
        for (int j = 0; j < MAP_SIZE; ++j)
            pMap->mCells[i * MAP_SIZE + j] = CELL_UNKNOWN;
}

/*
 * ロボットの計測と地図との相違を計算
 */
int DistanceScanToMap(const ScanData* pScan,        /* スキャンデータ */
                      const GridMap* pMap,          /* 占有格子地図 */
                      const RobotPosition2D* pPos)  /* ロボットの姿勢 */
{
    /* ロボットの姿勢と地図から, 計測の尤度の逆数に相当する値を計算 */
    /* パーティクルフィルタにおいて, 各パーティクルkが保持する
     * 地図m_{k, t - 1}と姿勢x_{k, t}から, 計測z_tの尤度の逆数を計算 */
    /* 計測z_tが, 地図m_{k, t - 1}や姿勢x_{k, t}から予想されるものと
     * 異なるとき, 関数の値は増加する(計測と地図との相違を表す) */
    
    /* 尤度の逆数に相当する値 */
    std::int64_t sumValue = 0;

    /* 地図に含まれるスキャン点の個数 */
    int pointsNum = 0;
    
    /* ロボットの座標(地図座標系) */
    int posX = pPos->mX;
    int posY = pPos->mY;
    
    /* ロボットの回転角(rad) */
    double cosTheta = std::cos(DEG_TO_RAD(pPos->mTheta));
    double sinTheta = std::sin(DEG_TO_RAD(pPos->mTheta));

    for (int i = 0; i < pScan->mPointsNum; ++i) {
        if (pScan->mPoints[i].mValue != CELL_NO_OBSTACLE) {
            /* スキャン点の座標(ロボット座標系) */
            double scanX = pScan->mPoints[i].mX;
            double scanY = pScan->mPoints[i].mY;

            /* スキャン点の座標をロボット座標系から, 地図座標系に変換 */
            double globalScanX = posX + cosTheta * scanX - sinTheta * scanY;
            double globalScanY = posY + sinTheta * scanX + cosTheta * scanY;
            
            /* スキャン点の座標(地図座標系)から, 格子のインデックスに変換 */
            int mapX = static_cast<int>(
                std::floor(globalScanX * CELLS_PER_METER + 0.5));
            int mapY = static_cast<int>(
                std::floor(globalScanY * CELLS_PER_METER + 0.5));

            /* 格子のインデックスが範囲内であることをチェック */
            if (mapX < 0 || mapX >= MAP_SIZE)
                continue;
            if (mapY < 0 || mapY >= MAP_SIZE)
                continue;
            
            /* スキャン点の位置が, 地図上で占有されていなければ,
             * (地図上の対応する格子の値がCELL_NO_OBSTACLEであれば)
             * 大きな値が加算される(計測と地図との異なり具合が大きい) */
            sumValue += pMap->mCells[mapY * MAP_SIZE + mapX];
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

/*
 * スキャンに基づいて格子の状態を更新
 */
void MapLaserPoint(
    GridMap* pMap,                   /* 占有格子地図 */
    int mapBeginX, int mapBeginY,    /* スキャンの始点 */
    int mapHoleX, int mapHoleY,      /* スキャンの延長された到達点 */
    int mapEndX, int mapEndY,        /* スキャンの到達点 */
    int cellValue,                   /* 占有状態を表す値 */
    int alpha)                       /* 混合率 (0から255) */
{
    /* スキャンに基づいて, 格子の値を更新 */

    /* スキャンの始点が地図からはみ出している場合は無視 */
    if (mapBeginX < 0 || mapBeginX >= MAP_SIZE)
        return;
    if (mapBeginY < 0 || mapBeginY >= MAP_SIZE)
        return;
    
    /* 格子のインデックスの範囲内に収める */
    int clippedHoleX = mapHoleX;
    int clippedHoleY = mapHoleY;

    if (clippedHoleX < 0) {
        /* スキャンの終点が左側にはみ出している場合 */
        clippedHoleY += (clippedHoleY - mapBeginY) *
                        (-clippedHoleX) /
                        (clippedHoleX - mapBeginX);
        clippedHoleX = 0;
    } else if (clippedHoleX >= MAP_SIZE) {
        /* スキャンの終点が右側にはみ出している場合 */
        clippedHoleY += (clippedHoleY - mapBeginY) *
                        (-(clippedHoleX - (MAP_SIZE - 1))) /
                        (clippedHoleX - mapBeginX);
        clippedHoleX = MAP_SIZE - 1;
    }

    if (clippedHoleY < 0) {
        /* スキャンの終点が下側にはみ出している場合 */
        clippedHoleX += (clippedHoleX - mapBeginX) *
                        (-clippedHoleY) /
                        (clippedHoleY - mapBeginY);
        clippedHoleY = 0;
    } else if (clippedHoleY >= MAP_SIZE) {
        /* スキャンの終点が上側にはみ出している場合 */
        clippedHoleX += (clippedHoleX - mapBeginX) *
                        (-(clippedHoleY - (MAP_SIZE - 1))) /
                        (clippedHoleY - mapBeginY);
        clippedHoleY = 0;
    }

    int deltaX = std::abs(mapHoleX - mapBeginX);
    int deltaY = std::abs(mapHoleY - mapBeginY);
    bool isSteep = deltaY > deltaX;

    int clippedDeltaX = std::abs(clippedHoleX - mapBeginX);
    int clippedDeltaY = std::abs(clippedHoleY - mapBeginY);

    int stepX = (mapHoleX > mapBeginX) ? 1 : -1;
    int stepY = (mapHoleY > mapBeginY) ? 1 : -1;
    int stepValueX = (cellValue > CELL_NO_OBSTACLE) ? 1 : -1;
    
    /* 直線の傾きが1未満になるように値を交換 */
    /* mapBeginX, mapBeginY, mapHoleX, mapHoleY,
     * mapEndX, mapEndYは交換しない */
    if (isSteep) {
        std::swap(deltaX, deltaY);
        std::swap(clippedDeltaX, clippedDeltaY);
        std::swap(stepX, stepY);
    }
    
    /* 穴(Hole)の半分の長さがholeDeltaXに格納される */
    /* mapEndX, mapEndY, mapHoleX, mapHoleYが交換されないので,
     * 直線の傾きが1以上かどうかによって使用する変数を適切に変える */
    int halfHoleWidth = (!isSteep) ? std::abs(mapEndX - mapHoleX) :
                                     std::abs(mapEndY - mapHoleY);
    
    /* Bresenhamの方法における格子の値の誤差 */
    int errValue = halfHoleWidth / 2;
    /* Bresenhamの方法におけるy座標の値の誤差 */
    int errY = clippedDeltaX;

    /* 1つ格子がずれたときの格子の値の変化 */
    int stepValue = (cellValue - CELL_NO_OBSTACLE) / halfHoleWidth;
    /* Bresenhamの方法における格子の値の誤差増加量 */
    int deltaValue = (cellValue - CELL_NO_OBSTACLE) -
                     halfHoleWidth * stepValue;

    /* 格子のx座標がhalfHoleWidthだけずれる間に,
     * 格子の値はCELL_NO_OBSTACLEからcellValueまで線形に変化するが,
     * 格子の値の変化は座標の変化よりも急である(傾きが1を超える) */
    /* 傾きが1を超えてしまうとBresenhamの方法が利用できないので,
     * 傾きが1を超えないように、halfHoleWidth * stepValueを
     * 格子の値の変化量から差し引く */
    /* 差し引かれると, 0からdeltaValueまでの格子の値の変化だけを
     * 考慮することになるので、各ループごとにstepValueを加算する
     * (誤差項のみを対象としてBresenhamの方法を用いている) */
    
    /* 格子の現在の値 */
    int currentValue = CELL_NO_OBSTACLE;
    /* 格子のインデックス */
    int cellX = mapBeginX;
    int cellY = mapBeginY;

    for (int x = 0; x <= clippedDeltaX; ++x, cellX += stepX) {
        /* 穴(Hole)の内側にいる場合 */
        if (x > deltaX - 2 * halfHoleWidth) {
            if (x <= deltaX - halfHoleWidth) {
                /* 穴(Hole)の左側にいる場合 */
                currentValue += stepValue;
                errValue -= deltaValue;

                if (errValue < 0) {
                    currentValue += stepValueX;
                    errValue += halfHoleWidth;
                }
            } else {
                /* 穴(Hole)の右側にいる場合 */
                currentValue -= stepValue;
                errValue += deltaValue;

                if (errValue > halfHoleWidth) {
                    currentValue -= stepValueX;
                    errValue -= halfHoleWidth;
                }
            }
        }

        /* 格子の値を線形補間で更新 */
        int oldValue = pMap->mCells[cellY * MAP_SIZE + cellX];
        int newValue = ((256 - alpha) * oldValue + alpha * currentValue) >> 8;
        pMap->mCells[cellY * MAP_SIZE + cellX] = newValue;
        
        errY -= 2 * clippedDeltaY;

        if (errY < 0) {
            cellY += stepY;
            errY += 2 * clippedDeltaX;
        }
    }
}

/*
 * 地図の更新
 */
void UpdateMap(
    const ScanData* pScan,          /* スキャンデータ */
    GridMap* pMap,                  /* 占有格子地図 */
    const RobotPosition2D* pPos,    /* ロボットの姿勢 */
    int integrationQuality,         /* 混合率 */
    int holeWidth)                  /* 穴のサイズ (格子の個数) */
{
    /* 地図の更新 */
    
    /* ロボットの座標(地図座標系) */
    int posX = pPos->mX;
    int posY = pPos->mY;

    /* ロボットの回転角(rad) */
    double cosTheta = std::cos(DEG_TO_RAD(pPos->mTheta));
    double sinTheta = std::sin(DEG_TO_RAD(pPos->mTheta));

    /* ロボットの座標に対応する格子のインデックス */
    int mapBeginX = static_cast<int>(
        std::floor(pPos->mX * CELLS_PER_METER + 0.5));
    int mapBeginY = static_cast<int>(
        std::floor(pPos->mY * CELLS_PER_METER + 0.5));

    for (int i = 0; i < pScan->mPointsNum; ++i) {
        /* スキャン点の座標(ロボット座標系) */
        double scanX = pScan->mPoints[i].mX;
        double scanY = pScan->mPoints[i].mY;

        /* スキャン点の座標を回転 */
        double rotatedScanX = cosTheta * scanX - sinTheta * scanY;
        double rotatedScanY = sinTheta * scanX + cosTheta * scanY;

        /* スキャン点の座標を地図座標系に変換 */
        double globalEndX = posX + rotatedScanX;
        double globalEndY = posY + rotatedScanY;

        /* スキャン点の座標(地図座標系)から, 格子のインデックスに変換 */
        int mapEndX = static_cast<int>(
            std::floor(globalEndX * CELLS_PER_METER + 0.5));
        int mapEndY = static_cast<int>(
            std::floor(globalEndY * CELLS_PER_METER + 0.5));
        
        /* 穴(Hole)の半径の半分の長さだけスキャンを延長 */
        double scanDist = std::sqrt(scanX * scanX + scanY * scanY);
        double holeScale = 1.0 + (holeWidth / 2.0) / scanDist;

        rotatedScanX *= holeScale;
        rotatedScanY *= holeScale;

        /* 延長されたスキャン点の地図座標系を計算 */
        double globalHoleX = posX + rotatedScanX;
        double globalHoleY = posY + rotatedScanY;

        /* 延長されたスキャン点の座標を, 格子のインデックスに変換 */
        int mapHoleX = static_cast<int>(
            std::floor(globalHoleX * CELLS_PER_METER + 0.5));
        int mapHoleY = static_cast<int>(
            std::floor(globalHoleY * CELLS_PER_METER + 0.5));

        /* スキャン点に障害物がないときは, 混合率を減らす */
        if (pScan->mPoints[i].mValue == CELL_NO_OBSTACLE)
            integrationQuality /= 4;

        int cellValue = (pScan->mPoints[i].mValue == CELL_NO_OBSTACLE) ?
            CELL_NO_OBSTACLE : CELL_OBSTACLE;

        MapLaserPoint(pMap,
                      mapBeginX, mapBeginY,
                      mapHoleX, mapHoleY,
                      mapEndX, mapEndY,
                      cellValue, integrationQuality);
    }
}

/*
 * モンテカルロ法による位置探索
 */
RobotPosition2D MonteCarloPositionSearch(
    std::default_random_engine& randEngine, /* 擬似乱数生成器 */
    ScanData* pScan,                        /* スキャンデータ */
    GridMap* pMap,                          /* 占有格子地図 */
    RobotPosition2D* pStartPos,             /* 探索開始点の座標 */
    double sigmaXY,                         /* 並進移動の標準偏差 */
    double sigmaTheta,                      /* 回転移動の標準偏差 */
    int maxIterNum)                         /* 最大の繰り返し数 */
{
    RobotPosition2D currentPos = *pStartPos;
    RobotPosition2D bestPos = *pStartPos;
    RobotPosition2D lastBestPos = *pStartPos;

    int currentDist = DistanceScanToMap(pScan, pMap, &currentPos);
    int bestDist = currentDist;
    int lastBestDist = currentDist;
    
    int iterNum = 0;
    
    /* ロボット位置の標準偏差 */
    std::normal_distribution<> normalDistXY(0.0, sigmaXY);
    /* ロボット回転角の標準偏差 */
    std::normal_distribution<> normalDistTheta(0.0, sigmaTheta);

    using DistParamType = std::normal_distribution<>::param_type;

    while (iterNum < maxIterNum) {
        /* ロボット位置を乱数で適当に変更 */
        currentPos = lastBestPos;
        currentPos.mX += normalDistXY(randEngine);
        currentPos.mY += normalDistXY(randEngine);
        currentPos.mTheta += normalDistTheta(randEngine);
        
        /* スキャンと現在位置との相違を計算 */
        currentDist = DistanceScanToMap(pScan, pMap, &currentPos);

        if (currentDist < bestDist) {
            /* 最善なロボット位置を更新 */
            bestPos = currentPos;
            bestDist = currentDist;
        } else {
            ++iterNum;
        }
        
        /* ロボット位置が一定回数以上改善されなかった場合 */
        if (iterNum > maxIterNum / 3) {
            /* 現在の標準偏差の下でロボット位置が改善されている場合 */
            if (bestDist < lastBestDist) {
                /* 現在の標準偏差の下での最善なロボット位置を記録 */
                lastBestPos = bestPos;
                lastBestDist = bestDist;

                /* イテレーション回数をリセット */
                iterNum = 0;

                /* 標準偏差を小さくして再度モンテカルロ探索を実行 */
                sigmaXY *= 0.5;
                sigmaTheta *= 0.5;

                /* 正規分布のパラメータを更新 */
                DistParamType distParamXY(0.0, sigmaXY);
                DistParamType distParamTheta(0.0, sigmaTheta);

                normalDistXY.param(distParamXY);
                normalDistTheta.param(distParamTheta);
            }
        }
    }

    return bestPos;
}

/*
 * センサデータからスキャンを生成
 */
void BuildScanFromSensorData(
    ScanData* pScan,                /* スキャンデータ */
    const SensorData* pSensorData,  /* センサデータ */
    double scanFrequency,           /* 1秒間のスキャン回数 (Hz) */
    int scanDetectionMargin,        /* 取り除く両端のデータ数 */
    int scanSize,                   /* センサデータの個数 */
    int scanSpan,                   /* 補間されるデータ数 */
    double scanAngleMin,            /* 角度の最小値 (deg) */
    double scanAngleMax,            /* 角度の最大値 (deg) */
    double scanDistNoDetection,     /* 障害物の未検知の距離 (m) */
    double holeWidth,               /* 穴のサイズ (格子の個数) */
    double robotVelocityXY,         /* ロボットの並進速度 (m/s) */
    double robotVelocityAngle)      /* ロボットの回転速度 (rad/s) */
{
    /* スキャンのデータ数を初期化 */
    pScan->mPointsNum = 0;

    /* 1秒間のセンサの回転角 (deg) */
    double rotationAnglePerSecond = scanFrequency * 360.0;

    /* ロボットの移動によるセンサデータのずれを考慮
     * ロボットの姿勢が得られてから, センサデータが取得されるまでの間に,
     * ロボットの並進と回転がセンサデータに加わるので,
     * センサデータからロボットの並進と回転を取り除く必要がある */
    /* ロボットの局所座標系におけるx座標のみを扱えばよい
     * センサが1度回転するときの並進量と姿勢の回転量 */
    double horizontalShift = robotVelocityXY / rotationAnglePerSecond;
    double rotationShift = robotVelocityAngle / rotationAnglePerSecond;

    /* センサデータをscanSpan個だけ補間することで精度を高める */
    
    for (int i = scanDetectionMargin;
         i < scanSize - scanDetectionMargin; ++i) {
        /* 各センサデータの距離 (mm) */
        int sensorValue = pSensorData->mValue[i];

        if (sensorValue == 0) {
            /* 障害物がない場合(距離が大きいため0にされている場合) */
            AppendScanFromSensorData(pScan, scanSpan, scanSize, i,
                                     scanAngleMin, scanAngleMax,
                                     horizontalShift, rotationShift,
                                     scanDistNoDetection, CELL_NO_OBSTACLE);
        } else if (sensorValue > holeWidth / 2.0) {
            /* 障害物である場合 */
            AppendScanFromSensorData(pScan, scanSpan, scanSize, i,
                                     scanAngleMin, scanAngleMax,
                                     horizontalShift, rotationShift,
                                     sensorValue, CELL_OBSTACLE);
        }
    }
}

/*
 * センサデータをスキャンに追加
 */
void AppendScanFromSensorData(
    ScanData* pScan,        /* スキャンデータ */
    int scanSpan,           /* 補間されるデータ数 */
    int scanSize,           /* センサデータの個数 */
    int sensorDataIndex,    /* データのインデックス */
    double scanAngleMin,    /* 角度の最小値 (deg) */
    double scanAngleMax,    /* 角度の最大値 (deg) */
    double horizontalShift, /* 並進方向のずれ (m/deg) */
    double rotationShift,   /* 回転角のずれ */
    double scanDist,        /* センサデータの距離値 (m) */
    int scanValue)          /* 占有状態を表す値 */
{
    /* センサデータの最大角と最小角との差 (deg) */
    double scanAngleRange = std::abs(scanAngleMax - scanAngleMin);

    for (int j = 0; j < scanSpan; ++j) {
        /* ロボットはセンサデータが取得されるまでの間に,
         * sensorAngle * rotationShiftだけ姿勢が回転しているので,
         * センサデータの角度はこの量だけ減少しており, 減少分を補う */
        double sensorAngle =
            static_cast<double>(sensorDataIndex * scanSpan + j) *
            scanAngleRange / (scanSize * scanSpan - 1);
        double angleDeg = scanAngleMin + sensorAngle * (1.0 + rotationShift);
        double angleRad = DEG_TO_RAD(angleDeg);
        
        /* ロボットはセンサデータが取得されるまでの間に,
         * sensorAngle * horizontalShiftだけ(現在の方向に)並進移動しているので,
         * センサデータの距離値はこの量だけ増加している */
        double scanX = scanDist * std::cos(angleRad);
        scanX += sensorAngle * horizontalShift;
        
        /* センサデータのy座標は変更する必要なし */
        double scanY = scanDist * std::sin(angleRad);
        
        /* スキャンデータをセット */
        pScan->mPoints[pScan->mPointsNum].mX = scanX;
        pScan->mPoints[pScan->mPointsNum].mY = scanY;
        pScan->mPoints[pScan->mPointsNum].mValue = scanValue;

        ++pScan->mPointsNum;
    }
}

/*
 * ロボットの動作
 */
void UpdateRobotPosition2D(
    RobotPosition2D* pPos,          /* ロボットの姿勢 */
    double wheelRadius,             /* 車輪の半径 (m) */
    double robotRadius,             /* 車輪間の距離の半分 (m) */
    int odometerCountPerTurn,       /* 1回転後のオドメータ値の増分 */
    double odometerRatio,           /* オドメータの重み付け値 */
    int lastOdometerCountLeft,      /* 以前のオドメータ値 (左側の車輪) */
    int lastOdometerCountRight,     /* 以前のオドメータ値 (右側の車輪) */
    int odometerCountLeft,          /* オドメータ値 (左側の車輪) */
    int odometerCountRight,         /* オドメータ値 (右側の車輪) */
    int deltaTime,                  /* 時間経過 (us) */
    double* pRobotVelocityXY,       /* ロボットの並進速度 (m/s) */
    double* pRobotVelocityAngle)    /* ロボットの回転角速度 (m/s) */
{
    /* odometerRatioは, 左右の車輪におけるオドメータ値の
     * 重み付けを行う(通常は1.0に設定される) */

    /* オドメータ値の変化 */
    double deltaOdometerLeft = static_cast<double>(
        odometerCountLeft - lastOdometerCountLeft);
    double deltaOdometerRight = static_cast<double>(
        odometerCountRight - lastOdometerCountRight);
    deltaOdometerRight *= odometerRatio;

    /* オドメータ値が1増加するときのロボットの移動量 (m) */
    double distPerCount = (2.0 * wheelRadius) * M_PI / odometerCountPerTurn;
    
    /* ロボットの並進移動量と回転量 */
    double deltaXY = distPerCount *
                     (deltaOdometerLeft + deltaOdometerRight) / 2.0;
    double deltaAngle = distPerCount *
                        (deltaOdometerRight - deltaOdometerLeft) /
                        (2.0 * robotRadius);

    /* ロボットの速度と角速度 */
    double robotVelocityXY = deltaXY * 1e6 / deltaTime;
    double robotVelocityAngle = deltaAngle * 1e6 / deltaTime;

    /* ロボットの位置の更新 */
    double thetaRad = DEG_TO_RAD(pPos->mTheta);

    pPos->mX += deltaXY * std::cos(thetaRad);
    pPos->mY += deltaXY * std::sin(thetaRad);
    pPos->mTheta += deltaAngle;

    if (pRobotVelocityXY != nullptr)
        *pRobotVelocityXY = robotVelocityXY;

    if (pRobotVelocityAngle != nullptr)
        *pRobotVelocityAngle = robotVelocityAngle;
}

/*
 * 逐次的な自己位置推定と地図構築
 */
void IterativeMapBuilding(
    std::default_random_engine& randEngine, /* 擬似乱数生成器 */
    SensorData* pSensorData,                /* センサデータ */
    GridMap* pMap,                          /* 占有格子地図 */
    RobotPosition2D* pPos,                  /* ロボットの姿勢 */
    double wheelRadius,                     /* 車輪の半径 (m) */
    double robotRadius,                     /* 車輪間の距離の半分 (m) */
    int odometerCountPerTurn,               /* 1回転後のオドメータ値の増分 */
    double odometerRatio,                   /* オドメータの重み付け値 */
    int lastOdometerCountLeft,              /* 以前のオドメータ値 (左側の車輪) */
    int lastOdometerCountRight,             /* 以前のオドメータ値 (右側の車輪) */
    int deltaTime,                          /* 時間経過 (us) */
    double scanFrequency,                   /* 1秒間のスキャン回数 (Hz) */
    int scanDetectionMargin,                /* 取り除く両端のデータ数 */
    int scanSize,                           /* センサデータの個数 */
    double scanAngleMin,                    /* 角度の最小値 (deg) */
    double scanAngleMax,                    /* 角度の最大値 (deg) */
    double scanDistNoDetection,             /* 障害物の未検知の距離 (m) */
    double holeWidth,                       /* 穴のサイズ (格子の個数) */
    double sigmaXY,                         /* 並進移動の標準偏差 (m) */
    double sigmaTheta,                      /* 回転移動の標準偏差 (m) */
    double sensorOffsetX,                   /* ロボット中心に対するセンサの相対位置 */
    double sensorOffsetY,                   /* ロボット中心に対するセンサの相対位置 */
    double accumulatedTravelDist,           /* 累積走行距離 (m) */
    double lastRobotVelocityXY,             /* 以前のロボットの並進速度 */
    double lastRobotVelocityAngle,          /* 以前のロボットの回転速度 */
    double* pRobotVelocityXY,               /* ロボットの並進速度 */
    double* pRobotVelocityAngle,            /* ロボットの回転速度 */
    int timeStep)                           /* 時間ステップ */
{
    double robotVelocityXY;
    double robotVelocityAngle;
    double thetaRad;
    double cosTheta;
    double sinTheta;

    ScanData currentScan;
    RobotPosition2D currentPos = *pPos;

    if (timeStep > 0) {
        /* ロボットの位置をオドメトリにより更新 */
        UpdateRobotPosition2D(&currentPos, wheelRadius, robotRadius,
                              odometerCountPerTurn, odometerRatio,
                              lastOdometerCountLeft, lastOdometerCountRight,
                              pSensorData->mOdometerCountLeft,
                              pSensorData->mOdometerCountRight,
                              deltaTime,
                              &robotVelocityXY, &robotVelocityAngle);
    } else {
        robotVelocityXY = 0.0;
        robotVelocityAngle = 0.0;
    }

    /* センサデータからスキャンを構築 */
    BuildScanFromSensorData(&currentScan, pSensorData,
                            scanFrequency, scanDetectionMargin,
                            scanSize, 3,
                            scanAngleMin, scanAngleMax,
                            scanDistNoDetection, holeWidth,
                            lastRobotVelocityXY, lastRobotVelocityAngle);

    /* モンテカルロ探索による自己位置推定 */

    thetaRad = DEG_TO_RAD(currentPos.mTheta);
    cosTheta = std::cos(thetaRad);
    sinTheta = std::sin(thetaRad);

    /* ロボット位置をセンサ位置に変換 */
    currentPos.mX += sensorOffsetX * cosTheta - sensorOffsetY * sinTheta;
    currentPos.mY += sensorOffsetX * sinTheta + sensorOffsetY * cosTheta;

    currentPos = MonteCarloPositionSearch(randEngine,
                                          &currentScan, pMap,
                                          &currentPos,
                                          sigmaXY, sigmaTheta,
                                          1000);

    thetaRad = DEG_TO_RAD(currentPos.mTheta);
    cosTheta = std::cos(thetaRad);
    sinTheta = std::sin(thetaRad);

    /* センサ位置をロボット位置に戻す */
    currentPos.mX -= (sensorOffsetX * cosTheta - sensorOffsetY * sinTheta);
    currentPos.mY -= (sensorOffsetX * sinTheta + sensorOffsetY * cosTheta);

    /* 累積走行距離の計算 */
    double travelDist = std::sqrt(
        (currentPos.mX - pPos->mX) * (currentPos.mX - pPos->mX) +
        (currentPos.mY - pPos->mY) * (currentPos.mY - pPos->mY));
    accumulatedTravelDist += travelDist;

    /* 地図の更新 */
    UpdateMap(&currentScan, pMap, &currentPos, 50, holeWidth);

    if (pRobotVelocityXY != nullptr)
        *pRobotVelocityXY = robotVelocityXY;

    if (pRobotVelocityAngle != nullptr)
        *pRobotVelocityAngle = robotVelocityAngle;
}

