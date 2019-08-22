
/* tiny_slam.cpp */

#include "tiny_slam.h"

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
int DistanceScanToMap(const ScanData* pScan,
                      const GridMap* pMap,
                      const RobotPosition2D* pPos)
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
void MapLaserPoint(GridMap* pMap,
                   int mapBeginX, int mapBeginY,
                   int mapHoleX, int mapHoleY,
                   int mapEndX, int mapEndY,
                   int cellValue, int alpha)
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
void UpdateMap(const ScanData* pScan,
               GridMap* pMap,
               const RobotPosition2D* pPos,
               int integrationQuality,
               int holeWidth)
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

