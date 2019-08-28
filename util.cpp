
/* util.cpp */

#include <fstream>
#include <limits>
#include <sstream>

#include "util.h"

/*
 * 生成した地図をPGM(Portable Graymap Format)形式の画像で保存
 */
bool SaveMapImagePgm(const GridMap* pMap,           /* 占有格子地図 */
                     const GridMap* pTrajectoryMap, /* ロボットの軌跡 */
                     const char* pFileName,         /* PGMファイル名 */
                     int mapWidth,                  /* 横方向の格子数 */
                     int mapHeight,                 /* 縦方向の格子数 */
                     int imageWidth,                /* 画像の横幅 (px) */
                     int imageHeight)               /* 画像の縦幅 (px) */
{
    std::stringstream strStream;
    std::ofstream outFile { pFileName };

    if (!outFile)
        return false;
    
    /* マジックナンバーの出力 */
    strStream << "P2" << '\n';

    /* 画像サイズの出力 */
    strStream << mapWidth << ' ' << mapHeight << '\n';

    /* ピクセルの輝度の最大値 */
    strStream << 255 << '\n';

    /* 軌跡と地図の出力 */
    for (int y = 0; y < imageHeight; ++y) {
        int mapY = (MAP_SIZE - 1) - (MAP_SIZE - imageHeight) / 2 - y;

        for (int x = 0; x < imageWidth; ++x) {
            int mapX = (MAP_SIZE - imageWidth) / 2 + x;
            int pixelValue;

            if (pTrajectoryMap->mCells[mapY * MAP_SIZE + mapX] == 0)
                /* 軌跡の出力 */
                pixelValue = 0;
            else
                /* 格子の占有状態の出力 */
                pixelValue = static_cast<int>(
                    pMap->mCells[mapY * MAP_SIZE + mapX]) >> 8;

            strStream << pixelValue << ' ';
        }

        strStream << '\n';
    }

    /* ファイルに書き込み */
    outFile << strStream.rdbuf();
    outFile.close();

    return true;
}

/*
 * スキャンデータを地図に反映
 */
void DrawScan(const ScanData* pScan,                /* スキャンデータ */
              GridMap* pScanMap,                    /* スキャン点を描画した地図 */
              const RobotPosition2D* pSensorPos)    /* センサの位置 */
{
    double cosTheta = std::cos(DEG_TO_RAD(pSensorPos->mTheta));
    double sinTheta = std::sin(DEG_TO_RAD(pSensorPos->mTheta));

    for (int i = 0; i < pScan->mPointsNum; ++i) {
        if (pScan->mPoints[i].mValue != CELL_NO_OBSTACLE) {
            /* スキャン点の座標を計算 (地図座標系) */
            double scanPosX = pSensorPos->mX;
            double scanPosY = pSensorPos->mY;

            scanPosX += (pScan->mPoints[i].mX * cosTheta -
                         pScan->mPoints[i].mY * sinTheta);
            scanPosY += (pScan->mPoints[i].mX * sinTheta +
                         pScan->mPoints[i].mY * cosTheta);
            
            /* スキャン点に対応する格子のインデックス */
            int scanX = static_cast<int>(std::floor(
                scanPosX * CELLS_PER_METER + 0.5));
            int scanY = static_cast<int>(std::floor(
                scanPosY * CELLS_PER_METER + 0.5));

            if (scanX < 0 || scanX >= MAP_SIZE)
                continue;
            if (scanY < 0 || scanY >= MAP_SIZE)
                continue;
            
            /* スキャン点に印をつける */
            pScanMap->mCells[scanY * MAP_SIZE + scanX] = 0;
        }
    }
}

/*
 * ループ閉じ込み後の位置を用いて地図を作成
 */
void DrawMap(GridMap* pMap,                 /* 占有格子地図 */
             GridMap* pScanMap,             /* スキャン点を描画した地図 */
             SensorData* pSensorDataArray,  /* センサデータ配列 */
             int scanNum,                   /* スキャンデータの個数 */
             double scanFrequency,          /* 1秒間のスキャン回数 (Hz) */
             int scanDetectionMargin,       /* 除去される両端のデータ数 */
             int scanSize,                  /* 1スキャン内のデータの個数 */
             double scanAngleMin,           /* 角度の最小値 (deg) */
             double scanAngleMax,           /* 角度の最大値 (deg) */
             double scanDistNoDetection,    /* 障害物がないと判定する距離 (m) */
             double holeWidth,              /* 穴のサイズ (m) */
             double offsetPosX,             /* センサの相対位置 (m) */
             double offsetPosY)             /* センサの相対位置 (m) */
{
    ScanData scanToMap;
    
    /* 地図を初期化 */
    InitializeGridMap(pMap);
    
    /* スキャン点のみを描画した地図 */
    if (pScanMap != nullptr)
        InitializeGridMap(pScanMap,
                          std::numeric_limits<std::uint16_t>::max());

    for (int i = 0; i < scanNum; ++i) {
        /* センサデータをスキャンデータに変換 */
        BuildScanFromSensorData(&scanToMap,
                                &(pSensorDataArray[i]),
                                scanFrequency,
                                scanDetectionMargin,
                                scanSize,
                                3,
                                scanAngleMin,
                                scanAngleMax,
                                scanDistNoDetection,
                                holeWidth);

        /* センサ融合後のロボット位置 */
        RobotPosition2D robotPos =
            pSensorDataArray[i].mPositions[POSITION_LOOP_CLOSURE];

        double cosTheta = std::cos(DEG_TO_RAD(robotPos.mTheta));
        double sinTheta = std::sin(DEG_TO_RAD(robotPos.mTheta));
        
        /* ロボット位置をセンサ位置に変換 */
        RobotPosition2D sensorPos = robotPos;
        sensorPos.mX += (offsetPosX * cosTheta - offsetPosY * sinTheta);
        sensorPos.mY += (offsetPosX * sinTheta + offsetPosY * cosTheta);

        /* 地図を更新 */
        UpdateMap(&scanToMap,
                  pMap,
                  &sensorPos,
                  50,
                  holeWidth);
        
        /* スキャン点を反映 */
        if (pScanMap != nullptr)
            DrawScan(&scanToMap, pScanMap, &sensorPos);
    }
}

