
/* util.h */

#ifndef UTIL_H
#define UTIL_H

#include "tiny_slam.h"

/*
 * 生成した地図をPGM(Portable Graymap Format)形式の画像で保存
 */
bool SaveMapImagePgm(const GridMap* pMap,           /* 占有格子地図 */
                     const GridMap* pTrajectoryMap, /* ロボットの軌跡 */
                     const char* pFileName,         /* PGMファイル名 */
                     int mapWidth,                  /* 横方向の格子数 */
                     int mapHeight,                 /* 縦方向の格子数 */
                     int imageWidth,                /* 画像の横幅 (px) */
                     int imageHeight);              /* 画像の縦幅 (px) */

/*
 * スキャンデータを地図に反映
 */
void DrawScan(const ScanData* pScan,                /* スキャンデータ */
              GridMap* pScanMap,                    /* スキャン点を描画した地図 */
              const RobotPosition2D* pSensorPos);   /* センサの位置 */

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
             double offsetPosY);            /* センサの相対位置 (m) */

#endif /* UTIL_H */

