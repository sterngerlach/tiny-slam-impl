
/* tiny_slam.h */

#ifndef TINY_SLAM_H
#define TINY_SLAM_H

#include <cstdint>
#include <cmath>

/* スキャン中に含まれるデータの個数 */
#define SCAN_SIZE           8192
/* 生成される地図のサイズ(縦横の格子の個数) */
#define MAP_SIZE            2048
/* 1つの格子の縦横のサイズ(m) */
#define CELL_SIZE           0.01
/* 1メートル当たりの格子の個数 */
#define CELLS_PER_METER     100
/* 格子が物体で占有されていないときの値 */
#define CELL_NO_OBSTACLE    65500
/* 格子が物体で占有されているときの値 */
#define CELL_OBSTACLE       0
/* 格子の状態が不明であるときの値 */
#define CELL_UNKNOWN        ((CELL_NO_OBSTACLE + CELL_OBSTACLE) / 2)

#ifndef M_PI
#define M_PI                3.14159265358979
#endif

#define DEG_TO_RAD(x)       ((x) * M_PI / 180.0)
#define RAD_TO_DEG(x)       ((x) * 180.0 / M_PI)

/*
 * 占有格子地図
 */
struct GridMap
{
    std::uint16_t mCells[MAP_SIZE * MAP_SIZE];
};

/*
 * スキャン点
 */
struct LaserPoint2D
{
    double        mX;       /* ロボット座標系におけるx座標 (m) */
    double        mY;       /* ロボット座標系におけるy座標 (m) */
    std::uint16_t mValue;   /* 占有状態を表す値 */
};

/*
 * スキャンデータ
 */
struct ScanData
{
    LaserPoint2D mPoints[SCAN_SIZE];    /* スキャン点の集合 */
    int          mPointsNum;            /* 実際のデータの個数 */
};

/*
 * ロボットの位置
 */
struct RobotPosition2D
{
    double mX;      /* 地図座標系におけるx座標 (m) */
    double mY;      /* 地図座標系におけるy座標 (m) */
    double mTheta;  /* 回転角 (deg) */
};

/*
 * 占有格子地図の初期化
 */
void InitializeGridMap(GridMap* pMap);

/*
 * ロボットの計測と地図との相違を計算
 */
int DistanceScanToMap(const ScanData* pScan,
                      const GridMap* pMap,
                      const RobotPosition2D* pPos);

/*
 * スキャンに基づいて格子の状態を更新
 */
void MapLaserPoint(GridMap* pMap,
                   int mapBeginX, int mapBeginY,
                   int mapHoleX, int mapHoleY,
                   int mapEndX, int mapEndY,
                   int cellValue, int alpha);

/*
 * 地図の更新
 */
void UpdateMap(const ScanData* pScan,
               GridMap* pMap,
               const RobotPosition2D* pPos,
               int integrationQuality,
               int holeWidth);

#endif /* TINY_SLAM_H */

