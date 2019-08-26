
/* tiny_slam.h */

#ifndef TINY_SLAM_H
#define TINY_SLAM_H

#include <cstdint>
#include <cmath>
#include <random>

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
 * センサデータ
 */
struct SensorData
{
    std::uint32_t mTimeStamp;           /* データ取得時のタイムスタンプ (us) */
    int           mValue[SCAN_SIZE];    /* 距離値の集合 (mm) */
    int           mDataNum;             /* 実際のデータの個数 */
    int           mOdometerCountLeft;   /* 左側の車輪のカウンタ */
    int           mOdometerCountRight;  /* 右側の車輪のカウンタ */
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
 * センサのパラメータ
 */
struct SensorInfo
{
    double mOffsetPosX;         /* ロボット中心に対するセンサの相対位置 */
    double mOffsetPosY;         /* ロボット中心に対するセンサの相対位置 */
    int    mScanSize;           /* センサデータの個数 */
    double mAngleMin;           /* 角度の最小値 (deg) */
    double mAngleMax;           /* 角度の最大値 (deg) */
    int    mDetectionMargin;    /* 取り除く両端のセンサデータの個数 */
    double mDistNoDetection;    /* 障害物を検知しない距離 (m) */
    double mFrequency;          /* 1秒間のスキャン回数 (Hz) */
};

/*
 * ロボットのパラメータ
 */
struct RobotInfo
{
    double mWheelRadius;            /* 車輪の半径 (m) */
    double mRobotRadius;            /* 車輪間の距離の半分 (m) */
    int    mOdometerCountPerTurn;   /* 1回転後のオドメータ値の増分 */
    double mOdometerRatio;          /* オドメータの重み付け値 */
};

/*
 * SLAMの状態
 */
struct SlamContext
{
    GridMap*        mpMap;                      /* 占有格子地図 */
    SensorInfo      mSensorInfo;                /* センサのパラメータ */
    RobotInfo       mRobotInfo;                 /* ロボットのパラメータ */
    RobotPosition2D mLastRobotPos;              /* ロボットの姿勢 */
    int             mLastOdometerCountLeft;     /* 以前のオドメータ値 (左側の車輪) */
    int             mLastOdometerCountRight;    /* 以前のオドメータ値 (右側の車輪) */
    std::uint32_t   mLastTimeStamp;             /* タイムスタンプ (us) */
    double          mLastRobotVelocityXY;       /* 以前のロボットの並進速度 (m/s) */
    double          mLastRobotVelocityAngle;    /* 以前のロボットの回転速度 (rad/s) */
    double          mAccumulatedTravelDist;     /* 累積走行距離 (m) */
    int             mHoleWidth;                 /* 穴のサイズ (格子の個数) */
    double          mSigmaXY;                   /* 並進移動の標準偏差 (m) */
    double          mSigmaTheta;                /* 回転移動の標準偏差 (m) */
};

/*
 * 2つのロボット位置間の距離を計算
 */
double DistanceRobotPosition2D(const RobotPosition2D* pPos0,
                               const RobotPosition2D* pPos1);

/*
 * 占有格子地図の初期化
 */
void InitializeGridMap(GridMap* pMap);

/*
 * ロボットの計測と地図との相違を計算
 */
int DistanceScanToMap(const ScanData* pScan,        /* スキャンデータ */
                      const GridMap* pMap,          /* 占有格子地図 */
                      const RobotPosition2D* pPos); /* ロボットの姿勢 */

/*
 * スキャンに基づいて格子の状態を更新
 */
void MapLaserPoint(
    GridMap* pMap,                   /* 占有格子地図 */
    int mapBeginX, int mapBeginY,    /* スキャンの始点 */
    int mapHoleX, int mapHoleY,      /* スキャンの延長された到達点 */
    int mapEndX, int mapEndY,        /* スキャンの到達点 */
    int cellValue,                   /* 占有状態を表す値 */
    int alpha);                      /* 混合率 (0から255) */

/*
 * 地図の更新
 */
void UpdateMap(
    const ScanData* pScan,          /* スキャンデータ */
    GridMap* pMap,                  /* 占有格子地図 */
    const RobotPosition2D* pPos,    /* ロボットの姿勢 */
    int integrationQuality,         /* 混合率 */
    int holeWidth);                 /* 穴のサイズ (格子の個数) */

/*
 * モンテカルロ法による位置探索
 */
RobotPosition2D MonteCarloPositionSearch(
    std::default_random_engine& randEngine, /* 擬似乱数生成器 */
    const ScanData* pScan,                  /* スキャンデータ */
    const GridMap* pMap,                    /* 占有格子地図 */
    const RobotPosition2D* pStartPos,       /* 探索開始点の座標 */
    double sigmaXY,                         /* 並進移動の標準偏差 */
    double sigmaTheta,                      /* 回転移動の標準偏差 */
    int maxIterNum);                        /* 最大の繰り返し数 */

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
    double robotVelocityAngle);     /* ロボットの回転速度 (rad/s) */

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
    int scanValue);         /* 占有状態を表す値 */

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
    double* pRobotVelocityAngle);   /* ロボットの回転角速度 (m/s) */

/*
 * SLAMの状態の初期化
 */
void InitializeSlamContext(
    SlamContext* pContext,              /* SLAMの状態 */
    GridMap* pMap,                      /* 占有格子地図 */
    const SensorInfo* pSensorInfo,      /* センサのパラメータ */
    const RobotInfo* pRobotInfo,        /* ロボットのパラメータ */
    const RobotPosition2D* pRobotPos,   /* ロボットの姿勢 */
    int holeWidth,                      /* 穴のサイズ (格子の個数) */
    double sigmaXY,                     /* 並進移動の標準偏差 (m) */
    double sigmaTheta);                 /* 回転移動の標準偏差 (m) */

/*
 * 逐次的な自己位置推定と地図構築
 */
void IterativeMapBuilding(
    std::default_random_engine& randEngine, /* 擬似乱数生成器 */
    const SensorData* pSensorData,          /* センサデータ */
    SlamContext* pContext);                 /* SLAMの状態 */

#endif /* TINY_SLAM_H */

