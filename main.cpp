
/* main.cpp */

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <iterator>
#include <random>
#include <sstream>
#include <string>
#include <vector>

#include "tiny_slam.h"
#include "util.h"

const size_t MaxSensorDataSize = 5000;
const int ScanSize = 682;

void PrintSensorData(const SensorData& sensorData)
{
    std::cout << "Time stamp (us): " << sensorData.mTimeStamp << ", "
              << "Odometer left: " << sensorData.mOdometerCountLeft << ", "
              << "Odometer right: " << sensorData.mOdometerCountRight << ", "
              << "The number of values: " << sensorData.mDataNum << ", "
              << "Values: ";

    std::for_each(std::begin(sensorData.mValue),
                  std::begin(sensorData.mValue) + sensorData.mDataNum,
                  [](int distValue) { std::cout << distValue << " "; });
    
    std::cout << '\n';
}

std::vector<std::string> SplitLine(const std::string& str)
{
    std::istringstream strStream { str };
    
    std::istream_iterator<std::string> strBegin { strStream };
    std::istream_iterator<std::string> strEnd;

    return std::vector<std::string>(strBegin, strEnd);
}

bool ReadSensorData(const char* fileName,
                    std::vector<SensorData>& sensorData)
{
    std::ifstream inFile(fileName);
    std::string line;

    if (!inFile)
        return false;

    while (std::getline(inFile, line)) {
        std::vector<std::string> splitTokens = SplitLine(line);
        SensorData readData;
        
        /* スキャン点の個数 */
        readData.mDataNum = 0;
        
        /* データが取得されたタイムスタンプ (us) */
        readData.mTimeStamp = std::stoi(splitTokens[0], nullptr, 10);

        /* 左側の車輪のオドメータ値 */
        readData.mOdometerCountLeft = std::stoi(splitTokens[2], nullptr, 10);
        /* 右側の車輪のオドメータ値 */
        readData.mOdometerCountRight = std::stoi(splitTokens[3], nullptr, 10);
        
        /* 各スキャン点への距離 (mm) */
        for (size_t i = 25; i < splitTokens.size(); ++i)
            if (readData.mDataNum < SCAN_SIZE)
                readData.mValue[readData.mDataNum++] =
                    std::stoi(splitTokens[i], nullptr, 10);

        sensorData.push_back(readData);
    }

    inFile.close();

    return true;
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << ' '
                  << "<data file name>" << ' '
                  << "[<loop index>]" << '\n';
        return EXIT_FAILURE;
    }

    /* 擬似乱数生成器の生成 */
    std::random_device seedGen;
    std::default_random_engine randEngine { seedGen() };
    
    /* オドメトリとスキャンデータを格納するファイル名 */
    std::string fileName = argv[1];

    size_t lastPeriodIndex = fileName.find_last_of('.');
    std::string dataName =
        (lastPeriodIndex == std::string::npos) ?
        fileName : fileName.substr(0, lastPeriodIndex);

    /* センサデータの読み込み */
    std::vector<SensorData> sensorData;

    if (!ReadSensorData(fileName.c_str(), sensorData)) {
        std::cerr << "Failed to load file \'" << fileName << "\'\n";
        return EXIT_FAILURE;
    }

    /* ループした時点でのデータのインデックス */
    int loopNum;
    int loopEndIndex[2];

    if (argc == 3) {
        /* ループするときのインデックスが指定されている場合 */
        loopNum = 2;
        loopEndIndex[0] = std::stoi(argv[2]);
        loopEndIndex[1] = static_cast<int>(sensorData.size());
    } else {
        /* データにループが含まれない場合 */
        loopNum = 1;
        loopEndIndex[0] = static_cast<int>(sensorData.size());
        loopEndIndex[1] = -1;
    }

    /* ロボットのパラメータの設定 */
    RobotInfo robotInfo;
    robotInfo.mWheelRadius = 0.077;
    robotInfo.mRobotRadius = 0.165;
    robotInfo.mOdometerCountPerTurn = 2000;
    robotInfo.mOdometerRatio = 1.0;

    /* センサのパラメータの設定 */
    SensorInfo sensorInfo;
    sensorInfo.mOffsetPosX = 0.145;
    sensorInfo.mOffsetPosY = 0.0;
    sensorInfo.mScanSize = ScanSize;
    sensorInfo.mAngleMin = -120.0;
    sensorInfo.mAngleMax = +120.0;
    sensorInfo.mDetectionMargin = 70;
    sensorInfo.mDistNoDetection = 4.0;
    sensorInfo.mFrequency = 10.0;

    /* 開始点の座標 */
    RobotPosition2D startPos;
    startPos.mX = 0.5 * MAP_SIZE / CELLS_PER_METER;
    startPos.mY = 0.6 * MAP_SIZE / CELLS_PER_METER;
    startPos.mTheta = 0.0;

    /* 地図の初期化 */
    GridMap* pResultMap = new GridMap();
    InitializeGridMap(pResultMap);
    
    /* ロボットの軌跡を記録するために用いる地図 */
    GridMap* pTrajectoryMap = new GridMap();
    InitializeGridMap(pTrajectoryMap);
    
    /* ループ閉じ込み(位置調整)のために用いる地図 */
    GridMap* pLoopMap = new GridMap();
    InitializeGridMap(pLoopMap);
    
    /* TinySLAMの実行 */
    SlamContext slamContext;

    int loopCount = 0;
    int loopStart = 0;
    RobotPosition2D loopStartPos = startPos;

    int scanIndex;
    
    /* 各ループごとに処理を実行 */
    /* ここでは途中のループ検出を考慮しない */
    for (loopCount = 0; loopCount < loopNum; ++loopCount) {
        /* SLAMの状態の初期化 */
        InitializeSlamContext(&slamContext,
                              pResultMap, &sensorInfo,
                              &robotInfo, &loopStartPos, 
                              0.6, 0.1, 20.0);

        /* 各センサデータごとの処理を開始 */
        for (scanIndex = loopStart;
             scanIndex < loopEndIndex[loopCount]; ++scanIndex) {
            /* センサデータを基に地図を構築 */
            IterativeMapBuilding(randEngine,
                                 &sensorData[scanIndex],
                                 &slamContext,
                                 true);

            /* 現在のロボットの姿勢を表示 */
            std::cerr << "Scan " << scanIndex << ", "
                      << "Robot position x: " << slamContext.mLastRobotPos.mX << ", "
                      << "y: " << slamContext.mLastRobotPos.mY << ", "
                      << "theta: " << slamContext.mLastRobotPos.mTheta << '\n';

            /* ループ閉じ込みに用いる地図を保存 */
            if (scanIndex == 50)
                *pLoopMap = *pResultMap;
            
            /* ロボットの軌跡を記録 */
            int mapX = static_cast<int>(std::floor(
                slamContext.mLastRobotPos.mX * CELLS_PER_METER + 0.5));
            int mapY = static_cast<int>(std::floor(
                slamContext.mLastRobotPos.mY * CELLS_PER_METER + 0.5));

            if (mapX >= 0 && mapX < MAP_SIZE &&
                mapY >= 0 && mapY < MAP_SIZE)
                pTrajectoryMap->mCells[mapY * MAP_SIZE + mapX] = 0;
        }
        
        /* 累積走行距離や姿勢を表示 */
        std::cerr << "Accumulated travel distance: "
                  << slamContext.mAccumulatedTravelDist << '\n';
        std::cerr << "Start position x: " << startPos.mX << ", "
                  << "y: " << startPos.mY << ", "
                  << "theta: " << startPos.mTheta << '\n';
        std::cerr << "End position x: " << slamContext.mLastRobotPos.mX << ", "
                  << "y: " << slamContext.mLastRobotPos.mY << ", "
                  << "theta: " << slamContext.mLastRobotPos.mTheta << '\n';

        /* 地図を画像として保存 */
        std::string forwardFileName = dataName;
        forwardFileName += "-loop";
        forwardFileName += std::to_string(loopCount);
        forwardFileName += "-forward.pgm";

        SaveMapImagePgm(pResultMap, pTrajectoryMap,
                        forwardFileName.c_str(),
                        MAP_SIZE, MAP_SIZE, MAP_SIZE, MAP_SIZE);

        /* 簡易的なループ閉じ込みの実行 */
        std::cerr << "Performing loop closure ..." << '\n';
        
        /* ロボット位置をセンサ位置に変換 */
        RobotPosition2D sensorPos = slamContext.mLastRobotPos;
        double cosTheta = std::cos(DEG_TO_RAD(sensorPos.mTheta));
        double sinTheta = std::sin(DEG_TO_RAD(sensorPos.mTheta));

        sensorPos.mX += (slamContext.mSensorInfo.mOffsetPosX * cosTheta -
                         slamContext.mSensorInfo.mOffsetPosY * sinTheta);
        sensorPos.mY += (slamContext.mSensorInfo.mOffsetPosX * sinTheta +
                         slamContext.mSensorInfo.mOffsetPosY * cosTheta);
        
        /* スキャンと地図との相違 */
        int bestDistScanAndMap;

        RobotPosition2D loopEndPos = LoopClosurePosition(
            randEngine,
            &sensorData[loopEndIndex[loopCount] - 1],
            pLoopMap,
            &sensorPos,
            slamContext.mSensorInfo.mFrequency,
            slamContext.mSensorInfo.mDetectionMargin,
            slamContext.mSensorInfo.mScanSize,
            slamContext.mSensorInfo.mAngleMin,
            slamContext.mSensorInfo.mAngleMax,
            slamContext.mSensorInfo.mDistNoDetection,
            slamContext.mHoleWidth,
            &bestDistScanAndMap);
        
        /* センサ位置をロボット位置に戻す */
        cosTheta = std::cos(DEG_TO_RAD(loopEndPos.mTheta));
        sinTheta = std::sin(DEG_TO_RAD(loopEndPos.mTheta));

        loopEndPos.mX -= (slamContext.mSensorInfo.mOffsetPosX * cosTheta -
                          slamContext.mSensorInfo.mOffsetPosY * sinTheta);
        loopEndPos.mY -= (slamContext.mSensorInfo.mOffsetPosX * sinTheta +
                          slamContext.mSensorInfo.mOffsetPosY * cosTheta);
        
        /* ループ閉じ込み後のロボットの姿勢 */
        std::cout << "Position after loop-closure x: " << loopEndPos.mX << ", "
                  << "y: " << loopEndPos.mY << ", "
                  << "theta: " << loopEndPos.mTheta << '\n';
        
        /* ループ閉じ込み前後の姿勢の変化量 */
        double distanceDiff = DistanceRobotPosition2D(
            &loopEndPos, &(slamContext.mLastRobotPos));
        double angleDiff = std::fabs(
            loopEndPos.mTheta - slamContext.mLastRobotPos.mTheta);

        std::cout << "Distance difference (m): " << distanceDiff << ", "
                  << "angle difference (deg): " << angleDiff << '\n';
        
        /* 地図をループ閉じ込みに用いたもので置き換える */
        *pResultMap = *pLoopMap;

        /* ロボットの軌跡を消去 */
        InitializeGridMap(pTrajectoryMap);

        /* SLAMの状態を再度リセット */
        InitializeSlamContext(&slamContext,
                              pResultMap, &sensorInfo, &robotInfo, &loopEndPos,
                              0.6, 0.1, 20.0);
        
        /* 復路のSLAMを実行 */
        for (scanIndex = loopEndIndex[loopCount] - 1;
             scanIndex >= loopStart; --scanIndex) {
            /* センサデータを再度用いて地図を構築 */
            IterativeMapBuilding(randEngine,
                                 &sensorData[scanIndex],
                                 &slamContext,
                                 false);

            /* 現在のロボットの姿勢を表示 */
            std::cerr << "Scan " << scanIndex << ", "
                      << "Robot position x: " << slamContext.mLastRobotPos.mX << ", "
                      << "y: " << slamContext.mLastRobotPos.mY << ", "
                      << "theta: " << slamContext.mLastRobotPos.mTheta << '\n';
            
            /* ロボットの軌跡を記録 */
            int mapX = static_cast<int>(std::floor(
                slamContext.mLastRobotPos.mX * CELLS_PER_METER + 0.5));
            int mapY = static_cast<int>(std::floor(
                slamContext.mLastRobotPos.mY * CELLS_PER_METER + 0.5));

            if (mapX >= 0 && mapX < MAP_SIZE &&
                mapY >= 0 && mapY < MAP_SIZE)
                pTrajectoryMap->mCells[mapY * MAP_SIZE + mapX] = 0;
        }

        /* 復路の累積走行距離や現在の姿勢を表示 */
        std::cerr << "Accumulated travel distance (backward): "
                  << slamContext.mAccumulatedTravelDist << '\n';
        
        /* 開始点との位置ずれ */
        distanceDiff = DistanceRobotPosition2D(
            &startPos, &(slamContext.mLastRobotPos));
        std::cerr << "Distance between start point: " << distanceDiff << '\n';

        /* 復路の地図を画像として保存 */
        std::string backwardFileName = dataName;
        backwardFileName += "-loop";
        backwardFileName += std::to_string(loopCount);
        backwardFileName += "-backward.pgm";

        SaveMapImagePgm(pResultMap, pTrajectoryMap,
                        backwardFileName.c_str(),
                        MAP_SIZE, MAP_SIZE, MAP_SIZE, MAP_SIZE);

        /* 簡易的なループ閉じ込みの実行 */
        std::cerr << "Performing loop closure (backward)..." << '\n';

        LoopClosureTrajectory(&(sensorData[loopStart]),
                              loopEndIndex[loopCount] - loopStart);
        
        /* ループ閉じ込みに用いる地図を更新 */
        *pLoopMap = *pResultMap;

        loopStartPos = loopEndPos;
        loopStart = loopEndIndex[loopCount];
    }
    
    /* 地図の破棄 */
    delete pLoopMap;
    pLoopMap = nullptr;

    delete pTrajectoryMap;
    pTrajectoryMap = nullptr;

    delete pResultMap;
    pResultMap = nullptr;

    return EXIT_SUCCESS;
}

