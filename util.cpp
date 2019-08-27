
/* util.cpp */

#include <fstream>
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

