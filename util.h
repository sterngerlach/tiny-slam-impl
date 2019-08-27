
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

#endif /* UTIL_H */

