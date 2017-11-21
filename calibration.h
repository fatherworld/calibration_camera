
#ifndef __Calibration_H_
#define __Calibration_H_
#include <stdio.h>
// #include <string>
#define InternalRef_LONG 9
#define DistortionCoef_LONG 5
#ifdef __cplusplus
#if __cplusplus
extern "C"
{
#endif
#endif 
    typedef struct _SizePic
    {
        int rowsHorns; //标定板上每行的角点数
        int colsHorns; //标定板上每列的角点数
        int square_size;               //实际测量得到的标定板上每个棋盘格的大小
    } SizePic;
    /* __cplusplus*/
    // filePath 文件夹存放拍摄棋盘格
    // InternalRef 输出内参 3 * 3 
    // DistortionCoef 输出畸变系数 5 * 1
    __declspec(dllexport) void  GetCameraMatrix(char* filePathName, double* InternalRef, double* DistortionCoef, SizePic sizepic);

   __declspec(dllexport) void ImageCorrection (char* filePathName, double* InternalRef, double* distCoeffs);

   //输出畸变图片，矫正一张
   __declspec(dllexport) void CorrectOneImage(unsigned char* input_pic, unsigned char* output_pic, int width, int height, int channels, SizePic sizepic);

#ifdef __cplusplus
#if  __cplusplus
}
#endif
#endif
#endif