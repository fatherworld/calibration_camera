
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
        int rowsHorns; //�궨����ÿ�еĽǵ���
        int colsHorns; //�궨����ÿ�еĽǵ���
        int square_size;               //ʵ�ʲ����õ��ı궨����ÿ�����̸�Ĵ�С
    } SizePic;
    /* __cplusplus*/
    // filePath �ļ��д���������̸�
    // InternalRef ����ڲ� 3 * 3 
    // DistortionCoef �������ϵ�� 5 * 1
    __declspec(dllexport) void  GetCameraMatrix(char* filePathName, double* InternalRef, double* DistortionCoef, SizePic sizepic);

   __declspec(dllexport) void ImageCorrection (char* filePathName, double* InternalRef, double* distCoeffs);

   //�������ͼƬ������һ��
   __declspec(dllexport) void CorrectOneImage(unsigned char* input_pic, unsigned char* output_pic, int width, int height, int channels, SizePic sizepic);

#ifdef __cplusplus
#if  __cplusplus
}
#endif
#endif
#endif