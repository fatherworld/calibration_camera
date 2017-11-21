#include "calibration.h"
#include <io.h> 
#include <direct.h>
#include <fstream>  
#include <string>
#include <vector>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
/*
����˵��
path �ļ���·��
files ���������ļ����µ��ļ����֣�����vector��
*/
static void GetAllFiles(string path, vector<string>& files)
{
    intptr_t   hFile = 0;
    //�ļ���Ϣ    
    struct _finddata_t fileinfo;//�����洢�ļ���Ϣ�Ľṹ��    
    string p;
    if ((hFile = _findfirst(p.assign(path).append("\\*.*").c_str(), &fileinfo)) != -1)  //��һ�β���  
    {
        do
        {
            if ((fileinfo.attrib &  _A_SUBDIR))  //������ҵ������ļ���  
            {
                if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)  //�����ļ��в���  
                {
                    files.push_back(p.assign(path).append("\\").append(fileinfo.name));
                    GetAllFiles(p.assign(path).append("\\").append(fileinfo.name), files);

                }
            }
            else //������ҵ��Ĳ������ļ���   
            {
                //files.push_back(p.assign(fileinfo.name));  //���ļ�·�����棬Ҳ����ֻ�����ļ���:    p.assign(path).append("\\").append(fileinfo.name)  

                files.push_back(p.assign(path).append("\\").append(fileinfo.name));
            }

        } while (_findnext(hFile, &fileinfo) == 0);

        _findclose(hFile); //��������  
    }
}



//��ȡ������ڲκͻ������

__declspec(dllexport) void GetCameraMatrix(char* filePathName, double* InternalRef, double* DistortionCoef, SizePic sizepic)
{
    int i, j, t;
    string filePath = filePathName;
    ofstream fout("caliberation_result_1.txt");  /* ����궨������ļ� */
    vector<string> fileName;
    GetAllFiles(filePath, fileName);

    int image_count = 0;  /* ͼ������ */
    Size image_size;  /* ͼ��ĳߴ� */
    Size board_size = Size(sizepic.colsHorns, sizepic.rowsHorns);

 //   Size board_size = Size(4, 6);    /* �궨����ÿ�С��еĽǵ��� */
    vector<Point2f> image_points_buf;  /* ����ÿ��ͼ���ϼ�⵽�Ľǵ� */
    vector<vector<Point2f>> image_points_seq; /* �����⵽�����нǵ� */
    int count = -1;//���ڴ洢�ǵ������
    bool no_boardCorners = false;
    for (int k = 0; k < fileName.size()-1; k++)
    {
        Mat imageInput = imread(fileName[k]);
        //Mat imageInput = imread(temp);
        /* ��ȡ�ǵ� */
        if (0 == findChessboardCorners(imageInput, board_size, image_points_buf))
        {
            no_boardCorners = true;
            cout << "can not find chessboard corners!\n"; //�Ҳ����ǵ�
            fout << fileName[k] << "û�з��ֽǵ㣬��ɾ����ͼƬ������������һ��ͼƬ���滻��ͼƬ" << "\n";
        }
    }
    if(no_boardCorners)
    exit(1);//ͳ���ļ���û�г��ֽǵ��ͼƬ����


    for (int k = 0; k < fileName.size()-1; k++)
    {
        image_count++;
        // ���ڹ۲�������
        cout << "image_count = " << image_count << endl;
        /* �������*/
        cout << "-->count = " << count;
        //char temp[200];
        //sprintf_s(temp, 200, "./srcImage/chess%d.bmp", k + 1);

        Mat imageInput = imread(fileName[k]);
        //Mat imageInput = imread(temp);
        if (image_count == 1)  //�����һ��ͼƬʱ��ȡͼ������Ϣ
        {
            image_size.width = imageInput.cols;
            image_size.height = imageInput.rows;
            cout << "image_size.width = " << image_size.width << endl;
            cout << "image_size.height = " << image_size.height << endl;
            cout << "image_size.type = " << imageInput.type() << endl;
        }
        /* ��ȡ�ǵ� */
        if (0 == findChessboardCorners(imageInput, board_size, image_points_buf))
        {
            cout << "can not find chessboard corners!\n"; //�Ҳ����ǵ�
            fout << fileName[k] << "û�з��ֽǵ㣬��ɾ����ͼƬ������������һ��ͼƬ���滻��ͼƬ" << "\n";
            exit(1);
        }
        else
        {
            Mat view_gray;
            cvtColor(imageInput, view_gray, CV_RGB2GRAY);
            /* �����ؾ�ȷ�� */
            find4QuadCornerSubpix(view_gray, image_points_buf, Size(5, 5)); //�Դ���ȡ�Ľǵ���о�ȷ��
            image_points_seq.push_back(image_points_buf);  //���������ؽǵ�
                                                           /* ��ͼ������ʾ�ǵ�λ�� */
            drawChessboardCorners(view_gray, board_size, image_points_buf, true); //������ͼƬ�б�ǽǵ�
            cv::namedWindow("Camera Calibration", 1);
            imshow("Camera Calibration", view_gray);//��ʾͼƬ
            waitKey(500);//��ͣ0.5S		
        }
    }
    int total = image_points_seq.size();
    cout << "total = " << total << endl;
    int CornerNum = board_size.width*board_size.height;  //ÿ��ͼƬ���ܵĽǵ���
    for (int ii = 0; ii < total; ii++)
    {
        if (0 == ii%CornerNum)// 24 ��ÿ��ͼƬ�Ľǵ���������ж������Ϊ����� ͼƬ�ţ����ڿ���̨�ۿ� 
        {
            int i = -1;
            i = ii / CornerNum;
            int j = i + 1;
            cout << "--> �� " << j << "ͼƬ������ --> : " << endl;
        }
        if (0 == ii % 3)	// ���ж���䣬��ʽ����������ڿ���̨�鿴
        {
            cout << endl;
        }
        else
        {
            cout.width(10);
        }
        //������еĽǵ�
        cout << " -->" << image_points_seq[ii][0].x;
        cout << " -->" << image_points_seq[ii][0].y;
    }
    cout << "�ǵ���ȡ��ɣ�\n";
    //������������궨
    cout << "��ʼ�궨������������";
    /*������ά��Ϣ*/
    //Size square_size = Size(10, 10);  /* ʵ�ʲ����õ��ı궨����ÿ�����̸�Ĵ�С */
    Size square_size = Size(sizepic.square_size, sizepic.square_size);  /* ʵ�ʲ����õ��ı궨����ÿ�����̸�Ĵ�С */

    vector<vector<Point3f>> object_points; /* ����궨���Ͻǵ����ά���� */
                                           /*�������*/
    Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* ������ڲ������� */
    vector<int> point_counts;  // ÿ��ͼ���нǵ������
    Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); /* �������5������ϵ����k1,k2,p1,p2,k3 */
    vector<Mat> tvecsMat;  /* ÿ��ͼ�����ת���� */
    vector<Mat> rvecsMat; /* ÿ��ͼ���ƽ������ */
                          /* ��ʼ���궨���Ͻǵ����ά���� */
    for (t = 0; t < image_count; t++)
    {
        vector<Point3f> tempPointSet;
        for (i = 0; i < board_size.height; i++)
        {
            for (j = 0; j < board_size.width; j++)
            {
                Point3f realPoint;
                /* ����궨�������������ϵ��z=0��ƽ���� */
                realPoint.x = i*square_size.width;
                realPoint.y = j*square_size.height;
                realPoint.z = 0;
                tempPointSet.push_back(realPoint);
            }
        }
     object_points.push_back(tempPointSet);
    }
    /* ��ʼ��ÿ��ͼ���еĽǵ��������ٶ�ÿ��ͼ���ж����Կ��������ı궨�� */
    for (i = 0; i < image_count; i++)
    {
        point_counts.push_back(board_size.width*board_size.height);
    }
    /* ��ʼ�궨 */
    calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
    for (i=0;i < InternalRef_LONG; i++)
    {
       InternalRef[i] = (double)cameraMatrix.at<double>(i/3, i%3);
       fout << "aaaaaaaa" << InternalRef[i] << endl;
    }
    for (i = 0; i < DistortionCoef_LONG; i++)
    {
        DistortionCoef[i] = (double)distCoeffs.at<double>(0,i);
        fout << "bbbbbbbb" << DistortionCoef[i] << endl;
    }
    fout << "����궨��ɣ�" << endl;
    fout << "����ڲ�������" << endl;
    fout << cameraMatrix << endl << endl;
    fout << "����ϵ����\n";
    fout << distCoeffs << endl << endl << endl;
    fout.close();
}

static void ImageCorrections(Mat &imageSource, Mat &newimage,double*& cameraMatrixss, double*& distCoeffss)
{
    Size image_size;
    image_size.width = imageSource.cols;
    image_size.height = imageSource.rows;
    int i = 0;
    Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* ������ڲ������� */
    Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); /* �������5������ϵ����k1,k2,p1,p2,k3 */
    for (;i < InternalRef_LONG; i++)
    {
        cameraMatrix.at<float>(i / 3, i % 3) = (float)cameraMatrixss[i];
    }
    for (i = 0; i < DistortionCoef_LONG; i++)
    {
        distCoeffs.at<float>(0, i) =(float)distCoeffss[i];
    }
    Mat mapx = Mat(image_size, CV_32FC1);
    Mat mapy = Mat(image_size, CV_32FC1);
    Mat R = Mat::eye(3, 3, CV_32F);
    initUndistortRectifyMap(cameraMatrix, distCoeffs, R, cameraMatrix, image_size, CV_32FC1, mapx, mapy);
    newimage = imageSource.clone();
    //��һ�ֲ���Ҫת������ķ�ʽ
 //   undistort(imageSource,newimage,cameraMatrix,distCoeffs);
    remap(imageSource, newimage, mapx, mapy, INTER_LINEAR);
    std::cout << "�������" << endl;
}


void __declspec(dllexport) ImageCorrection(char* filePathName, double* InternalRef, double* distCoeffs)
{

    string filePath = filePathName;
    vector<string> fileName;
    GetAllFiles(filePath, fileName);
    string outFile = "./CorrectImage";
    Mat imageSource;
    if (_access(outFile.c_str(), 00) != 0) {
        _mkdir(outFile.c_str());
    }
    for (int i = 0; i < fileName.size(); i++)
    {
        Mat newimage;
        imageSource = imread(fileName[i], 1);
        ImageCorrections(imageSource, newimage, InternalRef, distCoeffs);
        string str = fileName[i];
        int n = str.rfind("\\");
        string name = str.substr(n + 1, str.length() - n - 1); //��·���л�ȡ�ļ�������  

        name = outFile + "/" + name + "_Correct.bmp";

        imwrite(name, newimage);
        cout << name << endl;
        imshow("src", imageSource);
        imshow("correct", newimage);
        waitKey(10);
    }
}




__declspec(dllexport) void CorrectOneImage(unsigned char* input_pic, unsigned char* output_pic, int width, int height,int channels, SizePic sizepic)
{
    if (!input_pic || width == 0 || height == 0)
    {
        cout << "����ͼƬ����Ϊ��" << endl;
        return;
    }
    if (!output_pic)
    {
        cout << "���ͼƬ�ռ��С����Ϊ��" << endl;
        return;
    }
    double* InternalRef = (double*)malloc(sizeof(double) * 9);//�ڲ�ϵ��
    double* DistortionCoef = (double*)malloc(sizeof(double) * 5);//����ϵ��
    memset(InternalRef, 0, sizeof(double) * 9);
    memset(DistortionCoef, 0, sizeof(double) * 5);
    Size image_size;  /* ͼ��ĳߴ� */
    image_size.height = height;
    image_size.width = width;
    Mat imageInput(height,width,16, Scalar::all(0));//��������ͼƬ
    Mat imageOut(height, width, 16, Scalar::all(0));//�������ͼƬ
    memcpy(imageInput.data, input_pic, width*height*channels);
    namedWindow("sourceImage");
    imshow("sourceImage", imageInput);
    waitKey(1500);
    Size board_size = Size(sizepic.colsHorns, sizepic.rowsHorns);//���ñ궨����ÿ��ÿ�еĽǵ����

    vector<Point2f> image_points_buf;  /* ����ÿ��ͼ���ϼ�⵽�Ľǵ� */
    vector<vector<Point2f>> image_points_seq; /* �����⵽�����нǵ� */

    if (0 == findChessboardCorners(imageInput, board_size, image_points_buf))
    {
        cout << "can not find chessboard corners!\n"; //�Ҳ����ǵ�
        cout << "û�з��ֽǵ㣬��ɾ����ͼƬ������������һ��ͼƬ���滻��ͼƬ" << endl;
        exit(1);
    }
    else
    {
        Mat view_gray;
        cvtColor(imageInput, view_gray, CV_RGB2GRAY);
        /* �����ؾ�ȷ�� */
        find4QuadCornerSubpix(view_gray, image_points_buf, Size(5, 5)); //�Դ���ȡ�Ľǵ���о�ȷ��
        image_points_seq.push_back(image_points_buf);  //���������ؽǵ�
                                                       /* ��ͼ������ʾ�ǵ�λ�� */
        drawChessboardCorners(view_gray, board_size, image_points_buf, true); //������ͼƬ�б�ǽǵ�
        cv::namedWindow("Camera Calibration", 1);
        imshow("Camera Calibration", view_gray);//��ʾͼƬ
        waitKey(1500);//��ͣ0.5S		
    }

    Size square_size = Size(sizepic.square_size, sizepic.square_size);  /* ʵ�ʲ����õ��ı궨����ÿ�����̸�Ĵ�С */

    vector<vector<Point3f>> object_points; /* ����궨���Ͻǵ����ά���� */
                                           /*�������*/
    int i, j;
    vector<Point3f> tempPointSet;
    Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* ������ڲ������� */
    vector<int> point_counts;  // ÿ��ͼ���нǵ������
    Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); /* �������5������ϵ����k1,k2,p1,p2,k3 */
    vector<Mat> tvecsMat;  /* ÿ��ͼ�����ת���� */
    vector<Mat> rvecsMat; /* ÿ��ͼ���ƽ������ */
                            /* ��ʼ���궨���Ͻǵ����ά���� */
    for (i = 0; i < board_size.height; i++)
    {
        for (j = 0; j < board_size.width; j++)
        {
            Point3f realPoint;
            /* ����궨�������������ϵ��z=0��ƽ���� */
            realPoint.x = i*square_size.width;
            realPoint.y = j*square_size.height;
            realPoint.z = 0;
            tempPointSet.push_back(realPoint);
        }
    }
    object_points.push_back(tempPointSet);

    /* ��ʼ��ÿ��ͼ���еĽǵ��������ٶ�ÿ��ͼ���ж����Կ��������ı궨�� */
    point_counts.push_back(board_size.width*board_size.height);
    /* ��ʼ�궨 */
    calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
    for (i = 0; i < InternalRef_LONG; i++)
    {
        InternalRef[i] = (double)cameraMatrix.at<double>(i / 3, i % 3);//����ȡ�����ڲ���Ϣ���浽�ڴ�������
    }
    for (i = 0; i < DistortionCoef_LONG; i++)
    {
        DistortionCoef[i] = (double)distCoeffs.at<double>(0, i);//����ȡ���Ļ�����Ϣ���浽����������
    }
    ImageCorrections(imageInput,imageOut, InternalRef, DistortionCoef);
    namedWindow("correctImage");
    imshow("correctImage", imageOut);
    waitKey(1500);
    memcpy(output_pic, imageOut.data, width*height*channels);
    return;
}





