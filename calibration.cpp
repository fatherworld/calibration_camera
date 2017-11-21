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
参数说明
path 文件夹路径
files 返回所有文件夹下的文件名字，存在vector中
*/
static void GetAllFiles(string path, vector<string>& files)
{
    intptr_t   hFile = 0;
    //文件信息    
    struct _finddata_t fileinfo;//用来存储文件信息的结构体    
    string p;
    if ((hFile = _findfirst(p.assign(path).append("\\*.*").c_str(), &fileinfo)) != -1)  //第一次查找  
    {
        do
        {
            if ((fileinfo.attrib &  _A_SUBDIR))  //如果查找到的是文件夹  
            {
                if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)  //进入文件夹查找  
                {
                    files.push_back(p.assign(path).append("\\").append(fileinfo.name));
                    GetAllFiles(p.assign(path).append("\\").append(fileinfo.name), files);

                }
            }
            else //如果查找到的不是是文件夹   
            {
                //files.push_back(p.assign(fileinfo.name));  //将文件路径保存，也可以只保存文件名:    p.assign(path).append("\\").append(fileinfo.name)  

                files.push_back(p.assign(path).append("\\").append(fileinfo.name));
            }

        } while (_findnext(hFile, &fileinfo) == 0);

        _findclose(hFile); //结束查找  
    }
}



//获取焦点的内参和畸变参数

__declspec(dllexport) void GetCameraMatrix(char* filePathName, double* InternalRef, double* DistortionCoef, SizePic sizepic)
{
    int i, j, t;
    string filePath = filePathName;
    ofstream fout("caliberation_result_1.txt");  /* 保存标定结果的文件 */
    vector<string> fileName;
    GetAllFiles(filePath, fileName);

    int image_count = 0;  /* 图像数量 */
    Size image_size;  /* 图像的尺寸 */
    Size board_size = Size(sizepic.colsHorns, sizepic.rowsHorns);

 //   Size board_size = Size(4, 6);    /* 标定板上每行、列的角点数 */
    vector<Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */
    vector<vector<Point2f>> image_points_seq; /* 保存检测到的所有角点 */
    int count = -1;//用于存储角点个数。
    bool no_boardCorners = false;
    for (int k = 0; k < fileName.size()-1; k++)
    {
        Mat imageInput = imread(fileName[k]);
        //Mat imageInput = imread(temp);
        /* 提取角点 */
        if (0 == findChessboardCorners(imageInput, board_size, image_points_buf))
        {
            no_boardCorners = true;
            cout << "can not find chessboard corners!\n"; //找不到角点
            fout << fileName[k] << "没有发现角点，请删除该图片，或者再拍摄一张图片，替换该图片" << "\n";
        }
    }
    if(no_boardCorners)
    exit(1);//统计文件中没有出现角点的图片数量


    for (int k = 0; k < fileName.size()-1; k++)
    {
        image_count++;
        // 用于观察检验输出
        cout << "image_count = " << image_count << endl;
        /* 输出检验*/
        cout << "-->count = " << count;
        //char temp[200];
        //sprintf_s(temp, 200, "./srcImage/chess%d.bmp", k + 1);

        Mat imageInput = imread(fileName[k]);
        //Mat imageInput = imread(temp);
        if (image_count == 1)  //读入第一张图片时获取图像宽高信息
        {
            image_size.width = imageInput.cols;
            image_size.height = imageInput.rows;
            cout << "image_size.width = " << image_size.width << endl;
            cout << "image_size.height = " << image_size.height << endl;
            cout << "image_size.type = " << imageInput.type() << endl;
        }
        /* 提取角点 */
        if (0 == findChessboardCorners(imageInput, board_size, image_points_buf))
        {
            cout << "can not find chessboard corners!\n"; //找不到角点
            fout << fileName[k] << "没有发现角点，请删除该图片，或者再拍摄一张图片，替换该图片" << "\n";
            exit(1);
        }
        else
        {
            Mat view_gray;
            cvtColor(imageInput, view_gray, CV_RGB2GRAY);
            /* 亚像素精确化 */
            find4QuadCornerSubpix(view_gray, image_points_buf, Size(5, 5)); //对粗提取的角点进行精确化
            image_points_seq.push_back(image_points_buf);  //保存亚像素角点
                                                           /* 在图像上显示角点位置 */
            drawChessboardCorners(view_gray, board_size, image_points_buf, true); //用于在图片中标记角点
            cv::namedWindow("Camera Calibration", 1);
            imshow("Camera Calibration", view_gray);//显示图片
            waitKey(500);//暂停0.5S		
        }
    }
    int total = image_points_seq.size();
    cout << "total = " << total << endl;
    int CornerNum = board_size.width*board_size.height;  //每张图片上总的角点数
    for (int ii = 0; ii < total; ii++)
    {
        if (0 == ii%CornerNum)// 24 是每幅图片的角点个数。此判断语句是为了输出 图片号，便于控制台观看 
        {
            int i = -1;
            i = ii / CornerNum;
            int j = i + 1;
            cout << "--> 第 " << j << "图片的数据 --> : " << endl;
        }
        if (0 == ii % 3)	// 此判断语句，格式化输出，便于控制台查看
        {
            cout << endl;
        }
        else
        {
            cout.width(10);
        }
        //输出所有的角点
        cout << " -->" << image_points_seq[ii][0].x;
        cout << " -->" << image_points_seq[ii][0].y;
    }
    cout << "角点提取完成！\n";
    //以下是摄像机标定
    cout << "开始标定………………";
    /*棋盘三维信息*/
    //Size square_size = Size(10, 10);  /* 实际测量得到的标定板上每个棋盘格的大小 */
    Size square_size = Size(sizepic.square_size, sizepic.square_size);  /* 实际测量得到的标定板上每个棋盘格的大小 */

    vector<vector<Point3f>> object_points; /* 保存标定板上角点的三维坐标 */
                                           /*内外参数*/
    Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 摄像机内参数矩阵 */
    vector<int> point_counts;  // 每幅图像中角点的数量
    Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
    vector<Mat> tvecsMat;  /* 每幅图像的旋转向量 */
    vector<Mat> rvecsMat; /* 每幅图像的平移向量 */
                          /* 初始化标定板上角点的三维坐标 */
    for (t = 0; t < image_count; t++)
    {
        vector<Point3f> tempPointSet;
        for (i = 0; i < board_size.height; i++)
        {
            for (j = 0; j < board_size.width; j++)
            {
                Point3f realPoint;
                /* 假设标定板放在世界坐标系中z=0的平面上 */
                realPoint.x = i*square_size.width;
                realPoint.y = j*square_size.height;
                realPoint.z = 0;
                tempPointSet.push_back(realPoint);
            }
        }
     object_points.push_back(tempPointSet);
    }
    /* 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板 */
    for (i = 0; i < image_count; i++)
    {
        point_counts.push_back(board_size.width*board_size.height);
    }
    /* 开始标定 */
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
    fout << "相机标定完成：" << endl;
    fout << "相机内参数矩阵：" << endl;
    fout << cameraMatrix << endl << endl;
    fout << "畸变系数：\n";
    fout << distCoeffs << endl << endl << endl;
    fout.close();
}

static void ImageCorrections(Mat &imageSource, Mat &newimage,double*& cameraMatrixss, double*& distCoeffss)
{
    Size image_size;
    image_size.width = imageSource.cols;
    image_size.height = imageSource.rows;
    int i = 0;
    Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 摄像机内参数矩阵 */
    Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
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
    //另一种不需要转换矩阵的方式
 //   undistort(imageSource,newimage,cameraMatrix,distCoeffs);
    remap(imageSource, newimage, mapx, mapy, INTER_LINEAR);
    std::cout << "保存结束" << endl;
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
        string name = str.substr(n + 1, str.length() - n - 1); //从路径中获取文件的名字  

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
        cout << "输入图片不能为空" << endl;
        return;
    }
    if (!output_pic)
    {
        cout << "输出图片空间大小不能为空" << endl;
        return;
    }
    double* InternalRef = (double*)malloc(sizeof(double) * 9);//内参系数
    double* DistortionCoef = (double*)malloc(sizeof(double) * 5);//畸变系数
    memset(InternalRef, 0, sizeof(double) * 9);
    memset(DistortionCoef, 0, sizeof(double) * 5);
    Size image_size;  /* 图像的尺寸 */
    image_size.height = height;
    image_size.width = width;
    Mat imageInput(height,width,16, Scalar::all(0));//缓存输入图片
    Mat imageOut(height, width, 16, Scalar::all(0));//缓存输出图片
    memcpy(imageInput.data, input_pic, width*height*channels);
    namedWindow("sourceImage");
    imshow("sourceImage", imageInput);
    waitKey(1500);
    Size board_size = Size(sizepic.colsHorns, sizepic.rowsHorns);//设置标定板上每行每列的角点个数

    vector<Point2f> image_points_buf;  /* 缓存每幅图像上检测到的角点 */
    vector<vector<Point2f>> image_points_seq; /* 保存检测到的所有角点 */

    if (0 == findChessboardCorners(imageInput, board_size, image_points_buf))
    {
        cout << "can not find chessboard corners!\n"; //找不到角点
        cout << "没有发现角点，请删除该图片，或者再拍摄一张图片，替换该图片" << endl;
        exit(1);
    }
    else
    {
        Mat view_gray;
        cvtColor(imageInput, view_gray, CV_RGB2GRAY);
        /* 亚像素精确化 */
        find4QuadCornerSubpix(view_gray, image_points_buf, Size(5, 5)); //对粗提取的角点进行精确化
        image_points_seq.push_back(image_points_buf);  //保存亚像素角点
                                                       /* 在图像上显示角点位置 */
        drawChessboardCorners(view_gray, board_size, image_points_buf, true); //用于在图片中标记角点
        cv::namedWindow("Camera Calibration", 1);
        imshow("Camera Calibration", view_gray);//显示图片
        waitKey(1500);//暂停0.5S		
    }

    Size square_size = Size(sizepic.square_size, sizepic.square_size);  /* 实际测量得到的标定板上每个棋盘格的大小 */

    vector<vector<Point3f>> object_points; /* 保存标定板上角点的三维坐标 */
                                           /*内外参数*/
    int i, j;
    vector<Point3f> tempPointSet;
    Mat cameraMatrix = Mat(3, 3, CV_32FC1, Scalar::all(0)); /* 摄像机内参数矩阵 */
    vector<int> point_counts;  // 每幅图像中角点的数量
    Mat distCoeffs = Mat(1, 5, CV_32FC1, Scalar::all(0)); /* 摄像机的5个畸变系数：k1,k2,p1,p2,k3 */
    vector<Mat> tvecsMat;  /* 每幅图像的旋转向量 */
    vector<Mat> rvecsMat; /* 每幅图像的平移向量 */
                            /* 初始化标定板上角点的三维坐标 */
    for (i = 0; i < board_size.height; i++)
    {
        for (j = 0; j < board_size.width; j++)
        {
            Point3f realPoint;
            /* 假设标定板放在世界坐标系中z=0的平面上 */
            realPoint.x = i*square_size.width;
            realPoint.y = j*square_size.height;
            realPoint.z = 0;
            tempPointSet.push_back(realPoint);
        }
    }
    object_points.push_back(tempPointSet);

    /* 初始化每幅图像中的角点数量，假定每幅图像中都可以看到完整的标定板 */
    point_counts.push_back(board_size.width*board_size.height);
    /* 开始标定 */
    calibrateCamera(object_points, image_points_seq, image_size, cameraMatrix, distCoeffs, rvecsMat, tvecsMat, 0);
    for (i = 0; i < InternalRef_LONG; i++)
    {
        InternalRef[i] = (double)cameraMatrix.at<double>(i / 3, i % 3);//将获取到的内参信息缓存到内存数组中
    }
    for (i = 0; i < DistortionCoef_LONG; i++)
    {
        DistortionCoef[i] = (double)distCoeffs.at<double>(0, i);//将获取到的畸变信息缓存到畸变数组中
    }
    ImageCorrections(imageInput,imageOut, InternalRef, DistortionCoef);
    namedWindow("correctImage");
    imshow("correctImage", imageOut);
    waitKey(1500);
    memcpy(output_pic, imageOut.data, width*height*channels);
    return;
}





