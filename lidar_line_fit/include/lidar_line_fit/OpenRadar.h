#pragma once
#include <iostream>
#include <cmath>
#include <cv.h>
#include <highgui.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <message_filters/subscriber.h>

using namespace std;
#include <vector>
#include "WeightedFit.h"
#define UTM30LX

static int RadarImageWdith  = 500;
static int RadarImageHeight = 500;
//������ɫ
/*Colour      Red      Green      Blue      ֵ     
    ��ɫ   White    255    255    255    16777215     
    ��ɫ    Red    255    0    0    255     
    ���ɫ    Dark    Red    128    0    0    128     
    ��ɫ    Green    0    255    0    65280     
    ����ɫ    Dark    Green    0    128    0    32768     
    ��ɫ    Blue    0    0    255    16711680       
    �Ϻ�ɫ    Magenta    255    0    255    16711935            
    ����    Dark    Cyan    0    128    128    8421376     
    ��ɫ    Yellow    255    255    0    65535     
    ��ɫ    Brown    128    128    0    32896  
 */
static int usualColor[15] = {16777215,255,128,65280,32768,
                             16711680,16711935,8421376,65535,32896 }; /*<10�ֳ��õ���ɫ*/

class OpenRadar
{
public:
    OpenRadar(ros::NodeHandle nh);
    ~OpenRadar(void);

    vector<double>RadarRho;
    vector<double>BreakedRadarRho;

    vector<double>RadarTheta;
    vector<double>BreakedRadarTheta;

    vector<double>SepRadarRho;//�ֶ�֮���Rho,����Ҳ���в��
    vector<double>SepRadarTheta;

    vector<int>RadarX;
    vector<int>RadarY;

    vector<int>BreakIndex;
    vector<LinePara>FittedLine;

    void start();

    //���ݶ�ȡ
    void RadarRead(const sensor_msgs::LaserScan::ConstPtr &scan);
    void CreateRadarImage(IplImage* RadarImage,vector<double>& RadarRho,vector<double>& RadarTheta);
    //���ݷֶ�
    int BreakRadarRho();//��ֵ���ѡȡ
    int PolyContourFit(int* X, int* Y, int n , float Eps);
    void PolyContourFit_iter(const vector<int>&,const vector<int>& ,const vector<double>& ,const vector<double>&,int&);
    int BreakPolyLine();//�����߲������
    void FitLine(vector<LinePara>& FittedLine,vector<double>& RadarRho,vector<double>& RadarTheta);//ֱ�����
    void DrawRadarLine(vector<LinePara>& FittedLine,IplImage* RadarImage);


    int frameCnt;
    int lineCnt;
    double scan_range_max_;
    double eps_;
    IplImage* RadarImage;
    ros::NodeHandle nh_;
    ros::Subscriber scan_sub;
};

