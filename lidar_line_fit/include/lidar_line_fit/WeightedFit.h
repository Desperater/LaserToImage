
#ifndef WEIGHTED_FIT_H
#define WEIGHTED_FIT_H
#include <cmath>
#include <cstdlib>
#include "QSort.h"
#define MAX_FITPOINTS_CNT 1000
//#include "ImgProcess.h"
#define K   5.0//(4.685 / 0.6745)     /// ��Ȩ����е�ϵ��
typedef struct
{
    int x;
    int y;
}iPoint;
typedef struct{
    double a;//y = a*x + b
    double b;
    double Rho; // �ö�ֱ�ߵ����
    iPoint startPoint;
    iPoint endPoint;
}LinePara;

int Med(int R[] , int Cnt);// ��ȡ��ֵ
int CalW(int X[] , int Y[] , int Cnt , LinePara * EstLinePara , int W[]);
int FitPara(int X[] , int Y[] , int Cnt ,LinePara * EstLinePara , int W[]);
int WeightedFit(int X[] , int Y[] , int Cnt , LinePara * EstLinePara);
#define cmp_pts( x, y )   ( x < y )    //  ���ڿ�������Ƚ�x < y , �õ��Ľ����Ŷ��������
#endif
