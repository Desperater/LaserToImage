#include "WeightedFit.h"

CV_IMPLEMENT_QSORT( IntQSort, int, cmp_pts )  // �ú��������������庯��IntQSort���ڿ�������
int W[MAX_FITPOINTS_CNT];// =(int * )malloc(sizeof(int) * Cnt);// Ȩֵϵ��	
int WeightedFit(int X[] , int Y[] , int Cnt , LinePara * EstLinePara)
{
    // ��Ȩ��С���˷�
    // Cnt: ���ݵ����
    // EstLinePara : ֱ����ϵĹ���ֵ������������С���˷�����õ�
    // ������С���˽��й���
    int * Tmp;
    int FlagFlip = 0;// �Ƿ��X,Y���з�ת��
    //FitPara(X , Y , Cnt , EstLinePara , NULL);
    //if(abs(EstLinePara->a) > 1 || EstLinePara->a == NAN || EstLinePara->a == -NAN)
    if( abs(X[0] - X[Cnt - 1]) < abs(Y[0] - Y[Cnt - 1]) )
    {
        // �ö�ֱ��Ϊб�ʴ���1
        // ��45���߽��з�ת
        // �� X �� Y ���з�ת
        Tmp = X;
        X = Y;
        Y = Tmp;
        FlagFlip = 1;  // ��ת
    }
    int i = 0;
    if(W == NULL)
        return -1;
    // ����20��
    for(i = 0 ; i < 20 ; i++)
    {
        // ����Ȩֵ
        CalW(X , Y ,Cnt , EstLinePara , W );
        FitPara(X,Y,Cnt,EstLinePara,W);// ����Ȩֵ��ϲ���
    }
    //free(W);
   // EstLinePara->Dis = abs(EstLinePara->b)/(sqrt(EstLinePara->a * EstLinePara->a + EstLinePara->b * EstLinePara->b));
    if(FlagFlip == 0)
    {
        // δ��ת
        EstLinePara->Rho = atan(EstLinePara->a);
    }
    else
    {
        // ��ת��
        if(abs(EstLinePara->a) < 0.00001)
        {
            EstLinePara->a = 100000;
        }	     
        else
        {
            EstLinePara->a =1.0/ EstLinePara->a;
        }	 
        EstLinePara->b = - EstLinePara->b * (EstLinePara->a);
        EstLinePara->Rho = atan(EstLinePara->a);
    }

    //X Y��ת�����ٷ�ת��ȥ
    if(FlagFlip == 1)
    {
        // �ö�ֱ��Ϊб�ʴ���1
        // ��45���߽��з�ת
        // �� X �� Y ���з�ת
        Tmp = X;
        X = Y;
        Y = Tmp;
    }
    //�����߶ε������˵�
   if (abs(EstLinePara->a) >= 30000)
   {
       EstLinePara->startPoint.y = Y[0];
       EstLinePara->startPoint.x = (Y[0] - EstLinePara->b)/EstLinePara->a;
        
       EstLinePara->endPoint.y = Y[Cnt-1];
       EstLinePara->endPoint.x = (Y[Cnt-1] - EstLinePara->b)/EstLinePara->a;
   }else {
       EstLinePara->startPoint.x = X[0];
       EstLinePara->startPoint.y = EstLinePara->a* X[0] + EstLinePara->b;

       EstLinePara->endPoint.x = X[Cnt-1];
       EstLinePara->endPoint.y = EstLinePara->a* X[Cnt-1] + EstLinePara->b;
   }

    //double d;
    //for (int i = 0 ; i < Cnt; i++)
    //{
    //    if (EstLinePara->a >= 30000)
    //    {
    //        d = abs( EstLinePara->a * X[i] - Y[i] + EstLinePara->b)/sqrt(EstLinePara->a*EstLinePara->a + 1);
    //        if (1/*d < 300*/)
    //        {
    //            EstLinePara->startPoint.x = X[i];
    //            EstLinePara->startPoint.y = EstLinePara->a*X[i] + EstLinePara->b;
    //            break;
    //        }
    //    }

    //   
    //}

    //for (int i = Cnt-1 ; i >=0; i--)
    //{
    //    d = abs( EstLinePara->a * X[i] - Y[i] + EstLinePara->b)/sqrt(EstLinePara->a*EstLinePara->a + 1);
    //    if (1/*d < 300*/)
    //    {
    //        EstLinePara->endPoint.x = X[i];
    //        EstLinePara->endPoint.y = EstLinePara->a*X[i] + EstLinePara->b;
    //        break;
    //    }
    //}
    return 0;
}


int Med(int R[] , int Cnt)// ��ȡ��ֵ
{
    //qsort(R , Cnt , sizeof(R[0]) , Cmp);
    IntQSort(R , Cnt , 0);
    return R[Cnt/2];
}
int CalW(int X[] , int Y[] , int Cnt , LinePara * EstLinePara , int W[] )
{
    int i = 0;
    double a = (double)EstLinePara->a;
    double b = (double)EstLinePara->b;

    int Median = 0;
    double u;
    double tmp;
    for(i = 0; i < Cnt ; i++)
    {
        tmp = (int)abs(Y[i] - a * X[i] - b );
        W[i]=tmp;
    }
    Median = Med(W , Cnt);
    Median = Median > 2 ? Median : 2;

    for(i = 0 ; i < Cnt ; i++)
    {
        u =(double)( W[i]/(K * Median) );

        if(u < 1)
        {
            W[i] =(int)((1 - u * u) * (1 - u * u) * 100);   //��W��Χ������0-100
            //W[i] = (int)((1-u)*(1-u)*100);
        }
        else{
            W[i] = 0;
        }
    }

    return 0;
}
int FitPara(int X[] , int Y[] , int Cnt ,LinePara * EstLinePara , int W[])
{

    int i = 0;
    long long P1 = 0; // sum(wi*xi*yi);
    long long P2 = 0; // sum(wi * xi * xi)
    long long P3 = 0; // sum(wi * xi)
    long long P4 = 0; // sum(wi * yi)
    long long P5 = 0; // sum(wi)
    if(W == NULL) // ֱ�ӽ�����С������ϣ����������ݵ�Ȩֵ���
    {
        //
        for( i = 0 ; i < Cnt ;i++)
        {
            P1 +=  X[i] * Y[i];
            P2 +=  X[i] * X[i];
            P3 +=  X[i];
            P4 +=  Y[i];
            P5 +=  1;
        }
    }
    else{ //��Ȩ��С�������
        for( i = 0 ; i < Cnt ;i++)
        {
            P1 += W[i] * X[i] * Y[i];
            P2 += W[i] * X[i] * X[i];
            P3 += W[i] * X[i];
            P4 += W[i] * Y[i];
            P5 += W[i];
        }
    }

    EstLinePara->a =  ( ((double)(((double)P1) * ((double)P5) - P4 * P3)) 
                    / ( (double)(((double)P2) * ((double)P5) - P3 * P3)));
    EstLinePara->b = (P1 - P2 * EstLinePara->a)/P3;
    return 0; 
}