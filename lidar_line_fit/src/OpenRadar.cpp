#include "OpenRadar.h"
#include <opencv2/highgui/highgui.hpp>
#include "linefitting.h"
#include "houghlines.h"
// #include "lsdlines.h"


OpenRadar::OpenRadar(ros::NodeHandle nh):nh_(nh)
{
    RadarImage = cvCreateImage(cvSize(RadarImageWdith,RadarImageHeight),IPL_DEPTH_8U,3);
    // cvNamedWindow("Radar",1);
    // cvNamedWindow("BreakedRadar",1);
    // cvNamedWindow("line",1);
    lineCnt = 0;
    frameCnt=0;
    scan_range_max_=0.;
    eps_ = 200.0;
}
OpenRadar::~OpenRadar(void)
{
    cvReleaseImage(&RadarImage);
    // cvDestroyWindow("Radar");
    // cvDestroyWindow("line");
    // cvDestroyWindow("BreakedRadar");
}
void OpenRadar::start(){
    scan_sub = nh_.subscribe("/scan",2,&OpenRadar::RadarRead,this);
}
void OpenRadar::RadarRead(const sensor_msgs::LaserScan::ConstPtr &scan){
    int rho = 0;
    double theta = 0;
    double deltaTeta = scan->angle_increment;

    int totalCnt = scan->ranges.size();;
    scan_range_max_ = scan->range_max;
    //cout<<"successed to read"<<endl;
    RadarRho.clear();
    RadarTheta.clear();
    BreakedRadarRho.clear();
    BreakedRadarTheta.clear();
    SepRadarRho.clear();


    for (int j = 0; j < totalCnt; j++) {      
        if (scan->ranges[j] < scan->range_min || scan->ranges[j]>scan->range_max)
            RadarRho.push_back(0.0);
        else
            RadarRho.push_back(static_cast<double>(scan->ranges[j]));
        RadarTheta.push_back(theta);
        theta += deltaTeta;
    }

    CreateRadarImage(RadarImage, RadarRho, RadarTheta); 
    cvWaitKey(1);

    return;
}

void OpenRadar::CreateRadarImage(IplImage* RadarImage,vector<double>& RadarRho_,vector<double>& RadarTheta_){
    //RadarImage = cvCreateImage(cvSize(RadarImageWdith,RadarImageHeight),IPL_DEPTH_8U,1);
    cvZero(RadarImage);
    //�����ļ���һ��Բ��
    //int halfWidth  = RadarImageWdith/2;
    //int halfHeight = RadarImageHeight/2;
    int dx =  RadarImageWdith/2;
    int dy =  RadarImageHeight/2;

    // cvCircle(RadarImage, cvPoint(dx,dy),3, CV_RGB(255,0,0), -1, 8,0);

    int x,y;
    double theta,rho;
    unsigned char * pPixel = 0;
    


    //��ɫ
    int colorIndex = 0,colorRGB;
    int R = 255,G = 0,B = 0;
    int pointCnt = 0;
  
    cv::Mat src_binary = cv::Mat::zeros(RadarImageWdith,RadarImageHeight,CV_8UC1);

    unsigned char* image_data = new unsigned char[RadarImageHeight*RadarImageWdith];
    memset(image_data,0,sizeof(unsigned char)*RadarImageHeight*RadarImageWdith);
    int laser_beam_num=0;
    
    for (int i = 0; i < RadarRho_.size();i++)
    {
        //theta = (pointCnt/4.0 - 45)*pi/180;
        theta = RadarTheta_.at(i);
        rho = RadarRho_.at(i);
        if (rho < 0)
        {
           //�״����ݶϵ��־
            colorRGB = usualColor[colorIndex];
            R = colorRGB/65536;
            G = (colorRGB%65536)/256;
            B = colorRGB%256; 
            colorIndex = (colorIndex + 1)%10;
            continue;
        }else {
            pointCnt++;
        }
        if(scan_range_max_ != 0.)
            rho = rho / (scan_range_max_) * RadarImageHeight*0.5;

        x = (int)(rho*cos(theta)) + dx;
        y = (int)(-rho*sin(theta))+ dy;
        if (x >= 0 && x < RadarImageWdith && y >= 0 && y < RadarImageHeight)
        {
            if(rho > 1e-4f && image_data[y*RadarImageWdith+x] == 0 )
                ++laser_beam_num;
            pPixel = (unsigned char*)RadarImage->imageData + y*RadarImage->widthStep + 3*x;
            pPixel[0] = B;
            pPixel[1] = G;
            pPixel[2] = R;
            image_data[y*RadarImageWdith+x] = 255;
        }else{
            // cout<<"x: "<<x<<"  y: "<<y<<endl;
        }
    }
    // cv::imshow("srcbinary",src_binary);

    cv::RNG rng(12345);
    // cv::Mat img,src_gray,img_binary;
    cv::Mat img = cv::Mat(RadarImage);

    std::vector<line_float_t> lines;    
    std::vector<line_param_t> merged_lines;
    boundingbox_t bbox={0,0,RadarImageHeight,RadarImageWdith};

    HoughLineDetector detector(1,0.00581718236208,15,0,30);
    detector.detect((unsigned char*)image_data,RadarImageHeight,RadarImageWdith,bbox);
    if(detector.handlLines(merged_lines)){
        std::vector<line_param_t>::const_iterator it = merged_lines.begin();
        while (it!=merged_lines.end())
        {
            cv::Point pt1(int(floor(it->start_point.x)), int(floor(it->start_point.y)));
            cv::Point pt2(int(floor(it->end_point.x)), int(floor(it->end_point.y)));
            // printf("angles %f  b  %f\n",it->angle,it->b);
            // cv::line(image, pt1, pt2, color);
            cv::line(img, pt1, pt2, cv::Scalar(rng.uniform(0, 255), rng.uniform(0, 255),rng.uniform(0, 255)));
            it++;
        }
       
    }
    int merged_line_size = (int)merged_lines.size();
    // printf("%d\n",merged_line_size);
    int laser_in_line_num = 0;
    if( merged_line_size == 2){
        for(int k=0;k<2;++k){
            int y1 = int(floor(merged_lines[k].start_point.y));
            int y2 = int(floor(merged_lines[k].end_point.y));
            int x1 = int(floor(merged_lines[k].start_point.x));
            int x2 = int(floor(merged_lines[k].end_point.x));
            // printf("x1  %d x2  %d  y1  %d  y2  %d\n",x1,x2,y1,y2);
            int start_y,end_y,start_x,end_x;
            assert(y1!=y2);
            assert(x1!=x2);
            if(y1>y2){
                start_y = y2;
                end_y = y1;
            }else{
                start_y = y1;
                end_y = y2;
            }
            if(x1>x2){
                start_x = x2;
                end_x = x1;
            }else{
                start_x = x1;
                end_x = x2;
            }

            float angle = merged_lines[k].angle;    
            float slope = tan(angle/180*PI);
            float inteception = merged_lines[k].b;
            if(abs(angle) <= 45){                 
                for(int i=start_x;i<=end_x;++i){
                    int mid_j = int(floor(i*slope + inteception));
                    if(mid_j < 9){
                        for(int j = mid_j;j<mid_j+9;++j)
                            if(image_data[j*RadarImageWdith+i] == 255)
                                laser_in_line_num++;
                    }else{
                        for(int j = mid_j-4;j<mid_j+4;++j)
                            if(image_data[j*RadarImageWdith+i] == 255)
                                laser_in_line_num++;
                    }
                }
            }else{
                for(int i=start_y;i<=end_y;++i){
                    int mid_j = int(floor((i- inteception)/slope));
                    if(mid_j < 9){
                        for(int j = mid_j;j<mid_j+9;++j)
                            if(image_data[i*RadarImageWdith+j] == 255)
                                laser_in_line_num++;
                    }else{
                        for(int j = mid_j-4;j<mid_j+4;++j)
                            if(image_data[i*RadarImageWdith+j] == 255)
                                laser_in_line_num++;
                    }
                }
            }
            // for(int j = start_y;j<end_y;++j)
            //     for(int i=start_x;i<end_x;++i){
            //         if(image_data[j*RadarImageWdith+i] == 255)
            //             laser_in_line_num++;
            //     }        
        }
        float line_point_ratio = (float)laser_in_line_num/laser_beam_num;
        printf("line num is %d  %d  %f\n",laser_in_line_num,laser_beam_num,line_point_ratio );
        if(line_point_ratio > 0.8){
            detector.judgeHallway(merged_lines);
        }else{
            printf("no hallway\n");
        } 
    }else{
        printf("no hallway\n");
    } 
    
    // lines = detetor.getLines();

    // cout << "** Number of lines detected: " << lines.size() << endl;

    // std::vector<line_float_t>::const_iterator it = lines.begin();
    // while (it!=lines.end())
    // {
    //     cv::Point pt1(int(floor(it->startx)), int(floor(it->starty)));
    //     cv::Point pt2(int(floor(it->endx)), int(floor(it->endy)));
    //     // cv::line(image, pt1, pt2, color);
    //     cv::line(img, pt1, pt2, cv::Scalar(255, 255,255));
    //     it++;
    // }
    

    cv::imshow("corners",img);
    delete[] image_data;
}

int OpenRadar::BreakRadarRho(){
    int breakCnt = 0;
    int rho = 0;
    
    double lastRho = RadarRho.at(0);
    double theta = RadarTheta.at(0);

    iPoint last_point;
    last_point.x = int(lastRho * cos(theta));
    last_point.y = int(lastRho * sin(theta));

    int dis = 0;
    int Dmax = 50;  // 点集分割的阈值

    BreakedRadarRho.clear();
    BreakedRadarTheta.clear();

    BreakedRadarRho.push_back(lastRho);
    BreakedRadarTheta.push_back(theta);

    for (int i = 1; i< RadarRho.size();i++)
    {
        iPoint current_point;
       rho = RadarRho.at(i);
       theta = RadarTheta.at(i);    
       current_point.x = int(rho * cos(theta));
       current_point.y = int(rho * sin(theta));

       BreakedRadarRho.push_back(rho);
       BreakedRadarTheta.push_back(theta);
        
    //    dis = abs(rho - lastRho);
        dis = sqrt((current_point.x-last_point.x)*(current_point.x-last_point.x)+(current_point.y-last_point.y)*(current_point.y-last_point.y));
       if (dis < Dmax)
       {
          
       }else {
          BreakedRadarRho.push_back(-1);
          BreakedRadarTheta.push_back(1000.0);
          breakCnt++;
       }
       lastRho = rho;
       last_point=current_point;
    }
    BreakedRadarRho.push_back(-1);
    BreakedRadarTheta.push_back(1000.0);
    cout<<"breakCnt: "<<breakCnt<<endl;
    return breakCnt;
}

// 进行多边形拟合： Points : 轮廓上的点      n -- 轮廓点数目  Eps -- 拟合精度
// 返回值： 若该轮廓段需要分段，则返回分段点在该轮廓点列中的索引，否则，返回 0 表示不需要分段
// 这里是整个算法计算复杂性最大的一个地方
// 为了提高程序运行效率，对点到直线的距离计算进行改进：
// 多边形拟合中的直线是由点列中的点决定的
// 为了计算点到直线的距离，
// 采用坐标系旋转，将直线旋转到x轴方向，这样点到直线的距离即为各个点
// 在坐标旋转后的y值的绝对值
// 同时，坐标旋转矩阵在该次运算中为定值，只需一次计算，不需要多次的开方或三角计算
int OpenRadar::PolyContourFit( int* X, int* Y, int n , float Eps ) // ���������㣬�ö������ϸ�������    
{
    double dis = sqrt((double)(((X[0] - X[n - 1])*(X[0] - X[n - 1])) +  
                     ((Y[0] - Y[n - 1])* (Y[0] - Y[n - 1]))));
    double cosTheta = (X[n- 1] - X[0]) / dis;
    double sinTheta = - ( Y[n- 1] - Y[0] )/dis;
    double MaxDis = 0;
    int i ;
    int MaxDisInd = -1;
    double dbDis;
    for(i = 1 ; i < n - 1 ; i++)
    {
        // 进行坐标旋转，求旋转后的点到x轴的距离
        dbDis = abs( (Y[i] - Y[0]) * cosTheta + (X[i] - X[0])* sinTheta);
        if( dbDis > MaxDis)
        {
            MaxDis = dbDis;
            // cout<<"Max distance is "<< MaxDis<<endl;
            MaxDisInd = i;
        }
    }
    if(MaxDis > Eps)
    {
        return MaxDisInd;
            //    cout << "Line 1 : " << endl;
            //    cout << "Start :" << Points[0].x << "  " << Points[0].y  << " --- " << Points[MaxDisInd].x << "  " << Points[MaxDisInd].y << endl;
            //    cout << "�Ƕȣ� "<<180 * atan2(Points[0].y - Points[MaxDisInd].y , Points[0].x - Points[MaxDisInd].x ) / 3.1415926;
            //    cout << "Line 2 :" << endl;
            //    cout << "Start :" << Points[MaxDisInd].x << "  " << Points[MaxDisInd].y  << " --- " << Points[n - 1].x << "  " << Points[n - 1].y << endl;
            //    cout << "�Ƕȣ� "<< 180 * atan2(Points[n - 1].y - Points[MaxDisInd].y , Points[n - 1].x - Points[MaxDisInd].x ) / 3.1415926;
    }
    //    else{
    //        cout << "Line 1 : " << endl;
    //        cout << "Start :" << Points[0].x << "  " << Points[0].y  << " --- " << Points[n - 1].x << "  " << Points[n - 1].y << endl;
    //        cout << "�Ƕȣ� "<<180 * atan2(Points[n - 1].y - Points[0].y , Points[n - 1].x - Points[0].x ) / 3.1415926;

    //    }
    return 0;
}
void OpenRadar::PolyContourFit_iter(const vector<int>& X,const vector<int>& Y,const vector<double>& rho,const vector<double>& theta,int& lineCnt){
    int queue_size = (int)X.size();
    if(queue_size <= 3)
        return;
    double dis = sqrt((double)(((X[0] - X[queue_size - 1])*(X[0] - X[queue_size - 1])) +  
                     ((Y[0] - Y[queue_size - 1])* (Y[0] - Y[queue_size - 1]))));
    double cosTheta = (X[queue_size - 1] - X[0]) / dis;
    double sinTheta = - ( Y[queue_size - 1] - Y[0] )/dis;

    int i;
    double dbDis;

    SepRadarRho.push_back(rho.at(0));
    SepRadarTheta.push_back(theta.at(0));

    for(i=1;i < queue_size-1;++i){
        // 进行坐标旋转，求旋转后的点到x轴的距离
        dbDis = abs( (Y[i] - Y[0]) * cosTheta + (X[i] - X[0])* sinTheta);

        SepRadarRho.push_back(rho.at(i));
        SepRadarTheta.push_back(theta.at(i));

        if(dbDis > eps_){

            lineCnt ++;

            SepRadarRho.push_back(-1);
            SepRadarTheta.push_back(1000.0);

            vector<int> tmp_X(X.begin()+i+1,X.end());
            vector<int> tmp_Y(Y.begin()+i+1,Y.end());
            vector<double> tmp_rho(rho.begin()+i+1,rho.end());
            vector<double> tmp_theta(theta.begin()+i+1,theta.end());

            assert(X.begin()+i+1 != X.end());
            assert(Y.begin()+i+1 != Y.end());
            assert(rho.begin()+i+1 != rho.end());
            assert(theta.begin()+i+1 != theta.end());
            
            
            // std::copy(X.begin()+i+1,X.end(),tmp_X.begin());
            // std::copy(Y.begin()+i+1,Y.end(),tmp_Y.begin());
            // std::copy(rho.begin()+i+1,rho.end(),tmp_rho.begin());
            // std::copy(theta.begin()+i+1,theta.end(),tmp_theta.begin());

            PolyContourFit_iter(tmp_X,tmp_Y,tmp_rho,tmp_theta,lineCnt);
            break;
        }
        lineCnt ++;
    }
    return;
}


//将折线拆多段
int OpenRadar::BreakPolyLine(){
    int rho = 0;
    double theta = 0.0;
    // int X[1200] = {0};
    // int Y[1200] = {0};
    // int rhoCopy[1200] = {0};
    // double thetaCopy[1200] = {0};
    std::vector<int> X = {0};
    std::vector<int> Y = {0};
    std::vector<double> rhoCopy = {0};
    std::vector<double> thetaCopy = {0};
    int pointCnt = 0;
    int lineCnt = 0;
    int N = 0;
    SepRadarRho.clear();
    SepRadarTheta.clear();

    for (int i = 0; i < BreakedRadarRho.size();i++)
    {
        rho = BreakedRadarRho.at(i);
        theta = BreakedRadarTheta.at(i);

        if (rho < 0)
        {
            if (pointCnt > 200)//分割后每个点集内点的个数
            {
                // N = PolyContourFit(X , Y, pointCnt, 20);
                // //cout<<"N: "<<N<<endl;
                // //cout<<"pointCnt: "<<pointCnt<<endl;
                // //�������ֱ�ߣ���Ҫ����µĶϵ�
                // if (N == 0 )
                // {
                //     lineCnt++;
                //     //cout<<"Line"<<endl;
                //     for (int j = 0; j < pointCnt;j++)
                //     {
                       
                //        SepRadarRho.push_back(rhoCopy[j]);
                //        SepRadarTheta.push_back(thetaCopy[j]);
                //     }
                //     SepRadarRho.push_back(-1);
                //     SepRadarTheta.push_back(1000.0);

                // }else if (N > 0)
                // {
                //     lineCnt += 2;

                //     for (int j = 0; j < N;j++)
                //     {
                //         SepRadarRho.push_back(rhoCopy[j]);
                //         SepRadarTheta.push_back(thetaCopy[j]);
                //     }
                //     SepRadarRho.push_back(-1);
                //     SepRadarTheta.push_back(1000.0);

                //     for (int j = N; j< pointCnt;j++)
                //     {
                //         SepRadarRho.push_back(rhoCopy[j]);
                //         SepRadarTheta.push_back(thetaCopy[j]);
                //     }
                //     SepRadarRho.push_back(-1);
                //     SepRadarTheta.push_back(1000.0);
                // }
                PolyContourFit_iter(X,Y,rhoCopy,thetaCopy,lineCnt);
                cout<<" line num "<< lineCnt << endl;
            }
            pointCnt = 0;
            // X.clear();
            // Y.clear();
            // rhoCopy.clear();
            // thetaCopy.clear();
            continue;
        }
        X.push_back(rho*cos(theta));
        Y.push_back(rho*sin(theta));
        rhoCopy.push_back(rho);
        thetaCopy.push_back(theta);
        pointCnt++;
    }
    //cout<<"lineCnt: "<<lineCnt<<endl;
    return lineCnt;
}


void OpenRadar::FitLine(vector<LinePara>& FittedLine,vector<double>& RadarRho,vector<double>& RadarTheta){

    double rho = 0;
    double theta = 0.0;
    int X[1200] = {0};
    int Y[1200] = {0};
    int pointCnt = 0;
    LinePara tmpLinePara;
    FittedLine.clear();
    for (int i = 0 ; i < RadarRho.size();i++)
    {
        rho = RadarRho.at(i);
        theta = RadarTheta.at(i);

        if (rho < 0)
        {
            WeightedFit(X ,Y ,pointCnt,&tmpLinePara);
            FittedLine.push_back(tmpLinePara);
            pointCnt = 0;
            continue;
        }

        X[pointCnt] = rho*cos(theta);
        Y[pointCnt] = -rho*sin(theta);
        pointCnt++;
    }
    for (int i = 0; i < FittedLine.size();i++)
    {
        // cout<<"a: "<<FittedLine.at(i).a<<"  b: "<<FittedLine.at(i).b<<" ";
        // cout<<"x0: "<<FittedLine.at(i).startPoint.x<<" "
        //     <<"y0: "<<FittedLine.at(i).startPoint.y<<" "
        //     <<"x1: "<<FittedLine.at(i).endPoint.x<<" "
        //     <<"y1: "<<FittedLine.at(i).endPoint.y<<endl;
    }
    cout<<endl;
}

void OpenRadar::DrawRadarLine(vector<LinePara>& FittedLine,IplImage* RadarImage){
    //�����ļ���һ��Բ��
    int dx =  RadarImageWdith*2/3;
    int dy =  RadarImageHeight*3/4;
    cvCircle(RadarImage, cvPoint(dx,dy),3, CV_RGB(0,255,255), -1, 8,0);
    int x1,y1,x2,y2;
    //��ɫ
    int colorIndex = 0,colorRGB;
    int R = 255,G = 0,B = 0;
    for (int i = 0; i < FittedLine.size();i++)
    {
        //�״����ݶϵ��־
        colorRGB = usualColor[colorIndex];
        R = colorRGB/65536;
        G = (colorRGB%65536)/256;
        B = colorRGB%256; 
        colorIndex = (colorIndex + 1)%10;

        x1 = FittedLine.at(i).startPoint.x;
        y1 = FittedLine.at(i).startPoint.y;

        x2 = FittedLine.at(i).endPoint.x;
        y2 = FittedLine.at(i).endPoint.y;

        //ת��ΪRadarͼ����ϵ��
        x1 = x1 + dx;
        y1 = y1 + dy;
        x2 = x2  + dx;
        y2 = y2 + dy;
        cout<<"x1: "<<x1<<" y1: "<<y1<<" x2: "<<x2<<" y2: "<<y2<<endl;
        cvLine(RadarImage,cvPoint(x2,y2),cvPoint(x1,y1),CV_RGB(R,G,B),2,8,0);
    }
}