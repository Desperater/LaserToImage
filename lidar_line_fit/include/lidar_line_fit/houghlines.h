
#ifndef __HOUGH_LINE_H_
#define __HOUGH_LINE_H_



#include <vector>
#include <cmath>
#include <map>
#include <assert.h>

#ifndef PI
#define PI (3.1415926535897932384626433832795)
#endif

typedef enum _HOUGH_LINE_TYPE_CODE
{
	HOUGH_LINE_STANDARD = 0,				  //standad hough line
	HOUGH_LINE_PROBABILISTIC = 1,			  //probabilistic hough line

}HOUGH_LINE_TYPE_CODE;

typedef enum _TWO_LINE_STATE{
	LINES_COLINEAR = 0,				// 共线
	LINES_PARALLEL = 1,				// 平行
	LINES_NON_PARALLEL = 2,				// 夹角
} LINE_STATE;


typedef struct
{
	int x;                              
	int y;                            
	int width;                        
	int height;                         
}boundingbox_t;

template<typename T>
struct point_t{
	T x;
	T y;
};
typedef point_t<float> point2f;
typedef point_t<int> point2i;

typedef struct{
	point2f start_point;
	point2f end_point;
	float angle;
	float b;
	bool is_merged = false;
} line_param_t;

typedef struct
{
	float startx;
	float starty;
	float endx;
	float endy;
}line_float_t;


/*
@function    HoughLineDetector
@param       [in]      src:						  image,single channel
@param       [in]      w:                         width of image
@param       [in]      h:                         height of image
@param       [in]      scaleX:                    downscale factor in X-axis
@param       [in]      scaleY:                    downscale factor in Y-axis
@param       [in]      CannyLowThresh:            lower threshold for the hysteresis procedure in canny operator
@param       [in]      CannyHighThresh:           higher threshold for the hysteresis procedure in canny operator
@param       [in]      HoughRho:                  distance resolution of the accumulator in pixels
@param       [in]      HoughTheta:                angle resolution of the accumulator in radians
@param       [in]      MinThetaLinelength:        standard: for standard and multi-scale hough transform, minimum angle to check for lines.
												  propabilistic: minimum line length. Line segments shorter than that are rejected
@param       [in]      MaxThetaGap:               standard: for standard and multi-scale hough transform, maximum angle to check for lines
												  propabilistic: maximum allowed gap between points on the same line to link them
@param       [in]      HoughThresh:               accumulator threshold parameter. only those lines are returned that get enough votes ( >threshold ).
@param       [in]      _type:                     hough line method: HOUGH_LINE_STANDARD or HOUGH_LINE_PROBABILISTIC
@param       [in]      bbox:                      boundingbox to detect
@param       [in/out]  lines:                     result
@return：										  0:ok; 1:error
@brief：     _type: HOUGH_LINE_STANDARD:		  standard hough line algorithm
					HOUGH_LINE_PROBABILISTIC	  probabilistic hough line algorithm
					
When HOUGH_LINE_STANDARD runs, the line points might be the position outside the image coordinate

standard:		try (src,w,h,scalex,scaley,70,150, 1, PI/180, 0, PI, 100, HOUGH_LINE_STANDARD, bbox, line)
propabilistic:  try (src,w,h,scalex,scaley,70,150, 1, PI/180, 30, 10, 80, HOUGH_LINE_STANDARD, bbox, line)
*/
int HoughLineDetector_(unsigned char *src, int w, int h,
	float scaleX, float scaleY, float CannyLowThresh, float CannyHighThresh,
	float HoughRho, float HoughTheta, float MinThetaLinelength, float MaxThetaGap, int HoughThresh,
	HOUGH_LINE_TYPE_CODE _type,
	boundingbox_t bbox, std::vector<line_float_t> &lines);

inline bool compare_points(const point2f& p1,const point2f& p2){
	// if(p1.y==p2.y)
	// 	return p1.x > p2.x;
	// else
	// 	return p1.y > p2.y;
	return (p1.x > p2.x || (abs(p1.x-p2.x)<1e-4f && p1.y > p2.y));
}

inline bool compare_lines(const line_param_t& line_a,const line_param_t& line_b){
		return compare_points(line_a.end_point,line_b.end_point);
		// if(compare_points(line_a.end_point,line_b.end_point))
			
		// 	if(compare_points(line_a.start_point,line_b.end_point)){
}



class HoughLineDetector{
public:
	HoughLineDetector()=delete;
	HoughLineDetector(double rho,double theta,int vote,double minLength=0,double maxGap=0):
			delta_rho_(rho),delta_theta_(theta),min_vote_(vote),min_length_(minLength),max_gap_in_line_(maxGap),scale_x_(1.0),scale_y_(1.0)
	{
		angle_threshold_ = 2.0;
		intercept_threshold_ = 200.0;
		two_line_distance_threshold_ = 15;
		constigous_line_distance_ = 150;
	}
	~HoughLineDetector(){
	};

	bool detect(unsigned char* img,int width, int height,boundingbox_t bbox_);
	std::vector<line_float_t> getLines(){return lines_;};

	bool handlLines(std::vector<line_param_t>& merged_lines);

	bool judgeHallway(const std::vector<line_param_t>& v);
	
private:
	LINE_STATE judge_two_lines(const line_param_t& line_a,const line_param_t& line_b);
	line_param_t mergeLines(const line_param_t& line_a,const line_param_t& line_b);
	bool getLineParam();

	
	inline float two_point_distance(const point2f& a,const point2f& b){
		return sqrt((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y));
	}
	inline bool origin_between_lines(const line_param_t& line_a,const line_param_t& line_b){
		float slope_a = tan(line_a.angle*PI/180);
		float slope_b = tan(line_b.angle*PI/180);
		float sqrt_a = sqrt(1+slope_a*slope_a);
		float sqrt_b = sqrt(1+slope_b*slope_b);

		float dis_a = abs(slope_a*origin_point_.x +line_a.b-origin_point_.y);
		float dis_b = abs(slope_b*origin_point_.x +line_b.b-origin_point_.y);
		float dis_two_lines = two_line_distance(line_a,line_b);
		if(abs(dis_a-dis_b)<dis_two_lines)
			return true;
		else
			return false;
	}
	inline float two_line_distance(const line_param_t& line_a,const line_param_t& line_b ){
		float slope_a = tan(line_a.angle*PI/180);
		float slope_b = tan(line_b.angle*PI/180);
		float sqrt_a = sqrt(1+slope_a*slope_a);
		float sqrt_b = sqrt(1+slope_b*slope_b);

		float dis_a_start = abs(slope_b*line_a.start_point.x +line_b.b-line_a.start_point.y);
		float dis_a_end   = abs(slope_b*line_a.end_point.x +line_b.b-line_a.end_point.y);

		float dis_b_start = abs(slope_a*line_b.start_point.x +line_a.b-line_b.start_point.y);
		float dis_b_end   = abs(slope_a*line_b.end_point.x +line_a.b-line_b.end_point.y);

		float max_dis = dis_a_start > dis_a_end? dis_a_start:dis_a_end;
		float min_dis = dis_a_start < dis_a_end? dis_a_start:dis_a_end;

		max_dis = max_dis>dis_b_start? max_dis:dis_b_start;
		max_dis = max_dis>dis_b_end?max_dis:dis_b_end;

		min_dis = min_dis < dis_b_start? min_dis:dis_b_start;
		min_dis = min_dis < dis_b_end?min_dis:dis_b_end;
	
		return (dis_a_start+dis_b_start+dis_a_end+dis_b_end-max_dis-min_dis)*0.5;
	}
	inline bool lines_need_merge(const line_param_t& line_a,const line_param_t& line_b ){
		if(compare_points(line_a.end_point,line_b.end_point)){
			if(compare_points(line_a.start_point,line_b.end_point)){
				float dis = two_point_distance(line_a.start_point,line_b.end_point);
				// printf("two_point_distance  %f \n",dis);
				if(dis < constigous_line_distance_)
					return true;
				else
					return false;
			}else{
				return true;
			}				
		}else{
			if(compare_points(line_b.start_point,line_a.end_point)){
				float dis = two_point_distance(line_a.end_point,line_b.start_point);
				// printf("two_point_distance  %f \n",dis);				
				if(dis < constigous_line_distance_)
					return true;
				else
					return false;
			}else{
				return true;
			}		
		}
	}
private:

	std::vector<line_float_t> lines_;
	int min_vote_;			//每条直线的最小得票数量
	double delta_rho_;			//直线检测的距离分辨率
	double delta_theta_;		//直线检测角度分辨率
	double min_length_;			//最小直线长度
	double max_gap_in_line_;	//直线上允许存在的最大空隙

	int total_hit_beams_num_;
	point2i origin_point_;

	float scale_x_;
	float scale_y_;
	boundingbox_t bbox_;

	std::vector<line_param_t> line_params_;

	//some threshold
	float angle_threshold_;					//判断直线平行的角度阈值
	float intercept_threshold_;				//判断直线共线的截距阈值
	float two_line_distance_threshold_;		//两条直线的距离阈值
	float constigous_line_distance_;		//共线直线的距离阈值
};
#endif /* HOUGH_H */
