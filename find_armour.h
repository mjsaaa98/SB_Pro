#ifndef FIND_ARMOUR_H
#define FIND_ARMOUR_H

//#include <opencv2/opencv.hpp>
//#include <opencv2/core.hpp>
//#include <opencv2/highgui.hpp>
//#include <opencv2/imgproc.hpp>
#include "serialport.h"
#include "Header.h"
//#define KALMANXY_OPEN
#define SHOWDEBUG
using namespace cv;
using namespace std;
extern FileStorage fs;
/**
 * @brief Sort_RotatedRect  按旋转矩形的中心点x方向降序排列
 * @param r1  RotatedRect
 * @param r2  RotatedRect
 * @return
 */
static inline bool Sort_RotatedRect(RotatedRect r1,RotatedRect r2)
{
    return r1.center.x<r2.center.x;
}
/**
 * @brief SortArmorCenterX  在X方向升序排列
 * @param p1  Point2f
 * @param p2  Point2f
 * @return
 */
static inline bool SortArmorCenterX(Point2f p1,Point2f p2)
{
    return p1.x<p2.x;
}
/**
 * @brief SortArmorCenterY  在Y方向降序排列
 * @param p1  Point2f
 * @param p2  Point2f
 * @return
 */
static inline bool SortArmorCenterY(Point2f p1,Point2f p2)
{
    return p1.y>p2.y;
}

static inline bool Sort_Areas(float a1,float a2)
{
    return a1<a2;
}

/**
 * @brief The find_armour class  找装甲板的类
 */
class find_armour
{
public:
    find_armour();
    Mat roi(Mat,Point,float);
    float Point_Angle(const Point2f &p1,const Point2f &p2);
    void image_preprocess(int mode,Mat src,Mat &);
    void search_armour(Mat&img,Mat&dst);
    void get_Light();
    void src_get_armor();
    void get_armor(Mat& image,Mat &dst,int mode,bool Show_Left);
public:
    int isfind;
    int ismiddle;
    vector<Armordata> Armordatas;
    vector<Point2f> ArmorPoints;
    Armordata LastArmor;
private:
    void clear_data();   //切换指令时清空所有数据
    void Clear();   //清空所有数据结构
    vector<float> diameters;
    vector<Point2f> armour_center;
    vector<float> big_diameters;
    vector<Point2f> big_armour_center;
    vector<vector<RotatedRect> > Armorlists;
    vector<Point2f> Rotate_Point;
    vector<vector<Point2f> > Rotate_Points;
    vector<vector<Point2f> > big_Rotate_Points;
    vector<RotatedRect> fir_armor,result_armor;
    vector<Vec4f> contours_para;
    vector<int> CellMaxs;
    float x1,x2,y1,y2;
    float y_dist_wucha_ROI,height_d_wucha_ROI;
    float a1,a2,area_min,min_rate,max_rate,height_d_wucha,y_dist_wucha;
private:
    //与截图有关的参数
    int isROIflag;
    int last_mode;
    Point2f last_center;
    Mat dstROI;
    float last_d;
    float last_angle;
    Size2f last_size;
private:
    int vmin = 10, vmax = 256, smin = 30;
    int find_armor_flag=0;
    int isfindflag = 0;

    Mat frame, hsv, hue,hue1, mask,mask1, hist, hist1,histimg = Mat::zeros(640, 480, CV_8UC3),histimg1 = Mat::zeros(640, 480, CV_8UC3), backproj,backproj1;

};

#endif // FIND_ARMOUR_H
