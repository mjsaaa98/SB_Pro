#include "find_armour.h"
/**
 * @brief find_armour::find_armour  构造函数 完成一些从yaml读取数据的操作
 * @param f  写入yaml文件的类
 */
find_armour::find_armour()
{
    fs["y_dist_wucha_ROI"] >> y_dist_wucha_ROI;
    fs["height_d_wucha_ROI"] >> height_d_wucha_ROI;
    fs["area_min"] >> area_min;
    fs["angle1"] >> a1;
    fs["angle2"] >> a2;
    fs["y_dist_wucha"] >> y_dist_wucha;
    fs["height_d_wucha"] >> height_d_wucha;
    fs["min_rate"] >> min_rate;
    fs["max_rate"] >> max_rate;
    isROIflag = 0;
    Clear();
    clear_data();
}

/**
 * @brief find_armour::Clear  清空所有数据
 */
void find_armour::Clear()
{
    fir_armor.clear();
    result_armor.clear();
    Armorlists.clear();
    contours_para.clear();
    CellMaxs.clear();
    Armordatas.clear();
    ArmorPoints.clear();
    isfind = 0;
    ismiddle =0;
}
/**
 * @brief find_armour::clear_data  切换指令时清空所有数据
 */
void find_armour::clear_data()
{
    x1 = 0;
    x2 = 0;
    y1 = 0;
    y2 = 0;
}


/**
 * @brief find_armour::roi   截图
 * @param img
 * @param center
 * @param d
 * @param cols
 * @param rows
 * @return
 */
Mat find_armour::roi(Mat img,Point center,float d)
{
    Mat roi;
    int cols = img.cols;
    int rows = img.rows;
    x1 = center.x-d*1;
    x2 = center.x+d*1;
    y1 = center.y-d*0.8;
    y2 = center.y+d*0.8;
    if(x1<=0) x1 = 1;
    if(x2>cols) x2 = cols-1;
    if(y1<=0) y1 = 1;
    if(y2>=rows) y2 = rows-1;
    roi = img(Range(y1,y2),Range(x1,x2));
    return roi;
}



/**
 * @brief find_armour::image_preprocess  图像预处理
 * @param mode  识别模式
 * @param src  input  输入图像
 * @param dst  output 输出图像
 */
void find_armour::image_preprocess(int mode,Mat src,Mat &dst)
{
//    Mat gray;
//    vector <Mat> planne;
//    Mat k = getStructuringElement(MORPH_RECT,Size(11,11));
//    Mat k1 = getStructuringElement(MORPH_RECT,Size(7,7));
    if(mode==2)
    {
        //    蓝色通道减去红色通道
        Mat gray;
        vector <Mat> planne;
        Mat k = getStructuringElement(MORPH_RECT,Size(11,11));
        Mat k1 = getStructuringElement(MORPH_RECT,Size(7,7));

        split(src,planne);
        cvtColor(src,gray,CV_BGR2GRAY);
        threshold(gray,gray,50,255,THRESH_BINARY);
//        dilate(gray,gray,k);

        subtract(planne[0],planne[2],dst);

        threshold(dst,dst,50,255,THRESH_BINARY);
        dilate(dst,dst,k);
        dst = dst&gray;
        dilate(dst,dst,k1);
    }
    else if(mode==1)
    {

    }
    //        Mat gray;
    ////        Mat k = getStructuringElement(MORPH_RECT,Size(3,3));
    //        Mat k1 = getStructuringElement(MORPH_RECT,Size(7,7));
    //        cvtColor(src,gray,CV_BGR2GRAY);
    //        threshold(gray,gray,110,255,THRESH_BINARY);
    //        Mat img_blue;
    //        int iLowH, iHighH, iLowS, iHighS, iLowV, iHighV;

    //        fs["iLowH"]>>iLowH;
    //        fs["iHighH"]>>iHighH;
    //        fs["iLowS"]>>iLowS;
    //        fs["iHighS"]>>iHighS;
    //        fs["iLowV"]>>iLowV;
    //        fs["iHighV"]>>iHighV;

    //        cvtColor(src, img_blue, COLOR_BGR2HSV);
    //    //    vector<Mat> planes;
    //    //    split(img_blue,planes);
    //    //    planes[2]-=30;
    //    //    merge(planes,img_blue);
    //        inRange(img_blue, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), dst);

    //        //open and close operation.
    ////        Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
    ////        Mat kernel1 = getStructuringElement(MORPH_RECT, Size(3, 3));
    //    //    Mat kernel2 = getStructuringElement(MORPH_RECT, Size(11,11));

    //    //    morphologyEx(dst, dst, MORPH_CLOSE, kernel);

    //    //    morphologyEx(dst, dst, MORPH_OPEN, kernel1);
    //    //    morphologyEx(dst, dst, CV_MOP_OPEN, kernel1);
    //        dilate(dst,dst,k1);
    //    //    morphologyEx(dst, dst, CV_MOP_CLOSE, kernel1);
    //    //        erode(dst,dst,kernel);
    //        dst = dst&gray;
    //        dilate(dst,dst,k1);



    //    Mat gray;
    //    Mat k = getStructuringElement(MORPH_RECT,Size(3,3));
    //    Mat k1 = getStructuringElement(MORPH_RECT,Size(7,7));
    //    cvtColor(src,gray,CV_BGR2GRAY);
    //    imshow("gray",gray);
    //    threshold(gray,gray,110,255,THRESH_BINARY);
}


/**
 * @brief find_armour::find_blue4
 * @param img  input  输入相机原始图像
 * @param dst  output 输出图像
 * @param RRect output 输出旋转矩形，用于解算出角度
 * @param mode  input 输入指令
 * @return Mat
 */
void find_armour::get_armor(Mat& img,Mat& dst,int mode,bool Show_Left)
{
    Clear();
    //判断是否切换命令了
    if(isROIflag==0)
    {
        clear_data();
        image_preprocess(mode,img,dst);  //图片预处理
        search_armour(img,dst);
    }
    else
    {
#ifdef SHOW_DEBUG
        if(Show_Left)
        {
            cout<<"L_in"<<endl;
        }
        else
        {
            cout<<"R_in"<<endl;
        }
#endif
        //截取本帧图片，只对截图操作
        img_ROI = roi(img,LastArmor.armor_center,LastArmor.diameter);
        image_preprocess(mode,img_ROI,dst);  //图片预处理
        search_armour(img,dst);
        //当前截图区域没找到，以半径扩展
        if(Armordatas.size()==0)
        {

            LastArmor.diameter = LastArmor.diameter*1.3;
            if(x1==1||x2==img.cols-1||y1==1||y2==img.rows-1)
            {
                isROIflag = 0;
            }
        }
    }
}

/**
 * @brief find_armour::get_Light   剔出领近的灯柱
 */
void find_armour::get_Light()
{
    size_t size = fir_armor.size();
    vector<RotatedRect> Groups;
    int cellmaxsize;
    if(fir_armor.size()<1) return;
    Groups.push_back(fir_armor[0]);
    cellmaxsize = fir_armor[0].size.height * fir_armor[0].size.width;
    if(cellmaxsize > 3500) cellmaxsize = 0;
    int maxsize;
    for(int i=1;i<size;i++){
        if(fir_armor[i].center.x - fir_armor[i-1].center.x <10){
            maxsize = fir_armor[i].size.height * fir_armor[i].size.width;
            if(maxsize > 3500) continue;
            if(maxsize > cellmaxsize) cellmaxsize = maxsize;
            Groups.push_back(fir_armor[i]);
        }else{
            Armorlists.push_back(Groups);\
            CellMaxs.push_back(cellmaxsize);
            cellmaxsize = 0;
            maxsize = 0;
            Groups.clear();
            //if(fir_armor[i].size.height * fir_armor[i].size.width > 2500) continue;
            Groups.push_back(fir_armor[i]);
            cellmaxsize = fir_armor[i].size.height * fir_armor[i].size.width;
        }
        //std::cout<<"max:"<<cellmaxsize<<std::endl;
        //sizescale = (float)fir_armor[i].size.height/(float)fir_armor[i].size.width;
        //std::cout<<"scale:"<<sizescale<<" width:"<<fir_armor[i].size.width<<std::endl;
    }
    Armorlists.push_back(Groups);
    CellMaxs.push_back(cellmaxsize);
    size = Armorlists.size();
    for(int i=0;i<size;i++){
        int Gsize = Armorlists[i].size();
        int GroupMax = CellMaxs[i];
        if(GroupMax > 5){
            for(int j=0;j<Gsize;j++){
                maxsize = Armorlists[i][j].size.height * Armorlists[i][j].size.width;
                if(maxsize == GroupMax){
                    result_armor.push_back(Armorlists[i][j]);
                    RotatedRect r = Armorlists[i][j];
                    double real_h = r.size.height < r.size.width ? r.size.width : r.size.height;
                    Vec4f contour_para(0,0,0,0);
                    contour_para[0] = real_h;
                    contour_para[1] = r.center.x;
                    contour_para[2] = r.center.y;
                    contour_para[3] = r.angle;
                    contours_para.push_back(contour_para);
                }
            }
        }
    }

}

/**
 * @brief find_armour::src_get_armor  在原图（截图）中找装甲板
 */
void find_armour::src_get_armor()
{
    int size = result_armor.size();
    if(size < 2){
        return;
    }
    Point2f center_point1,center_point2;
    Point2f _pt[4],pt[4];
    float height1,height2;
    float angle1,angle2;
    float area1,area2,area_rate;
    float x1,x2;
    float y1,y2;
    float y_dist,x_dist,min_h,height_d,K,x2h_rate,/*angle_diff,*/max_h;
    float angle_of_Rotated,height_of_Rotated;
    if(size==2){
        height1 = contours_para[0][0];
        x1 = contours_para[0][1];
        y1 = contours_para[0][2];
        angle1 = fabs(contours_para[0][3]);
        center_point1 = result_armor[0].center;
        area1 = result_armor[0].size.height * result_armor[0].size.width;
        result_armor[0].points(_pt);
         /**
          * pt
          * 0 2
          * 1 3
        */
        if(angle1 > 50.0)
        {
            pt[0] = _pt[3];
            pt[1] = _pt[0];
        }
        else
        {
            pt[0] = _pt[2];
            pt[1] = _pt[3];
        }

        height2 = contours_para[1][0];
        x2 = contours_para[1][1];
        y2 = contours_para[1][2];
        angle2 = fabs(contours_para[1][3]);
        center_point2 = result_armor[1].center;
        area2 = result_armor[1].size.height * result_armor[1].size.width;

        float angle_d = fabs(angle2-angle1);
        y_dist = fabs(y2-y1);
        if(y1>y2) y_dist = y1-y2;
        else y_dist = y2-y1;
        x_dist = x2-x1;
        min_h = min(height1,height2);
        max_h = max(height1,height2);
        height_d = fabs(height2-height1);
        if(center_point1.x != center_point2.x)
        {
            K = (center_point1.y - center_point2.y) / (center_point1.x - center_point2.x);
            //装甲板右边的点
            result_armor[1].points(_pt);
            if(angle2 > 50.0)
            {
                pt[2] = _pt[2];
                pt[3] = _pt[1];
            }else{
                pt[2] = _pt[1];
                pt[3] = _pt[0];
            }
            //面积比
            if(area1 > area2){
                area_rate = area1 / area2;
            }else{
                area_rate = area2 / area1;
            }
            angle_of_Rotated = MAX(Point_Angle(pt[0],pt[2]),Point_Angle(pt[1],pt[3]));//旋转矩形的角度
            height_of_Rotated = MAX(MAX(result_armor[0].size.width,result_armor[1].size.width),
                                    MAX(result_armor[0].size.height,result_armor[1].size.height)); //旋转矩形的
            x2h_rate = x_dist/height_of_Rotated;
            //get circle diameter
            float d=sqrt(pow(contours_para[0][1]-contours_para[1][1],2)
                    +pow(contours_para[0][2]-contours_para[1][2],2));
//            float dh_rate = max(d/height1,d/height2);
//            cout<<"angle::"<<angle1<<" "<<angle2<<"rate:"<<x2h_rate<<" "<<dh_rate<<" "<<height_d<<endl;
//            cout<<"y_dist::"<<y_dist<<" "<<angle_d<<endl;
            if(y_dist<0.4*(height1+height2)&&(angle_d<20||angle_d>50)
                   &&fabs(K)<0.5&&angle_of_Rotated<20&&area_rate<3.0&&x2h_rate>=0.8&&x2h_rate<=5&&/*dh_rate<4.5&&*/height_d<0.4*max_h)
            {
                Armordata pushdata;
                Point center=Point2f((x1+x2)*0.5,(y1+y2)*0.5);
                pushdata.armor_points[0] = pt[0];
                pushdata.armor_points[1] = pt[1];
                pushdata.armor_points[2] = pt[2];
                pushdata.armor_points[3] = pt[3];
                pushdata.diameter = d;
                pushdata.armor_center = center;
                ArmorPoints.push_back(center);
                if(x2h_rate>3.5)
                {// big_armor
                    pushdata.armor = big_armor;
                }
                Armordatas.push_back(pushdata);

            }
        }
    }
    else if(size>=3){
        for(int i=0;i<size-1;i++)
        {
            height1 = contours_para[i][0];
            x1 = contours_para[i][1];
            y1 = contours_para[i][2];
            angle1 = fabs(contours_para[i][3]);
            center_point1 = result_armor[i].center;
            area1 = result_armor[i].size.height * result_armor[i].size.width;
            result_armor[i].points(_pt);
             /**
              * pt
              * 0 2
              * 1 3
            */
            if(angle1 > 50.0)
            {
                pt[0] = _pt[3];
                pt[1] = _pt[0];
            }
            else
            {
                pt[0] = _pt[2];
                pt[1] = _pt[3];
            }
            for(int j = i+1;j<size;j++)
            {
                height2 = contours_para[j][0];
                x2 = contours_para[j][1];
                y2 = contours_para[j][2];
                angle2 = fabs(contours_para[j][3]);

                center_point2 = result_armor[j].center;
                area2 = result_armor[j].size.height * result_armor[j].size.width;

                float angle_d = fabs(angle2-angle1);
                y_dist = fabs(y2-y1);
                if(y1>y2) y_dist = y1-y2;
                else y_dist = y2-y1;
                x_dist = x2-x1;
                min_h = min(height1,height2);
                max_h = max(height1,height2);
                height_d = fabs(height2-height1);
                if(center_point1.x != center_point2.x)
                {
                    K = (center_point1.y - center_point2.y) / (center_point1.x - center_point2.x);

                    //装甲板右边的点
                    result_armor[j].points(_pt);
                    if(angle2 > 50.0)
                    {
                        pt[2] = _pt[2];
                        pt[3] = _pt[1];
                    }else{
                        pt[2] = _pt[1];
                        pt[3] = _pt[0];
                    }
                    //面积比
                    if(area1 > area2){
                        area_rate = area1 / area2;
                    }else{
                        area_rate = area2 / area1;
                    }
                    angle_of_Rotated = MAX(Point_Angle(pt[0],pt[2]),Point_Angle(pt[1],pt[3]));//旋转矩形的角度

                    height_of_Rotated = MAX(MAX(result_armor[i].size.width,result_armor[j].size.width),
                                            MAX(result_armor[i].size.height,result_armor[j].size.height));
                    x2h_rate = x_dist/height_of_Rotated;
                    //get circle diameterArmorPoints.push_back(armor_center);
                    float d=sqrt(pow(contours_para[i][1]-contours_para[j][1],2)
                            +pow(contours_para[i][2]-contours_para[j][2],2));
//                    float dh_rate = max(d/height1,d/height2);
                    if(isROIflag==0)
                    {
//                    cout<<"Rate::"<<x2h_rate<<" "<<fabs(K)<<" "<<endl;
//                    cout<<"IN?"<<endl;
                        if(y_dist<=0.4*(height1+height2)&&(angle_d<20||angle_d>60)
                               &&fabs(K)<0.4&&angle_of_Rotated<30&&area_rate<3.0&&x2h_rate>=0.8&&x2h_rate<=4.5&&/*dh_rate<4.5&&*/height_d<0.45*(height1+height2))
                        {
//                            cout<<"IN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<endl;
                            Armordata pushdata;
                            Point center=Point2f((x1+x2)*0.5,(y1+y2)*0.5);
                            pushdata.armor_points[0] = pt[0];
                            pushdata.armor_points[1] = pt[1];
                            pushdata.armor_points[2] = pt[2];
                            pushdata.armor_points[3] = pt[3];
                            pushdata.diameter = d;
                            pushdata.armor_center = center;
                            ArmorPoints.push_back(center);
                            if(x2h_rate>3.5)
                            {// big_armor
                                pushdata.armor = big_armor;
                            }
                            Armordatas.push_back(pushdata);
                        }
                    }
                    else
                    {//size>3+截图
                        if(y_dist<0.45*(height1+height2)
                               &&fabs(K)<0.5&&angle_of_Rotated<20&&area_rate<3.5&&x2h_rate>=0.8&&x2h_rate<=4.5&&/*dh_rate<4.5&&*/height_d<0.5*max_h)
                        {
                            Armordata pushdata;
                            Point center=Point2f((x1+x2)*0.5,(y1+y2)*0.5);
                            pushdata.armor_points[0] = pt[0];
                            pushdata.armor_points[1] = pt[1];
                            pushdata.armor_points[2] = pt[2];
                            pushdata.armor_points[3] = pt[3];
                            pushdata.diameter = d;
                            pushdata.armor_center = center;
                            ArmorPoints.push_back(center);
                            if(x2h_rate>3.5)
                            {
                                pushdata.armor = big_armor;
                            }
                            ArmorPoints.push_back(center);
                        }
                    }
                }
            }
        }
    }
}

/**
 * @brief find_armour::search_armour  寻找装甲板的方法。分为截图和不截图两种情况
 * @param img input img
 * @param dst input dst
 * @param armour_center  output save center of armor
 * @param diameters      output save diameter of armor
 * @param flags          input  is a flag to get ROI or not
 * @param Rotated_angles    output save RotatedRect's Angle
 * @param Heights        output save Height
 */
void find_armour::search_armour(Mat &img,Mat &dst)
{
    vector<vector<Point> > contours;

    //findcontuors最后一个参数是偏移量，对于截图来说，巨好用
    findContours(dst,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_SIMPLE,Point(x1,y1));

    int num = contours.size();   //contour's amount
    for(int i = 0;i<num;i++)
    {
        RotatedRect r = minAreaRect(contours[i]);
        double real_h = r.size.height < r.size.width ? r.size.width : r.size.height;
        double real_w = r.size.height < r.size.width ? r.size.height : r.size.width;
        if(real_h<1.1*real_w) continue;
        //长宽分明时。筛去平躺的矩形
        if((r.size.height>r.size.width&&r.angle>-30)
            ||(r.size.height<r.size.width&&r.angle<-60))
        {
#ifdef SHOW_DEBUG
            Mat vertice;
            boxPoints(r,vertice);
            for(int k = 0;k<4;k++)
            {
                Point p1 = Point(vertice.row(k));
                int n = (k+1)%4;
                Point p2 = Point(vertice.row(n));
                line(img,p1,p2,Scalar(0,255,0),2);
            }
#endif
            fir_armor.push_back(r);
        }
    }
    num = fir_armor.size();
    if(num>=4){
        //灯柱数量大于等于4,去掉最小矩形。
        list<float> fir_armor_areas;
        for(int i = 0;i<num;i++)
        {
            fir_armor_areas.push_back(fir_armor[i].size.width*fir_armor[i].size.width);
        }
        fir_armor_areas.sort();
        fir_armor_areas.pop_front();
        while(fir_armor_areas.front()>fir_armor_areas.back()*10)
        {
//            cout<<"去掉小矩形"<<endl;
            fir_armor_areas.pop_back();  //去掉小面积
        }
    }
    sort(fir_armor.begin(),fir_armor.end(),Sort_RotatedRect);
    get_Light();
//    cout<<result_armor.size()<<endl;
    sort(result_armor.begin(),result_armor.end(),Sort_RotatedRect);
    src_get_armor();
}
