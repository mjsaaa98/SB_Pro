#include "Header.h"
#include <ImgFactory.h>
#include "camera_calibration.h"
#include "armorpredict.h"
#include "CRC_Check.h"
#include "serialport.h"
#include "solvepnp.h"
#include "stereo_vision.h"
#include "v4l2_set.h"
FileStorage fs("./../SB_Pro/canshu.yaml",FileStorage::READ);
void Read_Img(VideoCapture &cap, Mat &src)
{
//    QTime time;
//    time.start();
    cap >> src;
//    cout<<"time_read:"<<time.elapsed()<<"ms"<<endl;

}
int main()
{
    //
    camera_two_calibration();

/// =======================================chushihua=================================
    // open the camera and local the camera
    int fd1 = open("/dev/video0",O_RDWR);
    int fd2 = open("/dev/video1",O_RDWR);
    v4l2_set vs1(fd1),vs2(fd2);
    vs1.set_saturation(128);      //饱和度
    vs1.set_exposure(22);     //曝光
    vs2.set_saturation(128);      //饱和度
    vs2.set_exposure(22);     //曝光
//    vs1.set_contrast(64);
//    vs2.set_contrast(32);
    int camnum1 = vs1.set_camnum();
    int camnum2 = vs2.set_camnum();
    VideoCapture camera1(0),camera2(1);
    camera1.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    camera1.set(CV_CAP_PROP_FRAME_HEIGHT,720);
    camera2.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    camera2.set(CV_CAP_PROP_FRAME_HEIGHT,720);
    if (!camera1.isOpened()||!camera2.isOpened())
    {
        cout << "Failed!"<<endl;
        return 0;
    }
    VideoCapture cap_left,cap_right;
    bool camstatus[2] = {false,false};
    if(camnum1 == 2){
        cap_left = camera1;
        camstatus[0] = true;
    }
    else if(camnum2 == 2){
        cap_left = camera2;
        camstatus[0] = true;
    }
    else{
        camstatus[0] = false;
    }

    if(camnum1 == 1){
        cap_right = camera1;
        camstatus[1] = true;
    }
    else if(camnum2 == 1){
        cap_right = camera2;
        camstatus[1] = true;
    }
    else{
        camstatus[1] = false;
    }
    cout << "摄像机初始化完成!!" << endl;

/// =======================================chushihuawancheng=================================
//    AngleSolver Left_PnP,Right_PnP;
    stereo_vision Stereo;
    FileStorage stereo_yaml("/home/s305-nuc5/Downloads/build-SB_Pro-Desktop-Debug/camera_calibrate.yaml",FileStorage::READ);
    Mat L_camera_matrix,R_camera_matrix,L_dist_matrix,R_dist_matrix;
    stereo_yaml["cameraMatrixL"] >> L_camera_matrix;
    stereo_yaml["cameraMatrixR"] >> R_camera_matrix;
    stereo_yaml["distCoeffL"] >> L_dist_matrix;
    stereo_yaml["distCoeffR"] >> R_dist_matrix;

//    //chushihua wei xiaozhuangjiaban
//    Left_PnP.Init(L_camera_matrix,L_dist_matrix,13.5,6.5);
//    Right_PnP.Init(R_camera_matrix,R_dist_matrix,13.5,6.5);
//    Left_PnP.set_Axis(110,110,90);
//    Right_PnP.set_Axis(110,110,90);

    //初始化双目
    Stereo.setAxis(90,35,20);

    find_armour L_find_armour,R_find_armour;
    ArmorPredict A_Predict;
    vector<AbsPosition> Positions;

    vector<Point2f> Left_Points,Right_Points;
    vector<Armordata> Left_Armordata, Right_Armordata;
    Mat L_frame,R_frame;
    int mode = 2;
    int small_dis_i = -1;
#ifdef OPEN_SERIAL
    SerialPort sp;
    sp.initSerialPort();
#endif
    if(!camstatus[0] || !camstatus[1])
    {
        cout<<"左右摄像头有误！！"<<endl;
        return 0;
    }
    while(1)
    {
//#ifdef PRINT
//        QTime time;
//        time.start();
//#endif
        std::thread L_read(Read_Img,ref(cap_left),ref(L_frame));
        std::thread R_read(Read_Img,ref(cap_right),ref(R_frame));
        L_read.join();
        R_read.join();
//#if PRINT
//        cout<<"L_time_read:"<<time.elapsed()<<"ms"<<endl;
//#endif

//#ifdef OPEN_SERIAL
//        sp.get_Mode(mode);
//#endif

        Mat L_dst,R_dst;
//        thread L_get_armor(&find_armour::get_armor,&L_find_armour,ref(L_frame),ref(L_dst),mode,true);
//        thread R_get_armor(&find_armour::get_armor,&L_find_armour,ref(R_frame),ref(R_dst),mode,false);
//        L_get_armor.join();
//        R_get_armor.join();
        L_find_armour.get_armor(L_frame,L_dst,mode,true);
        R_find_armour.get_armor(R_frame,R_dst,mode,false);

        Left_Points = L_find_armour.ArmorPoints;
        Right_Points = R_find_armour.ArmorPoints;
        Left_Armordata = L_find_armour.Armordatas;
        Right_Armordata = R_find_armour.Armordatas;

        size_t Left_size = Left_Points.size();
        size_t Right_size = Right_Points.size();

//        cout<<"Size:::::::::::::::::::::::::::"<<Left_size<<" "<<Right_size<<endl;
#ifdef PRINT
        cout<<"Left_size&Right_size:"<<Left_size<<"    "<<Right_size<<endl;
        for(size_t i=0;i<Left_Points.size();i++){
            cout<<"L_YOUT:"<<Left_Points[i].y<<"   ";
        }
        for(size_t i=0;i<Right_Points.size();i++){
            cout<<"R_YOUT:"<<Right_Points[i].y<<"  ";
        }
        cout<<endl;
#endif

/// ========================双目矫正+测距===================================================
        if(Left_size == 0 || Right_size == 0){
            memset(&A_Predict.Vision,0,sizeof(VisionData));
        }else{


#ifdef SHOW_DEBUG
            for(int i =0 ;i < Left_size;i++)
            {
                circle(L_frame,Left_Points[i],30,Scalar(0,0,255),5);
            }
            for(int i =0 ;i < Right_size;i++)
            {
                circle(R_frame,Right_Points[i],30,Scalar(0,0,255),5);
            }
#endif

            Positions.clear();  //清空容器
            if(Left_size == Right_size){
//                cout<<"Points pipei!!!!!!!!!!!!!!"<<endl;
                Stereo.get_location(Left_Points,Right_Points,Positions);
                if(Positions.size()==0) continue;
                small_dis_i = A_Predict.Predict(Positions);
#ifdef SHOW_DEBUG
                circle(L_frame,Left_Points[small_dis_i],40,Scalar(255,0,0),5);
                circle(R_frame,Right_Points[small_dis_i],40,Scalar(255,0,0),5);
#endif  //SHOW_DEBUG
                L_find_armour.LastArmor = Left_Armordata[A_Predict.Result.index];
                R_find_armour.LastArmor = Right_Armordata[A_Predict.Result.index];

#ifdef RRINT
                cout<<"LCenter"<<L_find_armour.LastArmor.armor_center<<" "<<"R_center"<<R_find_armour.LastArmor.armor_center<<endl;
#endif  //PRINT
                L_find_armour.isROIflag = 1;
                R_find_armour.isROIflag = 1;
            }
//            else if(Left_size>Right_size)
//            {
//                cout<<"左边点多，开始选点！"<<endl;

//                for (int i = 0;i<Left_size;i++)
//                {
//                    cout<<"L_Y:"<<Left_Points[i]<<" ";
//                }
//                cout<<endl;
//                for (int i = 0;i<Right_size;i++)
//                {
//                    cout<<"R_Y:"<<Right_Points[i]<<" ";
//                }
//                cout<<endl;

//                size_t t = Left_size-Right_size;
//                int i=0,j,s;
//                vector<Point2f> temp;
//                //开始匹配选点
//                for(s = 0;s<Right_size;s++)
//                {
//                    for(i; i<=t+s ;i++)
//                    {
//                        cout<<"here"<<endl;
//                        if(103<=abs((Left_Points[i].y-Right_Points[s].y))&&abs((Left_Points[i].y-Right_Points[s].y))<=115
//                                &&Left_Points[i].x>Right_Points[s].x
//                                /*&&250<=Left_Points[i].x-Right_Points[s].x&&Left_Points[i].x-Right_Points[s].x<=280*/)
//                        {
//                            cout<<"in"<<endl;
//                            temp.push_back(Left_Points[i]);
//                            i++;
//                            break;
//                        }
//                    }
//                    if(i==t+s+1&&s<Right_size-1)
//                    {
//                        for (int j = i;j<Left_size;j++)
//                        {
//                            temp.push_back(Left_Points[j]);
//                        }
//                        break;
//                    }
//                }
//                Left_Points.clear();
//                Left_Points = temp;
//                Left_size = Left_Points.size();
//                if(Left_size == Right_size){
//                    cout<<"选点成功！"<<endl;
//                    for (int i = 0;i<Left_size;i++)
//                    {
//                        cout<<"Xuan_L_Y:"<<Left_Points[i]<<" "<<Right_Points[i]<<endl;
//                    }
//                    Stereo.get_location(Left_Points,Right_Points,Positions);
//                    if(Positions.size()==-1) continue;
//                    small_dis_i = A_Predict.Predict(Positions);
//    #ifdef SHOW_DEBUG
//                    circle(L_frame,Left_Points[small_dis_i],40,Scalar(255,0,0),5);
//                    circle(R_frame,Right_Points[small_dis_i],40,Scalar(255,0,0),5);
//    #endif
//                    L_find_armour.LastArmor = Left_Armordata[A_Predict.Result.index];
//                    R_find_armour.LastArmor = Right_Armordata[A_Predict.Result.index];
//                    L_find_armour.isROIflag = 1;
//                    R_find_armour.isROIflag = 1;
//                }
//                else{
//                    cout<<"Failed!"<<endl;
//                }
//            }
//            else{
//                cout<<"右边点多，开始选点！"<<endl;
//                size_t t = Right_size-Left_size;
//                int i=0,j,s;
//                vector<Point2f> temp;
//                //开始匹配选点
//                for(s = 0;s<Left_size;s++)
//                {
//                    for(i; i<=t+s ;i++)
//                    {
//                        if(103<=abs((Left_Points[s].y-Right_Points[i].y))&&abs((Left_Points[s].y-Right_Points[i].y))<=115
//                                &&Left_Points[s].x>Right_Points[i].x
//                                /*&&250<=(Left_Points[s].x-Right_Points[i].x)&&(Left_Points[s].x-Right_Points[i].x)<=280*/)
//                        {
//                            cout<<"Y最短距离"<<abs((Left_Points[s].y-Right_Points[i].y))<<endl;
//                            temp.push_back(Right_Points[i]);
//                            i++;
//                            break;
//                        }
//                        cout<<"Y距离"<<abs((Left_Points[s].y-Right_Points[i].y))<<endl;
//                    }
//                    if(i==t+s+1&&s<Left_size-1)
//                    {
//                        for (int j = i;j<Right_size;j++)
//                        {
//                            temp.push_back(Right_Points[j]);
//                        }
//                        break;
//                    }
//                }
//                Right_Points.clear();
//                Right_Points = temp;
//                Right_size = Right_Points.size();
//                if(Left_size == Right_size){
//                    cout<<"选点成功！"<<endl;
//                    for (int i = 0;i<Left_size;i++)
//                    {
//                        cout<<"L_Y:"<<Left_Points[i]<<" "<<Right_Points[i]<<endl;
//                    }
//                    Stereo.get_location(Left_Points,Right_Points,Positions);
//                    if(Positions.size()==-1) continue;
//                    small_dis_i = A_Predict.Predict(Positions);
//    #ifdef SHOW_DEBUG
//                    circle(L_frame,Left_Points[small_dis_i],40,Scalar(255,0,0),5);
//                    circle(R_frame,Right_Points[small_dis_i],40,Scalar(255,0,0),5);
//    #endif
//                    L_find_armour.LastArmor = Left_Armordata[A_Predict.Result.index];
//                    R_find_armour.LastArmor = Right_Armordata[A_Predict.Result.index];
//                    L_find_armour.isROIflag = 1;
//                    R_find_armour.isROIflag = 1;
//                }
//                else{
//                    cout<<"Failed!"<<endl;
//                }
//            }
        }

////------------------------单目------------------------------------
//            else if(Left_size > Right_size){
//                Left_PnP.get_location(Left_Armordata,Positions);
//                A_Predict.Predict(Positions);
//            }
//            else if(Left_size < Right_size){
//                Right_PnP.get_location(Right_Armordata,Positions);
//                A_Predict.Predict(Positions);
//            }

//                cout<<"time:"<<(t3-t1)/getTickFrequency()*1000<<endl;

#ifdef SHOW_DEBUG
        char screen_data[100];
        sprintf(screen_data,"dis:%fm",A_Predict.Vision.dis.f/1000);
        putText(L_frame,screen_data,Point(100,100),1,5,Scalar(255,255,255));
        imshow("LEFT_img",L_frame);
//        imshow("LEFT_dst",L_dst);
        imshow("RIGHT_img",R_frame);
//        imshow("RIGHT_dst",R_dst);
#endif
#ifdef OPEN_SERIAL
        sp.TransformData(A_Predict.Vision);
#endif
#ifdef SHOW_DEBUG
        int i = waitKey(1);
        if( i=='q') break;
#endif
    }
//    else
//    {
//        cout<<"shexiangtou de shunxu youwu!!"<<endl;
//    }
    camera1.release();
    camera2.release();
    return 0;
}
