#include "ImgFactory.h"


ImgFactory::ImgFactory(){
    stop_pro = false;
    L_handle_flag = false;
    R_handle_flag = false;
    Show_falg = 1;
#ifdef CAMERA_DEBUG
    mode = 2;
#else
    mode = 0;
#endif
}

/**
 * @brief ImgFactory::Left_read  读取zuobianshexiangtou图像的一个线程
 */
void ImgFactory::Left_read(VideoCapture &camera0){
    Mat frame;
    while (1)
    {
        while(L_handle_flag==true);  //可以处理标志位还为真，说明图片还没被传进去处理，一直等待。
//        double t1=0,t3 = 0;
//        t1 = getTickCount();
        camera0 >> frame;
//        if (frame.empty()) break;
//        imshow("L",frame);
//        t3 = getTickCount();
//        int fps_read = (t3-t1)/getTickFrequency()*1000;
//        cout<<"time_read:"<<fps_read<<"ms"<<endl;
        //加锁,在列表插入值，同时禁止读取
        Lock.lock();
        L_list_of_frame.push_back(frame.clone());
        L_handle_flag = true;
        Lock.unlock();
    }
}

/**
 * @brief ImgFactory::Right_read  读取yuobianshexiangtou图像的一个线程
 */
void ImgFactory::Right_read(VideoCapture &camera){
    Mat frame;
    while (1)
    {
        while(R_handle_flag==true);  //可以处理标志位还为真，说明图片还没被传进去处理，一直等待。
        camera >> frame;
//        if (frame.empty()) break;
        imshow("R",frame);
        //加锁,在列表插入值，同时禁止读取
        Lock.lock();
        R_list_of_frame.push_back(frame.clone());
        R_handle_flag = true;
        Lock.unlock();
    }
}
/**
 * @brief ImgFactory::Img_handle  处理图像线程
 */
void ImgFactory::Img_handle(){
    AngleSolver Left_PnP,Right_PnP;
    stereo_vision Stereo;
    FileStorage stereo_yaml("/home/mjs/RP/build-SB_detect_armor-Desktop_Qt_5_10_0_GCC_64bit-Debug/camera_calibrate.yaml",FileStorage::READ);
    Mat L_camera_matrix,R_camera_matrix,L_dist_matrix,R_dist_matrix;
    stereo_yaml["cameraMatrixL"] >> L_camera_matrix;
    stereo_yaml["cameraMatrixR"] >> R_camera_matrix;
    stereo_yaml["distCoeffL"] >> L_dist_matrix;
    stereo_yaml["distCoeffR"] >> R_dist_matrix;
    //chushihua wei xiaozhuangjiaban
    Left_PnP.Init(L_camera_matrix,L_dist_matrix,13.5,6.5);
    Right_PnP.Init(R_camera_matrix,R_dist_matrix,13.5,6.5);
    Left_PnP.set_Axis(110,110,90);
    Right_PnP.set_Axis(110,110,90);
    //chushihau shuangmu
    Stereo.setAxis(110,110,90);

    find_armour L_find_armour,R_find_armour;
    ArmorPredict A_Predict;
    vector<AbsPosition> Positions;

    vector<Point2f> Left_Points,Right_Points;
    vector<Armordata> Left_Armordata, Right_Armordata;
#ifdef OPEN_SERIAL
    SerialPort sp;
    sp.initSerialPort();
#endif
    while(1)
    {
        Mat L_dst,R_dst;
        double t1=0,t3 = 0;
        t1 = getTickCount();
#ifdef OPEN_SERIAL
        sp.get_Mode(mode,receive_data,isreceiveflag);
#endif
        while(L_handle_flag==false||R_handle_flag==false);
        Lock.lock();
        L_frame = L_list_of_frame.back();
        L_list_of_frame.clear();
        L_handle_flag = false;
        R_frame = R_list_of_frame.back();
        R_list_of_frame.clear();
        R_handle_flag = false;
        Lock.unlock();
        L_find_armour.get_armor(L_frame,L_dst,mode,true);
        R_find_armour.get_armor(R_frame,R_dst,mode,false);
        Left_Points = L_find_armour.ArmorPoints;
        Right_Points = R_find_armour.ArmorPoints;
        Left_Armordata = L_find_armour.Armordatas;
        Right_Armordata = R_find_armour.Armordatas;

        size_t Left_size = Left_Points.size();
        size_t Right_size = Right_Points.size();
        cout<<":"<<Left_size<<"    "<<Right_size<<endl;
        if(Left_size == 0 && Right_size == 0){
            memset(&A_Predict.Vision,0,sizeof(VisionData));
        }else if(Left_size!=0 || Right_size != 0){
            if(Left_size == Right_size){
                cout<<"in"<<endl;
                int small_dis_i;
                Stereo.get_location(Left_Points,Right_Points,Positions);
                small_dis_i = A_Predict.Predict(Positions);
                circle(L_frame,Left_Points[small_dis_i],30,Scalar(0,0,255),10);
                circle(R_frame,Right_Points[small_dis_i],30,Scalar(0,0,255),10);
//                putText(L_frame,string(Left_Points[small_dis_i].x),Left_Points[small_dis_i],FONT_HERSHEY_SIMPLEX,3,Scalar(255,255,255));
//                putText(R_frame,string(Right_Points[small_dis_i].x),Right_Points[small_dis_i],FONT_HERSHEY_SIMPLEX,3,Scalar(255,255,255));
                cout<<"Left:"<<Left_Points[small_dis_i].x<<endl<<"Right:"<<Right_Points[small_dis_i].x<<endl;
                L_find_armour.LastArmor = Left_Armordata[A_Predict.Result.index];
                R_find_armour.LastArmor = Right_Armordata[A_Predict.Result.index];
            }
//            else if(Left_size > Right_size){
//                Left_PnP.get_location(Left_Armordata,Positions);
//                A_Predict.Predict(Positions);
//            }
//            else if(Left_size < Right_size){
//                Right_PnP.get_location(Right_Armordata,Positions);
//                A_Predict.Predict(Positions);
//            }

        }
            imshow("LEFT_img",L_frame);
            imshow("LEFT_dst",L_dst);
            imshow("RIGHT_img",R_frame);
            imshow("RIGHT_dst",R_dst);

            t3 = getTickCount();
            cout<<"dis:"<<A_Predict.Result.z<<"m"<<endl;
            cout<<"time:"<<(t3-t1)/getTickFrequency()*1000<<endl;
#ifdef OPEN_SERIAL
        sp.TransformData(A_Predict.Vision);
#endif
        int i = waitKey(1);
        if( i=='q') break;
    }
#ifdef OPEN_SERIAL
    sp.Close();
#endif
}
