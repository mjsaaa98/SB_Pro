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
#ifdef PRINT
        double t1=0,t3 = 0;
        t1 = getTickCount();
#endif
        while(L_handle_flag==true);  //可以处理标志位还为真，说明图片还没被传进去处理，一直等待。
        camera0 >> frame;
        if (frame.empty()) break;
#ifdef PRINT
        t3 = getTickCount();
        int fps_read = (t3-t1)/getTickFrequency()*1000;
        cout<<"L_time_read:"<<fps_read<<"ms"<<endl;
#endif
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
#ifdef PRINT
        double t1=0,t3 = 0;
        t1 = getTickCount();
#endif
        while(R_handle_flag==true);  //可以处理标志位还为真，说明图片还没被传进去处理，一直等待。
        camera >> frame;
//        if (frame.empty()) break;
#ifdef PRINT
        t3 = getTickCount();
        int fps_read = (t3-t1)/getTickFrequency()*1000;
        cout<<"R_time_read:"<<fps_read<<"ms"<<endl;
#endif
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
    FileStorage stereo_yaml("/home/s305-nuc5/Downloads/build-SB_Pro-Desktop-Debug/camera_calibrate.yaml",FileStorage::READ);
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
    Stereo.setAxis(120,50,20);

    find_armour L_find_armour,R_find_armour;
    ArmorPredict A_Predict;
    vector<AbsPosition> Positions;

    vector<Point2f> Left_Points,Right_Points;
    vector<Armordata> Left_Armordata, Right_Armordata;
    char screen_data[100];
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
//        int CommanID;
//        sp.get_Mode(CommanID);
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
#ifdef PRINT
        cout<<":"<<Left_size<<"    "<<Right_size<<endl;
        for(size_t i=0;i<Left_Points.size();i++){
            cout<<"L_YOUT:"<<Left_Points[i].y<<"   ";
        }
        for(size_t i=0;i<Right_Points.size();i++){
            cout<<"R_YOUT:"<<Right_Points[i].y<<"  ";
        }
        cout<<endl;
#endif
//***************************************双目矫正+测距***************************************
        //只有一边观察到
        if(Left_size == 0 || Right_size == 0){
            memset(&A_Predict.Vision,0,sizeof(VisionData));
        }else /*if(Left_size!=0 && Right_size != 0)*/{
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
            Positions.clear();  //qingkong neicun
            if(Left_size == Right_size){
//                cout<<"Points pipei!!!!!!!!!!!!!!"<<endl;
                int small_dis_i=100;
                Stereo.get_location(Left_Points,Right_Points,Positions);
                small_dis_i = A_Predict.Predict(Positions);
#ifdef SHOW_DEBUG
                circle(L_frame,Left_Points[small_dis_i],40,Scalar(255,0,0),5);
                circle(R_frame,Right_Points[small_dis_i],40,Scalar(255,0,0),5);
#endif
//                cout<<"Left:"<<Left_Points[small_dis_i].x<<endl<<"Right:"<<Right_Points[small_dis_i].x<<endl;
//                cout<<"LeftYYYY:"<<Left_Points[small_dis_i].y<<endl<<"Right:"<<Right_Points[small_dis_i].y<<endl;
                L_find_armour.LastArmor = Left_Armordata[A_Predict.Result.index];
                R_find_armour.LastArmor = Right_Armordata[A_Predict.Result.index];
                L_find_armour.isROIflag = 1;
                R_find_armour.isROIflag = 1;
            }
//            else if(Left_size>Right_size)
//            {
//                cout<<"左边点多，开始选点！"<<endl;
//                size_t t = Left_size-Right_size;
//                int i=0,j,s;
//                vector<Point2f> temp;
//                //开始匹配选点
//                for(s = 0;s<Right_size;s++)
//                {
//                    for(i; i<=t+s ;i++)
//                    {
//                        if(72<=abs((Left_Points[i].y-Right_Points[s].y))&&abs((Left_Points[i].y-Right_Points[s].y))<=87
//                                /*&&250<=Left_Points[i].x-Right_Points[s].x&&Left_Points[i].x-Right_Points[s].x<=280*/)
//                        {
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
//                if(Left_size == Right_size){
//                    cout<<"选点成功！"<<endl;
//                    int small_dis_i;
//                    Stereo.get_location(Left_Points,Right_Points,Positions);
//                    small_dis_i = A_Predict.Predict(Positions);
//    #ifdef SHOW_DEBUG
//                    circle(L_frame,Left_Points[small_dis_i],40,Scalar(255,0,0),5);
//                    circle(R_frame,Right_Points[small_dis_i],40,Scalar(255,0,0),5);
//    #endif
//    //                cout<<"Left:"<<Left_Points[small_dis_i].x<<endl<<"Right:"<<Right_Points[small_dis_i].x<<endl;
//                    cout<<"LeftYYYY:"<<Left_Points[small_dis_i].y<<endl<<"Right:"<<Right_Points[small_dis_i].y<<endl;
//                    L_find_armour.LastArmor = Left_Armordata[A_Predict.Result.index];
//                    R_find_armour.LastArmor = Right_Armordata[A_Predict.Result.index];
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
//                        if(70<=abs((Left_Points[s].y-Right_Points[i].y))&&abs((Left_Points[s].y-Right_Points[i].y))<=85
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
//                    int small_dis_i;
//                    Stereo.get_location(Left_Points,Right_Points,Positions);
//                    small_dis_i = A_Predict.Predict(Positions);
//    #ifdef SHOW_DEBUG
//                    circle(L_frame,Left_Points[small_dis_i],40,Scalar(255,0,0),5);
//                    circle(R_frame,Right_Points[small_dis_i],40,Scalar(255,0,0),5);
//    #endif
//    //                putText(L_frame,string(Left_Points[small_dis_i].x),Left_Points[small_dis_i],FONT_HERSHEY_SIMPLEX,3,Scalar(255,255,255));
//    //                putText(R_frame,string(Right_Points[small_dis_i].x),Right_Points[small_dis_i],FONT_HERSHEY_SIMPLEX,3,Scalar(255,255,255));
//    //                cout<<"Left:"<<Left_Points[small_dis_i].x<<endl<<"Right:"<<Right_Points[small_dis_i].x<<endl;
////                    cout<<"LeftYYYY:"<<Left_Points[small_dis_i].y<<endl<<"Right:"<<Right_Points[small_dis_i].y<<endl;
//                    L_find_armour.LastArmor = Left_Armordata[A_Predict.Result.index];
//                    R_find_armour.LastArmor = Right_Armordata[A_Predict.Result.index];
//                }
//            }

////------------------------单目------------------------------------
//            else if(Left_size > Right_size){
//                Left_PnP.get_location(Left_Armordata,Positions);
//                A_Predict.Predict(Positions);
//            }
//            else if(Left_size < Right_size){
//                Right_PnP.get_location(Right_Armordata,Positions);
//                A_Predict.Predict(Positions);
//            }

        }
                t3 = getTickCount();
//                cout<<"time:"<<(t3-t1)/getTickFrequency()*1000<<endl;#ifdef OPEN_SERIAL

        sprintf(screen_data,"dis:%fm   time:%fs",A_Predict.Vision.dis.f/1000,(t3-t1)/getTickFrequency()*1000);
        putText(L_frame,screen_data,Point(100,100),1,5,Scalar(255,255,255));
#ifdef SHOW_DEBUG
        imshow("LEFT_img",L_frame);
        imshow("LEFT_dst",L_dst);
        imshow("RIGHT_img",R_frame);
        imshow("RIGHT_dst",R_dst);
#endif
//      cout<<"dis:"<<A_Predict.Result.z/1000.0<<"m"<<endl;
#ifdef OPEN_SERIAL
        sp.TransformData(A_Predict.Vision);
#endif
#ifdef SHOW_DEBUG
        int i = waitKey(1);
        if( i=='q') break;
#endif
    }
#ifdef OPEN_SERIAL
    sp.Close();
#endif
}
