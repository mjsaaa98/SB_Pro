#include "Header.h"
#include <ImgFactory.h>
#include "camera_calibration.h"
//#include "RMVideoCapture.h"
#include "armorpredict.h"
#include "CRC_Check.h"
#include "serialport.h"
#include "solvepnp.h"
#include "stereo_vision.h"
#include "v4l2_set.h"
//#define DEBUG
FileStorage fs("./../SB_Pro/canshu.yaml",FileStorage::READ);
int main()
{
    // jiang biaoding canshu xieru wenjian
    camera_two_calibration();

/// =======================================chushihua=================================
    // open the camera and local the camera
    int fd1 = open("/dev/video0",O_RDWR);
    int fd2 = open("/dev/video1",O_RDWR);
    v4l2_set vs1(fd1),vs2(fd2);
//    vs1.set_saturation(70);      //饱和度
    vs1.set_exposure(10);     //曝光
//    vs2.set_saturation(150);      //饱和度
    vs2.set_exposure(10);     //曝光
//    vs1.set_contrast(64);
//    vs2.set_contrast(32);
    int camnum1 = vs1.set_camnum();
    int camnum2 = vs2.set_camnum();
    VideoCapture camera1(0),camera2(1);
    camera1.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    camera1.set(CV_CAP_PROP_FRAME_HEIGHT,720);
    camera2.set(CV_CAP_PROP_FRAME_WIDTH,1280);
    camera2.set(CV_CAP_PROP_FRAME_HEIGHT,720);
    if (!camera1.isOpened())
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
    cout << "摄像机打开完成" << endl;

/// =======================================chushihuawancheng=================================
    if(camstatus[0] && camstatus[1])
    {
        ImgFactory imgfactory;
        std::thread t1(&ImgFactory::Left_read,&imgfactory,ref(cap_left));
        std::thread t2(&ImgFactory::Right_read,&imgfactory,ref(cap_right));
        std::thread t3(&ImgFactory::Img_handle,&imgfactory);
        t1.join();
        t2.join();
        t3.join();
    }
    else
    {
        cout<<"shexiangtou de shunxu youwu!!"<<endl;
    }
    camera1.release();
    camera2.release();
    return 0;
}
