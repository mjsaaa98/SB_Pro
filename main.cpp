#include "Header.h"
#include <ImgFactory.h>
#include "camera_calibration.h"
#include "RMVideoCapture.h"
#include "armorpredict.h"
#include "CRC_Check.h"
#include "serialport.h"
#include "solvepnp.h"
#include "stereo_vision.h"
//#define DEBUG

int main()
{
    // jiang biaoding canshu xieru wenjian
    camera_two_calibration();

/// =======================================chushihua=================================

    // open the camera and local the camera
    RMVideoCapture cap0("/dev/video1",3);
    if(cap0.fd!=-1){
        cap0.setVideoFormat(1280,720,1);
        cap0.setExposureTime(0, 10);
        //cap0.setSaturation(100);
        cap0.startStream();
        cap0.info();
    }
    else{
        cap0.camnum = -1;
        cout << "can't open the camera" << endl;
        return 0;
    }

    RMVideoCapture cap1("/dev/video2",3);
    if(cap1.fd!=-1){
        cap1.setVideoFormat(1280,720,1);
        cap1.setExposureTime(0,10);
        //cap1.setSaturation(100);
        cap1.startStream();
        cap1.info();
    }
    else{
        cap1.camnum = -1;
        cout << "can't open the camera" << endl;
        return 0;
    }
    RMVideoCapture cap_left,cap_right;
    bool camstatus[2] = {false,false};
    if(cap0.camnum == 2){
        cap_left = cap0;
        camstatus[0] = true;
    }
    else if(cap1.camnum == 2){
        cap_left = cap1;
        camstatus[0] = true;
    }
    else{
        camstatus[0] = false;
    }

    if(cap0.camnum == 3){
        cap_right = cap0;
        camstatus[1] = true;
    }
    else if(cap1.camnum == 3){
        cap_right = cap1;
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
    cap0.closeStream();
    cap1.closeStream();
    return 0;
}
