#ifndef IMGFACTORY_H
#define IMGFACTORY_H
#include <find_armour.h>
#include <solvepnp.h>
#include <thread>
#include <mutex>
#include <RMVideoCapture.h>
#include <armorpredict.h>
#include "stereo_vision.h"
#include "Header.h"
#define BUFF_SIZE 1


//#define OPEN_SERIAL
#define CAMERA_DEBUG
class ImgFactory
{
    bool L_handle_flag;
    bool R_handle_flag;
    std::mutex Lock;

private:
    bool stop_pro;
    bool Show_falg;
    int mode;
    Mat L_frame,R_frame;

    list<Mat> L_list_of_frame;
    list<Mat> R_list_of_frame;
public:
    ImgFactory();
    void Left_read(VideoCapture &cap);
    void Right_read(VideoCapture &cap);
    void Img_handle();
};

#endif // IMGFACTORY_H
