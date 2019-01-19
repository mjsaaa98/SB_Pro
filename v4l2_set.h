#ifndef V4L2_SET_H
#define V4L2_SET_H

#include "v4l2_set.h"
#include<opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <linux/videodev2.h>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>

using namespace std;
using namespace cv;

class v4l2_set
{
public:
    v4l2_set(int);
    void set_contrast(int c);
    void set_gain(int c);
    void set_exposure(int c);
    void set_white_balance(int c);
    void set_brightness(int c);
    void set_saturation(int c);
    int set_camnum();
    int xioctl(int fd, int request, void *arg);
private:
    int h;
    int fd;
};

#endif // V4L2_SET_H
