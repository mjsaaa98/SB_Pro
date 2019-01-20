#include "v4l2_set.h"

//a class to set camera parameter

v4l2_set::v4l2_set(int handle)
{
    h = handle;
    fd = h;
}

void v4l2_set::set_contrast(int c)
{

//set value
    struct v4l2_control ctrl0;

    ctrl0.id = V4L2_CID_EXPOSURE_AUTO;
    ctrl0.value = 1;
    ioctl(fd,VIDIOC_S_CTRL,&ctrl0);
//    cout<<ctrl0.value<<endl;

    ctrl0.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    ctrl0.value = c;

    ioctl(fd,VIDIOC_S_CTRL,&ctrl0);
    cout<<ctrl0.value<<endl;
    //cout<<ctrl0.value<<endl;
}
void v4l2_set::set_exposure(int c)
{
    struct v4l2_control ctrl0;
    ctrl0.id = V4L2_CID_EXPOSURE_AUTO;
    ctrl0.value = 1;
    ioctl(fd,VIDIOC_S_CTRL,&ctrl0);
//    cout<<ctrl0.value<<endl;
    ctrl0.id = V4L2_CID_EXPOSURE;
//    int a = ioctl(fd,VIDIOC_G_CTRL,&ctrl0);
//    cout<<"===="<<a<<endl;
    ctrl0.id = V4L2_CID_EXPOSURE_ABSOLUTE;
    ctrl0.value = c;
    if (ioctl(fd,VIDIOC_S_CTRL,&ctrl0)!=-1)
        cout<<"曝光值:"<<ctrl0.value<<endl;
}
void v4l2_set::set_gain(int c)
{
    struct v4l2_control ctrl0;
    ctrl0.id = V4L2_CID_GAIN;
    ctrl0.value = c;
    ioctl(fd,VIDIOC_S_CTRL,&ctrl0);
    //cout<<ctrl0.value<<endl;
}

void v4l2_set::set_white_balance(int c)
{
    struct v4l2_control ctrl0;
    ctrl0.id = V4L2_CID_WHITE_BALANCE_TEMPERATURE;
    ctrl0.value = c;
    ioctl(fd,VIDIOC_S_CTRL,&ctrl0);
    //cout<<ctrl0.value<<endl;

}

void v4l2_set::set_brightness(int c)
{
    struct v4l2_control ctrl0;
    ctrl0.id = V4L2_CID_BRIGHTNESS;
    ctrl0.value = c;
    ioctl(fd,VIDIOC_S_CTRL,&ctrl0);
    //cout<<ctrl0.value<<endl;


}

void v4l2_set::set_saturation(int c)
{
    struct v4l2_control ctrl0;
    ctrl0.id = V4L2_CID_SATURATION;
    ctrl0.value = c;
    ioctl(fd,VIDIOC_S_CTRL,&ctrl0);
    if(ioctl(fd,VIDIOC_S_CTRL,&ctrl0) != -1)
        cout<<"饱和度:"<<ctrl0.value<<endl;
}

int v4l2_set::set_camnum(){
    struct v4l2_capability caps = {};
    if (-1 == xioctl(fd, VIDIOC_QUERYCAP, &caps)) {
            perror("Querying Capabilities\n");
            return -1;
    }
    int camnum = caps.bus_info[17] -48;
    std::cout<<"------cam-----number----:"<<camnum<<std::endl;

    return camnum;
}

int v4l2_set::xioctl(int fd, int request, void *arg){
    int r;
    do r = ioctl (fd, request, arg);
    while (-1 == r && EINTR == errno);
    return r;
}
