#ifndef SERIALPROT_H
#define SERIALPROT_H

/**
 *@class  SerialPort
 *@brief  set serialport,recieve and send
 *@param  int fd
 */
#include <sys/types.h>
#include <unistd.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <iostream>
#include "CRC_Check.h"
using namespace std;

#define TRUE 1
#define FALSE 0

//模式
#define CmdID0 0x00; //关闭视觉
#define CmdID1 0x01; //识别红色
#define CmdID2 0x02; //识别蓝色


#define CmdID3 0x03; //小幅
#define CmdID4 0x04; //大幅

#define BAUDRATE 115200
#define UART_DEVICE "/dev/ttyUSB0"


//C_lflag
#define ECHOFLAGS (ECHO | ECHOE | ECHOK | ECHONL)


typedef union{
    float f;
    unsigned char c[4];
}float2uchar;

typedef union{
    int16_t d;
    unsigned char c[2];
}int16uchar;


typedef struct
{
    float2uchar yaw_angle;
    float2uchar pitch_angle;
    float2uchar dis;
    int ismiddle;
    int isfind;
}VisionData;
static double serialtimer, timerlast;

class SerialPort
{
  private:
    int fd; //串口号
    int speed, databits, stopbits, parity;
    unsigned char rdata[255]; //raw_data
    unsigned char Tdata[30];  //transfrom data
  public:
    SerialPort();
    SerialPort(char *);
    void initSerialPort();
    void set_Brate();
    int set_Bit(int, int, int);
    int set_disp_mode(int);
    void get_Mode(int &mode);
    void TransformTarPos(const VisionData &data);
    void send(int p, int yaw, int dis, char flag, int);
    void TransformData(const VisionData &data);
    void Close();
};

#endif
