/*
 *  RPLIDAR
 *  Simple Data Grabber Demo App
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2019 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <stdio.h>
#include <stdlib.h>

#include "rplidar.h" //RPLIDAR standard sdk, all-in-one header

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <string.h>
#include <unistd.h>
#include <netdb.h>

#include <opencv2/opencv.hpp>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>
#include <opencv/cxcore.h>
#include <math.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

const float laserR = 150.0;
const float laserPhi = 0.0;

struct areaRectangle {
	cv::Point2f min;
	cv::Point2f max;
};

struct areaBox {
	cv::Point2f p0, p1, p2, p3;
};

const float PI = 3.14159;
#include <unistd.h>
static inline void delay(_word_size_t ms){
    while (ms >= 1000){
        usleep(1000 * 1000);
        ms -= 1000;
    };
    if (ms != 0)
        usleep(ms * 1000);
}

using namespace rp::standalone::rplidar;
int view3Send[9], view3Recv[9];

int sockSend, sockRecv;
int portSend, portRecv;
char IPSend[32], IPRecv[32];
int yes = 1;
struct sockaddr_in addrSend, addrRecv;
unsigned char buf[256];
const int offsetAngleDefault = -90; //  180;
int offsetAngle;

const int maxX = 500;
const int maxY = 500;


const cv::Scalar colorData[] = 
{cv::Scalar(  0,   0, 255), cv::Scalar(255,   0,   0), cv::Scalar(255,   0, 255),
 cv::Scalar(  0, 255,   0), cv::Scalar(  0, 255, 255), cv::Scalar(255, 255,   0),
 cv::Scalar(255, 255, 255)};

RPlidarDriver * drv; //  = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);

struct pointHistory {
    cv::Point2f p;
    int ttl;
};

const int maxPointData = 10000;
cv::Point2f pointData[maxPointData];
pointHistory pointHistory[maxPointData];

void print_usage(int argc, const char * argv[])
{
    printf("Simple LIDAR data grabber for RPLIDAR.\n"
           "Version: " RPLIDAR_SDK_VERSION "\n"
           "Usage:\n"
           "%s <com port> <baudrate> SnedIPAddress SendPort ReceiveIPAddress ReceivePort\n"
           "The default baudrate is 115200(for A2) or 256000(for A3). Please refer to the datasheet for details.\n"
           , argv[0]);
}

u_result rplidar_capture(RPlidarDriver * drv,  rplidar_response_measurement_node_t nodes[], size_t   count)
{
    u_result ans;
    
    // rplidar_response_measurement_node_t nodes[8192];
    // size_t   count = _countof(nodes);
    // count = _countof(nodes);

    printf("waiting for data...\n");

    // fetech extactly one 0-360 degrees' scan
    ans = drv->grabScanData(nodes, count);
    if (IS_OK(ans) || ans == RESULT_OPERATION_TIMEOUT) {
        drv->ascendScanData(nodes, count);
    } else {
        printf("error code: %x\n", ans);
    }

    return ans;
}

float deg180(float degree) {
    while (degree >  180) degree -= 360;
    while (degree < -180) degree += 360;
    return degree;
}

float degPos(rplidar_response_measurement_node_t node){
    return deg180(((node.angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f) + offsetAngle);
}

void init_udp(int argc, const char * argv[]) {
    int i;
    
    for(i = 1; i < 9; i++) view3Send[i] = i + 3;

    offsetAngle = offsetAngleDefault;
    if (argc < 4) {
        portSend = 9180; portRecv = 9182;
        strcpy(IPSend, "127.0.1.1");
        strcpy(IPRecv, "127.0.1.1");
    } else {
        strcpy(IPSend, argv[3]);
        portSend = atoi(argv[4]);
        strcpy(IPRecv, argv[5]);
        portRecv = atoi(argv[6]);
        if (argc == 8) offsetAngle = deg180(atoi(argv[7]));
    }
    printf("UDP connection: send %s:%d, recv %s:%d\n", IPSend, portSend, IPRecv, portRecv);

    sockSend = socket(AF_INET, SOCK_DGRAM, 0);
    addrSend.sin_family = AF_INET;
    addrSend.sin_port = htons(portSend);
    addrSend.sin_addr.s_addr = inet_addr(IPSend);
    // addrSend.sin_addr.s_addr = inet_addr("255.255.255.255");
    printf("setsockopt: %d\n", setsockopt(sockSend, SOL_SOCKET, SO_BROADCAST, (char *)&yes, sizeof(yes)));

    sockRecv = socket(AF_INET, SOCK_DGRAM, 0);
    addrRecv.sin_family = AF_INET;
    addrRecv.sin_port = htons(portRecv);
    addrRecv.sin_addr.s_addr = inet_addr(IPRecv); // INADDR_ANY;
    bind(sockRecv, (struct sockaddr *)&addrRecv, sizeof(addrRecv));

    memset(buf, 0, sizeof(buf));
}

void init_rplidar(int argc, const char * argv[]) {
    const char * opt_com_path = NULL;
    _u32         opt_com_baudrate = 115200;
    u_result     op_result;

    if (argc < 2) {
        print_usage(argc, argv);
        // exit(-1);
        opt_com_path = "/dev/ttyUSB0";
        // opt_com_baudrate = 115200;
    } else {
        opt_com_path = argv[1];
    }
    if (argc > 2) opt_com_baudrate = strtoul(argv[2], NULL, 10);
    // create the driver instance
    drv = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);

    if (!drv) {
        fprintf(stderr, "insufficent memory, exit\n");
        exit(-2);
    }

    rplidar_response_device_health_t healthinfo;
    rplidar_response_device_info_t devinfo;

    do {
        // try to connect
        if (IS_FAIL(drv->connect(opt_com_path, opt_com_baudrate))) {
            fprintf(stderr, "Error, cannot bind to the specified serial port %s.\n"
                , opt_com_path);
            break;
        }

        // retrieving the device info
        ////////////////////////////////////////
        op_result = drv->getDeviceInfo(devinfo);

        if (IS_FAIL(op_result)) {
            if (op_result == RESULT_OPERATION_TIMEOUT) {
                // you can check the detailed failure reason
                fprintf(stderr, "Error, operation time out.\n");
            } else {
                fprintf(stderr, "Error, unexpected error, code: %x\n", op_result);
                // other unexpected result
            }
            break;
        }

        // print out the device serial number, firmware and hardware version number..
        printf("RPLIDAR S/N: ");
        for (int pos = 0; pos < 16 ;++pos) {
            printf("%02X", devinfo.serialnum[pos]);
        }
        printf("\n"
                "Version: " RPLIDAR_SDK_VERSION "\n"
                "Firmware Ver: %d.%02d\n"
                "Hardware Rev: %d\n"
                , devinfo.firmware_version>>8
                , devinfo.firmware_version & 0xFF
                , (int)devinfo.hardware_version);


        // check the device health
        ////////////////////////////////////////
        op_result = drv->getHealth(healthinfo);
        if (IS_OK(op_result)) { // the macro IS_OK is the preperred way to judge whether the operation is succeed.
            printf("RPLidar health status : ");
            switch (healthinfo.status) {
            case RPLIDAR_STATUS_OK:
                printf("OK.");
                break;
            case RPLIDAR_STATUS_WARNING:
                printf("Warning.");
                break;
            case RPLIDAR_STATUS_ERROR:
                printf("Error.");
                break;
            }
            printf(" (errorcode: %d)\n", healthinfo.error_code);

        } else {
            fprintf(stderr, "Error, cannot retrieve the lidar health code: %x\n", op_result);
            break;
        }


        if (healthinfo.status == RPLIDAR_STATUS_ERROR) {
            fprintf(stderr, "Error, rplidar internal error detected. Please reboot the device to retry.\n");
            // enable the following code if you want rplidar to be reboot by software
            // drv->reset();
            break;
        }
        drv->startMotor();

        // take only one 360 deg scan and display the result as a histogram
        ////////////////////////////////////////////////////////////////////////////////
        //
        if (IS_FAIL(drv->startScan( 0,1 ))) // you can force rplidar to perform scan operation regardless whether the motor is rotating
        {
            fprintf(stderr, "Error, cannot start the scan operation.\n");
            exit(-1);
        }
        break;
    } while(1);
}

//
// sizeRate : size rate for image data.
// if you set 1dot = 5cm, please set sizeRate = 50.0 * maxX;
float sizeRate = 10000.0;

// 
// convert from real x-y coordinates to the dot x-y coordinates.
// at the center of the image is (0, 0).
float cvX(float x) { return maxX / 2 + x * maxX / sizeRate; }
float cvY(float y) { return maxY / 2 + y * maxY / sizeRate; }

//
// convert from real x-y to dot x-y as the point type
cv::Point2f cvP(cv::Point2f P) {
    cv::Point2f r;
    r.x = cvX(P.x);
    r.y = cvY(P.y);
    return r;
}

//
// put the square mark at point P on the img.
// P is x-y code; 
void putSquareXY(cv::Mat img, cv::Point2f P, int size, cv::Scalar scalar) {
    cv::Point2f startP, endP, sizeP;
    sizeP.x = size * maxX / sizeRate / 2;
    sizeP.y = size * maxY / sizeRate / 2;

    startP.x = cvX(P.x) - size * maxX / sizeRate / 2;
    startP.y = cvY(P.y) - size * maxY / sizeRate / 2;
    endP.x   = cvX(P.x) + size * maxX / sizeRate / 2;
    endP.y   = cvY(P.y) + size * maxY / sizeRate / 2;

    cv::rectangle(img, startP, endP, scalar);
}

//
// put the square mark at P (P is pole with R and angle) on img.
// R is the radius, and angle is the degree.
void putSquareR(cv::Mat img, float angle, float r, int size, cv::Scalar scalar) {
    cv::Point2f circleP;
    circleP.x = cos(angle / 180.0 * PI) * r;
    circleP.y = sin(angle / 180.0 * PI) * r;
    putSquareXY(img, circleP, size, scalar);
}

//
// draw the line between P1 and P2 on the img.
void putLineXY(cv::Mat img, cv::Point2f P1, cv::Point2f P2, int size, cv::Scalar scalar) {
    cv::line(img, cvP(P1), cvP(P2), scalar, size,CV_AA);
}

// draw the line using pole coordinates.
void putLineR(cv::Mat img, float angle1, float r1, float angle2, float r2, int size, cv::Scalar scalar) {
    cv::Point2f p1, p2;
    p1.x = cos(angle1 / 180.0 * PI) * r1;
    p1.y = sin(angle1 / 180.0 * PI) * r1;
    p2.x = cos(angle2 / 180.0 * PI) * r2;
    p2.y = sin(angle2 / 180.0 * PI) * r2;
    putLineXY(img, p1, p2, size, scalar);
}

//
// draw the circle at point P on the img.
void putCircle(cv::Mat img, cv::Point2f P, int size, cv::Scalar scalar) {
    cv::circle(img, cvP(P), size * maxX / sizeRate, scalar, size * maxX / sizeRate / 5, CV_AA);
}

// get the points data from LRF.
void getAllData(cv::Mat img, rplidar_response_measurement_node_t nodes[], size_t count){
    // draw the distance information
    float r, angle;
    cv::Point2f p;
    for (int pos = 0; pos < (int)count; pos++) {
        angle = degPos(nodes[pos]);
        r = (nodes[pos].distance_q2/4.0f);
        p.x = cos(angle) * r;
        p.y = sin(angle) * r;
        putLineXY(img, p, cv::Point2f(0, 0), 10, cv::Scalar(0, 128, 0));
        putSquareXY(img, p, 50, cv::Scalar(0, 255, 0));
        // printf("%d/%d, r: %f, angle %f (%f, %f )\n", pos, (int)count, r, angle, x, y);
    }
}

float getLRFData(cv::Mat img, float robotX, float robotY, float robotPhi, float minDeg, float maxDeg, rplidar_response_measurement_node_t nodes[], size_t count) {
    float angle, r, distance;
    int pointNo = 0;
    bool rotFlag = (minDeg < maxDeg);

    distance = FLT_MAX;
printf("minDeg: %f, maxDeg: %f \n", minDeg, maxDeg);
    cv::Point2f p;
    p.x = cos((minDeg - robotPhi)/ 180.0 * PI) * 250.0 + 0.0;
    p.y = sin((minDeg - robotPhi)/ 180.0 * PI) * 250.0;
    putSquareXY(img, p, 100, cv::Scalar(0, 0, 255));
    p.x = cos((maxDeg - robotPhi) / 180.0 * PI) * 250.0 + 0.0;
    p.y = sin((maxDeg - robotPhi) / 180.0 * PI) * 250.0;
    putSquareXY(img, p, 100, cv::Scalar(64, 64, 255));
    putCircle(img, cv::Point2f(0, 0), 100, cv::Scalar(128, 128, 255));
    for (int pos = 0; pos < (int)count; pos++) {
        angle = deg180(degPos(nodes[pos]) - robotPhi); // deg180(((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f) + offsetAngle);
        if ((rotFlag && (minDeg <= angle && angle <= maxDeg)) ||  
            (!rotFlag && (angle <= maxDeg || minDeg <= angle))) {
            r = (nodes[pos].distance_q2/4.0f);
            if (r != 0 && r < distance) distance = r;
            // putSquareR(img, angle, r, 50, cv::Scalar(0, 255, 0));
            cv::Point2f circleP;
            circleP.x = cos(angle / 180.0 * PI) * r + robotX;
            circleP.y = sin(angle / 180.0 * PI) * r + robotY;
            putSquareXY(img, circleP, 50, cv::Scalar(0, 255, 0));
            if (circleP.x !=0 && circleP.y !=0 ) {
                pointData[pointNo].x = circleP.x;
                pointData[pointNo++].y = circleP.y;
            }
// printf("minDeg %f, angle %f, maxDeg %f, r: %f\n", minDeg, angle, maxDeg, r);
        }
    }
    pointData[pointNo].x = 0;
    pointData[pointNo].y = 0;
    // view3Send[2] = (int)distance;
    return (float)distance;
}

float getLRFData(cv::Mat img, float minDeg, float maxDeg, rplidar_response_measurement_node_t nodes[], size_t count) {
    return getLRFData(img, 0, 0, 0, minDeg, maxDeg, nodes, count);
}

double lineRad(cv::Point2f line) {
	double d = line.x * line.x + line.y * line.y;
	double result;
	if (d == 0) return 0;
	result = acosf(line.x / sqrtf(d));
	if (result == NAN) {
		result = line.x > 0 ? 0 : PI;
		printf("NAN detected PI = %f (%f, %f)\n", result, line.x, line.y);
	}
	if (line.y < 0) result = PI * 2.0 - result;
	return result;
}	

double lineRad(areaRectangle line) {
    cv::Point2f lineRadRet;
    lineRadRet.x = line.max.x - line.min.x;
    lineRadRet.y = line.max.y - line.min.y;
    return lineRad(lineRadRet);
}


bool checkInclude(double min1, double max1, double min2, double max2) {
    double tmp;
    if (min1 > max1) { tmp = min1; min1 = max1; max1 = tmp; }
    if (min2 > max2) { tmp = min2; min2 = max2; max2 = tmp; }
    if (min1 <= min2 && min2 <= max1) return true;
    if (min1 <= max2 && max2 <= max1) return true;
    if (min2 <= min1 && min1 <= max2) return true;
    if (min2 <= max1 && max1 <= max2) return true;
    return false;
}

bool checkSameLine(areaRectangle line1, areaRectangle line2) {
    double radDiff = 5.0 / 180.0 * PI;
    // return (fabs(lineRad(line1) - lineRad(line2)) <= radDiff) ;
    return ((fabs(lineRad(line1) - lineRad(line2)) <= radDiff) &&
        (checkInclude(line1.min.x, line1.max.x, line2.min.x, line2.max.x)) &&
        (checkInclude(line1.min.y, line1.max.y, line2.min.y, line2.max.y)));
}


void recognizeLine(cv::Mat img, float minDeg, float maxDeg, rplidar_response_measurement_node_t nodes[], size_t count) {
    float x, y, oldX = 0, oldY = 0;
    float leftAngle, leftR, rightAngle, rightR = 0; // , leftX, leftY;
    cv::Point2f leftP, rightP, oldP, old2P, secondP;
    bool findFlag;
    int lineColor = 0;
    const int threshold = 5;
    int counter = 0;
    bool rotFlag = (minDeg < maxDeg);

    for (int pos = 0; pos < (int)count; pos++) {
        // at first, find the left edge that has some distance.
        leftAngle = degPos(nodes[pos]); // deg180(((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f) + offsetAngle);
        // if (minDeg <= leftAngle && leftAngle <= maxDeg) {
        if ((rotFlag && (minDeg <= leftAngle && leftAngle <= maxDeg)) ||
            (!rotFlag && (leftAngle <= maxDeg || minDeg <= leftAngle))) {

            leftR = (nodes[pos].distance_q2/4.0f);
            findFlag = false;
            if (leftR > 0) { // find the left edge
                leftP.x = cos(leftAngle / 180.0 * PI) * leftR;
                leftP.y = sin(leftAngle / 180.0 * PI) * leftR;
                oldP = old2P = secondP = leftP;
                // 
                // find the right edge
                areaRectangle leftLine, rightLine, secondLine, lastLine;
                leftLine.min.x = rightLine.min.x = leftP.x;
                leftLine.min.y = rightLine.min.y = leftP.y;
                secondLine = leftLine;
                int rightPos;
                counter = 0;
                for (rightPos = pos; rightPos < (int)count; rightPos++) {
                    rightAngle = degPos(nodes[rightPos]);
                    if (maxDeg < rightAngle) break;
                    rightR = (nodes[rightPos].distance_q2/4.0f);
                    if (rightR > 0) {
                        rightP.x = cos(rightAngle / 180.0 * PI) * rightR;
                        rightP.y = sin(rightAngle / 180.0 * PI) * rightR;
                        if (oldP != leftP) {
                            double leftRad, rightRad;
                            leftLine.max = oldP;
                            rightLine.max = rightP;
                            // line1: left-oldP, line2: left-rightP
                            // check the same direction for line1 and line2.
                            if (!(checkSameLine(leftLine, rightLine))) break;
                            lastLine = rightLine;
                        } else {
                            secondP = rightP;
                            secondLine.max = rightP;
                        }
                        old2P = oldP;
                        oldP = rightP;
                        counter++;
                    } else {
                        // break;
                        // skip
                    }
                }
                rightPos--;
                
                if (rightR > 0 && counter > threshold && 
                    leftAngle < maxDeg &&
                    checkSameLine(secondLine, lastLine)) {
                    findFlag = true;
                    pos = rightPos + 1;
                }
                if (findFlag) {
                    rightP = oldP;
                    putLineXY(img, leftP, rightP, 3, colorData[lineColor]);
                    if (++lineColor > 8) lineColor = 0;
                    findFlag = false;
                }
            }
        }
    }
    printf("count: %d\n", (int)count);
}

int main(int argc, const char * argv[]) {
    rplidar_response_measurement_node_t nodes[8192];
    size_t   count = _countof(nodes);
    int i;

    unsigned char checkSum;

    init_udp(argc, argv);
    init_rplidar(argc, argv);

    //
    // init for opencv
    //
    cv::Mat img  = cv::Mat::zeros(maxX, maxY, CV_8UC3);
    cv::Mat img2 = cv::Mat::zeros(maxX, maxY, CV_8UC3);
    cv::Vec3b pointColor;

    // cv::line(img, cv::Point(maxX, 300), cv::Point(400, 300), cv::Scalar(255,0,0), 10, CV_AA);
    cv::imshow("Sample", img);
    cv::waitKey(1);

    for(int i = 0; i < maxPointData; i++) {
        pointData[i].x = 0;
        pointData[i].y = 0;
        pointHistory[i].p.x = 0;
        pointHistory[i].p.y = 0;
        pointHistory[i].ttl = 0;
    }

    do {

        //
        // program for view3.
        //
        if (recv(sockRecv, buf, sizeof(buf), MSG_WAITALL) > 0) {
            if (buf[0] == 0) { // we can receive only Message 0.
                for (i = 1; i < 9; i++) {
                    view3Recv[i] = (buf[i * 4 + 3] << 24) + (buf[i * 4 + 2] << 16) + (buf[i * 4 + 1] << 8) + (buf[i * 4]);
                    printf("INT%d = %x, ", i, view3Recv[i]);
                }
                printf("\n");
                view3Send[1] = view3Recv[1] + 1;        // seq number
            }
       

            //
            // get the distance data from LRF.
            //

            int result = 0;
	        count = _countof(nodes);
	        count = 360 * 2;

	        if (IS_FAIL(rplidar_capture(drv, nodes, count))) {
                fprintf(stderr, "Error, cannot grab scan data.\n");
                break;
            }
            img = cv::Mat::zeros(maxX, maxY, CV_8UC3);
            cv::Point2f robot;
            robot.x = -cos((laserPhi + offsetAngle)/ 180.0 * PI) * laserR;
            robot.y = -sin((laserPhi + offsetAngle)/ 180.0 * PI) * laserR;
            putCircle(img, robot, 250, cv::Scalar(128, 128, 128));
            sizeRate = 10000.0;

            switch (view3Recv[2]) {
                case 1:
                    result = nodes[(int)count / 2].distance_q2/4.0f;
                    break;
                case 2:
                    // getAllData(img, nodes, count);
                    getLRFData(img, -180, 180, nodes, count);
		            break;
                case 3:
                    // the same method for HOKUYO function block
                    // Input
                    //   INT2 ... Angle
                    //   INT3 ... Opening angle
                    // Output(return)
                    //   INT1 ... Distance
                    float angle, openingAngle, minDeg, maxDeg;
                    openingAngle = view3Recv[4];
                    angle = deg180(view3Recv[3]);
                    minDeg = deg180(angle - openingAngle / 2.0);
                    maxDeg = deg180(angle + openingAngle / 2.0);
                    result = (int)getLRFData(img, minDeg, maxDeg, nodes, count);
                    break;
                case 4:
                    sizeRate = 5000.0;
                    // float angle, openingAngle, minDeg, maxDeg;
                    openingAngle = view3Recv[4];
                    angle = deg180(view3Recv[3]);
                    minDeg = deg180(angle - openingAngle / 2.0);
                    maxDeg = deg180(angle + openingAngle / 2.0);
                    result = (int)getLRFData(img, minDeg, maxDeg, nodes, count);
                    recognizeLine(img, minDeg, maxDeg, nodes, count); 
                    break;
                case 5: // make the map
                    float robotX, robotY, robotPhi;
                    int pointNo;
                    sizeRate = 50.0 * maxX;
                    openingAngle = view3Recv[4];
                    angle = deg180(view3Recv[3]);
                    robotX   = view3Recv[5];
                    robotY   = view3Recv[6];
                    robotPhi = view3Recv[7];
                    minDeg = deg180(angle - openingAngle / 2.0);
                    maxDeg = deg180(angle + openingAngle / 2.0);
                    result = (int)getLRFData(img, robotX, robotY, robotPhi, minDeg, maxDeg, nodes, count);
                    for (int x = 0; x < maxX; x++) {
                        for(int y = 0; y < maxY; y++) {
                            pointColor = img2.at<cv::Vec3b>(x, y);
                            for(int i = 0; i < 3; i++)
                                if (pointColor[i] > 0) pointColor[i]--;
                            img2.at<cv::Vec3b>(x, y) = pointColor;
                        }
                    }
                    for (int i = 0; i < maxPointData; i++) {
                        if (pointData[i].x == 0 && pointData[i].y == 0) break;
                        putSquareXY(img2, pointData[i], sizeRate / maxX, cv::Scalar(255, 255, 255));
                    }    
                    img2.copyTo(img);
                    break; 
	        	case 0xffffffff:
                    drv->stop();
                    drv->stopMotor();

                    RPlidarDriver::DisposeDriver(drv);
                    return 0;

                default:
                    break;
	        }

            cv::imshow("Sample", img);
            cv::waitKey(1);

            if (result != 0) view3Send[2] = result * 1;
            printf("dist = %d\n", result);
            buf[0] = 0;
            buf[1] = 36;
            buf[2] = 0;
            buf[3] = 0; // checkSum;
            for (i = 1; i < 9; i++) {
                buf[i * 4    ] =  view3Send[i]        & 0xff;
                buf[i * 4 + 1] = (view3Send[i] >>  8) & 0xff;
                buf[i * 4 + 2] = (view3Send[i] >> 16) & 0xff;
                buf[i * 4 + 3] = (view3Send[i] >> 24) & 0xff;
            }
            checkSum = 0;
            for (i = 0; i < 36; i++) checkSum += buf[i];
            buf[3] = 0xff - checkSum;
            // printf("checkSum = %d\n", checkSum);
            sendto(sockSend, buf, 36, 0, (struct sockaddr *)&addrSend, sizeof(addrSend));

            // printf("sendto finished\n");
        // }
        // usleep(150000);
        }
    } while(1);

    drv->stop();
    drv->stopMotor();

    RPlidarDriver::DisposeDriver(drv);
    return 0;
}
