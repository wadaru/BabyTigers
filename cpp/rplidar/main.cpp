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
#include <math.h>

#ifndef _countof
#define _countof(_Array) (int)(sizeof(_Array) / sizeof(_Array[0]))
#endif

#include <unistd.h>
static inline void delay(_word_size_t ms){
    while (ms>=1000){
        usleep(1000*1000);
        ms-=1000;
    };
    if (ms!=0)
        usleep(ms*1000);
}

using namespace rp::standalone::rplidar;
int view3Send[9], view3Recv[9];

int sockSend, sockRecv;
int portSend, portRecv;
char IPSend[32], IPRecv[32];
int yes = 1;
struct sockaddr_in addrSend, addrRecv;
unsigned char buf[256];
const int offsetAngleDefault = 180;
int offsetAngle;

const int maxX = 500;
const int maxY = 500;

RPlidarDriver * drv; //  = RPlidarDriver::CreateDriver(DRIVER_TYPE_SERIALPORT);


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
        exit(-1);
    }

    opt_com_path = argv[1];

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

void putPoint(cv::Mat img, int x, int y, int size, cv::Scalar scalar) {

    cv::rectangle(img, cv::Point(maxX / 2 -  size / 2 + x, maxY / 2 - size / 2 + y), cv::Point(maxX / 2 + size / 2 + x, maxY / 2 + size / 2 + y), scalar);
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
    cv::Mat img = cv::Mat::zeros(maxX, maxY, CV_8UC3);
    // cv::line(img, cv::Point(maxX, 300), cv::Point(400, 300), cv::Scalar(255,0,0), 10, CV_AA);
    cv::imshow("Sample", img);
    cv::waitKey(1);

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
	        count = 360;

	        if (IS_FAIL(rplidar_capture(drv, nodes, count))) {
                fprintf(stderr, "Error, cannot grab scan data.\n");
                break;
            }
            switch (view3Recv[2]) {
                case 1:
                    result = nodes[(int)count / 2].distance_q2/4.0f;
                    break;
                case 2:
                    // draw the distance information
            	    float x, y, r, angle;
            	    img = cv::Mat::zeros(maxX, maxY, CV_8UC3);
		            putPoint(img, 0, 0, 20, cv::Scalar(128, 128, 128));
            	    for (int pos = 0; pos < (int)count; pos++) {
                		angle = (deg180((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f) + offsetAngle) / 180.0 * 3.14159;
                        r = (nodes[pos].distance_q2/4.0f);
                        x = cos(angle) * r * maxX / 5000.0;
                        y = sin(angle) * r * maxY / 5000.0;
                        putPoint(img, x, y, 3, cv::Scalar(0, 255, 0));
                        // printf("%d/%d, r: %f, angle %f (%f, %f )\n", pos, (int)count, r, angle, x, y);
                    }
                    cv::imshow("Sample", img);
                    cv::waitKey(1);
		            break;
                case 3:
                    // the same method for HOKUYO function block
                    // Input
                    //   INT2 ... Angle
                    //   INT3 ... Opening angle
                    // Output(return)
                    //   INT1 ... Distance
                    float openingAngle, minDeg, maxDeg, distance; // angle and r are already defined.
                    img = img = cv::Mat::zeros(maxX, maxY, CV_8UC3);
                    putPoint(img, 0, 0, 20, cv::Scalar(128, 128, 128));
                    openingAngle = view3Recv[4];
                    distance = FLT_MAX;
                    angle = deg180(view3Recv[3]);
                    minDeg = deg180(angle - openingAngle / 2.0);
                    maxDeg = deg180(angle + openingAngle / 2.0);
                    for (int pos = 0; pos < (int)count; pos++) {
                        angle = deg180(((nodes[pos].angle_q6_checkbit >> RPLIDAR_RESP_MEASUREMENT_ANGLE_SHIFT)/64.0f) + offsetAngle);
                        if (minDeg <= angle && angle <= maxDeg) {
                            r = (nodes[pos].distance_q2/4.0f);
                            if (r != 0 && r < distance) distance = r;
                            // printf("minDeg %f, maxDeg %f, deg %f, dist: %f\n", minDeg, maxDeg, angle, r);
                            angle = angle / 180.0 * 3.14159;
                            r = (nodes[pos].distance_q2/4.0f);
                            x = cos(angle) * r * maxX / 5000.0;
                            y = sin(angle) * r * maxY / 5000.0;
                            putPoint(img, x, y, 3, cv::Scalar(0, 255, 0));
                        }
                    }
                    // view3Send[2] = (int)distance;
                    result = (int)distance;
                    cv::imshow("Sample", img);
                    cv::waitKey(1);

                    break;
	        	case 0xffffffff:
                    drv->stop();
                    drv->stopMotor();

                    RPlidarDriver::DisposeDriver(drv);
                    return 0;

                default:
                    break;
	        }

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
