#include "urg_sensor.h"
#include "urg_utils.h"
#include "open_urg_sensor.h"

#include <stdlib.h>
#include <stdio.h>

#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <unistd.h>

int view3Send[9], view3Recv[9];
const char *device = "/dev/ttyACM0";
long baudrate_or_port = 115200;
urg_connection_type_t connection_type = URG_SERIAL;

static void print_data(urg_t *urg, long data[], int data_n, long time_stamp)
{
#if 1
    int front_index;

    (void)data_n;

    // \~japanese 前方のデータのみを表示
    // \~english Shows only the front step
    front_index = urg_step2index(urg, 0);
    printf("%ld [mm], (%ld [msec])\n", data[front_index], time_stamp);

#else
    (void)time_stamp;

    int i;
    long min_distance;
    long max_distance;

    // \~japanese 全てのデータの X-Y の位置を表示
    // \~english Prints the X-Y coordinates for all the measurement points
    urg_distance_min_max(urg, &min_distance, &max_distance);
    for (i = 0; i < data_n; ++i) {
	long l = data[i];
	double radian;
	long x;
	long y;

	if ((l <= min_distance) || (l >= max_distance)) {
	    continue;
	}
	radian = urg_index2rad(urg, i);
	x = (long)(l * cos(radian));
	y = (long)(l * sin(radian));
	printf("(%ld, %ld), ", x, y);
    }
    printf("\n");
#endif
}


int main(int argc, char *argv[])
{
    //
    // init for URG
    //

    enum {
	CAPTURE_TIMES = -1,
    };
    urg_t urg;
    long *data = NULL;
    long time_stamp;
    int n;
    int i;

    // if (open_urg_sensor(&urg, argv, argv) < 0) {
    //     return 1;
    // }
    if (urg_open(&urg, connection_type, device, baudrate_or_port) < 0) {
	printf("urg_open error\n");
	return 1;
    }

    data = (long *)malloc(urg_max_data_size(&urg) * sizeof(data[0]));
    if (!data) {
	perror("urg_max_index()");
	return 1;
    }

    // urg_start_measurement(&urg, URG_DISTANCE, URG_SCAN_INFINITY, 0);

    // \~japanese ?~C~G?~C??~B??~O~V?~W
    // \~english Gets measurement data
#if 0
    // \~japanese ?~C~G?~C??~B??~A??~O~V?~W?~D?~[??~B~R?~I?~[??~A~Y?~B~K?| ??~P~H
    // \~english Case where the measurement range (start/end steps) is defined
    urg_set_scanning_parameter(&urg,
			       urg_deg2step(&urg, -90),
			       urg_deg2step(&urg, +90), 0);
#endif

    //
    // init for view3
    //
    int sockSend, sockRecv;
    int portSend, portRecv;
    char IPSend[16], IPRecv[16];
    // int i;
    int yes = 1;
    struct sockaddr_in addrSend, addrRecv;

    for(i = 1; i < 9; i++) view3Send[i] = -1;
    unsigned char buf[256];
    int checkSum;

    sockSend = socket(AF_INET, SOCK_DGRAM, 0);

    if (argc != 5) {
        portSend = 9180; portRecv = 9182;
        strcpy(IPSend, "127.0.1.1");
	strcpy(IPRecv, "127.0.1.1");
    } else {
	strcpy(IPSend, argv[1]);
	portSend = atoi(argv[2]);
	strcpy(IPRecv, argv[3]);
	portRecv = atoi(argv[4]);
    }

    printf("UDP connection: send %s:%d, recv %s:%d\n", IPSend, portSend, IPRecv, portRecv);

    addrSend.sin_family = AF_INET;
    addrSend.sin_port = htons(portSend);
    addrSend.sin_addr.s_addr = inet_addr(IPSend);
    // addr.sin_addr.s_addr = inet_addr("255.255.255.255");
    setsockopt(sockSend, SOL_SOCKET, SO_BROADCAST, (char *)&yes, sizeof(yes));

    sockRecv = socket(AF_INET, SOCK_DGRAM, 0);
    addrRecv.sin_family = AF_INET;
    addrRecv.sin_port = htons(portRecv);
    addrRecv.sin_addr.s_addr = inet_addr(IPRecv); // INADDR_ANY;
    bind(sockRecv, (struct sockaddr *)&addrRecv, sizeof(addrRecv));

    memset(buf, 0, sizeof(buf));

    //
    // main loop
    //

    for(;;){

	if (recv(sockRecv, buf, sizeof(buf), MSG_DONTWAIT) > 0) {
	    if (buf[0] == 0) { // we can receive only Message 0.
		for (i = 1; i < 9; i++) {
		    view3Recv[i] = (buf[i * 4 + 3] << 24) + (buf[i * 4 + 2] << 16) + (buf[i * 4 + 1] << 8) + (buf[i * 4]);
		    printf("INT%d = %x, ", i, view3Recv[i]);
		}
		printf("\n");
		view3Send[1] = view3Recv[1] + 1;        // seq number
	    }
	    if (view3Recv[1] == 0xff) { //finish
		printf("finish!\n");
		free(data);
		urg_close(&urg);
		close(sockSend);
		close(sockRecv);
		return 0;
	    }
	}

	urg_start_measurement(&urg, URG_DISTANCE, 1, 0);
	n = urg_get_distance(&urg, data, &time_stamp);
	urg_stop_measurement(&urg);
	if (n <= 0) {
	    printf("urg_get_distance: %s\n", urg_error(&urg));
	    // free(data);
	    // urg_close(&urg);
	    // return 1;
	    continue;
	}
	
	// print_data(&urg, data, n, time_stamp);
	print_data(&urg, data, n, time_stamp);
	view3Send[1] = data[urg_step2index(&urg, 0)];

	// send data
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
	printf("checkSum = %d\n", checkSum);
	sendto(sockSend, buf, 36, 0, (struct sockaddr *)&addrSend, sizeof(addrSend));

	usleep(100000);
    }

    // \~japanese 切断
    // \~english Disconnects
    free(data);
    urg_close(&urg);

#if defined(URG_MSC)
    getchar();
#endif
    return 0;
}
