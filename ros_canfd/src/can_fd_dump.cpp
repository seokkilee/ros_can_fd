#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

int main(int argc, char **argv)
{
	int s, i; 
	int nbytes;
	struct sockaddr_can addr;
	struct ifreq ifr;
	struct canfd_frame frame;
	int enable_canfd = 1;

	printf("CAN Sockets Receive Demo\r\n");

	if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
		perror("Socket");
		return 1;
	}

	strcpy(ifr.ifr_name, "vcan0" );
	ioctl(s, SIOCGIFINDEX, &ifr);

	memset(&addr, 0, sizeof(addr));
	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
		perror("Bind");
		return 1;
	}
	setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd));

	nbytes = read(s, &frame, sizeof(struct canfd_frame));

 	if (nbytes < 0) {
		perror("Read");
		return 1;
	}

	printf("0x%03X [%d] ",frame.can_id, frame.len);

	for (i = 0; i < frame.len; i++)
		printf("%02X ",frame.data[i]);

	if (close(s) < 0) {
		perror("Close");
		return 1;
	}

	return 0;
}
