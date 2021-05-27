//#include <errno.h>
#include <poll.h>
//#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <time.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

int
main(int argc, char* argv[])
{
	int busfd;
	struct ifreq ifr;
	struct sockaddr_can addr;
	struct pollfd pollfds[2];
	short int rpm = 0;
	short int manfpress = 0;
	short int throttle = 0;
	short int coolpress = 0;
	short int fuelpress = 0;
	short int afr = 0;
	short int iat = 0;
	short int speed = 0;

	busfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (busfd < 0) {
		return EXIT_FAILURE;
	}

	strncpy(ifr.ifr_name, "vcan0", IFNAMSIZ);
	if (ioctl(busfd, SIOCGIFINDEX, &ifr) < 0) {
		return EXIT_FAILURE;
	}

	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

	if (bind(busfd, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
		return EXIT_FAILURE;
	}

	if (setsockopt(busfd, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0) == -1) {
		return EXIT_FAILURE;
	}

	pollfds[0].fd = busfd;
	pollfds[0].events = POLLOUT;

	srand(time(0));

	while (1) {
		if (poll(pollfds, 1, 100) < 0) {
			return EXIT_FAILURE;
		}
		else if (pollfds[0].revents & (POLLERR|POLLHUP|POLLNVAL)) {
			return EXIT_FAILURE;
		}
		else if (pollfds[0].revents & POLLOUT) {
			struct iovec iov;
			struct msghdr msg;
			struct can_frame frame;
			char ctrlmsg[CMSG_SPACE(sizeof(struct timeval) + 3*sizeof(struct timespec) + sizeof(__u32))];
			struct cmsghdr* cmsg;

			iov.iov_base = &frame;
			iov.iov_len = sizeof(frame);
			msg.msg_name = &addr;
			msg.msg_namelen = sizeof(addr);
			msg.msg_iov = &iov;
			msg.msg_iovlen = 1;
			msg.msg_control = &ctrlmsg;
			msg.msg_controllen = sizeof(ctrlmsg);
			msg.msg_flags = 0;
			frame.can_id = 0x360;
			frame.can_dlc = 8;
			rpm = (rpm+10) % 4000;
			throttle = (throttle + 10) % 1000;
			manfpress = rand()%100 + 500;
			coolpress = rand()%66 + 99;
			frame.data[0] = rpm & 0x255;
			frame.data[1] = rpm >> 8;
			frame.data[2] = manfpress & 0x255;
			frame.data[3] = manfpress >> 8;
			frame.data[4] = throttle & 0x255;
			frame.data[5] = throttle >> 8;
			frame.data[6] = coolpress & 0x255;
			frame.data[7] = coolpress >> 8;

			if (sendmsg(busfd, &msg, 0) < 0) {
				return EXIT_FAILURE;
			}

			frame.can_id = 0x370;
			speed = (speed + 15) % 2000;
			frame.data[0] = speed & 0x255;
			frame.data[1] = speed >> 8;

			if (sendmsg(busfd, &msg, 0) < 0) {
				return EXIT_FAILURE;
			}

			frame.can_id = 0x361;
			fuelpress = rand()%600 + 4200;
			frame.data[0] = fuelpress & 0x255;
			frame.data[1] = fuelpress >> 8;

			if (sendmsg(busfd, &msg, 0) < 0) {
				return EXIT_FAILURE;
			}

			frame.can_id = 0x368;
			afr = rand()%2000 + 9999;
			frame.data[0] = afr & 0x255;
			frame.data[1] = afr >> 8;

			if (sendmsg(busfd, &msg, 0) < 0) {
				return EXIT_FAILURE;
			}

			frame.can_id = 0x3E0;
			iat = rand()%20 + 250;
			frame.data[2] = iat & 0x255;
			frame.data[3] = iat >> 8;

			if (sendmsg(busfd, &msg, 0) < 0) {
				return EXIT_FAILURE;
			}

			usleep(100000);
		}
	}

	return EXIT_SUCCESS;
}

