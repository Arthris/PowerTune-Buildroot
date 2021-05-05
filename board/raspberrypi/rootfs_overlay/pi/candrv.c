#include <arpa/inet.h>
#include <errno.h>
#include <poll.h>
#include <pthread.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/eventfd.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

#include <ctype.h>
#include <fcntl.h>
#include <sys/time.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <linux/net_tstamp.h>

static const canid_t haltechv2[] = {
	0x360,
	0x361,
	0x362,
	0x363,
	0x368,
	0x369,
	0x36A,
	0x36B,
	0x36C,
	0x36D,
	0x36E,
	0x36F,
	0x370,
	0x371,
	0x372,
	0x373,
	0x374,
	0x375,
	0x3E0,
	0x3E1,
	0x3E2,
	0x3E3,
	0x3E4,
	0x470
};

static const struct {
	const char* chan;
	enum {S16, BIT} type;
	canid_t id;
	int b1;
	int b2;
	double scale;
	double offset;
	const char* unit;
	int ptid;
} map[] = {
        {"RPM",                   S16, 0x360, 0, 1, 1.0,   0.0,    "RPM",  179},
        {"Manifold Pressure",     S16, 0x360, 2, 3, 0.1,   0.0,    "kPA",  155},
        {"Throttle Position",     S16, 0x360, 4, 5, 0.1,   0.0,    "%",    202},
	{"Battery",		  S16, 0x372, 0, 1, 0.1,   0.0,    "V",    228},
        {"Coolant Pressure",      S16, 0x360, 6, 7, 0.1,   101.3,  "kPA",  29},
        {"Fuel Pressure",         S16, 0x361, 0, 1, 0.1,   101.3,  "kPA",  100},
        {"Oil Pressure",          S16, 0x361, 2, 3, 0.1,   101.3,  "kPA",  169},
        {"STG 1 DUTY",            S16, 0x362, 0, 1, 0.1,   0.0,    "%",    133},
        {"STG 2 DUTY",            S16, 0x362, 2, 3, 0.1,   0.0,    "%",    275},
        {"Target Boost",          S16, 0x372, 4, 5, 0.1,   0.0,    "%",     22},
        {"Lambda 1",              S16, 0x368, 0, 1, 0.001, 0.0,    "λ",      6},
        {"Fuel Compostion",       S16, 0x3E1, 4, 5, 0.1,   0.0,    "%",     84},
        {"Vehicle Speed",         S16, 0x370, 0, 1, 0.1,   0.0,    "km/h", 199},
        {"Coolant Temperature",   S16, 0x3E0, 0, 1, 0.1,   273.15, "C",    221},
        {"Air Temperature",       S16, 0x3E0, 2, 3, 0.1,   273.15, "C",    135},
        {"MIL",                   BIT, 0x3E4, 7, 0,   1,   0.0,    NULL,   157},
        {"Fuel Temperature",      S16, 0x3E0, 4, 5, 0.1,   273.15, "C",    101},
        {"Knock Level",           S16, 0x36A, 0, 1, .01,   0.0,    "dB",   137},
        {"GEAR POS",              S16, 0x470, 2, 3,   1,   0.0,    NULL,   106},

};

#define NHALTECHPKTS (sizeof(haltechv2)/sizeof(haltechv2[0]))
#define MAXPKTLEN 8
#define NMAPENTRIES (sizeof(map)/sizeof(map[0]))

static int stopev = -1;
static pthread_t thread;
static pthread_mutex_t mtx = PTHREAD_MUTEX_INITIALIZER;
static unsigned char procimg[NHALTECHPKTS*MAXPKTLEN] = {0};

static int
canidcmp(const void* id1, const void* id2)
{
	return 
		((*(canid_t*)id1) < (*(canid_t*)id2))? -1 :
		((*(canid_t*)id1) > (*(canid_t*)id2))? 1 :
		/*else*/0;
}

static void 
canrx_fail(const char* s, ...)
{
	va_list a;
	va_start(a, s);
	vfprintf(stderr, s, a);
	va_end(a);
	pthread_exit(NULL);
}

static void 
canrx_finalize(void* arg)
{
	eventfd_t u = 1;
	write(stopev, &u, sizeof(u));
}

static void*
canrx(void* arg)
{
	int busfd;
	struct ifreq ifr;
	struct sockaddr_can addr;
	const int tsflags = 
		SOF_TIMESTAMPING_RX_HARDWARE | SOF_TIMESTAMPING_TX_HARDWARE | SOF_TIMESTAMPING_RAW_HARDWARE | 
		SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_TX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE;
	struct pollfd pollfds[2];

	pthread_cleanup_push(canrx_finalize, NULL);

	busfd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
	if (busfd < 0) {
		canrx_fail("socket(PF_CAN) returned error: %s\n", strerror(errno));
	}

	strncpy(ifr.ifr_name, "can0", IFNAMSIZ);
	if (ioctl(busfd, SIOCGIFINDEX, &ifr) < 0) {
		canrx_fail("ioctl(SIOCGIFINDEX, %s) returned error: %s\n", ifr.ifr_name, strerror(errno));
	}

	addr.can_family = AF_CAN;
	addr.can_ifindex = ifr.ifr_ifindex;

#if(0)
	if (setsockopt(busfd, SOL_SOCKET, SO_TIMESTAMPING, &tsflags, sizeof(tsflags)) == -1) {
		canrx_fail("setsockopt(%s, SO_TIMESTAMPING) returned error: %s\n", ifr.ifr_name, strerror(errno));
	}
#endif

	if (bind(busfd, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
		canrx_fail("bind(%s) returned error: %s\n", ifr.ifr_name, strerror(errno));
	}

#if(0)
	if (setsockopt(busfd, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0) == -1) {
		canrx_fail("setsockopt(%s, CAN_RAW_FILTER) returned error: %s\n", ifr.ifr_name, strerror(errno));
	}
#endif

	pollfds[0].fd = busfd;
	pollfds[0].events = POLLIN;
	pollfds[1].fd = stopev;
	pollfds[1].events = POLLIN;

	while (1) {
		if (poll(pollfds, 2, 10000) < 0) {
			canrx_fail("canrx poll returned error: %s\n", strerror(errno));
		}
		else if (pollfds[1].revents & POLLIN) {
			break;
		}
		else if (pollfds[1].revents & (POLLERR|POLLHUP|POLLNVAL)) {
			canrx_fail("canrx stopev status indicates: %hXh\n", pollfds[1].revents);
		}
		else if (pollfds[0].revents & (POLLERR|POLLHUP|POLLNVAL)) {
			canrx_fail("canrx %s status indicates: %hXh\n", ifr.ifr_name, pollfds[0].revents);
		}
		else if (pollfds[0].revents & POLLIN) {
			struct iovec iov;
			struct msghdr msg;
			struct can_frame frame;
			char ctrlmsg[CMSG_SPACE(sizeof(struct timeval) + 3*sizeof(struct timespec) + sizeof(__u32))];
			struct cmsghdr* cmsg;
			canid_t can_id;
			canid_t* pkt;

			iov.iov_base = &frame;
			iov.iov_len = sizeof(frame);
			msg.msg_name = &addr;
			msg.msg_namelen = sizeof(addr);
			msg.msg_iov = &iov;
			msg.msg_iovlen = 1;
			msg.msg_control = &ctrlmsg;
			msg.msg_controllen = sizeof(ctrlmsg);
			msg.msg_flags = 0;

			if (recvmsg(busfd, &msg, 0) < 0) {
				canrx_fail("recvmsg(%s) returned error: %hd\n", ifr.ifr_name, strerror(errno));
			}

			if (frame.can_dlc > MAXPKTLEN) continue;

			can_id = frame.can_id & CAN_ERR_MASK;
			pkt = bsearch(&can_id, haltechv2, NHALTECHPKTS, sizeof(haltechv2[0]), canidcmp);
			if (NULL == pkt) continue;
			pthread_mutex_lock(&mtx);
			memcpy(&procimg[(pkt - haltechv2)*MAXPKTLEN], frame.data, frame.can_dlc);
			pthread_mutex_unlock(&mtx);

#if(0)
			flags = (frame.can_id & CAN_EFF_FLAG)? MSG_EXT : 0;
			flags |= (frame.can_id & CAN_ERR_FLAG)? MSG_ERR : 0;
			for (cmsg = CMSG_FIRSTHDR(&msg); 
				cmsg && (cmsg->cmsg_level == SOL_SOCKET);
				cmsg = CMSG_NXTHDR(&msg,cmsg)) {
				switch (cmsg->cmsg_type) {
				case SO_TIMESTAMP:
					memcpy(&timestamp, CMSG_DATA(cmsg), sizeof(rx->timestamp));
					flags |= MSG_TIME;
					break;
				case SO_TIMESTAMPING: {
					struct timespec *ts = (struct timespec*)CMSG_DATA(cmsg);
					timestamp.tv_sec = ts[0].tv_sec;
					timestamp.tv_usec = ts[0].tv_nsec/1000;
					flags |= MSG_TIME;
				}	break;
				default:
					break;
				}
			}
#endif
		}
	}

	pthread_cleanup_pop(1);
	return NULL;
}

static void
fail(const char* s, ...)
{
	va_list a;
	va_start(a, s);
	vfprintf(stderr, s, a);
	va_end(a);
	exit(EXIT_FAILURE);
}

static void
finalize(void)
{
	eventfd_t u = 1;
	write(stopev, &u, sizeof(u));
	pthread_join(thread, NULL);
}

int
main(int argc, char* argv[])
{
	struct sockaddr_in ptaddr;
	int ptsock;
	int ret;
	struct pollfd pollfds[2];
	int i = NMAPENTRIES;

	stopev = eventfd(0, EFD_NONBLOCK);
	if (-1 == stopev) {
		fail("eventfd failed: %s\n", strerror(errno));
	}

	ptsock = socket(AF_INET, SOCK_DGRAM, 0);
	if (-1 == ptsock) {
		fail("socket(AF_INET) returned error: %s\n", strerror(errno));
	}

	memset(&ptaddr, 0, sizeof(ptaddr));
	ptaddr.sin_family = AF_INET;
	ptaddr.sin_port = htons(45454);
	ptaddr.sin_addr.s_addr = INADDR_ANY;

	pthread_t thread;
	ret = pthread_create(&thread, NULL, &canrx, NULL);
	if (ret != 0) {
		fail("pthread_create returned error: %d\n", ret);
	}

	atexit(finalize);

	pollfds[0].fd = stopev;
	pollfds[0].events = POLLIN;
	pollfds[1].fd = ptsock;
	pollfds[1].events = POLLOUT;

	while (1) {
		if (i < NMAPENTRIES) {
			ret = poll(pollfds, 2, -1);
		}
		else {
			ret = poll(&pollfds[0], 1, 100);
		}

		if (-1 == ret) {
			if (EINTR == errno) {
				break;
			}
			else {
				fail("poll returned error: %s\n", strerror(errno));
			}
		}
		else if (pollfds[0].revents & POLLIN) {
			break;
		}
		else if (pollfds[0].revents & (POLLERR|POLLHUP|POLLNVAL)) {
			fail("stopev status indicates: %hXh\n", pollfds[0].revents);
		}
		else if (!(i < NMAPENTRIES)) {
			i = 0;
printf("Chris is a screwball\n");
		}
		else if (pollfds[1].revents & (POLLERR|POLLHUP|POLLNVAL)) {
			fail("ptsock status indicates: %hXh\n", pollfds[1].revents);
		}
		else if (pollfds[1].revents & POLLOUT) {
			char msg[32];
			canid_t* pkt;
			unsigned char* p;
			unsigned char* p2;
			short int raw;
			double val;

			pkt = bsearch(&map[i].id, haltechv2, NHALTECHPKTS, sizeof(haltechv2[0]), canidcmp);
			if (NULL == pkt) {++i; continue;}

			p = &procimg[(pkt - haltechv2)*MAXPKTLEN] + map[i].b1;

			switch (map[i].type) {
			case S16:
				p2 = &procimg[(pkt - haltechv2)*MAXPKTLEN] + map[i].b2;
				pthread_mutex_lock(&mtx);
				raw = (((short int)*p) << 8 | *p2);
				pthread_mutex_unlock(&mtx);
				val = raw*map[i].scale - map[i].offset;
				snprintf(msg, sizeof(msg), "%d,%.3f\n", map[i].ptid, val);
printf("%s %hd %f\n",map[i].chan,raw,val);
				break;
			case BIT:
				pthread_mutex_lock(&mtx);
				raw = *p;
				pthread_mutex_unlock(&mtx);
				raw &= (1<<map[i].b2);
				raw >>= map[i].b2;
printf("%s %hd\n",map[i].chan,raw);
				raw *= (map[i].scale < 0)? -1 : 1;
				snprintf(msg, sizeof(msg), "%d,%hd\n", map[i].ptid, raw);
				break;
			}

			sendto(ptsock, msg, strlen(msg), MSG_DONTWAIT, (const struct sockaddr*)&ptaddr, sizeof(ptaddr));
			++i;
		}
	}

	return EXIT_SUCCESS;
}

