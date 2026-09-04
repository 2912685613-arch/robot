#ifndef TCP_CLIENT
#define TCP_CLIENT


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <unistd.h>
#include <arpa/inet.h>

#include <unistd.h>
#include <errno.h>


extern void * tcp_recev(void *sockfd_adress);
extern void tcp_connect();
extern void tcp_disconnect();
extern int tcp_init();

extern int sockfd;

#endif