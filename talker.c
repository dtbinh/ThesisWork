/*
** talker.c -- a datagram "client" demo
*/

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

#define SERVERPORT 4950	// the port users will be connecting to
#define BUFFER_LENGTH 100

char readBuff[BUFFER_LENGTH];
char writeBuff[BUFFER_LENGTH];

int main(int argc, char *argv[])
{
        int fd_socket;
        struct sockaddr_in their_addr;
        //socklen_t fromlen;
        int broadcast=1;
        int numbytes;

        // Socket settings
	their_addr.sin_family = AF_INET;
	their_addr.sin_port = htons(SERVERPORT);
	their_addr.sin_addr.s_addr = htonl(INADDR_BROADCAST);
        memset(their_addr.sin_zero, '\0',sizeof(their_addr.sin_zero));
   
        // Create socket. SOCK_STREAM=TPC. SOCK_DGRAM=UDP. 
	fd_socket = socket(AF_INET, SOCK_DGRAM, 0);
	
	if (fd_socket == -1){
		perror("socket");
		exit(1);
	}
	
	// Activate UDP broadcasting
	if (setsockopt(fd_socket, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast)) == -1){
		perror("setup");
		exit(1);
	}
	
	strncpy(writeBuff,"THIS IS A BROADCAST TEST",BUFFER_LENGTH);
	
	while(1){
	if ((numbytes=sendto(fd_socket, writeBuff, BUFFER_LENGTH, 0, (struct sockaddr*) &their_addr, sizeof(their_addr))) == -1){
	perror("write");
	exit(1);
	}
        
	printf("talker: sent %d bytes. Message: %s\n", numbytes, writeBuff);
	sleep(1);
	}
	close(fd_socket);

	return 0;
}
