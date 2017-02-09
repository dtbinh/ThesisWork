#include <stdio.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>
#include <inttypes.h>
#include <wiringPi.h>

#define BUFFER_SIZE 1

int main(int argc, char **argv)
{
	uint8_t buff[BUFFER_SIZE]; // Read and write function
	
	// Pipe array
	int parentToChildFD[2];
	int childToParentFD[2];
	
	// Creating pipes between parent and child process
	if(pipe(parentToChildFD)==-1) printf("pipe error");
	if(pipe(childToParentFD)==-1) printf("pipe error");
	
	printf("From main process ID: %d\n", (int)getpid());
	
	switch(fork())
	{
		case -1:
		printf("Error\n");
		break;
		case 0:
			// close the write end of the parent to child pipe in the child
			if(close(parentToChildFD[1])==-1)
				printf("close error - parent to child write\n");
			
			// close the read end of the child to parent pipe in the child
			if(close(childToParentFD[0])==-1)
				printf("close error - child to parent read\n");
				
			printf("We are in the child prcess and our PID is: %d\n", (int)getpid());
			
			// for loop testing our pipe
			for(int i=0; i<100; i++)
			{
				buff[0] = (uint8_t)i;
				if(write(childToParentFD[1], buff, sizeof(buff)) != sizeof(buff)) printf("write error in child\n");
				if(read(parentToChildFD[0], buff, 1) == -1) printf("read error in child\n");
				printf("In child recieving %d from parent, PID: %d\n", buff[0], (int)getpid());
			}
			
			if(close(childToParentFD[1]) ==-1)
				printf("close error - child to parent write\n");
		
		break;
		default:
			// close the write end of the child to parent pipe in the parent
			if(close(childToParentFD[1])== -1)
				printf("close error - child to parent write\n");
			
			// close the read end of the parent to child pipe in the parent
			if(close(parentToChildFD[0])==-1)
				printf("close error - parent to child read\n");
				
			printf("We are in the parent process and our PID is: %d\n", (int)getpid());
			
			// for loop for testing our pipe
			for(int i=0; i<100; i++)
			{
				buff[0] = (uint8_t)i;
				if(write(parentToChildFD[1], buff, sizeof(buff)) != sizeof(buff)) printf("write error in parent\n");
				if(read(childToParentFD[0], buff, 1) == -1) printf("read error in parent\n");
				printf("In parent recieving %d from child, PID: %d\n", buff[0], (int)getpid());
			}
			if(close(parentToChildFD[1]) ==-1)
				printf("close error - parent to child write\n");
			wait(NULL);
		
	}
	
	return 0;
}

