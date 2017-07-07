#define _POSIX_C_SOURCE 199309L
#define _GNU_SOURCE
//#define DUMP(Var) (fprintf(stderr, #Var "= %p", Var))
//#define DUMP(varname, format) (fprintf(stderr, "%s=" format "\n")
#define GET_VARIABLE_NAME(Variable) (#Variable)
 
 
#include <stdlib.h>
#include <stdio.h>
#include <time.h>
#include <sched.h>
#include <sys/mman.h>
#include <string.h>

#define MY_PRIORITY (49) /* we use 49 as the PRREMPT_RT use 50
                            as the priority of kernel tasklets
                            and interrupt handler by default */
#define MAX_SAFE_STACK (8*1024) /* The maximum stack size which is
                                   guaranteed safe to access without
                                   faulting */
#define NSEC_PER_SEC 1E9 /* The number of nsecs per sec. */

void stack_prefault(void) {
        unsigned char dummy[MAX_SAFE_STACK];
        memset(dummy, 0, MAX_SAFE_STACK);
        return;
}

// Load settings file
int loadSettings(double *data, char* name, int size){
	// Create file pointer
	FILE *fp;
	
	float value;
	char string[20];
	int finish=0;
	//int lines=0;
	int i;
	
	
	// Open file and prepare for read
	fp=fopen("settings.txt", "r");
	// Check to see that file has opened
	if(fp==NULL){
		printf("File could not be opened for read\n");
	}
	else{
		while((fgets(string,20,fp)) != NULL && !finish){
			// Search for variable
			if(strstr(string,name) != NULL){
				//printf("%s\n",name);
				// Get variable data
				for(i=0;i<size;i++){
					if((fgets(string,20,fp)) != NULL && string[0]!='\n'){
						sscanf(string, "%f\n", &value);
						data[i]=(double)value;
						//printf("%f\n",value);
					}
					else{
						printf("Bad format or missing data on reading settings file\n");
					}
				}
				finish=1;
				printf("%s settings loaded\n",name);
			}
		}
		if(!finish){
			printf("%s not loaded (does not exist in settings.txt)\n",name);
		}
	}
	
	// Close file
	fclose(fp);
	return finish;
}

// Save settings file
void saveSettings(double *data, char* name, int size){
	// Create file pointer
	FILE *fpRead, *fpWrite;
	int i;
	char string[20];
	int finish=0;
	
	// Open file and prepare for write
	fpRead=fopen("settings.txt", "r");

	// Check to see that file has opened
	if(fpRead==NULL){
		printf("File could not be opened for write\n");
	}
	else{
		// check that variable does not exist in settings file
		while((fgets(string,20,fpRead)) != NULL && !finish){
			
			if(strstr(string,name) != NULL){
				finish=1; // found variable
			}
		}
	}
	
	// Close file
	fclose(fpRead);
	
	// Open file and prepare for write
	fpWrite=fopen("settings.txt", "a"); // "a" means append
	if(fpWrite==NULL){
		printf("File could not be opened for write\n");
	}
	else{
		if(!finish){	// if variable does not exist
			fprintf(fpWrite, "%s\n", name); // append variable name
			for(i=0;i<size;i++){
				fprintf(fpWrite, "%f\n", data[i]); // append content
			}
			fprintf(fpWrite, "\n"); // newline
			printf("%s settings saved\n",name);
		}
		else{	// if variable does exist
			printf("%s not saved (lready exists in settings.txt)\n", name);
		}
	}
	
	// Close file
	fclose(fpWrite);
}

// Matrix print function
void printmat(double *A, int m, int n){
    double *dptr;
    int j, i;
    dptr = A;
    for (j = 0; j < m; j++)
    {
        for (i = 0; i < n; i++)
        {
            printf("%6.4f\t", *(dptr+m*i+j));
        }
        printf("\n");
    }
    //printf("\n");
    return;
}

int main(int argc, char* argv[])
{
        struct timespec t, t_start, t_stop;
        struct sched_param param;
        //int interval = 50000000; /* 0.05s*/
        double accum;
        int interval_s = 5;

        /* Declare ourself as a real time task */

        param.sched_priority = MY_PRIORITY;
        if(sched_setscheduler(0, SCHED_FIFO, &param) == -1) {
                perror("sched_setscheduler failed");
                exit(-1);
        }
        
				//double Racc[9]={1,0,0,0,1,0,0,0,1};
			//double Rmag[12]={1,0,0,0,1,0,0,0,1,9,8,7};
			//double Rgyr[15]={1,0,0,0,1,0,0,0,1,4,7,8,9,2,4};
			//double Rekf[18]={1,0,0,0,1,0,0,0,1,1,0,0,0,1,0,0,0,1};
			double Racc[9]={0,0,0,0,0,0,0,0,0};
			double Rmag[12]={0,0,0,0,0,0,0,0,0,0,0,0};
			double Rgyr[15]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
			double Rekf[18]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
        

        /* Lock memory */
        if(mlockall(MCL_CURRENT) == -1) {
                perror("mlockall failed");
                exit(-2);
        }

        /* Pre-fault our stack */
        stack_prefault();

        clock_gettime(CLOCK_MONOTONIC ,&t);
		
		
        /* start after sampling time */
        t.tv_sec++;

        while(1) {
			// Time it
			clock_gettime(CLOCK_MONOTONIC, &t_start);
			
			/* wait until next shot */			
			clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &t, NULL);
			

			/* do the stuff */

			
			printmat(Racc,1,9);
			printmat(Rmag,1,12);
			printmat(Rgyr,1,15);
			printmat(Rekf,1,18);
			
			
			// try to load settings
			loadSettings(Racc,"Racc",sizeof(Racc)/sizeof(double));
			loadSettings(Rmag,"Rmag",sizeof(Rmag)/sizeof(double));
			loadSettings(Rgyr,"Rgyr",sizeof(Rgyr)/sizeof(double));
			loadSettings(Rekf,"Rekf",sizeof(Rekf)/sizeof(double));
			
			// try to save settings
			saveSettings(Racc,"Racc",sizeof(Racc)/sizeof(double));
			saveSettings(Rmag,"Rmag",sizeof(Rmag)/sizeof(double));
			saveSettings(Rgyr,"Rgyr",sizeof(Rgyr)/sizeof(double));
			saveSettings(Rekf,"Rekf",sizeof(Rekf)/sizeof(double));
			

			//saveSettings(Racc,"Racc",sizeof(Racc)/sizeof(double));
			//printf("Size of Racc: %i\n", sizeof(Racc)/sizeof(double));
			
			/* calculate next shot */
			//t.tv_nsec += interval;
			t.tv_sec += interval_s;

			while (t.tv_nsec >= NSEC_PER_SEC) {
				   t.tv_nsec -= NSEC_PER_SEC;
					t.tv_sec++;
			}
			
			clock_gettime(CLOCK_MONOTONIC, &t_stop);
			accum=(t_stop.tv_sec - t_start.tv_sec) + (t_stop.tv_nsec - t_start.tv_nsec) / NSEC_PER_SEC;
			printf("%lf\n",accum);
	}
}
