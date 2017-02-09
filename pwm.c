#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <wiringPi.h>
#include <softPwm.h>
#include <errno.h>

#define PWM1_PIN 1
#define PWM2_PIN 2
#define PWM3_PIN 3
#define PWM4_PIN 4

//0, 1, 2, 3, 4, 5, 6, 7

int main(int argc, char **argv)
{
	// Create pwm objects
	int pwm1=softPwmCreate(PWM1_PIN, 0, 1000);
	int pwm2=softPwmCreate(PWM2_PIN, 0 ,1000);
	int pwm3=softPwmCreate(PWM3_PIN, 0 ,1000);
	int pwm4=softPwmCreate(PWM4_PIN, 0 ,1000);
	
	// Check for successful creating
	if (pwm1!=0 || pwm2!=0 || pwm3!=0 || pwm4!=0){
		perror("pwm");
	}

	
	char input[64];
	while(1){
		if (fgets(input, sizeof(input), stdin)){
			int value = atoi(input);
			if (value>=0 && value<=1000){
				softPwmWrite(PWM1_PIN, value);
				softPwmWrite(PWM2_PIN, value);
				softPwmWrite(PWM3_PIN, value);
				softPwmWrite(PWM4_PIN, value);
				printf("PWM set to: %i\n", value);
			}
			else{
				printf("Error. Bad input format. Range: 0-1000\n");
			}
			memset(&input[0], 0, sizeof(input));
		}
	}
	
	return 0;
}

