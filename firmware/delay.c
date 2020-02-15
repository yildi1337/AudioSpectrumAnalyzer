/*----------------------------------------------------------------------------------
    Description:    Delay functions.
    Date:           05/26/2014
    Author:         Phillip Durdaut
----------------------------------------------------------------------------------*/

#include "main.h"

/*----------------------------------------------------------------------------------
  Public functions
----------------------------------------------------------------------------------*/

void delay_us(u32_t us)
{
	u64_t start_time;
	u64_t time_difference;
	struct timespec gettime_now;

	clock_gettime(CLOCK_REALTIME, &gettime_now);
	start_time = gettime_now.tv_nsec;
	while (1)
	{
		clock_gettime(CLOCK_REALTIME, &gettime_now);
		time_difference = gettime_now.tv_nsec - start_time;
		if (time_difference < 0)
			time_difference += 1000000000;
		if (time_difference > (us * 1000))
			break;
	}
}