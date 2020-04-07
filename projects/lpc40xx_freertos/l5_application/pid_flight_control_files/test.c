#include <cblas.h>
#include <stdio.h>
#include <stdlib.h>
/**#include <sys/dos.h>
#include <sys/time.h>**/
#include "../../l2_utils/matrices.h"

/**
 * gcc test.c /usr/lib/matrices.o -lm
 **/ 
/**void delay(int number_of_seconds) 
{ 
    // Converting time into milli_seconds 
    int milli_seconds = 1000 * number_of_seconds; 
    //printf("seconds, %d, to milliseconds, %d\n", number_of_seconds, milli_seconds);
    // Storing start time 
    clock_t start_time = clock(); 
  
    // looping till required time is not achieved 
    while (clock() < start_time + milli_seconds) 
        ; 
}**/
int main()
{
    
    double disp[3][2] = {
        {10, 12},
        {14, 15},
        {15, 6}
    };
    
	Matrix *m = constructor(3, 2);
    int i, j;
	for(i = 0; i < m->columns; i++){
		for(j = 0; j < m->rows; j++){
			m->numbers[i][j] = disp[i][j];
		}
	}

    // struggling with freeing memory
	for (int i = 0; i < m->rows; i++)
    {
        free(disp[i]);
    }
    free(disp);
	print(m);
    
    /**struct timeval start, stop;
    double secs = 0;

    gettimeofday(&start, NULL);

    // Do stuff  here
    delay(1000);

    gettimeofday(&stop, NULL);
    secs = (stop.tv_sec - start.tv_sec) * 1000.0;      // sec to ms
    secs += (stop.tv_usec - start.tv_usec) / 1000.0;
    printf("time taken %f\n",secs);**/
}