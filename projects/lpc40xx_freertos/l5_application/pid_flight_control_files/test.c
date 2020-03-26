#include <cblas.h>
#include <stdio.h>
#include <sys/dos.h>
#include <sys/time.h>

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
    struct timeval start, stop;
    double secs = 0;

    gettimeofday(&start, NULL);

    // Do stuff  here
    delay(1000);

    gettimeofday(&stop, NULL);
    secs = (stop.tv_sec - start.tv_sec) * 1000.0;      // sec to ms
    secs += (stop.tv_usec - start.tv_usec) / 1000.0;
    printf("time taken %f\n",secs);
}