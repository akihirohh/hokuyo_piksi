#ifndef LIDARREAD_H_
#define LIDARREAD_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <pthread.h>
#include <sys/time.h> //gettimeofday
#include <math.h>
#include <vector>
#include <algorithm>
#include <fstream>
#include "lidar.h"

#define PATH_MAX 100

#define NUM_LIDAR_ELEMENTS 1080

namespace lidarRead
{
	
	typedef struct str_thrdata
	{
		int b_loop;
		std::vector<long> d;	//vector of read distances
		int lidar_timestamp;
		int b_save;
		std::string filename;
		std::string timestamp;
	} thdata;
	
	int millis(timeval);
	void *lidarReading(void *ptr);
}

#endif