#ifndef LIDAR_H
#define LIDAR_H

/*!
  \file
  \brief new lidar program
  \author Edson K. Nakamura
*/

#include <iostream>
#include <stdio.h>

#include "Urg_driver.h"
#include "Connection_information.h"
#include "math_utilities.h"

using namespace qrk;
using namespace std;

namespace hokuyo
{
	void 			close(Urg_driver& urg);
	bool 			is_open(Urg_driver& urg); 
	int	       		lidar_init(int argc, const char **argv, Urg_driver& urg);
	vector<long> 	lidar_lecture(Urg_driver& urg);
	void         	print_data(const vector<long>& data, long time_stamp, Urg_driver& urg);  
}

#endif /* LIDAR_H */