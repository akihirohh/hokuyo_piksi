#include <iostream>
#include <stdio.h>

#include "lidar.h"
#include "Urg_driver.h"
#include "Connection_information.h"
#include "math_utilities.h"

using namespace qrk;
using namespace std;

namespace hokuyo
{
	bool is_open(Urg_driver& urg)
	{
		return urg.is_open();
	}
	void close(Urg_driver& urg)
	{
		std::cout << "\nLidar closed\n";
		urg.close();
	}
    int lidar_init(int argc, const char **argv, Urg_driver& urg)
    {
        Connection_information information(argc, argv);
        do
        {
            if (!urg.open(information.device_or_ip_name(), information.baudrate_or_port_number(), information.connection_type()))
            {
                cout << "Urg_driver::open(): " << information.device_or_ip_name() << ": " << urg.what() << endl;
                //return 1;
            }        
            else
            {
                cout << "URG OK!\n";
            }
            if (urg.max_data_size()<0) urg.close();
        }while(urg.max_data_size()<0);

    #if 1
        urg.set_scanning_parameter(urg.deg2step(-135), urg.deg2step(+135), 0);
    #endif
        enum { Capture_times = 1 };
        urg.start_measurement(Urg_driver::Distance, Urg_driver::Infinity_times, 0);
    }

    vector <long> lidar_lecture(Urg_driver& urg)
    {
        long time_stamp = 0;
        vector<long> data;
        int success = 0;
        success = urg.get_distance(data, &time_stamp);
        if(!success)
        {
            cout << "Urg_driver::get_distance(): " << urg.what() << endl;
            data.push_back(-1);
        }     
        else
        {
            data.push_back(time_stamp);
        }
        return data;
    }

    void print_data(const vector<long>& data, long time_stamp, Urg_driver& urg)
    {
        // Shows only the front step
        int front_index = urg.step2index(0);
        cout << data[front_index] << " [mm], (" << time_stamp << " [msec])" << endl;
    }    
}
