#include "lidarRead.h"
#include <iostream>

namespace lidarRead
{
	//Given a reference tStart, it returns time 
	//ellapsed in miliseconds
	int millis(timeval tStart)
	{
		struct timeval t;
		gettimeofday(&t,NULL);
		return (t.tv_sec - tStart.tv_sec)*1000 + (t.tv_usec - tStart.tv_usec)/1000;	
	}
	
	void *lidarReading( void *ptr )
	{
		FILE *fp;
		int status;
		char path[PATH_MAX];
		char * p;
		struct timeval t;
		printf("beginning thread\n");
		int k = 0;
		
		char const *arr0[] = { "pruebalidar", "-s", "/dev/ttyACM0" };
		Urg_driver urg0;
		thdata *data;
		data = (thdata *) ptr;
		
		std::fstream lidar_file(data->filename.c_str(),std::fstream::out);

  	hokuyo::lidar_init(3, arr0, urg0);
		data->d = hokuyo::lidar_lecture(urg0);
	    std::cout << "Front Lidar samples: ";
	    for (int i = 0; i< NUM_LIDAR_ELEMENTS; i+=NUM_LIDAR_ELEMENTS/20) std::cout << data->d[i] << "\t|\t";
		gettimeofday(&t,NULL);
		data->b_loop = 1;

		while(data->b_loop)
		{
			try
			{
				data->d = hokuyo::lidar_lecture(urg0); 
				
				if (data->d.back()==-1)
				{
					std::cout << "ERROR";
					hokuyo::close(urg0);
					hokuyo::lidar_init(3,arr0,urg0);
					data->d = hokuyo::lidar_lecture(urg0);
				}  
				data->lidar_timestamp = data->d.back();
				if(data->b_save)
				{
					lidar_file << data->timestamp;
					for (int i = 0; i < data->d.size()-1; i++) //last one is timestamp
						lidar_file << "|" << data->d[i];
					lidar_file << std::endl;
				}
			}
			catch (...)
			{
				if (urg0.max_data_size() < 0) 
				{
					std::cout << "\n\n\n!!!ERROR LIDAR!!!\n\n\n";
					data->b_loop = 0;					
				}      
			}  						
		}
		
		if(data->b_save) lidar_file.close();	
		return NULL;
	}	
}
