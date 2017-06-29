#include <iostream>
#include <fstream>
#include <string>
#include <stdlib.h> //exit
#include <pthread.h>

#include "auxFcn.h"
#include "lidarFcn.h"
#include "keyboardbreak.h"

#include <ctime>
#include <iomanip>

#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>

#include <libserialport.h>

#include <libsbp/sbp.h>
#include <libsbp/system.h>
#include <libsbp/navigation.h>

vector<long> lidarFront;
int USE_LIDAR = 0, USE_PIKSI = 0;

/* SBP structs that messages from Piksi will feed. */
msg_pos_llh_t         pos_llh;
msg_baseline_ned_t    baseline_ned;
msg_vel_ned_t         vel_ned;
msg_dops_t            dops;
msg_gps_time_t        gps_time;
msg_utc_time_t        utc_time;
msg_pos_ecef_t        pos_ecef;
msg_baseline_ecef_t   baseline_ecef;


/*
* SBP callback nodes must be statically allocated. Each message ID / callback
* pair must have a unique sbp_msg_callbacks_node_t associated with it.
*/
sbp_msg_callbacks_node_t baseline_ecef_node;
sbp_msg_callbacks_node_t baseline_ned_node;
sbp_msg_callbacks_node_t dops_node;
sbp_msg_callbacks_node_t gps_time_node;
sbp_msg_callbacks_node_t pos_llh_node;
sbp_msg_callbacks_node_t pos_ecef_node;
sbp_msg_callbacks_node_t utc_time_node;
sbp_msg_callbacks_node_t vel_ned_node;


/*
 * Callback functions to interpret SBP messages.
 * Every message ID has a callback associated with it to
 * receive and interpret the message payload.
 */


void sbp_baseline_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  baseline_ecef = *(msg_baseline_ecef_t *)msg;
}
void sbp_baseline_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  baseline_ned = *(msg_baseline_ned_t *)msg;
}
void sbp_dops_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  dops = *(msg_dops_t *)msg;
}
void sbp_gps_time_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  gps_time = *(msg_gps_time_t *)msg;
}
void sbp_pos_ecef_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  pos_ecef = *(msg_pos_ecef_t *)msg;
}
void sbp_pos_llh_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  pos_llh = *(msg_pos_llh_t *)msg;
}
void sbp_utc_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  utc_time = *(msg_utc_time_t *)msg;
}
void sbp_vel_ned_callback(u16 sender_id, u8 len, u8 msg[], void *context)
{
  vel_ned = *(msg_vel_ned_t *)msg;
}

void sbp_setup(sbp_state_t *sbp_state)
{
  /* SBP parser state must be initialized before sbp_process is called. */
	
  sbp_state_init(sbp_state);

  /* Register a node and callback, and associate them with a specific message ID. */	
  sbp_register_callback(sbp_state, SBP_MSG_BASELINE_ECEF, &sbp_baseline_ecef_callback, NULL, &baseline_ecef_node);
  sbp_register_callback(sbp_state, SBP_MSG_BASELINE_NED, &sbp_baseline_ned_callback, NULL, &baseline_ned_node);
  sbp_register_callback(sbp_state, SBP_MSG_DOPS, &sbp_dops_callback,NULL, &dops_node);
  sbp_register_callback(sbp_state, SBP_MSG_GPS_TIME, &sbp_gps_time_callback, NULL, &gps_time_node);
  sbp_register_callback(sbp_state, SBP_MSG_POS_ECEF, &sbp_pos_ecef_callback, NULL, &pos_ecef_node);
  sbp_register_callback(sbp_state, SBP_MSG_POS_LLH, &sbp_pos_llh_callback, NULL, &pos_llh_node);
  sbp_register_callback(sbp_state, SBP_MSG_UTC_TIME, &sbp_utc_callback, NULL, &utc_time_node);
  sbp_register_callback(sbp_state, SBP_MSG_VEL_NED, &sbp_vel_ned_callback, NULL, &vel_ned_node);
}

char *serial_port_name = NULL;
struct sp_port *piksi_port = NULL;
static sbp_msg_callbacks_node_t heartbeat_callback_node;

void usage() {
  fprintf(stderr, "usage: Use at least one of the devices: -p for Piksi and -l for LiDAR\n");
}

void setup_port()
{
  int result;

  result = sp_set_baudrate(piksi_port, 115200);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set port baud rate!\n");
    exit(EXIT_FAILURE);
  }

  result = sp_set_flowcontrol(piksi_port, SP_FLOWCONTROL_NONE);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set flow control!\n");
    exit(EXIT_FAILURE);
  }

  result = sp_set_bits(piksi_port, 8);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set data bits!\n");
    exit(EXIT_FAILURE);
  }

  result = sp_set_parity(piksi_port, SP_PARITY_NONE);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set parity!\n");
    exit(EXIT_FAILURE);
  }

  result = sp_set_stopbits(piksi_port, 1);
  if (result != SP_OK) {
    fprintf(stderr, "Cannot set stop bits!\n");
    exit(EXIT_FAILURE);
  }
}

u32 piksi_port_read(u8 *buff, u32 n, void *context)
{
  (void)context;
  u32 result;

  result = sp_blocking_read(piksi_port, buff, n, 0);

  return result;
}

//DATA_LOGGING
int countNewData = 0, logTime = 0, maxLogTime = 0, loop = 1;

struct timeval tStart;
void dataLogging(void *ptr);

int main(int argc, char *argv[])
{
	
	//Time
	struct timeval t1;
	int loopTime = 0, maxLoopTime = 0, totalTime = 0, previousLoopTime = 0, time1;
	
	//Loop
	int loop1=1;

	//LIDAR
	char *arr0[] = { "pruebalidar", "-s", "/dev/serial/by-path/platform-3f980000.usb-usb-0:1.4:1.0" };
	Urg_driver urg0;

	serial_port_name = "/dev/ttyUSB0";

	int opt, result = 0;
	sbp_state_t s;
	if (argc <= 1) {
		usage();
		exit(EXIT_FAILURE);
	}
	while ((opt = getopt(argc, argv, "pl")) != -1) {
		switch (opt) {
		case 'p':
			USE_PIKSI = 1;
			break;
		case 'l':
			USE_LIDAR = 1;
			break;
		case 'h':
			usage();
			exit(EXIT_FAILURE);
		}
	}

	if(USE_LIDAR || USE_PIKSI)
	{			
		pthread_t thread;
		pthread_create(&thread, NULL, (void *)dataLogging, (void *) NULL);        
	}    				
	if(USE_LIDAR)
	{
		lidar_init(3, arr0, urg0);
		lidarFront  =  lidar_lecture(urg0);
		std::cout << "Front Lidar samples: ";
		for (int i = 0; i< 1080; i+=216) cout << lidarFront[i] << "\t|\t";	
	}
		
	if(USE_PIKSI)
	{
		result = sp_get_port_by_name(serial_port_name, &piksi_port);
		if (result != SP_OK) {
			fprintf(stderr, "Cannot find provided serial port!\n");
			exit(EXIT_FAILURE);
		}

		result = sp_open(piksi_port, SP_MODE_READ);
		if (result != SP_OK) {
			fprintf(stderr, "Cannot open %s for reading!\n", serial_port_name);
			exit(EXIT_FAILURE);
		}
		setup_port();
		sbp_setup(&s);
	}

	gettimeofday(&tStart,NULL);
	gettimeofday(&t1,NULL); 

	//#############################################	
	//#############################################	
	//
	//					LOOP CODE 
	//
	//#############################################	
	//#############################################	
	while(loop == 1) //começo do loop do experimento
	{	
		try
		{
			if(USE_LIDAR) 
			{
				lidarFront  =  lidar_lecture(urg0);
				for (int i = 0; i< 1080; i+=216) cout << lidarFront[i] << "\t|\t";	
			}
		}
		catch (...)
		{
			if(USE_LIDAR && urg0.max_data_size() < 0) 
					std::cout << "\n\n\n!!!ERROR LIDAR!!!\n\n\n"; 
			loop = 1;        
		}   
		if(USE_PIKSI)
		{
			sbp_process(&s, &piksi_port_read);
			printf("\n############################\n");
			printf( "Absolute Position:\n");
			printf( "\tLatitude: %4.10lf", pos_llh.lat);
			printf( "\tLongitude: %4.10lf", pos_llh.lon);
			printf( "\tHeight: %4.10lf", pos_llh.height);
			printf( "\tSatellites: %02d", pos_llh.n_sats);
			printf("\tnFlags: %d", (int)pos_llh.flags);
			printf( "\n");
			printf( "Position ECEF (m):\n");
			printf( "\tx: %4.3f", pos_ecef.x);
			printf( "\ty: %4.3f", pos_ecef.y);
			printf( "\tz: %4.3f", pos_ecef.z);
			printf( "\n");
			printf( "Baseline ECEF (m):\n");
			printf( "\tx: %4.3f", (double)baseline_ecef.x/1000);
			printf( "\ty: %4.3f", (double)baseline_ecef.y/1000);
			printf( "\tz: %4.3f", (double)baseline_ecef.z/1000);
			printf( "\n");	
		}

		if (kbhit() && getchar()=='s') loop=0;
		previousLoopTime = loopTime;
		loopTime = micros(t1);
		gettimeofday(&t1,NULL);
		totalTime = micros(tStart);
		maxLoopTime = (loopTime > maxLoopTime) ? loopTime : maxLoopTime;			
		countNewData++;
	}
	//#############################################	
	//#############################################	
	//
	//				LOOP CODE (END)
	//
	//#############################################	
	//#############################################	
	std::cout << "Max loop time: " << maxLoopTime << "us" << endl; 

	if(USE_PIKSI)
	{
		result = sp_close(piksi_port);
		if (result != SP_OK) {
			fprintf(stderr, "Cannot close %s properly!\n", serial_port_name);
		}
		sp_free_port(piksi_port);
		free(serial_port_name);
	}
	
	if(loop == -1) cout << "\n###################################\n \n \t\tCHECK SENSORS \n\n###################################\n";    
}

//###############################################
//FUNCTIONS
//###############################################
void dataLogging(void *ptr)
{
	int oldCountNewData = 0;
	struct timeval tLoop, tLogging;
	std::string filename;
	std::fstream lidar_file;

	std::string filename_gps;
	std::fstream gps_file;

	if(USE_LIDAR)
	{
		filename.append(currentDateTime());
		filename.append("_lidar.txt");
		lidar_file.open(filename.c_str(),std::fstream::out);
		std::cout << filename << std::endl;
	} 

	if(USE_PIKSI)
	{
		filename_gps.append(currentDateTime());
		filename_gps.append("_vars.txt");
		gps_file.open(filename_gps.c_str(), std::ios_base::out);
		gps_file << "timestamp | utc_time_tow | utc_time_flags | utc_time_year |  utc_time_month | utc_time_day | utc_time_hours | utc_time_minutes | utc_time_seconds | utc_time_ns | pos_llh_tow | pos_llh_flags | pos_llh_h_accuracy | pos_llh_v_accuracy |  pos_llh_lat | pos_llh_lon | pos_llh_n_sats | baseline_ned_tow | baseline_ned_flags |  baseline_ned_n | baseline_ned_e | baseline_ned_d | dops_tow | dops_flags | dops_gdop | dops_pdop | dops_tdop | dops_hdop | dops_vdop | pos_ecef.tow |pos_ecef.flags | pos_ecef.n_sats | pos_ecef.x | pos_ecef.y | pos_ecef.z | baseline_ecef.tow | baseline_ecef.flags |baseline_ecef.n_sats | baseline_ecef.x | baseline_ecef.y |baseline_ecef.z | baseline_ecef.accuracy " <<  std::endl;
	}
  
	string timestamp;

	gettimeofday(&tLogging,NULL);
	while(loop == 1)
	{   
		gettimeofday(&tLoop,NULL);
		if(oldCountNewData != countNewData)
		{
			timestamp = currentTime() + std::to_string(millis(tLogging));
			if(USE_LIDAR)
			{
				lidar_file  << timestamp << "|";
				for(int i =0; i < lidarFront.size(); i++)
				{
						lidar_file << lidarFront[i] << "|";
				}
				lidar_file << std::endl;
			}

			if(USE_PIKSI)
			{
				gps_file << timestamp << "|" << utc_time.tow << "|" << (int)utc_time.flags << "|" << utc_time.year << "|" << (int) utc_time.month << "|" << (int)utc_time.day << "|" << (int)utc_time.hours << "|" << (int)utc_time.minutes << "|" << (int)utc_time.seconds << "|" << utc_time.ns << "|" << pos_llh.tow << "|" << (int)pos_llh.flags << "|" << pos_llh.h_accuracy << "|" << pos_llh.v_accuracy << "|" <<  std::fixed << std::setprecision(10) << pos_llh.lat << "|" << std::fixed << std::setprecision(10) << pos_llh.lon << "|" << (int)pos_llh.n_sats << "|" << baseline_ned.tow << "|" << (int)baseline_ned.flags << "|" <<  baseline_ned.n << "|" << baseline_ned.e << "|" << baseline_ned.d << "|" << dops.tow << "|" << (int)dops.flags << "|" << dops.gdop << "|" << dops.pdop << "|" << dops.tdop << "|" << dops.hdop << "|" << dops.vdop << "|" << pos_ecef.tow << "|" << (int)pos_ecef.flags << "|" << (int) pos_ecef.n_sats << "|" << pos_ecef.x << "|" << pos_ecef.y << "|" << pos_ecef.z << "|" << baseline_ecef.tow << "|" << (int)baseline_ecef.flags << "|" << (int)baseline_ecef.n_sats << "|" << baseline_ecef.x << "|" << baseline_ecef.y << "|" <<baseline_ecef.z << "|" << baseline_ecef.accuracy << std::endl;
			}

			std::cout << "DATA_LOGGING: " << millis(tLoop) << "\tts:  " << timestamp << "\tmaxDataLogging: " << maxLogTime << endl;
			oldCountNewData = countNewData;				
		}    
		logTime = millis(tLoop);
		maxLogTime = (logTime > maxLogTime) ? logTime : maxLogTime;
	}
	std::cout << "\n###################################\n\nmaxLogTime: " << maxLogTime<< "ms\n###################################\n\n";
	if(USE_LIDAR) lidar_file.close();	
	if(USE_PIKSI) gps_file.close();
	pthread_exit(0);
}
