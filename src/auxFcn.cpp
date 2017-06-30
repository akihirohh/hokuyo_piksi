#include "auxFcn.h"

//###########################################
//AUX########################################

//For data logging purpose. It returns a string
//with current time to uniquely name log files


const std::string currentTime(){
	time_t		now = time(0);
	struct tm	tstruct;
	char		buf[80];
	tstruct = *localtime(&now);
	strftime(buf,sizeof(buf), "%Y%m%d%H%M%S", &tstruct);
	return buf;
}
const std::string currentDateTime(){
	time_t		now = time(0);
	struct tm	tstruct;
	char		buf[80];
	tstruct = *localtime(&now);
	strftime(buf,sizeof(buf), "%Y%m%d_%Hh%Mm%Ss",&tstruct);
	return buf;
}
//Given a reference tStart, it returns time
//ellapsed in microseconds
int micros(timeval tStart)
{
	struct timeval t;
	gettimeofday(&t,NULL);
	return (t.tv_sec - tStart.tv_sec)*1000000 + t.tv_usec - tStart.tv_usec;	
}
//Given a reference tStart, it returns time 
//ellapsed in miliseconds
int millis(timeval tStart)
{
	struct timeval t;
	gettimeofday(&t,NULL);
	return (t.tv_sec - tStart.tv_sec)*1000 + (t.tv_usec - tStart.tv_usec)/1000;	
}
