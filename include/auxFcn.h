#ifndef AUXFCN_H_INCLUDED
#define AUXFCN_H_INCLUDED

#include <iostream>
#include <sys/time.h> //gettimeofday

const std::string currentDateTime();
const std::string currentTime();	
int micros(timeval tStart);
int millis(timeval tStart);

#endif
