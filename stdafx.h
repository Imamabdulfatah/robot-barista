#ifndef SERIALCLASS_H_INCLUDED
#define SERIALCLASS_H_INCLUDED

#define ARDUINO_WAIT_TIME 2000

#include <stdlib.h>

#include "targetver.h"

#include <stdio.h>
#include <tchar.h>
#include <windows.h>

class Serial1
{
private:
	//Serial comm handler
	HANDLE hSerial1;
	//Connection status
	bool connected;
	//Get various information about the connection
	COMSTAT status;
	//Keep track of last error
	DWORD errors;

public:
	//Initialize Serial communication with the given COM port
	Serial1(const char *portName1);
	//Close the connection
	~Serial1();
	//Read data in a buffer, if nbChar is greater than the
	//maximum number of bytes available, it will return only the
	//bytes available. The function return -1 when nothing could
	//be read, the number of bytes actually read.
	int ReadData(char *buffer, unsigned int nbChar);
	//Writes data from a buffer through the Serial connection
	//return true on success.
	bool WriteData(const char *buffer, unsigned int nbChar);
	//Check if we are actually connected
	bool IsConnected();


};

#endif // SERIALCLASS_H_INCLUDED