/*  Copyright (c) 2016 Macchina
 *
 *  Permission is hereby granted, free of charge, to any person obtaining
 *  a copy of this software and associated documentation files (the
 *  "Software"), to deal in the Software without restriction, including
 *  without limitation the rights to use, copy, modify, merge, publish,
 *  distribute, sublicense, and/or sell copies of the Software, and to
 *  permit persons to whom the Software is furnished to do so, subject to
 *  the following conditions:
 *
 *  The above copyright notice and this permission notice shall be included
 *  in all copies or substantial portions of the Software.
 *
 *  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 *  EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 *  MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 *  IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 *  CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 *  TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 *  SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *  
 *  LIN STACK for TJA1021
 *  v2.0
 *
 *  Short description: 
 *  Comunication stack for LIN and TJA1021 LIN transceiver. 
 *  Can be modified for any Arduino board with UART available and any LIN slave.
 *  
 *  Author: Blaž Pongrac B.S., RoboSap, Institute of Technology, Ptuj (www.robosap-institut.eu)
 *  
 *  Arduino IDE 1.6.9
 *  RoboSap, Institute of Technology, September 2016
*/ 

#include <Arduino.h>

/*
	Please, read Getting Started Guide first.
*/

class lin_stack
{
	public:
	// Constructors
	lin_stack(uint8_t serial_ch, uint8_t wake, float baud_def); // Master node constructor--doesn't need identity 
	lin_stack(uint8_t serial_ch, uint8_t wake, byte ident, float baud_def); //Slave node constructor with ID

	// Methods
	
	// Writing data to bus
	int write(byte add, byte data[], byte data_size); // write whole package
	int writeRequest(byte add); // Write header only
	int writeResponse(byte data[], byte data_size); // Write response only
	int writeStream(byte data[], byte data_size); // Writing user data to LIN bus
	int read(byte data[], byte data_size); // read data from LIN bus, checksum and ident validation
	int readStream(byte data[],byte data_size); // read data from LIN bus
	int setSerial(); // set up Seril communication for receiving data.
	
	// Private methods and variables
	private:
	unsigned long baud_rate = 10416; 	// baud rate for the LIN interface, defaults to 10417
	unsigned int period = 96; 			// calculated period, in mircoseconds, based on baud
	byte ch = 0; // which channel should be used
	byte identByte; // user defined Identification Byte.
	byte wakePin; // user defined pin for controlling wake function.
	int sleep(bool sleep_state); // method for controlling transceiver modes (0 - sleep, 1 - normal)
	int serial_pause(int no_bits); // for generating Synch Break
	boolean validateParity(byte ident); // for validating Identification Byte, can be modified for validating parity
	boolean validateChecksum(byte data[], byte data_size); // for validating Checksum Byte
};