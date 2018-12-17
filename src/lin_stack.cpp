/*  Original Copyright (c) 2016 Macchina
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
 *  LIN STACK for MCP2004/MCP2003
 *  v1.0
 *
 *  Short description: 
 *  Comunication stack for LIN and MCP2004 LIN transceiver. 
 *  Can be modified for any Arduino board with UART available and any LIN slave.
 *  
 *	Forked from Blaž Pongrac B.S., RoboSap, Institute of Technology, Ptuj (www.robosap-institut.eu)
 *	Modified by MAX PFEIFFER, September 2018.   
 */ 

#include <lin_stack.h>
// Including the Teensy hardware serial module 
#include <HardwareSerial.h>

/* LIN PACKET:
   It consist of:
    ___________ __________ _______ ____________ _________ 
   |           |          |       |            |         |
   |Synch Break|Synch Byte|ID byte| Data Bytes |Checksum |
   |___________|__________|_______|____________|_________|
   
   Every byte have start bit and stop bit and it is send LSB first.
   Synch Break - 13 bits of dominant state ("0"), followed by 1 bit recesive state ("1")
   Synch Byte - Byte for Bound rate syncronization, always 0x55
   ID Byte - consist of parity, length and address; parity is determined by LIN standard and depends from address and message length
   Data Bytes - user defined; depend on devices on LIN bus
   Checksum - inverted 256 checksum; data bytes are sumed up and then inverted
*/


// LIN MESSAGE CONSTRUCTORS 

/*-----------------------------------------------------------------------------------*/ 

// NEW CONSTRUCTORS
// Initialize the lin_stack object with the desired serial channel, wakeup pin and identity. 
lin_stack::lin_stack(byte ident, float baud_def){
	identByte = ident; // Saving the identy to the private variable
	baud_rate = baud_def;	// Save the baud rate to a private var 
	period = int(baud_rate/108.5);	// calculate the period from baud rate  
}

// Initialize the lin_stack object with just the channel and wakeup pin
lin_stack::lin_stack(float baud_def) {
	baud_rate = baud_def;	// Save the baud rate to a private var 
	period = int(baud_rate/108.5);	// calculate the period from baud rate  
}


/*-----------------------------------------------------------------------------------*/ 
// TODO: ADJUST THESE TO CONFORM TO NEW CONFOGIURATIONS 
// PUBLIC METHODS
// WRITE methods
// Creates a LIN packet and then send it via USART(Serial) interface.
int lin_stack::write(byte ident, byte data[], byte data_size){
	// Calculate checksum
	byte suma = 0;
	for(int i=0;i<data_size;i++) suma = suma + data[i];
	suma = suma + 1;
	byte checksum = 255 - suma;
	// Need a delay between the wake from sleep and the delay. Not sure we want the sleep 
	// actually. We should probably just wake the fucker up and have it on while the vehicle is 
	// on. This business of putting this shit to sleep every time doesn't make sense.. 
	// delayMicroseconds(100);	// thinking we might not need this 
	// Synch Break -- need this as the beginning of the LIN frame.. also need recessive bit?s 
	serial_pause(13); 
	// Send the data using the Serial 3 interface. 
	Serial3.begin(baud_rate);
	Serial3.write(0x55);	
	Serial3.write(ident);
	for(int i=0;i<data_size;i++) Serial3.write(data[i]);
	Serial3.write(checksum);
	Serial3.end();
	return 1;
}

// Writes just the PID to the bus 
int lin_stack::writeRequest(byte ident){
	// Create Header
	byte header[2]= {0x55, ident}; // ident is the identy of the device you wish to receive data from 
	// Synch Break
	serial_pause(13);
	// Send data via Serial interface
	Serial3.begin(baud_rate);
	Serial3.write(header,2);
	Serial3.end();
	return 1;
}

// Write a response to a PID, this includes the checksum?  
int lin_stack::writeResponse(byte data[], byte data_size){
	// Calculate checksum
	byte suma = 0;
	for(int i=0;i<data_size;i++) suma = suma + data[i];
	suma = suma + 1;
	byte checksum = 255 - suma;
	Serial3.begin(baud_rate);
	Serial3.write(data,data_size);
	Serial3.write(checksum);
	Serial3.end();
	return 1;
}

// Write a stream--without the checksum though.. 
int lin_stack::writeStream(byte data[], byte data_size){
	// Synch Break
	serial_pause(13);
	Serial3.begin(baud_rate);
	for(int i=0;i<data_size;i++) Serial3.write(data[i]);
	Serial3.end(); 

	return 1;
}

// READ methods
// Read LIN traffic and then proces it.
int lin_stack::setSerial(){ // Only needed when receiving signals, note there are no internall pull ups here now, hardware may need one. 
	Serial3.begin(baud_rate);
	return 1;
}

// Reads data from the bus, including the checksum. 
int lin_stack::read(byte data[], byte data_size){
	byte rec[data_size+3];						// How much data are we looking for? +3 
	if(Serial3.read() != -1){ 
		Serial3.readBytes(rec,data_size+3);		// Reads bytes from the serial bus 
		

		// This probabl has a problem when reading LIN from the heater 
		if((validateParity(rec[1]))&(validateChecksum(rec,data_size+3))){
			for(int j=0;j<data_size;j++){
			data[j] = rec[j+2];
			}
			return 1;
		}else{
			return -1;
		}	
	}
	return 0;
}

// Reads the data stream 
int lin_stack::readStream(byte data[],byte data_size){
	byte rec[data_size];
	if(Serial3.read() != -1){ 
		Serial3.readBytes(data,data_size);
		for(int j=0;j<data_size;j++){
			data[j] = rec[j];
		}
		return 1;
	}

	return 0;
}


// PRIVATE METHODS

//come back to the serial_pause method, this will likely need to be adjusted but not sure what to do with this yet...
// needs to be correctly pulling the line low, not sure how to do this at hardware level without 
// and delay influencing the transmission but the easiest thing seems to be to pull the pin low 
// manually, then high again after the micros delay, allowing the pin to be recaptured as a Serial pin

// Another alternative to this would be figuring out how to send the serial 13 bits of 0, i.e. 
// directly sending it some hex or binary. 

int lin_stack::serial_pause(int no_bits){
	// Calculate delay needed for 13 bits, depends on baud 
	uint16_t del = period*no_bits; // delay for number of bits (no-bits) in microseconds, depends on period
	Serial.print("Delay in microseconds should be; "); Serial.println(del);
	digitalWrite(8, LOW); // not sure whether or not this is working, but the delay could be problematic 
	delayMicroseconds(del);
	digitalWrite(8, HIGH);
	return 1;
}

// If the byte coming in is the same as the ident byte for this instance 
boolean lin_stack::validateParity(byte ident) {
	if(ident == identByte)
		return true;
	else
		return false;
}

boolean lin_stack::validateChecksum(unsigned char data[], byte data_size){
	byte checksum = data[data_size-1];
	byte suma = 0;
	for(int i=2;i<data_size-1;i++) suma = suma + data[i];
	byte v_checksum = 255 - suma - 1;
	if(checksum==v_checksum)
		return true;
	else
		return false;
} 

