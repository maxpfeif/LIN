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
 *  LIN STACK for MCP2004 
 *  v1.0
 *
 *  Short description: 
 *  Comunication stack for LIN and MCP2004 LIN transceiver. 
 *  Can be modified for any Arduino board with UART available and any LIN slave.
 *  
 *	Forked from Blaž Pongrac B.S., RoboSap, Institute of Technology, Ptuj (www.robosap-institut.eu)
 *	Authored by MAX PFEIFFER, September 2018.   
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

/*-----------------------------------------------------------------------------------*/ 

// NEW CONSTRUCTORS
// Initialize the lin_stack object with the desired serial channel, wakeup pin and identity. 
lin_stack::lin_stack(uint8_t serial_ch, uint8_t wake, byte ident, float baud_def){
	ch = serial_ch; // Saving the serial channel to a private variable 
	wakePin = wake; // Saving the wake pin to a private variable  
	identByte = ident; // Saving the identy to the private variable
	baud_rate = baud_def;	// Save the baud rate to a private var 
	period = int(baud_rate/108.5);	// calculate the period from baud rate  
	sleep(FALSE);	// Initialize the transciever in wake state
}

// Initialize the lin_stack object with just the channel and wakeup pin
lin_stack::lin_stack(uint8_t serial_ch, uint8_t wake, float baud_def) {
	ch = serial_ch; // Saving the serial channel to a private variable 
	wakePin = wake; // Saving the wake pin to a private variable 
	baud_rate = baud_def;	// Save the baud rate to a private var 
	period = int(baud_rate/108.5);	// calculate the period from baud rate  
	sleep(FALSE); //Initialize the transciever in wake state 
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
	// Start interface
	sleep(FALSE); // wakeup the device 

	// Synch Break -- need this as the beginning of the LIN frame 
	serial_pause(13);

	// Send data via Serial interface, depending on which serial port is being used 
	if(ch==1){						// If serial channel = 1
		Serial1.begin(baud_rate); 	// Begin serial at the lin_stack's baud 
		Serial1.write(0x55); 		// write Synch Byte to serial
		Serial1.write(ident); 		// write Identification Byte to serial
		for(int i=0;i<data_size;i++) Serial1.write(data[i]); // write data to serial
		Serial1.write(checksum); 	// write Checksum Byte to serial
		Serial1.end(); 				// clear Serial config
	}else if(ch==2){ 
		Serial2.begin(baud_rate); 
		Serial2.write(0x55); 
		Serial2.write(ident); 
		for(int i=0;i<data_size;i++) Serial2.write(data[i]); 
		Serial2.write(checksum);
		Serial2.end(); 
	} else if(ch==3){
		Serial3.begin(baud_rate);	
		Serial3.write(0x55);	
		Serial3.write(ident);
		for(int i=0;i<data_size;i++) Serial3.write(data[i]);
		Serial3.write(checksum);
		Serial3.end();
	} else if(ch==4){				
		Serial4.begin(baud_rate);	
		Serial4.write(0x55);		
		Serial4.write(ident);
		for(int i=0;i<data_size;i++) Serial4.write(data[i]);
		Serial4.write(checksum);
		Serial4.end();
	}
	sleep(TRUE); // Go to Sleep mode
	return 1;
}

int lin_stack::writeRequest(byte ident){
	// Create Header
	byte header[2]= {0x55, ident}; // ident is the identy of the device you wish to receive data from 
	// Start interface
	sleep(FALSE); // Wakeup the transciever 
	// Synch Break
	serial_pause(13);
	// Send data via Serial interface
	if(ch==1){ 						// if we are using channel 1 
		Serial1.begin(baud_rate);	// configure serial at desired baud rate
		Serial1.write(header,2); 	// write data to the serial port using recursive write 
		Serial1.end(); 				// clear Serial config
	}else if(ch==2){
		Serial2.begin(baud_rate); 
		Serial2.write(header,2); 
		Serial2.end();
	} else if(ch==3) {
		Serial3.begin(baud_rate);
		Serial3.write(header,2);
		Serial3.end();
	} else if (ch==4){
		Serial4.begin(baud_rate);
		Serial4.write(header,2);
		Serial4.end();
	}
	sleep(TRUE); // Go to Sleep mode
	return 1;
}

int lin_stack::writeResponse(byte data[], byte data_size){
	// Calculate checksum
	byte suma = 0;
	for(int i=0;i<data_size;i++) suma = suma + data[i];
	suma = suma + 1;
	byte checksum = 255 - suma;
	// Start interface
	sleep(FALSE); // Go to Normal mode
	// Send data via Serial interface
	if(ch==1){ 							// if we are using serial1
		Serial1.begin(baud_rate); 		// initialize the port at the correct rate 
		Serial1.write(data, data_size); // write data to serial
		Serial1.write(checksum); // write data to serial
		Serial1.end(); // clear Serial config
	}else if(ch==2){ 
		Serial2.begin(baud_rate);
		Serial2.write(data, data_size); 
		Serial2.write(checksum); 
		Serial2.end(); 
	} else if(ch==3) {
		Serial3.begin(baud_rate);
		Serial3.write(data,data_size);
		Serial3.write(checksum);
		Serial3.end();
	} else if(ch==4) {
		Serial4.begin(baud_rate);
		Serial4.write(data,data_size);
		Serial4.write(checksum);
		Serial4.end();
	}
	sleep(TRUE); // Go to Sleep mode
	return 1;
}

int lin_stack::writeStream(byte data[], byte data_size){
	// Start interface
	sleep(FALSE); // Go to Normal mode
	// Synch Break
	serial_pause(13);
	// Send data via Serial interface
	if(ch==1){ 
		Serial1.begin(baud_rate);
		for(int i=0;i<data_size;i++) Serial1.write(data[i]);
		Serial1.end();
	}else if(ch==2){
		Serial2.begin(baud_rate); 
		for(int i=0;i<data_size;i++) Serial2.write(data[i]);
		Serial2.end(); 
	} else if(ch==3){
		Serial3.begin(baud_rate);
		for(int i=0;i<data_size;i++) Serial3.write(data[i]);
		Serial3.end(); 
	} else if(ch==4){
		Serial4.begin(baud_rate);
		for(int i=0;i<data_size;i++) Serial4.write(data[i]);
		Serial4.end(); 
	}
	sleep(TRUE); // Go to Sleep mode
	return 1;
}

// READ methods
// Read LIN traffic and then proces it.
int lin_stack::setSerial(){ // Only needed when receiving signals, note there are no internall pull ups here now, hardware may need one. 
	if(ch==1){ 
		Serial1.begin(baud_rate); 
	} else if(ch==2){ 
		Serial2.begin(baud_rate);
	} else if(ch==3){
		Serial3.begin(baud_rate);
	} else if(ch==4){
		Serial4.begin(baud_rate);
	}
}

int lin_stack::read(byte data[], byte data_size){
	byte rec[data_size+3];
	if(ch==1){ // For LIN1 or Serial1
		if(Serial1.read() != -1){ // Check if there is an event on LIN bus
			Serial1.readBytes(rec,data_size+3);
			if((validateParity(rec[1]))&(validateChecksum(rec,data_size+3))){
				for(int j=0;j<data_size;j++){
				data[j] = rec[j+2];
				}
				return 1;
			}else{
				return -1;
			}	
		}
	}else if(ch==2){ 
		if(Serial2.read() != -1){ 
			Serial2.readBytes(rec,data_size+3);
			if((validateParity(rec[1]))&(validateChecksum(rec,data_size+3))){
				for(int j=0;j<data_size;j++){
				data[j] = rec[j+2];
				}
				return 1;
			}else{
				return -1;
			}	
		}
	}else if(ch==3){ 
		if(Serial3.read() != -1){ 
			Serial3.readBytes(rec,data_size+3);
			if((validateParity(rec[1]))&(validateChecksum(rec,data_size+3))){
				for(int j=0;j<data_size;j++){
				data[j] = rec[j+2];
				}
				return 1;
			}else{
				return -1;
			}	
		}
	}else if(ch==4){ 
		if(Serial4.read() != -1){ 
			Serial4.readBytes(rec,data_size+3);
			if((validateParity(rec[1]))&(validateChecksum(rec,data_size+3))){
				for(int j=0;j<data_size;j++){
				data[j] = rec[j+2];
				}
				return 1;
			}else{
				return -1;
			}	
		}
	}

	return 0;
}

int lin_stack::readStream(byte data[],byte data_size){
	byte rec[data_size];
	if(ch==1){ // For LIN1 or Serial1
		if(Serial1.read() != -1){ // Check if there is an event on LIN bus
			Serial1.readBytes(rec,data_size);
			for(int j=0;j<data_size;j++){
				data[j] = rec[j];
			}
			return 1;
		}
	}else if(ch==2){ 
		if(Serial2.read() != -1){ 
			Serial2.readBytes(data,data_size);
			for(int j=0;j<data_size;j++){
				data[j] = rec[j];
			}
			return 1;
		}
	}else if(ch==3){ 
		if(Serial3.read() != -1){ 
			Serial3.readBytes(data,data_size);
			for(int j=0;j<data_size;j++){
				data[j] = rec[j];
			}
			return 1;
		}
	}else if(ch==4){ 
		if(Serial4.read() != -1){ 
			Serial4.readBytes(data,data_size);
			for(int j=0;j<data_size;j++){
				data[j] = rec[j];
			}
			return 1;
		}
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
	unsigned int del = period*no_bits; // delay for number of bits (no-bits) in microseconds, depends on period
	if(ch=1){
		// tx is pin 1 
		digitalWrite(1, LOW); // should be able to pull this low with out initializing the pin...
		delayMicroseconds(del); // delay
	}else if(ch=2){
		digitalWrite(10, LOW);
		delayMicroseconds(del); // delay
	} else if(ch=3){
		digitalWrite(8, LOW);
		delayMicroseconds(del);
	} else if(ch=4){
		digitalWrite(32, LOW);
		delayMicroseconds(del);
	}
	return 1;
}

// Modified to write the sleep pin to high or low depending on the passed boolean
// True = Sleep 
// False = Wake
int lin_stack::sleep(bool sleep_state){
	if(sleep_state){ // Go to Normal mode
		digitalWrite(wake_pin, LOW);
	} else {
		digitalWrite(wake_pin, HIGH);
	}	
	delayMicroseconds(20); // According to TJA1021 datasheet this is needed for propper working
	return 1;
}

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
