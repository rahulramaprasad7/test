/* REFERENCE https://raspberry-projects.com/pi/programming-in-c/i2c/using-the-i2c-interface */
#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <stdint.h>
#include <sys/ioctl.h>			//Needed for I2C port
//#include <linux/i2c-dev.h>		//Needed for I2C port
#include "i2c-dev.h"
#include "MadgwickAHRS.h"
int main()
{
	int file_i2c, file_i2c_mag;
	//int length;
	uint8_t buffer[18] = {0};
	uint16_t ax =0, ay = 0, az = 0, gx = 0, gy = 0, gz = 0, mx = 0, my = 0, mz = 0;

	//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";
	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
		return -1;
	}
	
	int addr = 0x68;          //<<<<<The I2C address of the slave
	int addr_mag = 0x48; 
	if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
		//ERROR HANDLING; you can check errno to see what went wrong
		return -1 ;
	}
	if (ioctl(file_i2c_mag, I2C_SLAVE, addr_mag) < 0)
	{
		printf("Failed to acquire bus access for magnetometer and/or talk to slave.\n");
		//ERROR HANDLING; you can check errno to see what went wrong
		return -1 ;
	}
	
	/* New code */
	while(1)
	{
		/* Reading accelerometer values */
		buffer[0] = i2c_smbus_read_byte_data(file_i2c, 0x3B); // AccelerometerX High Byte
		buffer[1] = i2c_smbus_read_byte_data(file_i2c, 0x3C); // AccelerometerX Low Byte
		buffer[2] = i2c_smbus_read_byte_data(file_i2c, 0x3D); // AccelerometerY High Byte
		buffer[3] = i2c_smbus_read_byte_data(file_i2c, 0x3E); // AccelerometerY Low Byte
		buffer[4] = i2c_smbus_read_byte_data(file_i2c, 0x3F); // AccelerometerZ High Byte
		buffer[5] = i2c_smbus_read_byte_data(file_i2c, 0x40); // AccelerometerZ Low Byte

		/* Reading gyroscope values */
		buffer[6] = i2c_smbus_read_byte_data(file_i2c, 0x43); // GyroscopeX High Byte
		buffer[7] = i2c_smbus_read_byte_data(file_i2c, 0x44); // GyroscopeX Low Byte
		buffer[8] = i2c_smbus_read_byte_data(file_i2c, 0x45); // GyroscopeY High Byte
		buffer[9] = i2c_smbus_read_byte_data(file_i2c, 0x46); // GyroscopeY Low Byte
		buffer[10] = i2c_smbus_read_byte_data(file_i2c, 0x47); // GyroscopeZ High Byte
		buffer[11] = i2c_smbus_read_byte_data(file_i2c, 0x48); // GyroscopeZ Low Byte

		/* Reading Magnetometer values */
		buffer[12] = i2c_smbus_read_byte_data(file_i2c_mag, 0x04); // MagnetometerX High Byte
		buffer[13] = i2c_smbus_read_byte_data(file_i2c_mag, 0x03); // MagnetometerX Low Byte
		buffer[14] = i2c_smbus_read_byte_data(file_i2c_mag, 0x06); // MagnetometerY High Byte
		buffer[15] = i2c_smbus_read_byte_data(file_i2c_mag, 0x05); // MagnetometerY Low Byte
		buffer[16] = i2c_smbus_read_byte_data(file_i2c_mag, 0x08); // MagnetometerZ High Byte
		buffer[17] = i2c_smbus_read_byte_data(file_i2c_mag, 0x07); // MagnetometerZ Low Byte

		/* Converting the accelerometer values into 16 bits */
		ax = buffer[0] << 8 | buffer[1];
		ay = buffer[2] << 8 | buffer[3];
		az = buffer[4] << 8 | buffer[5];

		/* Converting the gyroscope values into 16 bits */
		gx = buffer[6] << 8 | buffer[7];
		gy = buffer[8] << 8 | buffer[9];
		gz = buffer[10] << 8 | buffer[11];

		/* Converting the gyroscope values into 16 bits */
		mx = buffer[12] << 8 | buffer[13];
		my = buffer[14] << 8 | buffer[15];
		mz = buffer[16] << 8 | buffer[17];

		MadgwickAHRSupdate((float)ax, (float)ay, (float)az, (float)gx, (float)gy, (float)gz, (float)mx, (float)my, (float)mz);	
	}

	/*New code */
	
	//----- READ BYTES -----
	// length = 128;			//<<< Number of bytes to read
	// if (read(file_i2c, buffer, length) != length)		//read() returns the number of bytes actually read, if it doesn't match then an error occurred (e.g. no response from the device)
	// {
	// 	//ERROR HANDLING: i2c transaction failed
	// 	printf("Failed to read from the i2c bus.\n");
	// }
	// else
	// {
	// 	//printf("Data read: %s\n", buffer);
	// 	//for (int i = 59; i < 73; i++)
	// 	//	printf("%x: %x\n ", i,buffer[i]);
	// }
	// MadgwickAHRSupdate((float)buffer[59],(float)buffer[61],(float)buffer[63],(float)buffer[67],(float)buffer[69],(float)buffer[71],0,0,0);	

	
	// //----- WRITE BYTES -----
	// buffer[0] = 0x01;
	// buffer[1] = 0x02;
	// length = 2;			//<<< Number of bytes to write
	// if (write(file_i2c, buffer, length) != length)		//write() returns the number of bytes actually written, if it doesn't match then an error occurred (e.g. no response from the device)
	// {
	// 	/* ERROR HANDLING: i2c transaction failed */
	// 	printf("Failed to write to the i2c bus.\n");
	// } 
}
