/*
 * main.c
 *
 *  Created on: Jan 30, 2017
 *      Author: vldmr
 *
 *  Note: This application upload FPGAs connected by SPI to Zynq 7010. GPIO from PS side by EMIO.
 */

#include <stdio.h>				// fprintf
#include <stdlib.h>
#include <stdint.h>			    // uint*
#include <string.h>				// strerror
#include <errno.h>				// errno
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <sys/utsname.h>		// uname
#include <sys/stat.h>			// stat
#include <linux/spi/spidev.h>

#define SUCCESS 				0
#define FAILURE 			   -1
#define DONE 		   			1
#define FPGA_A					0
#define FPGA_B					1
#define BUFF_LENGTH 		  256
#define SPI_BUFSIZ			131072   // is in /sys/module/spidev/parameters/bufsiz, responsible for max size of data transfer

/*
 * TODO:
 *  1 - Check version of kernel and settings GPIO_BASE
 *  2 - read bufsiz from module and set this value
 *  3 - read random number of spidev drivers
 */


// GPIO Pins
//#define GPIO_BASE		138 // For Linux kernel 3.17.0
#define GPIO_BASE		906 // For Linux kernel 4.6.0
#define A_PROG_B 		( 54 + GPIO_BASE )	// OUT; from 1 to 0
#define B_PROG_B 		( 55 + GPIO_BASE )	// OUT; from 1 to 0
#define A_DONE   		( 56 + GPIO_BASE )	// IN; from 0 to 1
#define B_DONE	 		( 57 + GPIO_BASE )	// IN; from 0 to 1
#define A_INIT_B 		( 58 + GPIO_BASE )	// IN; from 1 to 0
#define B_INIT_B 		( 59 + GPIO_BASE )	// IN; from 1 to 0
#define PIN_DIRECTION	( 60 + GPIO_BASE )	// OUT

typedef unsigned long u32;
typedef struct spi_ioc_transfer spi_transfer;

static const char *device = "/dev/spidev1.0";
static const char *dev_bufsiz = "/sys/module/spidev/parameters/bufsiz";		// now don't use
static const char *bit_file_a = "a.bit";
static const char *bit_file_b = "b.bit";
static const char LOW  = '0';
static const char HIGH = '1';

static int SPI_Send(int spi_handle, uint8_t *data_tx, uint8_t *data_rx, size_t length);
static int SPI_Init(int spi_handle);
static int fpga_load(int spi_handle);
static int GPIO_Read(int gpio_pin);
static int GPIO_Write(int gpio_pin, char value);
static int check_complete(int FPGA);

int main(void){
	int status = 0;

	int spi_handle = open(device, O_RDWR);
	if( spi_handle < 0 ){
		fprintf(stderr, "Couldn't connect to your SPI: %s\r\n", strerror(errno));
		return FAILURE;
	}

	status = SPI_Init(spi_handle);
	if( status == FAILURE ){
		printf("Bad init\r\n");
		close(spi_handle);
		exit(FAILURE);
	}

	status = fpga_load(spi_handle);
	if( status == FAILURE ){
		printf("Bad load fpga\r\n");
		close(spi_handle);
		exit(FAILURE);
	}

	close(spi_handle);
	return SUCCESS;
}

/**
 * @brief  Flashing FPGAs via SPI with GPIO
 *
 * @param  spi_handle - descriptor for /dev/spidev*
 *
 * @return SUCCESS - if all be OK and FPGAs were uploaded
 * 		   FAILURE - if all be bad and FPGA were not uploaded
 */
static int fpga_load(int spi_handle){
	u32 status = 0;
	struct stat file_a;

	// Upload Kintex A
	stat(bit_file_a, &file_a);
	u32 FileSize = file_a.st_size;	// get size of firmware file
	uint8_t *DestinationAddress = (uint8_t*) calloc( file_a.st_size, sizeof(uint8_t));

	int file_descr = open(bit_file_a, O_RDONLY);
	if( file_descr < 0 ){
		fprintf(stderr, "<Error> Couldn't open bitstream file: %s\r\n", strerror(errno));
		return FAILURE;
	}

	// Pointer to beginning of file .
	status = lseek(file_descr, 0, SEEK_SET);
	if( status ) {
		fprintf(stderr, "<Error> Couldn't set pointer to beginning of file: %s\r\n", strerror(errno));
		close(file_descr);
		return errno;
	}
	// Read data from file.
	status = read(file_descr, /*(void*)*/ DestinationAddress, FileSize);
	if( !status ) {
		fprintf(stderr, "<Error> Couldn't read file a.bit: %s\n", strerror(errno));
		close(file_descr);
		return errno;
	}
	close(file_descr);
	printf("Read %lu bytes from %s\r\n", status, bit_file_a);

	// Reset FPGAs by setting pin PROGB low (UG 470 (page 52)) and then up
	GPIO_Write(A_PROG_B, LOW);
	GPIO_Write(B_PROG_B, LOW);
	printf("Reset FPGAs\n\r");
	GPIO_Write(A_PROG_B, HIGH);
	GPIO_Write(B_PROG_B, HIGH);

	GPIO_Write(PIN_DIRECTION, HIGH); 		// Set direction (Kintex A (1); Kintex B (0))
	printf("Loading configuration file %s\n\r", bit_file_a);

	while( GPIO_Read(A_INIT_B) != DONE ){}	// wait while signal INIT_B doesn't up
	status = SPI_Send(spi_handle, DestinationAddress, NULL, FileSize);
	if( status == FAILURE ){
		fprintf(stderr, "Couldn't write data into spi device: %s\r\n", strerror(errno));
		return status;
	}

	/*
	 * Depending on the direction, check the status of busy:
	 * 	A - read GPIO (56)
	 * 							  != DONE ( == 1 )
	 * 	B - read GPIO (57)
	 */
	while( check_complete(FPGA_A) != DONE );

	// Upload Kintex B
	struct stat file_b;
	stat(bit_file_b, &file_b);
	FileSize = file_b.st_size;
	file_descr = open(bit_file_b, O_RDONLY);
	if( file_descr < 0 ){
		fprintf(stderr, "Couldn't open bitstream file: %s\r\n", strerror(errno));
		return FAILURE;
	}

	status = lseek(file_descr, 0, SEEK_SET);
	if( status ) {
		fprintf(stderr, "Couldn't set pointer to beginning of file: %s\r\n", strerror(errno));
		close(file_descr);
		return errno;
	}

	status = read(file_descr, /*(void*)*/ DestinationAddress, FileSize);
	if( !status ) {
		printf("Error reading file b.bit: %d\n", errno);
		close(file_descr);
		return errno;
	}
	close(file_descr);
	printf("Read %lu bytes from %s\r\n", status, bit_file_b);

	GPIO_Write(PIN_DIRECTION, LOW);
	printf("Loading configuration file %s\n\r", bit_file_b);

	status = SPI_Send(spi_handle, DestinationAddress, NULL, FileSize);
	if( status == FAILURE ){
		fprintf(stderr, "Couldn't write data into spi device: %s\r\n", strerror(errno));
		return status;
	}

	while( check_complete(FPGA_B) != DONE ){
		//printf(".");
	}

	free(DestinationAddress);
	return SUCCESS;
}

/**
 * @brief  Check bit of DONE, and return this;
 *
 * @param  FPGA - direction of Kintex; 0 - Kintex A, 1 - Kintex B;
 *
 * @return result - contain bit of DONE;
 */
static int check_complete(int FPGA){
	int result = 0;
	if( FPGA == FPGA_A )
		result = GPIO_Read(A_DONE);
	else if( FPGA == FPGA_B )
		result = GPIO_Read(B_DONE);

	return result;
}

/**
 * @brief  Read GPIO value from gpio_pin;
 *
 * @param  gpio_pin - the No. of GPIO pin;
 *
 * @return result - contain value (0/1) from GPIO;
 * 		   FAILURE - if something goes wrong, return -1;
 *
 * @note   Thus we read value from sysfs, we get ASCII (character),
 * 				and for more easy work in future we convert this in numeric;
 */
static int GPIO_Read(int gpio_pin){
	char value;
	int result = 0;
	int status = 0;
	int fd = 0;
	char gpio_val[80];

	snprintf(gpio_val, sizeof(gpio_val), "/sys/class/gpio/gpio%d/value", gpio_pin);
	fd = open(gpio_val, O_RDONLY);
	if( fd < 0 ){
		fprintf(stderr, "Couldn't open GPIO: %s\r\n", strerror(errno));
		return FAILURE;
	}

	status = lseek(fd, 0, SEEK_SET);
	if( status < 0 ){
		fprintf(stderr, "Couldn't read GPIO value: %s\r\n", strerror(errno));
		return FAILURE;
	}

	status = read(fd, &value, sizeof(value));
	if( status < 0 ){
		fprintf(stderr, "Couldn't read GPIO value: %s\r\n", strerror(errno));
		return FAILURE;
	}

	result = atoi(&value);
	return result;
}

/**
 * @brief  Write GPIO value into gpio_pin
 *
 * @param  gpio_pin - the No. of GPIO pin
 * 		   value    - a character synonym of 0/1 (LOW/HIGH)
 *
 * @return status - contain number of written bytes;
 * 		   FAILURE - if something goes wrong, return -1;
 */
static int GPIO_Write(int gpio_pin, char value){
	int fd = 0;
	int status = 0;
	char gpio_val[80];

	snprintf(gpio_val, sizeof(gpio_val), "/sys/class/gpio/gpio%d/value", gpio_pin);
	fd = open(gpio_val, O_WRONLY);
	if( fd < 0 ){
		fprintf(stderr, "Couldn't open GPIO: %s\r\n", strerror(errno));
		return FAILURE;
	}

	status = write(fd, &value, sizeof(value));
	if( status < sizeof(value) ){
		fprintf(stderr, "Couldn't write GPIO value: %s\r\n", strerror(errno));
		return FAILURE;
	}

	return status;
}

/**
 * @brief  SPI initialization;
 *
 * @param  spi_handle - handle (descriptor) of /dev/spidev;
 *
 * @return status - result of ioctl operation;
 */
static int SPI_Init(int spi_handle){
	int status = 0;
	uint8_t mode = SPI_MODE_0;
	uint8_t bits = 8;
	uint8_t lsb_msb = 1;		 	// 0 - MSB; other - LSB
	uint32_t speed = 0;  			// Hz

	status = ioctl(spi_handle, SPI_IOC_WR_MODE, &mode);							// set SPI mode (e.g. CPOL|CPHA)
	if( status == FAILURE )
		fprintf(stderr, "Couldn't write spi mode: %s\r\n", strerror(errno));

	status = ioctl(spi_handle, SPI_IOC_RD_MODE, &mode);							// check SPI mode
	if( status == FAILURE )
			fprintf(stderr, "Couldn't read spi mode: %s\r\n", strerror(errno));

	status = ioctl(spi_handle, SPI_IOC_RD_MAX_SPEED_HZ, &speed);				// read SPI speed (in Hz)
	if( status == FAILURE )
		fprintf(stderr, "Couldn't read spi speed: %s\r\n", strerror(errno));

	if( speed > 50000000 ){
		speed = 50000000;   // (50 MHz)
		status = ioctl(spi_handle, SPI_IOC_WR_MAX_SPEED_HZ, &speed);			// set SPI speed
		if( status == FAILURE )
				fprintf(stderr, "Couldn't write spi speed: %s\r\n", strerror(errno));
	}

	status = ioctl(spi_handle, SPI_IOC_WR_BITS_PER_WORD, &bits);				// set bits per word.
	if( status == FAILURE )														// In Linux driver of spi (in Zynq) you can set maximum of 8 bits, no more
		fprintf(stderr, "Couldn't write spi number of bits in transfer word: %s\r\n", strerror(errno));

	status = ioctl(spi_handle, SPI_IOC_RD_BITS_PER_WORD, &bits);				// check bits per word
	if( status == FAILURE )
		fprintf(stderr, "Couldn't read spi number of bits in transfer word: %s\r\n", strerror(errno));

	status = ioctl(spi_handle, SPI_IOC_RD_LSB_FIRST, &lsb_msb);					// read LSB or MSB mode
	if( status == FAILURE )
		fprintf(stderr, "Couldn't read spi the bit justification: %s\r\n", strerror(errno));

	/*
	printf("SPI Mode: %x \r\n", mode);
	printf("SPI Speed: %d Hz ( %d MHz )\r\n", speed, speed / 1000000);
	printf("SPI Bits per word: %d\r\n", bits);
	printf("SPI First bits is %s ( %d ) \r\n", lsb_msb > 0 ? "LSB" : "MSB", lsb_msb );
*/
	return status;
}

/**
 * @brief  SPI transfer data;
 *
 * @param  spi_handle - handle (descriptor) of /dev/spidev;
 * 		   data_tx    - data for send;
 * 		   data_rx	  - data for receive;
 * 		   length	  - size of transfer data;
 *
 * @return status - result of ioctl operation - size of sending bytes;
 *
 * @note   In one way - doesn't read in this example, but if change _FIXME row
 * 				you can also get data;
 */
static int SPI_Send(int spi_handle, uint8_t *data_tx, uint8_t *data_rx, __u32 length){
	int count = 0;
	int lost = 0;
	int status = 0;

	// We check our size of buffer (bufsiz) of spidev, and send by packets of determined length
	if( length > SPI_BUFSIZ ){
		count = length / SPI_BUFSIZ;
		if( length % SPI_BUFSIZ != 0 ){
			length = length % SPI_BUFSIZ;
			lost = 1;
			count++;
		}
	}

	spi_transfer *trans = (spi_transfer*) calloc( count, sizeof(spi_transfer));
	int i;
	for( i = 0; i < count; i++ ){
		trans[i].tx_buf = (unsigned long)( data_tx + i * SPI_BUFSIZ );
		trans[i].rx_buf = 0; //(unsigned long)( data_rx + i );  FIXME: you can fix this for receive data
		trans[i].len    = SPI_BUFSIZ;
		if( lost && ( ( ( count - i ) == 1 ) ) ) // if we have the residue, send this by residue size in last byte
			trans[i].len = length;
		status = ioctl(spi_handle, SPI_IOC_MESSAGE(1), &trans[i]);
		if( status < 0 )
			break;
	}

	return status;
}

