/*=============================================================================
 * Copyright (c) 2019, Miguel del Valle <m.e.delvallecamino@ieee.org>
 * All rights reserved.
 * License: bsd-3-clause (see LICENSE.txt)
 * Date: 2019/08/23
 * Version: rev0
 *===========================================================================*/
/*!
 *  @brief Example using sAPI, shows basic setup of sensor,
 *  	includes initialization of the interface and
 *      performing the sensor initialization.
 */
/*=====[Inclusions of function dependencies]=================================*/

#include "bmp280.h"
#include "sapi.h"
/*=====[Definition macros of private constants]==============================*/

/*=====[Definitions of extern global variables]==============================*/

/*=====[Definitions of public global variables]==============================*/

/*=====[Definitions of private global variables]=============================*/

void delay_ms(uint32_t period_ms);
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data,
		uint16_t length);
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data,
		uint16_t length);
int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data,
		uint16_t length);
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data,
		uint16_t length);
void print_rslt(const char api_name[], int8_t rslt);

/*=====[Main function, program entry point after power on or reset]==========*/

int main(void) {
	int8_t rslt;
	struct bmp280_dev bmp;
	struct bmp280_config conf;
	struct bmp280_uncomp_data ucomp_data;
	uint32_t pres32, pres64;
	int32_t temp32;
	double temp;
	double pres;
	static char uartBuff[10];

	/* Map the delay function pointer with the function responsible for implementing the delay */
	bmp.delay_ms = delay_ms;

	/* Assign device I2C address based on the status of SDO pin (GND for PRIMARY(0x76) & VDD for SECONDARY(0x77)) */
	bmp.dev_id = BMP280_I2C_ADDR_PRIM;

	/* Select the interface mode as I2C */
	bmp.intf = BMP280_I2C_INTF;

	/* Map the I2C read & write function pointer with the functions responsible for I2C bus transfer */
	bmp.read = i2c_reg_read;
	bmp.write = i2c_reg_write;

	/* To enable SPI interface: comment the above 4 lines and uncomment the below 4 lines */

	/*
	 * bmp.dev_id = 0;
	 * bmp.read = spi_reg_read;
	 * bmp.write = spi_reg_write;
	 * bmp.intf = BMP280_SPI_INTF;
	 */
	// ----- Setup -----------------------------------
	boardInit();
	uartConfig(UART_USB, 9600); // Inicializar periferico UART_USB
	i2cInit(I2C0, BME280_I2C_RATE);

	// ----- Init sensor -----------------------------------
	rslt = bmp280_init(&bmp);

	print_rslt(" bmp280_init status", rslt);

	/* Always read the current settings before writing, especially when
	 * all the configuration is not modified
	 */
	rslt = bmp280_get_config(&conf, &bmp);
	print_rslt(" bmp280_get_config status", rslt);

	/* configuring the temperature oversampling, filter coefficient and output data rate */
	/* Overwrite the desired settings */
	conf.filter = BMP280_FILTER_COEFF_2;

	/* Temperature oversampling set at 4x */
	conf.os_temp = BMP280_OS_4X;

	/* Pressure over sampling none (disabling pressure measurement) */
	conf.os_pres = BMP280_OS_4X;

	/* Setting the output data rate as 1HZ(1000ms) */
	conf.odr = BMP280_ODR_1000_MS;
	rslt = bmp280_set_config(&conf, &bmp);
	print_rslt(" bmp280_set_config status", rslt);

	/* Always set the power mode after setting the configuration */
	rslt = bmp280_set_power_mode(BMP280_NORMAL_MODE, &bmp);
	print_rslt(" bmp280_set_power_mode status", rslt);
	// ----- Repeat for ever -------------------------
	while (true) {
		/* Reading the raw data from sensor */
		rslt = bmp280_get_uncomp_data(&ucomp_data, &bmp);

		/* Getting the 32 bit compensated temperature */
		rslt = bmp280_get_comp_temp_32bit(&temp32, ucomp_data.uncomp_temp,
				&bmp);

		/* Getting the compensated pressure using 32 bit precision */
		rslt = bmp280_get_comp_pres_32bit(&pres32, ucomp_data.uncomp_press,
				&bmp);

		/* Getting the compensated pressure using 64 bit precision */
		rslt = bmp280_get_comp_pres_64bit(&pres64, ucomp_data.uncomp_press,
				&bmp);

		/* Getting the compensated temperature as floating point value */
		rslt = bmp280_get_comp_temp_double(&temp, ucomp_data.uncomp_temp, &bmp);
		/*printf("UT: %ld, T32: %ld, T: %f \r\n", ucomp_data.uncomp_temp, temp32,
		 temp);*/

		uartWriteString(UART_USB, "Temperature: ");
		floatToString(temp, uartBuff, 2);
		uartWriteString(UART_USB, uartBuff);
		uartWriteString(UART_USB, "\rGrados C");
		uartWriteString(UART_USB, "\r\n");

		uartWriteString(UART_USB, "Pressure: ");
		floatToString(pres, uartBuff, 2);
		uartWriteString(UART_USB, uartBuff);
		uartWriteString(UART_USB, "\rPa");
		uartWriteString(UART_USB, "\r\n");

		/* Getting the compensated pressure as floating point value */
		rslt = bmp280_get_comp_pres_double(&pres, ucomp_data.uncomp_press,
				&bmp);
		/*printf("UP: %ld, P32: %ld, P64: %ld, P64N: %ld, P: %f\r\n",
		 ucomp_data.uncomp_press,
		 pres32,
		 pres64,
		 pres64 / 256,
		 pres);*/

		/* Sleep time between measurements = BMP280_ODR_1000_MS */
		bmp.delay_ms(5000);
		gpioToggle(LED);
	}

	// YOU NEVER REACH HERE, because this program runs directly or on a
	// microcontroller and is not called by any Operating System, as in the
	// case of a PC program.
	return 0;
}

/*!
 *  @brief Function that creates a mandatory delay required in some of the APIs such as "bmg250_soft_reset",
 *      "bmg250_set_foc", "bmg250_perform_self_test"  and so on.
 *
 *  @param[in] period_ms  : the required wait time in milliseconds.
 *  @return void.
 *
 */
void delay_ms(uint32_t period_ms) {
	delay(period_ms);
	/* Implement the delay routine according to the target machine */
}

/*!
 *  @brief Function for writing the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose value is to be written.
 *  @param[in] length   : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_write(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data,
		uint16_t length) {

	/* Implement the I2C write routine according to the target machine. */
	int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */
	uint8_t nuevoLen = 2 * length;
	uint8_t transmitDataBuffer[nuevoLen];
	uint8_t i = 0;

	for (i = 0; i < length; i += 2) {
		transmitDataBuffer[i] = reg_addr + i;
		transmitDataBuffer[i + 1] = reg_data[i];
	}

	/*
	 * The parameter dev_id can be used as a variable to store the I2C address of the device
	 */

	/*
	 * Data on the bus should be like
	 * |------------+----------------------|
	 * | I2C action | Data                 |
	 * |------------+----------------------|
	 * | Start      | -                    |
	 * | Write      | (reg_addr)           |
	 * | Write      | (reg_data[0])        |
	 * | Write      | (reg_addr + 1)       |
	 * | Write      | (reg_data[1])        |
	 * | Write      | (....)               |
	 * | Write      | (reg_addr + len - 1) |
	 * | Write      | (reg_data[len - 1])  |
	 * | Stop       | -                    |
	 * |------------+----------------------|
	 */

	if (i2cWrite(I2C0, i2c_addr, transmitDataBuffer, nuevoLen, TRUE)) {
		rslt = BMP280_OK;
	} else {
		rslt = 5;	//@retval >0 -> Failure Info
	}

	return rslt;
	//return -1;
}

/*!
 *  @brief Function for reading the sensor's registers through I2C bus.
 *
 *  @param[in] i2c_addr : Sensor I2C address.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t i2c_reg_read(uint8_t i2c_addr, uint8_t reg_addr, uint8_t *reg_data,
		uint16_t length) {

	/* Implement the I2C read routine according to the target machine. */
	int8_t rslt = 0; /* Return 0 for Success, non-zero for failure */

	/*
	 * Data on the bus should be like
	 * |------------+---------------------|
	 * | I2C action | Data                |
	 * |------------+---------------------|
	 * | Start      | -                   |
	 * | Write      | (reg_addr)          |
	 * | Stop       | -                   |
	 * | Start      | -                   |
	 * | Read       | (reg_data[0])       |
	 * | Read       | (....)              |
	 * | Read       | (reg_data[len - 1]) |
	 * | Stop       | -                   |
	 * |------------+---------------------|
	 */

	if (i2cRead(I2C0, i2c_addr, &reg_addr, 1, TRUE, reg_data, length, TRUE)) {
		rslt = BMP280_OK;
	} else {
		rslt = 5; //
	}

	return rslt; //@retval >0 -> Failure Info
	//	return -1;
}

/*!
 *  @brief Function for writing the sensor's registers through SPI bus.
 *
 *  @param[in] cs           : Chip select to enable the sensor.
 *  @param[in] reg_addr     : Register address.
 *  @param[in] reg_data : Pointer to the data buffer whose data has to be written.
 *  @param[in] length       : No of bytes to write.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t spi_reg_write(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data,
		uint16_t length) {

	/* Implement the SPI write routine according to the target machine. */
	return -1;
}

/*!
 *  @brief Function for reading the sensor's registers through SPI bus.
 *
 *  @param[in] cs       : Chip select to enable the sensor.
 *  @param[in] reg_addr : Register address.
 *  @param[out] reg_data    : Pointer to the data buffer to store the read data.
 *  @param[in] length   : No of bytes to read.
 *
 *  @return Status of execution
 *  @retval 0 -> Success
 *  @retval >0 -> Failure Info
 *
 */
int8_t spi_reg_read(uint8_t cs, uint8_t reg_addr, uint8_t *reg_data,
		uint16_t length) {

	/* Implement the SPI read routine according to the target machine. */
	return -1;
}

/*!
 *  @brief Prints the execution status of the APIs.
 *
 *  @param[in] api_name : name of the API whose execution status has to be printed.
 *  @param[in] rslt     : error code returned by the API whose execution status has to be printed.
 *
 *  @return void.
 */
void print_rslt(const char api_name[], int8_t rslt) {
	if (rslt != BMP280_OK) {
		printf("%s\t", api_name);
		if (rslt == BMP280_E_NULL_PTR) {
			printf("Error [%d] : Null pointer error\r\n", rslt);
		} else if (rslt == BMP280_E_COMM_FAIL) {
			printf("Error [%d] : Bus communication failed\r\n", rslt);
		} else if (rslt == BMP280_E_IMPLAUS_TEMP) {
			printf("Error [%d] : Invalid Temperature\r\n", rslt);
		} else if (rslt == BMP280_E_DEV_NOT_FOUND) {
			printf("Error [%d] : Device not found\r\n", rslt);
		} else {
			/* For more error codes refer "sapi_bmp280.h" */
			printf("Error [%d] : Unknown error code\r\n", rslt);
		}
	} else {
		/* For more error codes refer "sapi_bmp280.h" */
		printf("BMP280_OK code[%d]\r\n", rslt);
	}
}

