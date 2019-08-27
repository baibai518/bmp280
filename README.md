# bme280
GY-BME/P 280 3.3 &amp; EDU CIAA (sapi)



a) Listado de archivos

	1) GY-BMP280-3.3_BG.pdf -> Presentacion del producto, ver punto b)
	2) BST-BMP280-DS001.pdf
	3) BST-BME280-DS001.pdf
	4) SRC e INC: Biblioteca del fabricante (BMP 280), adaptada a la sAPI, pero con Chip_ID 0x60 que corresponde al BME280. 

b) Bosch Sensortec BMP280 vs BME280 Sensor Confusion



b) Note: Posted on August 5, 2018 by lui_gough (extract)

(https://goughlui.com/2018/08/05/note-bosch-sensortec-bmp280-vs-bme280-sensor-confusion/)

Bosch Sensortec BME280 is a digital I2C sensor that can operate at 1.8 or 3.3V, providing temperature, humidity and barometric pressure measurements with high accuracy, low energy consumption and at low cost. This makes it ideal for weather stations, temperature monitoring for comfort, etc.

The BMP280 differs from the BME280 in lacking humidity measurement and having a different ID. This note help others in case they have been supplied incorrect products and how to diagnose it.

