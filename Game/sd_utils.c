#include "sd_utils.h"

#include "fatfs_sd.h"

FATFS fs;
FATFS *pfs;
FIL fil;
FRESULT fres;
DWORD fre_clust;
uint32_t totalSpace, freeSpace;

char file_data_buffer[100];

void transmit_uart(char *string) {
	uint8_t len = strlen(string);
	HAL_UART_Transmit(&huart2, (uint8_t*) string, len, 200);
}
void list_files_on_sd(void) {
	DIR dir;
	FILINFO fno;
	FRESULT res;

	res = f_opendir(&dir, "/"); // Abrir directorio raíz
	if (res == FR_OK) {
		transmit_uart("Archivos en la SD:\n");
		while (1) {
			res = f_readdir(&dir, &fno); // Leer archivo/directorio
			if (res != FR_OK || fno.fname[0] == 0)
				break;
			transmit_uart(fno.fname); // Enviar nombre por UART
			transmit_uart("\n");
		}
		f_closedir(&dir);
	} else {
		transmit_uart("Error al abrir el directorio\n");
	}
}

void erase_file(void) {
	FRESULT res;

	res = f_unlink("ascii-art.txt");
	if (res == FR_OK) {
		transmit_uart("Archivo eliminado correctamente\n");
	} else {
		transmit_uart("Error al eliminar el archivo\n");
	}
}
void SD_readLine(void) {
	while (f_gets(file_data_buffer, sizeof(file_data_buffer), &fil)) {
		char mRd[100];
		sprintf(mRd, "%s", file_data_buffer);
		transmit_uart(mRd);
	}
}
FRESULT SD_close(void) {
	FRESULT fres = f_close(&fil);
	if (fres == FR_OK) {
		transmit_uart("The file is closed\n");
	} else {
		transmit_uart("The file was not closed\n");
	}
	return fres;
}
FRESULT SD_open(const char *filename, BYTE mode) {
	FRESULT fres = f_open(&fil, filename, mode);
	if (fres == FR_OK) {
		transmit_uart("File opened successfully\n");
	} else {
		transmit_uart("File was not opened\n");
	}
	return fres;
}
FRESULT SD_unmount(void) {
	FRESULT fres = f_mount(NULL, "", 1);
	if (fres == FR_OK) {
		transmit_uart("The Micro SD card is unmounted\n");
	} else {
		transmit_uart("The Micro SD was not unmounted\n");
	}
	return fres;
}
FRESULT SD_mount(void) {
	FRESULT fres = f_mount(&fs, "", 0);
	if (fres == FR_OK) {
		transmit_uart("Micro SD card is mounted successfully!\n");
	} else {
		transmit_uart("Micro SD card's mount error!\n");
	}
	return fres;
}
FRESULT SD_createFile(const char *filename) {
	FRESULT fres = f_open(&fil, filename, FA_OPEN_APPEND | FA_WRITE | FA_READ);
	if (fres == FR_OK) {
		transmit_uart("File opened for writing/appending\n");
	} else {
		transmit_uart("File was not opened for writing/appending\n");
	}
	return fres;
}
FRESULT SD_write(const char *text) {
	if (f_puts(text, &fil) > 0) {
		transmit_uart("Text written successfully\n");
		return FR_OK;
	} else {
		transmit_uart("Failed to write text\n");
		return FR_DISK_ERR;
	}
}

void SD_readImageBin(const char *filename, uint8_t *dest, uint32_t size) {
	UINT br;

	if (SD_mount() == FR_OK) {
		if (SD_open(filename, FA_READ) == FR_OK) {
			f_read(&fil, dest, size, &br);
			f_close(&fil);
		}
		SD_unmount();
	}
}

void LCD_StreamBitmapFromSD(const char *filename, uint16_t x, uint16_t y,
		uint16_t width, uint16_t height) {
	uint8_t fila_buffer[640]; // 320 píxeles * 2 bytes por píxel
	UINT br;

	if (SD_mount() == FR_OK) {
		if (SD_open(filename, FA_READ) == FR_OK) {

			for (int j = 0; j < height; j++) {
				f_read(&fil, fila_buffer, width * 2, &br);  // Leer una fila completa

				SetWindows(x, y + j, x + width - 1, y + j);  // Solo una fila
				LCD_CMD(0x2C); // Comando de escritura

				HAL_GPIO_WritePin(LCD_RS_GPIO_Port, LCD_RS_Pin, GPIO_PIN_SET);
				HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_RESET);

				for (int i = 0; i < width * 2; i += 2) {
					LCD_DATA(fila_buffer[i]);     // LSB
					LCD_DATA(fila_buffer[i + 1]); // MSB
				}

				HAL_GPIO_WritePin(LCD_CS_GPIO_Port, LCD_CS_Pin, GPIO_PIN_SET);
			}

			f_close(&fil);
		}
		SD_unmount();
	}
}
