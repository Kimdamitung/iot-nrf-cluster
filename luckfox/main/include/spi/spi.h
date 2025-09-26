#ifndef _SPI_H_
#define _SPI_H_ 

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>

typedef enum {
    SPI0_CS1_M0 = 58,
    SPI0_CS0_M0 = 48,
    SPI0_CLK_M0 = 49,
    SPI0_MOSI_M0 = 50,
    SPI0_MISO_M0 = 51,
}spi_pin_t;

typedef enum {
    SPI0_CS0 = 0,
    SPI0_CS1,
}spi_number_t;

typedef struct {
    int (*spi_init)(spi_number_t number, uint8_t mode, uint8_t word, unsigned int speed);
    int (*spi_readinto)(uint8_t *buffer, size_t length);
    int (*spi_write)(const uint8_t *buffer, size_t length);
    int (*spi_write_readinto)(const uint8_t *tx_buffer, uint8_t *rx_buffer, size_t length);
    void (*spi_deinit)(void);
}spi_t;

extern spi_t SPI0;

#endif // _SPI_H_