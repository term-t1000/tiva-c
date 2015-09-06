#ifndef SPI_DATAFLASH_H
#define SPI_DATAFLASH_H

#include <stdint.h>


#define SPI_FLASH_OP_BLOCK_SIZE 264
#define SPI_FLASH_MAX_ADDRESS 2048


#define AT45D_BUFFER1_WRITE_CMD 0x84
#define AT45D_MMPAGE_TO_BUFFER1_CMD 0x53
#define AT45D_BUFFER1_READ 0xD4

// Buffer to Main Memory Page Program with Built-In Erase
#define AT45D_BUFFER1_TO_MMPAGE 0x83
// Status Register Read
#define AT45D_STATUS_REGISTER_READ 0xD7

#define SPI_DATAFLASH_BUF_MAX_SIZE 300
struct spi_flash_ctl {
    uint8_t buf[SPI_DATAFLASH_BUF_MAX_SIZE];
    // receive buffer should have at least the same size as number of bytes to send + to receive
    uint8_t *rbuf;
    int in_progress;
    int cmd;
    uint32_t pageid;
    int snd_sz;
    int rcv_sz;
    int total_sz;
    int rcv_err_cnt;
    int crc_err;
    int ignore_crc_check;
    int status_busy_wait;
} extern volatile flash_ctl;


int at45d_read_status_reg(int busy_wait);


#endif

