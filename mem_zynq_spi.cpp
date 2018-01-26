#include "mem_zynq_spi.h"

#include <byteswap.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <stdlib.h>
#include <sys/mman.h>
#include <arpa/inet.h>

#define SPIDEV               "/dev/spidev32766.0"


FpgaIF::FpgaIF() {
  // open spidev
  g_spi_fd = open(SPIDEV, O_RDWR);
  if (g_spi_fd <= 0) {
    perror("Device not found\n");
    exit(1);
  }

  printf("FPGA SPI device opened!\n");
}

FpgaIF::~FpgaIF() {
  close(g_spi_fd);
}

bool
FpgaIF::mem_write(uint32_t addr, uint8_t be, uint32_t wdata) {
  char wr_buf[9];

  wr_buf[0] = 0x02; // write command
  // address
  wr_buf[1] = addr >> 24;
  wr_buf[2] = addr >> 16;
  wr_buf[3] = addr >>  8;
  wr_buf[4] = addr >>  0;
  wr_buf[5] = wdata >> 24;
  wr_buf[6] = wdata >> 16;
  wr_buf[7] = wdata >>  8;
  wr_buf[8] = wdata >>  0;

  // write to spidev
  if (write(g_spi_fd, wr_buf, 9) != 9) {
    perror("Write Error");

    return false;
  }

  return true;
}

bool
FpgaIF::mem_read(uint32_t addr, uint32_t *rdata) {
  char wr_buf[256];
  char rd_buf[256];
  int i;

  struct spi_ioc_transfer* transfer = (struct spi_ioc_transfer*)malloc(sizeof(struct spi_ioc_transfer));

  memset(transfer, 0, sizeof(struct spi_ioc_transfer));

  transfer->tx_buf = (unsigned long)wr_buf;
  transfer->rx_buf = (unsigned long)rd_buf;
  transfer->len    = 14;

  memset(wr_buf, 0, transfer->len);
  memset(rd_buf, 0, transfer->len);

  wr_buf[0] = 0x0B; // read command
  // address
  wr_buf[1] = addr >> 24;
  wr_buf[2] = addr >> 16;
  wr_buf[3] = addr >> 8;
  wr_buf[4] = addr;

  // check if write was sucessful
  if (ioctl(g_spi_fd, SPI_IOC_MESSAGE(1), transfer) < 0) {
    perror("SPI_IOC_MESSAGE");
    return false;
  }

  // shift everything by one bit
  for(i = 0; i < transfer->len-1; i++) {
    rd_buf[i] = (rd_buf[i] << 1) | ((rd_buf[i+1] & 0x80) >> 7);
  }

  *rdata = htonl(*((int*)&rd_buf[9]));

  return true;
}

bool
FpgaIF::access(bool write, unsigned int addr, int size, char* buffer) {
  bool retval = true;
  uint32_t rdata;
  uint32_t wdata;
  uint8_t * ptr;
  uint8_t be;
  unsigned int t_addr;

  if (write) {
    // write
    
    /* BE is seems not implemented  in SPI cmds*/
    /* So we readback ~Align(4) data to writeback*/
    
    /* only write 1B for address 1.3*/
    ptr = (uint8_t *) &wdata;
    if (addr&0x03)
        this->mem_read(addr&(~0x03), &wdata);
        
    while(t_addr=(addr&0x03)) {
        
        printf("[e]addr =%x , v=%x", addr, *buffer);
        
        if (t_addr!=0x02){
            /* 1 or 3*/
         be = 1 << t_addr;   
         ptr[t_addr] = *((uint8_t *)buffer); 
         printf("odd write be=%x", be);
         retval = retval && this->mem_write(addr, be, wdata);
         addr   += 1;
         size   -= 1;
         buffer += 1;
        }
        if (t_addr==2)
        {
          if (size==1){
            be=0x04;
            ptr[2] = *((uint8_t *)buffer);
            retval = retval && this->mem_write(addr, be, wdata);
            size -= 1;
            addr += 1;
            buffer += 1;
          }
          else if(size>1){
            be=12;
            size -= 2;
            ptr[2] = *((uint8_t *)buffer);
            buffer++;
            ptr[3] = *((uint8_t *)buffer);
            retval = retval && this->mem_write(addr, be, wdata);
            addr += 2;
            buffer ++;            
          }
          else break;
        }
    }
             
        

    while (size >= 4) {
      retval = retval && this->mem_write(addr, 0xF, *((uint32_t*)buffer));
      addr   += 4;
      size   -= 4;
      buffer += 4;
    }
    if (size >0) {
        this->mem_read(addr&(~0x03), &wdata);
        memcpy(ptr, buffer, size);
        be = (1<<size) -1; //useless, but leave it here.
        retval = retval && this->mem_write(addr, be, wdata);
        printf("[2]addr =%x , v=%x", addr, wdata);
   }
  } else {
    // read
    //printf("read %d bytes\n", size);
    // bytes
    if (addr & 0x1) {
      be = addr&0x03; 
      
      retval = retval && this->mem_read(addr, &rdata);
    buffer[0] = ((char *)&rdata)[be];
      addr   += 1;
      size   -= 1;
      buffer += 1;
    }
     
    // half-words
    if (addr & 0x2) {
      retval = retval && this->mem_read((addr&~0x3), &rdata);
      be = addr&0x03;
      buffer[0] = ((char*)&rdata)[be];
      buffer[1] = ((char*)&rdata)[be+1];
      addr   += 2;
      size   -= 2;
      buffer += 2;
    }
    /* now aligned address */
    while (size >= 4) {
      retval = retval && this->mem_read(addr, &rdata);
      buffer[0] = rdata;
      buffer[1] = rdata >> 8;
      buffer[2] = rdata >> 16;
      buffer[3] = rdata >> 24;
      addr   += 4;
      size   -= 4;
      buffer += 4;
    }
    
     /* original code not correct, since here we should only
      * have 1-3B read on aligned address */
    // half-words
    retval = retval && this->mem_read(addr, &rdata);
    if (size<4)
     memcpy(buffer, (char *) &rdata, size);
    else
        printf("Error Condition on mem read\n");
#if 0    
    if (addr & 0x2) {
      retval = retval && this->mem_read(addr, &rdata);
      buffer[0] = rdata;
      buffer[1] = rdata >> 8;
      addr   += 2;
      size   -= 2;
      buffer += 2;
    }

    // bytes
    if (addr & 0x1) {
      retval = retval && this->mem_read(addr, &rdata);
      buffer[0] = rdata;
      addr   += 1;
      size   -= 1;
      buffer += 1;
    }
#endif     
  }

  return retval;
}
