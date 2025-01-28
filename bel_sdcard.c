// license:BSD-3-Clause
// copyright-holders:R. Belmont
/*
    SD Card emulation, SPI interface.
    Emulation by R. Belmont
    with changes by W. Toomey for r68k

    This emulates either an SDHC (SPI_SDCARD) or an SDV2 card
    (SPI_SDCARDV2). SDHC has a fixed 512 byte block size and the arguments
    to the read/write commands are block numbers. SDV2 has a variable
    block size defaulting to 512 and the arguments to the read/write
    commands are byte offsets.

    The block size set with CMD16 must match the underlying CHD block
    size if it's not 512.

    Adding the native 4-bit-wide SD interface is also possible; this
    should be broken up into a base SD Card class with SPI and SD
    frontends in that case.

    Multiple block read/write commands are not supported but would be
    straightforward to add.

    References:
    https://www.sdcard.org/downloads/pls/ (Physical Layer Simplified Specification)
    REF: tags are referring to the spec form above. 'Physical Layer Simplified Specification v8.00'

    http://www.dejazzer.com/ee379/lecture_notes/lec12_sd_card.pdf
    https://embdev.net/attachment/39390/TOSHIBA_SD_Card_Specification.pdf
    http://elm-chan.org/docs/mmc/mmc_e.html
*/

#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include "loglevel.h"

extern FILE *logfh;
extern int loglevel;
extern FILE *sdfh;		// The SD card file handle

static void do_command();

enum {
  //REF Table 4-1:Overview of Card States vs. Operation Mode
  SD_STATE_IDLE = 0,
  SD_STATE_READY,
  SD_STATE_IDENT,
  SD_STATE_STBY,
  SD_STATE_TRAN,
  SD_STATE_DATA,
  SD_STATE_DATA_MULTI,		// synthetical state for this implementation
  SD_STATE_RCV,
  SD_STATE_PRG,
  SD_STATE_DIS,
  SD_STATE_INA,

  // FIXME Existing states which must be revisited
  SD_STATE_WRITE_WAITFE,
  SD_STATE_WRITE_DATA
};

enum {
  SD_TYPE_V2 = 0,
  SD_TYPE_HC
};

static uint8_t m_data[520], m_cmd[6];

static int m_cmdidx;
static int m_state;
static int m_type;
static int m_ss, m_in_bit, m_clk_state;
static uint8_t m_in_latch, m_out_latch, m_cur_bit;
static uint16_t m_out_count, m_out_ptr, m_write_ptr, m_blksize;
static uint32_t m_blknext;
static bool m_bACMD;

static uint8_t DATA_RESPONSE_OK = 0x05;
static uint8_t DATA_RESPONSE_IO_ERROR = 0x0d;

static uint16_t get_u16be(uint8_t * buf) {
  return ((uint16_t) (buf[0] << 8) | buf[1]);
}

static uint32_t get_u32be(uint8_t * buf) {
  return ((uint16_t) (buf[0] << 24) | (buf[1] << 16) | buf[2] << 8 | buf[3]);
}

static void put_u16be(uint8_t * buf, uint16_t data) {
  buf[0] = (uint8_t) (data >> 8);
  buf[1] = (uint8_t) (data >> 0);
}

static void put_u32be(uint8_t * buf, uint32_t data) {
  buf[0] = (uint8_t) (data >> 24);
  buf[1] = (uint8_t) (data >> 16);
  buf[2] = (uint8_t) (data >> 8);
  buf[3] = (uint8_t) (data >> 0);
}

// Write a block at the given blk offset.
// Return 1 if OK, 1 otherwise
static int imagewrite(uint32_t blk, uint8_t * data) {
  int err, cnt;

  if (logfh != NULL && (loglevel & LOG_SDCARD) == LOG_SDCARD) {
    fprintf(logfh, "Block data:\n  ");
    for (int i = 0; i < m_blksize; i++) {
      fprintf(logfh, "%02x ", data[i]);
      if ((i % 16) == 15)
	fprintf(logfh, "\n  ");
    }
    fprintf(logfh, "\n");
  }

  // Change the block number to an offset
  if (m_type == SD_TYPE_HC)
    blk *= m_blksize;

  err = fseek(sdfh, blk, SEEK_SET);
  if (err == -1) {
    if (logfh != NULL && (loglevel & LOG_SDCARD) == LOG_SDCARD) {
      fprintf(logfh, "SD unable to fseek to %d for write\n", blk);
      return (0);
    }
  }
  cnt = fwrite(data, 1, m_blksize, sdfh);

  if (cnt == m_blksize)
    return (1);
  return (0);
}

// Read a block at the given blk offset.
// Return 1 if OK, 1 otherwise
static int imageread(uint32_t blk, uint8_t * data) {
  int err, cnt;

  // Change the block number to an offset
  if (m_type == SD_TYPE_HC)
    blk *= m_blksize;

  err = fseek(sdfh, blk, SEEK_SET);
  if (err == -1) {
    if (logfh != NULL && (loglevel & LOG_SDCARD) == LOG_SDCARD) {
      fprintf(logfh, "SD unable to fseek to %d for read\n", blk);
      return (0);
    }
  }
  cnt = fread(data, 1, m_blksize, sdfh);

  if (cnt == m_blksize)
    return (1);
  return (0);
}

static void change_state(int new_state) {
  // TODO validate if transition is valid using refs below.
  // WKT: I had to change a lot of the calls to this function below,
  // so this definitely needs looking at and refactoring.
  //
  // REF Figure 4-13:SD Memory Card State Diagram (Transition Mode)
  // REF Table 4-35:Card State Transition Table
  m_state = new_state;
}

void sdcard_init() {
  m_state = SD_STATE_IDLE;
  m_cmdidx = 0;
  m_ss = 0;
  m_in_bit = 0;
  m_clk_state = 0;
  m_in_latch = 0;
  m_out_latch = 0xFF;
  m_cur_bit = 0;
  m_out_count = 0;
  m_out_ptr = 0;
  m_write_ptr = 0;
  m_blksize = 512;
  m_blknext = 0;
  m_bACMD = false;
  // m_type = SD_TYPE_V2;
  m_type = SD_TYPE_HC;
}

// Record that there is data ready to send via SPI
static void send_data(uint16_t count, int new_state) {

  if (logfh != NULL && (loglevel & LOG_SDCARD) == LOG_SDCARD) {
    fprintf(logfh, "SDCARD response: %d bytes: ", count);
    if (count > 15) fprintf(logfh, "\n  ");
    for (int i = 0; i < count; i++) {
      fprintf(logfh, "%02x ", m_data[i]);
      if ((i % 16) == 15)
	fprintf(logfh, "\n  ");
    }
    fprintf(logfh, "\n");
  }
  m_out_ptr = 0;
  m_out_count = count;
  change_state(new_state);
}

// Return the next data byte to be sent via SPI
// or NULL if there are no bytes to send
uint8_t *spi_get_data(void) {
  uint8_t *dataptr;

  // No data
  if (m_out_count == 0)
    return (NULL);

  // We have reached end of data
  if (m_out_ptr == m_out_count) {
    m_out_ptr = m_out_count = 0;
    return (NULL);
  }
  // Get pointer to the data byte and
  // bump the index up for next time
  dataptr = &(m_data[m_out_ptr]);
  m_out_ptr++;
  return (dataptr);
}

// Absorb a byte from the SPI channel
void spi_latch_in(uint8_t m_in_latch) {

  // Bubble the existing command data down
  // and put the byte at the end
  for (int i = 0; i < 5; i++)
    m_cmd[i] = m_cmd[i + 1];

  m_cmd[5] = m_in_latch;

  switch (m_state) {
  case SD_STATE_IDLE:
    do_command();
    break;

  case SD_STATE_WRITE_WAITFE:
    if (m_in_latch == 0xFE) {
      m_state = SD_STATE_WRITE_DATA;
      m_out_latch = 0xFF;
      m_write_ptr = 0;
    }
    break;

  case SD_STATE_WRITE_DATA:
    m_data[m_write_ptr++] = m_in_latch;
    if (m_write_ptr == (m_blksize + 2)) {
      if (logfh != NULL && (loglevel & LOG_SDCARD) == LOG_SDCARD) {
	fprintf(logfh, "writing LBA %d (0x%x), data %02x %02x %02x %02x\n",
		m_blknext, m_blknext, m_data[0], m_data[1], m_data[2],
		m_data[3]);
      }
      if (imagewrite(m_blknext, &m_data[0])) {
	m_data[0] = DATA_RESPONSE_OK;
      } else {
	m_data[0] = DATA_RESPONSE_IO_ERROR;
      }

      // WKT: looking at the rosco code in bbsd.c, it does:
      // - send dummy FF byte
      // - send FE block start
      // - send 512 bytes of data
      // - send dummy FF FF checksum
      // - waits for card, i.e. expecting FF back
      // This is why I'm sending the DATA_RESPONSE then FF
      m_data[1] = 0xFF;
      send_data(2, SD_STATE_IDLE);

      // WKT: also clear the command buffer
      for (int i = 0; i < 6; i++)
	m_cmd[i] = 0xFF;
    }
    break;

  case SD_STATE_DATA_MULTI:
    do_command();
    if (m_state == SD_STATE_DATA_MULTI && m_out_count == 0) {
      m_data[0] = 0xFE;		// data token
      imageread(m_blknext++, &m_data[1]);
      uint16_t crc16 = 0;
      put_u16be(&m_data[m_blksize + 1], crc16);
      send_data(1 + m_blksize + 2, SD_STATE_DATA_MULTI);
    }
    break;

  default:
    // CMD0 - GO_IDLE_STATE
    if (((m_cmd[0] & 0x70) == 0x40) || (m_out_count == 0)) {
      do_command();
    }
    break;
  }
}

static void do_command() {
  if (((m_cmd[0] & 0xc0) == 0x40) && (m_cmd[5] & 1)) {
    if (logfh != NULL && (loglevel & LOG_SDCARD) == LOG_SDCARD) {
      fprintf(logfh, "SDCARD: cmd %02d 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
	      m_cmd[0] & 0x3f, m_cmd[1], m_cmd[2],
	      m_cmd[3], m_cmd[4], m_cmd[5]);
    }

    bool clean_cmd = true;

    // WKT: Looks like FUZIX wants FF before each command response
    switch (m_cmd[0] & 0x3f) {
    case 0:			// CMD0 - GO_IDLE_STATE
      if (sdfh != NULL) {
	m_data[0] = 0xFF;
	m_data[1] = 0x01;
	send_data(2, SD_STATE_IDLE);
      } else {
	m_data[0] = 0xFF;
	m_data[1] = 0x00;
	send_data(2, SD_STATE_INA);
      }
      break;

    case 1:			// CMD1 - SEND_OP_COND
      m_data[0] = 0xFF;
      m_data[1] = 0x00;
      send_data(2, SD_STATE_READY);
      break;

    case 8:			// CMD8 - SEND_IF_COND (SD v2 only)
      m_data[0] = 0xFF;
      m_data[1] = 0x01;
      m_data[2] = 0;
      m_data[3] = 0;
      m_data[4] = 0x01;
      m_data[5] = 0xaa;
      send_data(6, SD_STATE_IDLE);
      break;

    case 9:			// CMD9 - SEND_CSD
      m_data[0] = 0xFF;
      m_data[1] = 0x00;		// TODO
      send_data(2, SD_STATE_STBY);
      break;

    case 10:			// CMD10 - SEND_CID
      m_data[0] = 0xFF;
      m_data[1] = 0x00;		// initial R1 response
      m_data[2] = 0xFF;		// throwaway byte before data transfer
      m_data[3] = 0xFE;		// data token
      m_data[4] = 'M';		// Manufacturer ID - we'll use M for MAME
      m_data[5] = 'M';		// OEM ID - MD for MAMEdev
      m_data[6] = 'D';
      m_data[7] = 'M';		// Product Name - "MCARD"
      m_data[8] = 'C';
      m_data[9] = 'A';
      m_data[10] = 'R';
      m_data[11] = 'D';
      m_data[12] = 0x10;	// Product Revision in BCD (1.0)
      {
	uint32_t uSerial = 0x12345678;
	put_u32be(&m_data[13], uSerial);	// PSN - Product Serial Number
      }
      m_data[17] = 0x01;	// MDT - Manufacturing Date
      m_data[18] = 0x59;	// 0x15 9 = 2021, September
      m_data[19] = 0x00;	// CRC7, bit 0 is always 0
      {
	uint16_t crc16 = 0;
	put_u16be(&m_data[20], crc16);
      }
      send_data(4 + 16 + 2, SD_STATE_STBY);
      break;

    case 12:			// CMD12 - STOP_TRANSMISSION
      m_data[0] = 0xFF;
      m_data[1] = 0;
      send_data(2, m_state == SD_STATE_RCV ? SD_STATE_PRG : SD_STATE_TRAN);
      break;

    case 13:			// CMD13 - SEND_STATUS
      m_data[0] = 0xFF;
      m_data[1] = 0;		// TODO
      send_data(2, SD_STATE_STBY);
      break;

    case 16:			// CMD16 - SET_BLOCKLEN
      m_blksize = get_u16be(&m_cmd[3]);
      m_data[0] = 0xFF;
      m_data[1] = 0;
      send_data(2, SD_STATE_TRAN);
      break;

    case 17:			// CMD17 - READ_SINGLE_BLOCK
      if (sdfh != NULL) {
        m_data[0] = 0xFF;
	m_data[1] = 0x00;	// initial R1 response
	// data token occurs some time after the R1 response.
	// A2SD expects at least 1 byte of space between R1
	// and the data packet.
	m_data[2] = 0xFF;
	m_data[3] = 0xFE;	// data token
	uint32_t blk = get_u32be(&m_cmd[1]);
	if (m_type == SD_TYPE_V2) {
	  blk /= m_blksize;
	}
	if (logfh != NULL && (loglevel & LOG_SDCARD) == LOG_SDCARD) {
	  fprintf(logfh, "reading LBA %d (0x%x)\n", blk, blk);
	}
	imageread(blk, &m_data[4]);
	{
	  uint16_t crc16 = 0;
	  put_u16be(&m_data[m_blksize + 4], crc16);
	}
	send_data(4 + m_blksize + 2, SD_STATE_IDLE);
      } else {
	m_data[0] = 0xFF;	// show an error
	send_data(1, SD_STATE_IDLE);
      }
      break;

    case 18:			// CMD18 - CMD_READ_MULTIPLE_BLOCK
      if (sdfh != NULL) {
	m_data[0] = 0xFF;
	m_data[1] = 0x00;	// initial R1 response
	// data token occurs some time after the R1 response.  A2SD
	// expects at least 1 byte of space between R1 and the data
	// packet.
	m_blknext = get_u32be(&m_cmd[1]);
	if (m_type == SD_TYPE_V2) {
	  m_blknext /= m_blksize;
	}
      } else {
	m_data[0] = 0xFF;	// show an error
      }
      send_data(2, SD_STATE_DATA_MULTI);
      break;

    case 24:			// CMD24 - WRITE_BLOCK
      m_data[0] = 0xFF;
      m_data[1] = 0;
      m_blknext = get_u32be(&m_cmd[1]);
      if (m_type == SD_TYPE_V2) {
	m_blknext /= m_blksize;
      }
      send_data(2, SD_STATE_WRITE_WAITFE);
      break;

    case 41:
      if (m_bACMD)		// ACMD41 - SD_SEND_OP_COND
      {
	m_data[0] = 0xFF;
	m_data[1] = 0;
	send_data(2, SD_STATE_IDLE);
      } else			// CMD41 - illegal
      {
	m_data[0] = 0xFF;
	send_data(1, SD_STATE_INA);
      }
      break;

    case 55:			// CMD55 - APP_CMD
      m_data[0] = 0xFF;
      m_data[1] = 0x01;
      send_data(2, SD_STATE_IDLE);
      break;

    case 58:			// CMD58 - READ_OCR
      m_data[0] = 0xFF;
      m_data[1] = 0;
      if (m_type == SD_TYPE_HC) {
	m_data[2] = 0xC0;	// indicate SDHC support
      } else {
	m_data[2] = 0x80;
      }
      m_data[3] = 0;
      m_data[4] = 0;
      m_data[5] = 0;
      send_data(6, SD_STATE_IDLE);
      break;

    case 59:			// CMD59 - CRC_ON_OFF
      m_data[0] = 0xFF;
      m_data[1] = 0;
      // TODO CRC 1-on, 0-off
      send_data(2, SD_STATE_IDLE);
      break;

    default:
      if (logfh != NULL && (loglevel & LOG_SDCARD) == LOG_SDCARD) {
	fprintf(logfh, "SDCARD: Unsupported %02x\n", m_cmd[0] & 0x3f);
      }
      clean_cmd = false;
      break;
    }

    // if this is command 55, that's a prefix indicating the next command is an "app command" or "ACMD"
    if ((m_cmd[0] & 0x3f) == 55) {
      m_bACMD = true;
    } else {
      m_bACMD = false;
    }

    if (clean_cmd) {
      for (uint8_t i = 0; i < 6; i++) {
	m_cmd[i] = 0xFF;
      }
    }
  }
}
