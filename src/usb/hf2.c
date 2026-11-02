/*
 * HF2 over WebUSB for the UF2 bootloader (Forward Education, MicroRGB).
 *
 * MakeCode flashes UF2 targets through HF2 (pxt pxtlib/hf2.ts): a vendor interface with class
 * 0xFF, subclass 42 and two 64-byte bulk endpoints, framed as 64-byte packets whose first byte is
 * a flag (0x40 last packet of a command, 0x00 more to come) plus the payload length.
 *
 * Protocol and command layout follow microsoft/uf2-samdx1 (MIT) inc/uf2hid.h and src/hid.c.
 * Flash writes go through the same flash_nrf5x_* page cache as UF2 drag-and-drop, and
 * RESET_INTO_APP completes the update the same way msc_uf2.c does, so the app is marked valid.
 *
 * The BOS (WebUSB + Microsoft OS 2.0) descriptors let Chrome open the interface on Windows
 * without a driver (WinUSB). Layout from TinyUSB examples/device/webusb_serial (MIT).
 */

#include <string.h>

#include "nrfx.h"
#include "tusb.h"
#include "usb_desc.h"

#include "uf2/uf2.h"
#include "bootloader.h"
#include "bootloader_settings.h"
#include "flash_nrf5x.h"

#if CFG_TUD_VENDOR

//--------------------------------------------------------------------+
// HF2 protocol (uf2-samdx1 inc/uf2hid.h)
//--------------------------------------------------------------------+
#define HF2_CMD_BININFO               0x0001
#define HF2_CMD_INFO                  0x0002
#define HF2_CMD_RESET_INTO_APP        0x0003
#define HF2_CMD_RESET_INTO_BOOTLOADER 0x0004
#define HF2_CMD_START_FLASH           0x0005
#define HF2_CMD_WRITE_FLASH_PAGE      0x0006
#define HF2_CMD_CHKSUM_PAGES          0x0007
#define HF2_CMD_READ_WORDS            0x0008

#define HF2_MODE_BOOTLOADER           0x01

#define HF2_FLAG_CMDPKT_LAST          0x40
#define HF2_FLAG_CMDPKT_BODY          0x00
#define HF2_FLAG_MASK                 0xC0
#define HF2_SIZE_MASK                 63

#define HF2_STATUS_OK                 0x00
#define HF2_STATUS_INVALID_CMD        0x01
#define HF2_STATUS_EXEC_ERR           0x02

// Unit of WRITE_FLASH_PAGE and CHKSUM_PAGES: one UF2 block payload
#define HF2_PAGE_SIZE                 256
#define HF2_FLASH_SIZE                (512 * 1024)

// Largest command: 8-byte header, 4-byte address, one page. Responses are capped to fit the
// vendor TX FIFO, which is drained only after the handler returns.
#define HF2_BUF_SIZE                  (HF2_PAGE_SIZE + 64)
#define HF2_MAX_RESPONSE              (CFG_TUD_VENDOR_TX_BUFSIZE / 64 * 63 - 4)

#define DFU_MAGIC_UF2_RESET           0x57

typedef struct {
  uint32_t command_id;
  uint16_t tag;
  uint8_t  reserved0;
  uint8_t  reserved1;
  uint32_t data32[0];
} hf2_command_t;

static union {
  uint8_t  buf8[HF2_BUF_SIZE];
  uint32_t buf32[HF2_BUF_SIZE / 4];
  hf2_command_t cmd;
} _rx;
static uint16_t _rx_size;

static union {
  uint8_t  buf8[HF2_BUF_SIZE];
  uint16_t buf16[HF2_BUF_SIZE / 2];
  uint32_t buf32[HF2_BUF_SIZE / 4];
} _tx;

static bool _written; // any WRITE_FLASH_PAGE since boot

extern char infoUf2File[];

//--------------------------------------------------------------------+
// Helpers
//--------------------------------------------------------------------+

// CRC-16/XMODEM (poly 0x1021, init 0), as uf2-samdx1's add_crc
static uint16_t crc16_xmodem(uint8_t const *data, uint32_t len)
{
  uint16_t crc = 0;
  while ( len-- )
  {
    crc ^= (uint16_t) (*data++) << 8;
    for ( int i = 0; i < 8; i++ ) crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

static bool in_flash(uint32_t addr, uint32_t len)
{
  return addr < HF2_FLASH_SIZE && len <= HF2_FLASH_SIZE - addr;
}

static bool in_app_area(uint32_t addr, uint32_t len)
{
  return addr >= USER_FLASH_START && addr < USER_FLASH_END && len <= USER_FLASH_END - addr;
}

// Send `size` bytes of _tx as one HF2 message, in 64-byte packets
static void send_message(uint32_t size)
{
  uint8_t const *ptr = _tx.buf8;

  do
  {
    uint8_t pkt[64] = { 0 };
    uint32_t s = size > 63 ? 63 : size;

    pkt[0] = (size > 63 ? HF2_FLAG_CMDPKT_BODY : HF2_FLAG_CMDPKT_LAST) | s;
    memcpy(pkt + 1, ptr, s);
    tud_vendor_write(pkt, sizeof(pkt));

    ptr  += s;
    size -= s;
  } while ( size );
}

static void send_response(uint16_t tag, uint8_t status, uint32_t data_size)
{
  _tx.buf16[0] = tag;
  _tx.buf8[2]  = status;
  _tx.buf8[3]  = 0;
  send_message(4 + data_size);
}

static void finish(bool into_bootloader)
{
  dfu_update_status_t update_status;
  memset(&update_status, 0, sizeof(update_status));

  if ( into_bootloader )
  {
    flash_nrf5x_flush(true);
    NRF_POWER->GPREGRET = DFU_MAGIC_UF2_RESET;
    update_status.status_code = DFU_RESET;
    update_status.restart_into_bootloader = true;
  }
  else if ( _written )
  {
    // Same as a completed UF2 drag-and-drop (msc_uf2.c): mark the app valid, then start it
    flash_nrf5x_flush(true);
    update_status.status_code = DFU_UPDATE_APP_COMPLETE;
  }
  else
  {
    update_status.status_code = DFU_RESET;
    update_status.restart_into_bootloader = false;
  }

  bootloader_dfu_update_process(update_status);
}

//--------------------------------------------------------------------+
// Command handler
//--------------------------------------------------------------------+
static void process_command(uint32_t size)
{
  hf2_command_t const *cmd = &_rx.cmd;
  uint16_t const tag = cmd->tag;
  uint32_t const args = size >= 8 ? size - 8 : 0;
  uint8_t *data = _tx.buf8 + 4;

  if ( size < 8 )
  {
    return;
  }

  switch ( cmd->command_id )
  {
    case HF2_CMD_BININFO:
      _tx.buf32[1] = HF2_MODE_BOOTLOADER;
      _tx.buf32[2] = HF2_PAGE_SIZE;
      _tx.buf32[3] = HF2_FLASH_SIZE / HF2_PAGE_SIZE;
      _tx.buf32[4] = HF2_BUF_SIZE;
      _tx.buf32[5] = CFG_UF2_FAMILY_APP_ID;
      send_response(tag, HF2_STATUS_OK, 20);
      return;

    case HF2_CMD_INFO:
    {
      uint32_t len = strlen(infoUf2File);
      if ( len > HF2_MAX_RESPONSE ) len = HF2_MAX_RESPONSE;
      memcpy(data, infoUf2File, len);
      send_response(tag, HF2_STATUS_OK, len);
      return;
    }

    case HF2_CMD_START_FLASH:
      // Already in the bootloader: nothing to do
      send_response(tag, HF2_STATUS_OK, 0);
      return;

    case HF2_CMD_WRITE_FLASH_PAGE:
    {
      uint32_t const addr = cmd->data32[0];
      uint32_t const len  = args >= 4 ? args - 4 : 0;

      // Acknowledge first, so the host sends the next page while this one is written
      if ( len == 0 || len > HF2_PAGE_SIZE || (len & 3) || (addr & 3) || !in_app_area(addr, len) )
      {
        send_response(tag, HF2_STATUS_EXEC_ERR, 0);
        return;
      }

      send_response(tag, HF2_STATUS_OK, 0);
      flash_nrf5x_write(addr, &cmd->data32[1], len, true);
      _written = true;
      led_state(STATE_WRITING_STARTED);
      return;
    }

    case HF2_CMD_READ_WORDS:
    {
      uint32_t const addr = cmd->data32[0];
      uint32_t const num  = cmd->data32[1];

      if ( args < 8 || (addr & 3) || num * 4 > HF2_MAX_RESPONSE || !in_flash(addr, num * 4) )
      {
        send_response(tag, HF2_STATUS_EXEC_ERR, 0);
        return;
      }

      flash_nrf5x_flush(true);
      memcpy(data, (void const *) addr, num * 4);
      send_response(tag, HF2_STATUS_OK, num * 4);
      return;
    }

    case HF2_CMD_CHKSUM_PAGES:
    {
      uint32_t const addr = cmd->data32[0];
      uint32_t const num  = cmd->data32[1];

      if ( args < 8 || (addr % HF2_PAGE_SIZE) || num * 2 > HF2_MAX_RESPONSE || !in_flash(addr, num * HF2_PAGE_SIZE) )
      {
        send_response(tag, HF2_STATUS_EXEC_ERR, 0);
        return;
      }

      flash_nrf5x_flush(true);
      for ( uint32_t i = 0; i < num; i++ )
      {
        uint16_t crc = crc16_xmodem((uint8_t const *) (addr + i * HF2_PAGE_SIZE), HF2_PAGE_SIZE);
        memcpy(data + i * 2, &crc, 2);
      }
      send_response(tag, HF2_STATUS_OK, num * 2);
      return;
    }

    case HF2_CMD_RESET_INTO_APP:
      send_response(tag, HF2_STATUS_OK, 0);
      finish(false);
      return;

    case HF2_CMD_RESET_INTO_BOOTLOADER:
      send_response(tag, HF2_STATUS_OK, 0);
      finish(true);
      return;

    default:
      send_response(tag, HF2_STATUS_INVALID_CMD, 0);
      return;
  }
}

// Invoked by TinyUSB when data arrives on the vendor interface
void tud_vendor_rx_cb(uint8_t itf)
{
  (void) itf;
  uint8_t pkt[64];

  while ( tud_vendor_available() >= sizeof(pkt) )
  {
    tud_vendor_read(pkt, sizeof(pkt));

    uint8_t const flag = pkt[0] & HF2_FLAG_MASK;
    uint8_t const len  = pkt[0] & HF2_SIZE_MASK;

    // Serial packets (0x80, 0xC0) are not used by the bootloader
    if ( flag != HF2_FLAG_CMDPKT_BODY && flag != HF2_FLAG_CMDPKT_LAST ) continue;

    if ( _rx_size + len > sizeof(_rx.buf8) )
    {
      _rx_size = 0; // oversized message: drop it
      continue;
    }

    memcpy(_rx.buf8 + _rx_size, pkt + 1, len);
    _rx_size += len;

    if ( flag == HF2_FLAG_CMDPKT_LAST )
    {
      uint32_t size = _rx_size;
      _rx_size = 0;
      process_command(size);
    }
  }
}

//--------------------------------------------------------------------+
// BOS: WebUSB and Microsoft OS 2.0 descriptors
//--------------------------------------------------------------------+
enum
{
  VENDOR_REQUEST_WEBUSB    = 1,
  VENDOR_REQUEST_MICROSOFT = 2
};

#define BOS_TOTAL_LEN      (TUD_BOS_DESC_LEN + TUD_BOS_WEBUSB_DESC_LEN + TUD_BOS_MICROSOFT_OS_DESC_LEN)
#define MS_OS_20_DESC_LEN  0xB2

static uint8_t const desc_bos[] =
{
  TUD_BOS_DESCRIPTOR(BOS_TOTAL_LEN, 2),
  TUD_BOS_WEBUSB_DESCRIPTOR(VENDOR_REQUEST_WEBUSB, 0), // no landing page
  TUD_BOS_MS_OS_20_DESCRIPTOR(MS_OS_20_DESC_LEN, VENDOR_REQUEST_MICROSOFT)
};

uint8_t const * tud_descriptor_bos_cb(void)
{
  return desc_bos;
}

static uint8_t const desc_ms_os_20[] =
{
  // Set header: length, type, windows version, total length
  U16_TO_U8S_LE(0x000A), U16_TO_U8S_LE(MS_OS_20_SET_HEADER_DESCRIPTOR), U32_TO_U8S_LE(0x06030000), U16_TO_U8S_LE(MS_OS_20_DESC_LEN),

  // Configuration subset header: length, type, configuration index, reserved, configuration total length
  U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_CONFIGURATION), 0, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN-0x0A),

  // Function subset header: length, type, first interface, reserved, subset length
  U16_TO_U8S_LE(0x0008), U16_TO_U8S_LE(MS_OS_20_SUBSET_HEADER_FUNCTION), ITF_NUM_VENDOR, 0, U16_TO_U8S_LE(MS_OS_20_DESC_LEN-0x0A-0x08),

  // Compatible ID: WINUSB
  U16_TO_U8S_LE(0x0014), U16_TO_U8S_LE(MS_OS_20_FEATURE_COMPATBLE_ID), 'W', 'I', 'N', 'U', 'S', 'B', 0x00, 0x00,
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,

  // Registry property: DeviceInterfaceGUIDs
  U16_TO_U8S_LE(MS_OS_20_DESC_LEN-0x0A-0x08-0x08-0x14), U16_TO_U8S_LE(MS_OS_20_FEATURE_REG_PROPERTY),
  U16_TO_U8S_LE(0x0007), U16_TO_U8S_LE(0x002A),
  'D', 0x00, 'e', 0x00, 'v', 0x00, 'i', 0x00, 'c', 0x00, 'e', 0x00, 'I', 0x00, 'n', 0x00, 't', 0x00, 'e', 0x00,
  'r', 0x00, 'f', 0x00, 'a', 0x00, 'c', 0x00, 'e', 0x00, 'G', 0x00, 'U', 0x00, 'I', 0x00, 'D', 0x00, 's', 0x00, 0x00, 0x00,
  U16_TO_U8S_LE(0x0050),
  // {4F2E2A1C-8E3B-4C5D-9B7A-2F6E1D0C3B5A}: MicroRGB HF2 interface (Forward, 2026-11-02)
  '{', 0x00, '4', 0x00, 'F', 0x00, '2', 0x00, 'E', 0x00, '2', 0x00, 'A', 0x00, '1', 0x00, 'C', 0x00, '-', 0x00,
  '8', 0x00, 'E', 0x00, '3', 0x00, 'B', 0x00, '-', 0x00, '4', 0x00, 'C', 0x00, '5', 0x00, 'D', 0x00, '-', 0x00,
  '9', 0x00, 'B', 0x00, '7', 0x00, 'A', 0x00, '-', 0x00, '2', 0x00, 'F', 0x00, '6', 0x00, 'E', 0x00, '1', 0x00,
  'D', 0x00, '0', 0x00, 'C', 0x00, '3', 0x00, 'B', 0x00, '5', 0x00, 'A', 0x00, '}', 0x00, 0x00, 0x00, 0x00, 0x00
};

TU_VERIFY_STATIC(sizeof(desc_ms_os_20) == MS_OS_20_DESC_LEN, "Incorrect size");

bool tud_vendor_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const * request)
{
  if ( stage != CONTROL_STAGE_SETUP ) return true;

  if ( request->bmRequestType_bit.type == TUSB_REQ_TYPE_VENDOR &&
       request->bRequest == VENDOR_REQUEST_MICROSOFT && request->wIndex == 7 )
  {
    return tud_control_xfer(rhport, request, (void*)(uintptr_t) desc_ms_os_20, MS_OS_20_DESC_LEN);
  }

  // No WebUSB landing page, nothing else supported: stall
  return false;
}

#endif
