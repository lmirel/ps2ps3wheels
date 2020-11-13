/*
 Copyright (c) 2015 Mathieu Laurendeau <mat.lau@laposte.net>
 License: GPLv3

 2019 mirel.t.lazar@gmail.com
 adapted for USB message extraction
 */

#include <gusb.h>
#include <gserial.h>
#include <protocol.h>
#include <adapter.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <gpoll.h>
#include <gtimer.h>
#include <names.h>
#include <prio.h>
#include <sys/time.h>

#define ENDPOINT_MAX_NUMBER USB_ENDPOINT_NUMBER_MASK

#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"

#define PRINT_ERROR_OTHER(MESSAGE) fprintf(stderr, "%s:%d %s: %s\n", __FILE__, __LINE__, __func__, MESSAGE);
#define PRINT_TRANSFER_WRITE_ERROR(ENDPOINT,MESSAGE) fprintf(stderr, "%s:%d %s: write transfer failed on endpoint %hhu with error: %s\n", __FILE__, __LINE__, __func__, ENDPOINT & USB_ENDPOINT_NUMBER_MASK, MESSAGE);
#define PRINT_TRANSFER_READ_ERROR(ENDPOINT,MESSAGE) fprintf(stderr, "%s:%d %s: read transfer failed on endpoint %hhu with error: %s\n", __FILE__, __LINE__, __func__, ENDPOINT & USB_ENDPOINT_NUMBER_MASK, MESSAGE);

static int usb = -1;
static int adapter = -1;
static int init_timer = -1;

static s_usb_descriptors * descriptors = NULL;
static unsigned char desc[MAX_DESCRIPTORS_SIZE] = {};
static unsigned char * pDesc = desc;
static s_descriptorIndex descIndex[MAX_DESCRIPTORS] = {};
static s_descriptorIndex * pDescIndex = descIndex;
static s_endpointConfig endpoints[MAX_ENDPOINTS] = {};
static s_endpointConfig * pEndpoints = endpoints;

static uint8_t descIndexSent = 0;
static uint8_t endpointsSent = 0;

static uint8_t inPending = 0;

static uint8_t serialToUsbEndpoint[2][ENDPOINT_MAX_NUMBER] = {};
static uint8_t usbToSerialEndpoint[2][ENDPOINT_MAX_NUMBER] = {};

#define ENDPOINT_ADDR_TO_INDEX(ENDPOINT) (((ENDPOINT) & USB_ENDPOINT_NUMBER_MASK) - 1)
#define ENDPOINT_DIR_TO_INDEX(ENDPOINT) ((ENDPOINT) >> 7)
#define S2U_ENDPOINT(ENDPOINT) serialToUsbEndpoint[ENDPOINT_DIR_TO_INDEX(ENDPOINT)][ENDPOINT_ADDR_TO_INDEX(ENDPOINT)]
#define U2S_ENDPOINT(ENDPOINT) usbToSerialEndpoint[ENDPOINT_DIR_TO_INDEX(ENDPOINT)][ENDPOINT_ADDR_TO_INDEX(ENDPOINT)]

static struct {
  uint16_t length;
  s_endpointPacket packet;
} inPackets[ENDPOINT_MAX_NUMBER] = {};

static uint8_t inEpFifo[MAX_ENDPOINTS] = {};
static uint8_t nbInEpFifo = 0;

static volatile int done;

#define EP_PROP_IN    (1 << 0)
#define EP_PROP_OUT   (1 << 1)
#define EP_PROP_BIDIR (1 << 2)

#define EP_PROP_INT   (1 << 3)
#define EP_PROP_BLK   (1 << 4)
#define EP_PROP_ISO   (1 << 5)

/** Fanatec setup: sudo ./usbxtract --tty /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 --device 0eb7:0e04
#USB extractor 0.10.4d
#i:initializing USB proxy with device 0eb7:0e04
#i:using device: VID 0x0eb7 PID 0x0e04 PATH 01:01:01:02
#i:starting
#i:sending descriptors
#i.DAT@0000: 0x00, 0x145, 0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0xb7, 0x0e, 0x04, 0x0e, 0x76, 0x04, 0x01, 0x09, 0x00, 0x01, 0x04, 0x03, 0x09, 0x04, 0x09, 0x02, 0x29, 0x00, 0x01, 0x01, 0x00, 0x80, 0x28, 0x09, 0x04, 0x00, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00, 0x09, 0x21, 0x11, 0x01, 0x21, 0x01, 0x22, 0xa0, 0x00, 0x07, 0x05, 0x03, 0x03, 0x40, 0x00, 0x05, 0x07, 0x05, 0x84, 0x03, 0x40, 0x00, 0x05, 0x10, 0x03, 0x46, 0x00, 0x61, 0x00, 0x6e, 0x00, 0x61, 0x00, 0x74, 0x00, 0x65, 0x00, 0x63, 0x00, 0x56, 0x03, 0x46, 0x00, 0x41, 0x00, 0x4e, 0x00, 0x41, 0x00, 0x54, 0x00, 0x45, 0x00, 0x43, 0x00, 0x20, 0x00, 0x43, 0x00, 0x53, 0x00, 0x4c, 0x00, 0x20, 0x00, 0x45, 0x00, 0x6c, 0x00, 0x69, 0x00, 0x74, 0x00, 0x65, 0x00, 0x20, 0x00, 0x57, 0x00, 0x68, 0x00, 0x65, 0x00, 0x65, 0x00, 0x6c, 0x00, 0x20, 0x00, 0x42, 0x00, 0x61, 0x00, 0x73, 0x00, 0x65, 0x00, 0x20, 0x00, 0x50, 0x00, 0x6c, 0x00, 0x61, 0x00, 0x79, 0x00, 0x53, 0x00, 0x74, 0x00, 0x61, 0x00, 0x74, 0x00, 0x69, 0x00, 0x6f, 0x00, 0x6e, 0x00, 0x20, 0x00, 0x34, 0x00, 0x05, 0x01, 0x09, 0x05, 0xa1, 0x01, 0x85, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x32, 0x09, 0x35, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, 0x04, 0x81, 0x02, 0x09, 0x39, 0x15, 0x00, 0x25, 0x07, 0x35, 0x00, 0x46, 0x3b, 0x01, 0x65, 0x14, 0x75, 0x04, 0x95, 0x01, 0x81, 0x42, 0x65, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x0e, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x0e, 0x81, 0x02, 0x06, 0x00, 0xff, 0x09, 0x20, 0x75, 0x06, 0x95, 0x01, 0x81, 0x02, 0x05, 0x01, 0x09, 0x33, 0x09, 0x34, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, 0x02, 0x81, 0x02, 0x06, 0x00, 0xff, 0x09, 0x21, 0x95, 0x36, 0x81, 0x02, 0x85, 0x05, 0x09, 0x22, 0x95, 0x1f, 0x91, 0x02, 0x85, 0x03, 0x0a, 0x21, 0x27, 0x95, 0x2f, 0xb1, 0x02, 0xc0, 0x06, 0xf0, 0xff, 0x09, 0x40, 0xa1, 0x01, 0x85, 0xf0, 0x09, 0x47, 0x95, 0x3f, 0xb1, 0x02, 0x85, 0xf1, 0x09, 0x48, 0x95, 0x3f, 0xb1, 0x02, 0x85, 0xf2, 0x09, 0x49, 0x95, 0x0f, 0xb1, 0x02, 0x85, 0xf3, 0x0a, 0x01, 0x47, 0x95, 0x07, 0xb1, 0x02, 0xc0,
#i.DAT@0000: 0x01, 0x30, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00, 0x12, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x00, 0x16, 0x00, 0x00, 0x02, 0x00, 0x00, 0x29, 0x00, 0x3f, 0x00, 0x01, 0x03, 0x09, 0x04, 0x10, 0x00, 0x4f, 0x00, 0x09, 0x03, 0x09, 0x04, 0x56, 0x00, 0xa5, 0x00, 0x00, 0x22, 0x00, 0x00, 0xa0, 0x00,
#i:ready descriptors
#i:ready descriptors
#i.DAT@0000: 0x02, 0x06, 0x03, 0x03, 0x40, 0x84, 0x03, 0x40,
#i:ready indexes
#i:ready
#i.DAT@0000: 0x04, 0x00,
#i.DAT@0000: 0x04, 0x30, 0x03, 0x21, 0x27, 0x04, 0x18, 0x06, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0f, 0x38, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
#i.DAT@0000: 0x04, 0x08, 0xf3, 0x00, 0x38, 0x38, 0x00, 0x00, 0x00, 0x00,
#i.DAT@0000: 0x04, 0x00,

-vs-
#w.UNK@0000: 0x03, 0x00,                                                                                                                                                                                                                                                        #w.UNK@0000: 0x00, 0xfe, 0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0xb7, 0x0e, 0x04, 0x0e, 0x76, 0x04, 0x01, 0x09, 0x00, 0x01, 0x04, 0x03, 0x09, 0x04, 0x09, 0x02, 0x29, 0x00, 0x01, 0x01, 0x00, 0x80, 0x28, 0x09, 0x04, 0x00, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00, 0x09, 0
x21, 0x11, 0x01, 0x21, 0x01, 0x22, 0xa0, 0x00, 0x07, 0x05, 0x03, 0x03, 0x40, 0x00, 0x05, 0x07, 0x05, 0x84, 0x03, 0x40, 0x00, 0x05, 0x10, 0x03, 0x46, 0x00, 0x61, 0x00, 0x6e, 0x00, 0x61, 0x00, 0x74, 0x00, 0x65, 0x00, 0x63, 0x00, 0x56, 0x03, 0x46, 0x00, 0x41, 0x00, 0x4e, 0x0
0, 0x41, 0x00, 0x54, 0x00, 0x45, 0x00, 0x43, 0x00, 0x20, 0x00, 0x43, 0x00, 0x53, 0x00, 0x4c, 0x00, 0x20, 0x00, 0x45, 0x00, 0x6c, 0x00, 0x69, 0x00, 0x74, 0x00, 0x65, 0x00, 0x20, 0x00, 0x57, 0x00, 0x68, 0x00, 0x65, 0x00, 0x65, 0x00, 0x6c, 0x00, 0x20, 0x00, 0x42, 0x00, 0x61, 0x00, 0x73, 0x00, 0x65, 0x00, 0x20, 0x00, 0x50, 0x00, 0x6c, 0x00, 0x61, 0x00, 0x79, 0x00, 0x53, 0x00, 0x74, 0x00, 0x61, 0x00, 0x74, 0x00, 0x69, 0x00, 0x6f, 0x00, 0x6e, 0x00, 0x20, 0x00, 0x34, 0x00, 0x05, 0x01, 0x09, 0x05, 0xa1, 0x01, 0x85, 0x01, 0x09, 0x30, 0x09, 0x31, 0
x09, 0x32, 0x09, 0x35, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, 0x04, 0x81, 0x02, 0x09, 0x39, 0x15, 0x00, 0x25, 0x07, 0x35, 0x00, 0x46, 0x3b, 0x01, 0x65, 0x14, 0x75, 0x04, 0x95, 0x01, 0x81, 0x42, 0x65, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x0e, 0x15, 0x00, 0x25, 0x0
1, 0x75, 0x01, 0x95, 0x0e, 0x81, 0x02, 0x06, 0x00, 0xff, 0x09, 0x20, 0x75, 0x06, 0x95, 0x01, 0x81, 0x02, 0x05, 0x01, 0x09, 0x33, 0x09, 0x34, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95,                                                                                    #w.UNK@0000: 0x00, 0x47, 0x02, 0x81, 0x02, 0x06, 0x00, 0xff, 0x09, 0x21, 0x95, 0x36, 0x81, 0x02, 0x85, 0x05, 0x09, 0x22, 0x95, 0x1f, 0x91, 0x02, 0x85, 0x03, 0x0a, 0x21, 0x27, 0x95, 0x2f, 0xb1, 0x02, 0xc0, 0x06, 0xf0, 0xff, 0x09, 0x40, 0xa1, 0x01, 0x85, 0xf0, 0x09, 0x47, 0
x95, 0x3f, 0xb1, 0x02, 0x85, 0xf1, 0x09, 0x48, 0x95, 0x3f, 0xb1, 0x02, 0x85, 0xf2, 0x09, 0x49, 0x95, 0x0f, 0xb1, 0x02, 0x85, 0xf3, 0x0a, 0x01, 0x47, 0x95, 0x07, 0xb1, 0x02, 0xc0,
#w.UNK@0000: 0x01, 0x30, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00, 0x12, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x00, 0x16, 0x00, 0x00, 0x02, 0x00, 0x00, 0x29, 0x00, 0x3f, 0x00, 0x01, 0x03, 0x09, 0x04, 0x10, 0x00, 0x4f, 0x00, 0x09, 0x03, 0x09, 0x04, 0x56, 0x00, 0xa5, 0
x00, 0x00, 0x22, 0x00, 0x00, 0xa0, 0x00,          
#w.UNK@0000: 0x01, 0x30, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00, 0x12, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x00, 0x16, 0x00, 0x00, 0x02, 0x00, 0x00, 0x29, 0x00, 0x3f, 0x00, 0x01, 0x03, 0x09, 0x04, 0x10, 0x00, 0x4f, 0x00, 0x09, 0x03, 0x09, 0x04, 0x56, 0x00, 0xa5, 0
x00, 0x00, 0x22, 0x00, 0x00, 0xa0, 0x00,
#w.UNK@0000: 0x00, 0x00,
#w.UNK@0000: 0x02, 0x06, 0x03, 0x03, 0x40, 0x84, 0x03, 0x40,
#w.UNK@0000: 0x02, 0x06, 0x03, 0x03, 0x40, 0x84, 0x03, 0x40,
#w.UNK@0000: 0x02, 0x00,
#w.UNK@0000: 0x00, 0x00,
#w.UNK@0000: 0x00, 0x00,
#w.UNK@0000: 0x01, 0x00,
#w.UNK@0000: 0x01, 0x00,
#w.UNK@0000: 0x00, 0x00,
#w.UNK@0000: 0x02, 0x00,
#w.UNK@0000: 0x02, 0x00,
#w.UNK@0000: 0x02, 0x00,
#w.UNK@0000: 0x04, 0x08, 0x21, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
#w.UNK@0000: 0x04, 0x00,
*** Fanatec/Logitech PID: sudo /opt/mfc/usbxtract --tty /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 --device 046d:c260
//i.WHL 0806:046d
#w.UNK@0000: 0x00, 0xef, 0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x6d, 0x04, 0x60, 0xc2, 0x00, 0x89, 0x01, 0x02, 0x00, 0x01, 0x04, 0x03, 0x09, 0x04, 0x09, 0x02, 0x29, 0x00, 0x01, 0x01, 0x00, 0x80, 0x64, 0x09, 0x04, 0x00, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00, 0x09, 0x21, 0x10, 0x01, 0x00, 0x01, 0x22, 0xa0, 0x00, 0x07, 0x05, 0x03, 0x03, 0x40, 0x00, 0x05, 0x07, 0x05, 0x84, 0x03, 0x40, 0x00, 0x05, 0x10, 0x03, 0x46, 0x00, 0x61, 0x00, 0x6e, 0x00, 0x61, 0x00, 0x74, 0x00, 0x65, 0x00, 0x63, 0x00, 0x05, 0x01, 0x09, 0x04, 0xa1, 0x01, 0x85, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x32, 0x09, 0x35, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, 0x04, 0x81, 0x02, 0x09, 0x39, 0x15, 0x00, 0x25, 0x07, 0x35, 0x00, 0x46, 0x3b, 0x01, 0x65, 0x14, 0x75, 0x04, 0x95, 0x01, 0x81, 0x42, 0x65, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x0e, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x0e, 0x81, 0x02, 0x06, 0x00, 0xff, 0x09, 0x20, 0x75, 0x06, 0x95, 0x01, 0x81, 0x02, 0x05, 0x01, 0x09, 0x33, 0x09, 0x34, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, 0x02, 0x81, 0x02, 0x06, 0x00, 0xff, 0x09, 0x21, 0x95, 0x36, 0x81, 0x02, 0x85, 0x05, 0x09, 0x22, 0x95, 0x1f, 0x91, 0x02, 0x85, 0x03, 0x0a, 0x21, 0x27, 0x95, 0x2f, 0xb1, 0x02, 0xc0, 0x06, 0xf0, 0xff, 0x09, 0x40, 0xa1, 0x01, 0x85, 0xf0, 0x09, 0x47, 0x95, 0x3f, 0xb1, 0x02, 0x85, 0xf1, 0x09, 0x48, 0x95, 0x3f, 0xb1, 0x02, 0x85, 0xf2, 0x09, 0x49, 0x95, 0x0f, 0xb1, 0x02, 0x85, 0xf3, 0x0a, 0x01, 0x47, 0x95, 0x07, 0xb1, 0x02, 0xc0,
#w.UNK@0000: 0x01, 0x28, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00, 0x12, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x00, 0x16, 0x00, 0x00, 0x02, 0x00, 0x00, 0x29, 0x00, 0x3f, 0x00, 0x01, 0x03, 0x09, 0x04, 0x10, 0x00, 0x4f, 0x00, 0x00, 0x22, 0x00, 0x00, 0xa0, 0x00,
#w.UNK@0000: 0x01, 0x28, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00, 0x12, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x00, 0x16, 0x00, 0x00, 0x02, 0x00, 0x00, 0x29, 0x00, 0x3f, 0x00, 0x01, 0x03, 0x09, 0x04, 0x10, 0x00, 0x4f, 0x00, 0x00, 0x22, 0x00, 0x00, 0xa0, 0x00,
#w.UNK@0000: 0x02, 0x06, 0x03, 0x03, 0x40, 0x84, 0x03, 0x40,
#w.UNK@0000: 0x02, 0x06, 0x03, 0x03, 0x40, 0x84, 0x03, 0x40,
#w.UNK@0000: 0x02, 0x00,
#w.UNK@0000: 0x04, 0x08, 0x80, 0x06, 0x02, 0x03, 0x09, 0x04, 0x02, 0x00,
#w.UNK@0000: 0x05, 0x00,
#w.UNK@2308: 0x04, 0x08, 0x21, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
#w.UNK@2308: 0x04, 0x00,

*** Fanatec/PC mode: sudo ./usbxtract --tty /dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_0001-if00-port0 --device 0eb7:0005
//i.WHL 0806:0eb7
#w.UNK@0000: 0x00, 0xfe, 0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0xb7, 0x0e, 0x05, 0x00, 0x76, 0x04, 0x01, 0x09, 0x00, 0x01, 0x04, 0x03, 0x09, 0x04, 0x09, 0x02, 0x29, 0x00, 0x01, 0x01, 0x00, 0x80, 0x28, 0x09, 0x04, 0x00, 0x00, 0x02, 0x03, 0x00, 0x00, 0xfe, 0x09, 0x21, 0x11, 0x01, 0x21, 0x01, 0x22, 0x26, 0x01, 0x07, 0x05, 0x81, 0x03, 0x40, 0x00, 0x01, 0x07, 0x05, 0x02, 0x03, 0x40, 0x00, 0x01, 0x10, 0x03, 0x46, 0x00, 0x61, 0x00, 0x6e, 0x00, 0x61, 0x00, 0x74, 0x00, 0x65, 0x00, 0x63, 0x00, 0x56, 0x03, 0x46, 0x00, 0x41, 0x00, 0x4e, 0x00, 0x41, 0x00, 0x54, 0x00, 0x45, 0x00, 0x43, 0x00, 0x20, 0x00, 0x43, 0x00, 0x53, 0x00, 0x4c, 0x00, 0x20, 0x00, 0x45, 0x00, 0x6c, 0x00, 0x69, 0x00, 0x74, 0x00, 0x65, 0x00, 0x20, 0x00, 0x57, 0x00, 0x68, 0x00, 0x65, 0x00, 0x65, 0x00, 0x6c, 0x00, 0x20, 0x00, 0x42, 0x00, 0x61, 0x00, 0x73, 0x00, 0x65, 0x00, 0x20, 0x00, 0x50, 0x00, 0x6c, 0x00, 0x61, 0x00, 0x79, 0x00, 0x53, 0x00, 0x74, 0x00, 0x61, 0x00, 0x74, 0x00, 0x69, 0x00, 0x6f, 0x00, 0x6e, 0x00, 0x20, 0x00, 0x34, 0x00, 0x05, 0x01, 0x09, 0x04, 0xa1, 0x01, 0x85, 0x01, 0x09, 0x39, 0x15, 0x00, 0x25, 0x07, 0x35, 0x00, 0x46, 0x3b, 0x01, 0x65, 0x14, 0x75, 0x04, 0x95, 0x01, 0x81, 0x02, 0x65, 0x00, 0x25, 0x01, 0x45, 0x01, 0x05, 0x09, 0x19, 0x01, 0x29, 0x6c, 0x75, 0x01, 0x95, 0x7c, 0x81, 0x02, 0x05, 0x01, 0x09, 0x30, 0x09, 0x32, 0x09, 0x35, 0x09, 0x31, 0x27, 0xff, 0xff, 0x00, 0x00, 0x47, 0xff, 0xff, 0x00, 0x00, 0x75, 0x10, 0x95, 0x04, 0x81, 0x02, 0x09, 0x33, 0x09, 0x34, 0x15, 0x80, 0x25, 0x7f, 0x35, 0x80, 0x45, 0x7f, 0x75, 0x08, 0x95, 0x02, 0x81, 0x02,
#w.UNK@0000: 0x00, 0xcd, 0x09, 0x36, 0x15, 0x00, 0x26, 0xff, 0x00, 0x35, 0x00, 0x46, 0xff, 0x00, 0x95, 0x01, 0x81, 0x02, 0x09, 0x37, 0x15, 0x80, 0x25, 0x7f, 0x35, 0x80, 0x45, 0x7f, 0x95, 0x01, 0x81, 0x02, 0x06, 0x00, 0xff, 0x09, 0x01, 0x95, 0x05, 0x81, 0x02, 0x09, 0x02, 0x95, 0x07, 0x91, 0x02, 0xc0, 0x05, 0x01, 0x09, 0x04, 0xa1, 0x01, 0x85, 0x02, 0x09, 0x39, 0x09, 0x39, 0x09, 0x39, 0x09, 0x39, 0x15, 0x00, 0x25, 0x07, 0x35, 0x00, 0x46, 0x3b, 0x01, 0x65, 0x14, 0x75, 0x04, 0x95, 0x04, 0x81, 0x02, 0x65, 0x00, 0x25, 0x01, 0x45, 0x01, 0x05, 0x09, 0x19, 0x01, 0x29, 0x3f, 0x75, 0x01, 0x95, 0x40, 0x81, 0x02, 0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x32, 0x09, 0x33, 0x09, 0x34, 0x09, 0x35, 0x09, 0x36, 0x09, 0x36, 0x27, 0xff, 0xff, 0x00, 0x00, 0x47, 0xff, 0xff, 0x00, 0x00, 0x75, 0x10, 0x95, 0x08, 0x81, 0x02, 0x09, 0x37, 0x09, 0x37, 0x09, 0x37, 0x09, 0x37, 0x15, 0x80, 0x25, 0x7f, 0x35, 0x80, 0x45, 0x7f, 0x75, 0x08, 0x95, 0x04, 0x81, 0x02, 0x06, 0x00, 0xff, 0x09, 0x01, 0x95, 0x03, 0x81, 0x02, 0x09, 0x02, 0x95, 0x07, 0x91, 0x02, 0xc0, 0x05, 0x01, 0x09, 0x3a, 0xa1, 0x01, 0x85, 0x03, 0x05, 0x01, 0x09, 0x3b, 0x15, 0x00, 0x26, 0xff, 0x00, 0x35, 0x00, 0x46, 0xff, 0x00, 0x75, 0x08, 0x95, 0x3f, 0x81, 0x02, 0x06, 0x00, 0xff, 0x09, 0x01, 0x91, 0x02, 0xc0,
#w.UNK@0000: 0x01, 0x30, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00, 0x12, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x00, 0x16, 0x00, 0x00, 0x02, 0x00, 0x00, 0x29, 0x00, 0x3f, 0x00, 0x01, 0x03, 0x09, 0x04, 0x10, 0x00, 0x4f, 0x00, 0x09, 0x03, 0x09, 0x04, 0x56, 0x00, 0xa5, 0x00, 0x00, 0x22, 0x00, 0x00, 0x26, 0x01,
#w.UNK@0000: 0x01, 0x30, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00, 0x12, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x00, 0x16, 0x00, 0x00, 0x02, 0x00, 0x00, 0x29, 0x00, 0x3f, 0x00, 0x01, 0x03, 0x09, 0x04, 0x10, 0x00, 0x4f, 0x00, 0x09, 0x03, 0x09, 0x04, 0x56, 0x00, 0xa5, 0x00, 0x00, 0x22, 0x00, 0x00, 0x26, 0x01,
#w.UNK@0000: 0x00, 0x00,
#w.UNK@0000: 0x02, 0x06, 0x81, 0x03, 0x40, 0x02, 0x03, 0x40,
#w.UNK@0000: 0x02, 0x06, 0x81, 0x03, 0x40, 0x02, 0x03, 0x40,
#w.UNK@0000: 0x02, 0x00,
#w.UNK@0000: 0x04, 0x08, 0x80, 0x06, 0xfe, 0x03, 0x09, 0x04, 0x02, 0x00,
#w.UNK@0000: 0x05, 0x00,
#w.UNK@2308: 0x04, 0x08, 0x21, 0x0a, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
#w.UNK@2308: 0x04, 0x00,

*** PS2:Logitech Drifing Force Pro
#i:initializing USB proxy with device 046d:c294
#i:using device: VID 0x046d PID 0xc294 PATH 01:01:01:02
#i:starting
#i:sending descriptors
#i.DAT@0000: 0x00, 0x103, 0x12, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x08, 0x6d, 0x04, 0x94, 0xc2, 0x00, 0x00, 0x03, 0x01, 0x00, 0x01, 0x04, 0x03, 0x09, 0x04, 0x09, 0x02, 0x29, 0x00, 0x01, 0x01, 0x00, 0x80, 0x28, 0x09, 0x04, 0x00, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00, 0x09, 0x21, 0x00, 0x01, 0x21, 0x01, 0x22, 0x84, 0x00, 0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x0a, 0x07, 0x05, 0x02, 0x03, 0x08, 0x00, 0x0a, 0x12, 0x03, 0x4c, 0x00, 0x6f, 0x00, 0x67, 0x00, 0x69, 0x00, 0x74, 0x00, 0x65, 0x00, 0x63, 0x00, 0x68, 0x00, 0x2e, 0x03, 0x4c, 0x00, 0x6f, 0x00, 0x67, 0x00, 0x69, 0x00, 0x74, 0x00, 0x65, 0x00, 0x63, 0x00, 0x68, 0x00, 0x20, 0x00, 0x44, 0x00, 0x72, 0x00, 0x69, 0x00, 0x76, 0x00, 0x69, 0x00, 0x6e, 0x00, 0x67, 0x00, 0x20, 0x00, 0x46, 0x00, 0x6f, 0x00, 0x72, 0x00, 0x63, 0x00, 0x65, 0x00, 0x05, 0x01, 0x09, 0x04, 0xa1, 0x01, 0xa1, 0x02, 0x95, 0x01, 0x75, 0x0a, 0x14, 0x26, 0xff, 0x03, 0x34, 0x46, 0xff, 0x03, 0x09, 0x30, 0x81, 0x02, 0x95, 0x0c, 0x75, 0x01, 0x25, 0x01, 0x45, 0x01, 0x05, 0x09, 0x19, 0x01, 0x29, 0x0c, 0x81, 0x02, 0x95, 0x02, 0x06, 0x00, 0xff, 0x09, 0x01, 0x81, 0x02, 0x05, 0x01, 0x26, 0xff, 0x00, 0x46, 0xff, 0x00, 0x95, 0x01, 0x75, 0x08, 0x81, 0x02, 0x25, 0x07, 0x46, 0x3b, 0x01, 0x75, 0x04, 0x65, 0x14, 0x09, 0x39, 0x81, 0x42, 0x75, 0x01, 0x95, 0x04, 0x65, 0x00, 0x06, 0x00, 0xff, 0x09, 0x01, 0x25, 0x01, 0x45, 0x01, 0x81, 0x02, 0x05, 0x01, 0x95, 0x01, 0x75, 0x08, 0x26, 0xff, 0x00, 0x46, 0xff, 0x00, 0x09, 0x31, 0x81, 0x02, 0x09, 0x35, 0x81, 0x02, 0xc0, 0xa1, 0x02, 0x26, 0xff, 0x00, 0x46, 0xff, 0x00, 0x95, 0x07, 0x75, 0x08, 0x09, 0x03, 0x91, 0x02, 0xc0, 0xc0,
#i.DAT@0000: 0x01, 0x30, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00, 0x12, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x00, 0x16, 0x00, 0x00, 0x02, 0x00, 0x00, 0x29, 0x00, 0x3f, 0x00, 0x03, 0x03, 0x09, 0x04, 0x12, 0x00, 0x51, 0x00, 0x01, 0x03, 0x09, 0x04, 0x2e, 0x00, 0x7f, 0x00, 0x00, 0x22, 0x00, 0x00, 0x84, 0x00,
#i:ready descriptors
#i:ready descriptors
#i.DAT@0000: 0x02, 0x06, 0x81, 0x03, 0x08, 0x02, 0x03, 0x08,
#i:ready indexes
#i:ready
#i.DAT@0000: 0x05, 0x00,

*/

#define MAX_AXIS_VALUE_8BITS 255
#define MAX_AXIS_VALUE_10BITS 1023
#define MAX_AXIS_VALUE_14BITS 16383
#define MAX_AXIS_VALUE_16BITS 65535

#define CENTER_AXIS_VALUE_8BITS (MAX_AXIS_VALUE_8BITS/2+1)
#define CENTER_AXIS_VALUE_10BITS (MAX_AXIS_VALUE_10BITS/2+1)
#define CENTER_AXIS_VALUE_14BITS (MAX_AXIS_VALUE_14BITS/2+1)
#define CENTER_AXIS_VALUE_16BITS (MAX_AXIS_VALUE_16BITS/2+1)

//Fanatec CSL Elite PS4 / Logitech G29
#define G29_SQUARE_MASK     0x10
#define G29_CROSS_MASK      0x20
#define G29_CIRCLE_MASK     0x40
#define G29_TRIANGLE_MASK   0x80
#define G29_PB_IDX          6
#define G29_HATUP_MASK      0x00
#define G29_HATDN_MASK      0x04
#define G29_HATLT_MASK      0x06
#define G29_HATRT_MASK      0x02
#define G29_HB_IDX          6

#define G29_L1_MASK         0x01
#define G29_R1_MASK         0x02
#define G29_L2_MASK         0x04
#define G29_R2_MASK         0x08
#define G29_SHARE_MASK      0x10
#define G29_OPTIONS_MASK    0x20
#define G29_L3_MASK         0x40
#define G29_R3_MASK         0x80
#define G29_LB_IDX          7

#define G29_PS_MASK         0x01
#define G29_PS_IDX          8

#define G29_GEAR_SHIFTER_1_MASK  0x01
#define G29_GEAR_SHIFTER_2_MASK  0x02
#define G29_GEAR_SHIFTER_3_MASK  0x04
#define G29_GEAR_SHIFTER_4_MASK  0x08
#define G29_GEAR_SHIFTER_5_MASK  0x10
#define G29_GEAR_SHIFTER_6_MASK  0x20
#define G29_GEAR_SHIFTER_R_MASK  0x80

#define G29_ENTER_MASK      0x01
#define G29_DIAL_DOWN_MASK  0x02
#define G29_DIAL_UP_MASK    0x04
#define G29_MINUS_MASK      0x08
#define G29_PLUS_MASK       0x10
//--
#define DF_CROSS_MASK      0x0400
#define DF_SQUARE_MASK     0x0800
#define DF_CIRCLE_MASK     0x1000
#define DF_TRIANGLE_MASK   0x2000
#define DF_R1_MASK         0x4000
#define DF_L1_MASK         0x8000

#define DF_R2_MASK         0x01
#define DF_L2_MASK         0x02
#define DF_SELECT_MASK     0x04
#define DF_START_MASK      0x08
#define DF_R3_MASK         0x10
#define DF_L3_MASK         0x20
#define GTF_PB_IDX          1 //push buttons index: SQ, CR, TR, CI
#define GTF_LB_IDX          1 //lateral buttons: L1, R1..

typedef struct __attribute__((packed))
{
  unsigned char endpoint;
  unsigned short buttonsAndWheel; // 10 LSB = wheel, 6 MSB = buttons
  unsigned char buttons;
  unsigned char unknown;
  unsigned char hat;
  unsigned char gasPedal;
  unsigned char brakePedal;
} s_report_dfPs2;
#if 0
static s_report_dfPs2 default_report =
{
  .endpoint = 0x81,
  .buttonsAndWheel = CENTER_AXIS_VALUE_10BITS,
  .buttons = 0x00,
  .unknown = 0x7f,
  .hat = 0x08,
  .gasPedal = MAX_AXIS_VALUE_8BITS,
  .brakePedal = MAX_AXIS_VALUE_8BITS,
};
#endif
static s_report_dfPs2 whl_ps2_df_report =
{
  .endpoint = 0x81,
  .buttonsAndWheel = CENTER_AXIS_VALUE_10BITS,
  .buttons = 0x00,
  .unknown = 0x7f,
  .hat = 0x08,
  .gasPedal = MAX_AXIS_VALUE_8BITS,
  .brakePedal = MAX_AXIS_VALUE_8BITS,
};

#define WHL_PS2_DF_REPORT_LEN  8
//--
/*
whl left-right
|            -- -- -- -- -- -- -- xx xx -- --
#i.WHL@0004: 06 09 81 08 00 00 7e 14 20 ff ff
*/
typedef struct __attribute__((packed))
{
  unsigned char endpoint;
  unsigned char hatAndButtons;
  unsigned char buttons;  //start, select, l1, l2, l3, r1, r2, r3
  unsigned char buttons2; //red/dial:button, left, right, plus, minus
  unsigned char buttons3; //MSB(B0)PS button
  unsigned short wheel;   // 16bit = wheel axis + LSB(B7) GT button
  unsigned char gasPedal;
  unsigned char brakePedal;
} s_report_dfgtPs3;
//
static s_report_dfgtPs3 whl_ps3_dfgt_report =
{
  .endpoint = 0x81,
  .hatAndButtons = 0x08,
  .buttons = 0x00,
  .buttons2 = 0x00,
  .buttons3 = 0x7e,
  .wheel = CENTER_AXIS_VALUE_14BITS, // 16bit wheel
  .gasPedal = MAX_AXIS_VALUE_8BITS,
  .brakePedal = MAX_AXIS_VALUE_8BITS,
};
#define WHL_PS3_DFGT_REPORT_LEN  9
//--
#define G27_CROSS_MASK      0x10
#define G27_SQUARE_MASK     0x20
#define G27_CIRCLE_MASK     0x40
#define G27_TRIANGLE_MASK   0x80

#define G27_R1_MASK         0x0001
#define G27_L1_MASK         0x0002
#define G27_R2_MASK         0x0004
#define G27_L2_MASK         0x0008

#define G27_SELECT_MASK     0x0010
#define G27_START_MASK      0x0020
#define G27_R3_MASK         0x0040
#define G27_L3_MASK         0x0080

#define G27_SHIFTER_1_MASK     0x0100
#define G27_SHIFTER_2_MASK     0x0200
#define G27_SHIFTER_3_MASK     0x0400
#define G27_SHIFTER_4_MASK     0x0800
#define G27_SHIFTER_5_MASK     0x1000
#define G27_SHIFTER_6_MASK     0x2000

#define G27_SHIFTER_CENTER 0x80
#define G27_SHIFTER_LEFT   0x55
#define G27_SHIFTER_RIGHT  0xab
#define G27_SHIFTER_DOWN   0x00
#define G27_SHIFTER_UP     0xff

#define G27_SHIFTER_DEFAULT_BITS    0x9c
#define G27_SHIFTER_REVERSE_ENGAGED 0x40
#define G27_SHIFTER_STICK_DOWN      0x01

#define G27_R4_MASK         0x4000
#define G27_R5_MASK         0x8000

#define G27_L4_MASK         0x0001
#define G27_L5_MASK         0x0002

#if 0
static s_report_g27Ps3 default_report =
{
  .hatAndButtons = 0x08,
  .buttons = 0x0000,
  .buttonsAndWheel = (CENTER_AXIS_VALUE_14BITS << 2),
  .gasPedal = MAX_AXIS_VALUE_8BITS,
  .brakePedal = MAX_AXIS_VALUE_8BITS,
  .clutchPedal = MAX_AXIS_VALUE_8BITS,
  .shifter = { G27_SHIFTER_CENTER, G27_SHIFTER_CENTER, G27_SHIFTER_DEFAULT_BITS },
};
#endif
typedef struct __attribute__((packed))
{
  unsigned char endpoint;
  unsigned char hatAndButtons;
  unsigned char buttons;
  unsigned char buttons2;
  unsigned short buttonsAndWheel; // 16bit = wheel axis
  unsigned char gasPedal;
  unsigned char brakePedal;
  unsigned char clutchPedal;
  unsigned char shifter_x;
  unsigned char shifter_y;
  unsigned char shifter_b;
} s_report_g27Ps3;

static s_report_g27Ps3 whl_ps3_g27_report =
{
  .endpoint = 0x81,
  .hatAndButtons = 0x08,
  .buttons = 0x00,
  .buttons2 = 0x00,
  .buttonsAndWheel = CENTER_AXIS_VALUE_14BITS<<2, // 14bit wheel|L4|L5
  .gasPedal = MAX_AXIS_VALUE_8BITS,
  .brakePedal = MAX_AXIS_VALUE_8BITS,
  .clutchPedal = MAX_AXIS_VALUE_8BITS,
  .shifter_x = 0x80,
  .shifter_y = 0x80,
  .shifter_b = 0x9C,
};

#define WHL_PS3_G27_REPORT_LEN  12
//--

int spoof_device_index = -1; //spf_0x046d_0xc294_idx; //device spoof index
//
#if 1
int whl_ps2_df_convert (char *rep, int rl);
int ffb_ps2_df_convert (unsigned char *ffbin, unsigned char *ffbot, int inlen);
int whl_ps3_dfgt_convert (char *rep, int rl);
int ffb_ps3_dfgt_convert (unsigned char *ffbin, unsigned char *ffbot, int inlen);
int whl_ps3_g27_convert (char *rep, int rl);
int ffb_ps3_g27_convert (unsigned char *ffbin, unsigned char *ffbot, int inlen);

typedef struct {
  int vid;
  int pid;
  int (*whl)(char *rep, int rl);
  int (*ffb)(unsigned char *ffbin, unsigned char *ffbot, int inlen);
  char *whl_report;
  int ffb_out_ep;
  char *pdv;
} proc_list;

proc_list spoof_handlers[] = {
    //PS4:Fanatec CSL Elite Pro
    {0x0EB7, 0x0E04, NULL, NULL, NULL, 0x03, "PS4:Fanatec CSL Elite Pro",},
    //PS2:Logitech Driving Force
    {0x046D, 0xC294, whl_ps2_df_convert, ffb_ps2_df_convert, (char *)&whl_ps2_df_report, 0x03, "PS2:Logitech Driving Force",},
    //PS3:Logitech Driving Force GT
    {0x046D, 0xC29A, whl_ps3_dfgt_convert, ffb_ps3_dfgt_convert, (char *)&whl_ps3_dfgt_report, 0x03, "PS3:Logitech Driving Force GT",},
    //PS3:Logitech G27
    {0x046D, 0xC29B, whl_ps3_g27_convert, ffb_ps3_g27_convert, (char *)&whl_ps3_g27_report, 0x03, "PS3:Logitech G27",},
    {0x0000, 0x0000, NULL, NULL, NULL, 0x00, NULL},
};
int spoof_handlers_index = -1;
char *whl_report = NULL;
#endif
//
typedef struct spoof_pkt {
  unsigned char type;
  unsigned int  len;
  unsigned char pkt[512];
} SPOOF_PKT;

static SPOOF_PKT devs_spoof[] = {
  //Fanatec CSL Elite Pro - PS4
  //#i:using device: VID 0x0eb7 PID 0x0e04 PATH 01:01:01:02
  //opt: --device 0eb7:0e04
  //endpoint: OUT INTERRUPT 3                                                              
  //endpoint: IN INTERRUPT 4                                                                    
  //#s2u[0]: 00 00 03 00 00 00 00 00 00 00 00 00 00 00 00                                  
  //#u2s[0]: 00 00 03 00 00 00 00 00 00 00 00 00 00 00 00                                  
  //#s2u[1]: 00 00 00 84 00 00 00 00 00 00 00 00 00 00 00                                  
  //#u2s[1]: 00 00 00 84 00 00 00 00 00 00 00 00 00 00 00 
  //VID&PID debug message
  {E_TYPE_DEBUG, 0x06, {0x0E, 0xB7, 0x0E, 0x04, 0x81, 0x03}},
  //0: descriptors
  //#i.DAT@0000: 0x00, 0x145, 0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0xb7, 0x0e, 0x04, 0x0e, 0x76, 0x04, 0x01, 0x09, 0x00, 0x01, 0x04, 0x03, 0x09, 0x04, 0x09, 0x02, 0x29, 0x00, 0x01, 0x01, 0x00, 0x80, 0x28, 0x09, 0x04, 0x00, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00, 0x09, 0x21, 0x11, 0x01, 0x21, 0x01, 0x22, 0xa0, 0x00, 0x07, 0x05, 0x03, 0x03, 0x40, 0x00, 0x05, 0x07, 0x05, 0x84, 0x03, 0x40, 0x00, 0x05, 0x10, 0x03, 0x46, 0x00, 0x61, 0x00, 0x6e, 0x00, 0x61, 0x00, 0x74, 0x00, 0x65, 0x00, 0x63, 0x00, 0x56, 0x03, 0x46, 0x00, 0x41, 0x00, 0x4e, 0x00, 0x41, 0x00, 0x54, 0x00, 0x45, 0x00, 0x43, 0x00, 0x20, 0x00, 0x43, 0x00, 0x53, 0x00, 0x4c, 0x00, 0x20, 0x00, 0x45, 0x00, 0x6c, 0x00, 0x69, 0x00, 0x74, 0x00, 0x65, 0x00, 0x20, 0x00, 0x57, 0x00, 0x68, 0x00, 0x65, 0x00, 0x65, 0x00, 0x6c, 0x00, 0x20, 0x00, 0x42, 0x00, 0x61, 0x00, 0x73, 0x00, 0x65, 0x00, 0x20, 0x00, 0x50, 0x00, 0x6c, 0x00, 0x61, 0x00, 0x79, 0x00, 0x53, 0x00, 0x74, 0x00, 0x61, 0x00, 0x74, 0x00, 0x69, 0x00, 0x6f, 0x00, 0x6e, 0x00, 0x20, 0x00, 0x34, 0x00, 0x05, 0x01, 0x09, 0x05, 0xa1, 0x01, 0x85, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x32, 0x09, 0x35, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, 0x04, 0x81, 0x02, 0x09, 0x39, 0x15, 0x00, 0x25, 0x07, 0x35, 0x00, 0x46, 0x3b, 0x01, 0x65, 0x14, 0x75, 0x04, 0x95, 0x01, 0x81, 0x42, 0x65, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x0e, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x0e, 0x81, 0x02, 0x06, 0x00, 0xff, 0x09, 0x20, 0x75, 0x06, 0x95, 0x01, 0x81, 0x02, 0x05, 0x01, 0x09, 0x33, 0x09, 0x34, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, 0x02, 0x81, 0x02, 0x06, 0x00, 0xff, 0x09, 0x21, 0x95, 0x36, 0x81, 0x02, 0x85, 0x05, 0x09, 0x22, 0x95, 0x1f, 0x91, 0x02, 0x85, 0x03, 0x0a, 0x21, 0x27, 0x95, 0x2f, 0xb1, 0x02, 0xc0, 0x06, 0xf0, 0xff, 0x09, 0x40, 0xa1, 0x01, 0x85, 0xf0, 0x09, 0x47, 0x95, 0x3f, 0xb1, 0x02, 0x85, 0xf1, 0x09, 0x48, 0x95, 0x3f, 0xb1, 0x02, 0x85, 0xf2, 0x09, 0x49, 0x95, 0x0f, 0xb1, 0x02, 0x85, 0xf3, 0x0a, 0x01, 0x47, 0x95, 0x07, 0xb1, 0x02, 0xc0,
  {0x00, 0x145, {0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0xb7, 0x0e, 0x04, 0x0e, 0x76, 0x04, 0x01, 0x09, 0x00, 0x01, 0x04, 0x03, 0x09, 0x04, 0x09, 0x02, 0x29, 0x00, 0x01, 0x01, 0x00, 0x80, 0x28, 0x09, 0x04, 0x00, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00, 0x09, 0x21, 0x11, 0x01, 0x21, 0x01, 0x22, 0xa0, 0x00, 0x07, 0x05, 0x03, 0x03, 0x40, 0x00, 0x05, 0x07, 0x05, 0x84, 0x03, 0x40, 0x00, 0x05, 0x10, 0x03, 0x46, 0x00, 0x61, 0x00, 0x6e, 0x00, 0x61, 0x00, 0x74, 0x00, 0x65, 0x00, 0x63, 0x00, 0x56, 0x03, 0x46, 0x00, 0x41, 0x00, 0x4e, 0x00, 0x41, 0x00, 0x54, 0x00, 0x45, 0x00, 0x43, 0x00, 0x20, 0x00, 0x43, 0x00, 0x53, 0x00, 0x4c, 0x00, 0x20, 0x00, 0x45, 0x00, 0x6c, 0x00, 0x69, 0x00, 0x74, 0x00, 0x65, 0x00, 0x20, 0x00, 0x57, 0x00, 0x68, 0x00, 0x65, 0x00, 0x65, 0x00, 0x6c, 0x00, 0x20, 0x00, 0x42, 0x00, 0x61, 0x00, 0x73, 0x00, 0x65, 0x00, 0x20, 0x00, 0x50, 0x00, 0x6c, 0x00, 0x61, 0x00, 0x79, 0x00, 0x53, 0x00, 0x74, 0x00, 0x61, 0x00, 0x74, 0x00, 0x69, 0x00, 0x6f, 0x00, 0x6e, 0x00, 0x20, 0x00, 0x34, 0x00, 0x05, 0x01, 0x09, 0x05, 0xa1, 0x01, 0x85, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x32, 0x09, 0x35, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, 0x04, 0x81, 0x02, 0x09, 0x39, 0x15, 0x00, 0x25, 0x07, 0x35, 0x00, 0x46, 0x3b, 0x01, 0x65, 0x14, 0x75, 0x04, 0x95, 0x01, 0x81, 0x42, 0x65, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x0e, 0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x0e, 0x81, 0x02, 0x06, 0x00, 0xff, 0x09, 0x20, 0x75, 0x06, 0x95, 0x01, 0x81, 0x02, 0x05, 0x01, 0x09, 0x33, 0x09, 0x34, 0x15, 0x00, 0x26, 0xff, 0x00, 0x75, 0x08, 0x95, 0x02, 0x81, 0x02, 0x06, 0x00, 0xff, 0x09, 0x21, 0x95, 0x36, 0x81, 0x02, 0x85, 0x05, 0x09, 0x22, 0x95, 0x1f, 0x91, 0x02, 0x85, 0x03, 0x0a, 0x21, 0x27, 0x95, 0x2f, 0xb1, 0x02, 0xc0, 0x06, 0xf0, 0xff, 0x09, 0x40, 0xa1, 0x01, 0x85, 0xf0, 0x09, 0x47, 0x95, 0x3f, 0xb1, 0x02, 0x85, 0xf1, 0x09, 0x48, 0x95, 0x3f, 0xb1, 0x02, 0x85, 0xf2, 0x09, 0x49, 0x95, 0x0f, 0xb1, 0x02, 0x85, 0xf3, 0x0a, 0x01, 0x47, 0x95, 0x07, 0xb1, 0x02, 0xc0,}},
  //#i.DAT@0000: 0x01, 0x30, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00, 0x12, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x00, 0x16, 0x00, 0x00, 0x02, 0x00, 0x00, 0x29, 0x00, 0x3f, 0x00, 0x01, 0x03, 0x09, 0x04, 0x10, 0x00, 0x4f, 0x00, 0x09, 0x03, 0x09, 0x04, 0x56, 0x00, 0xa5, 0x00, 0x00, 0x22, 0x00, 0x00, 0xa0, 0x00,
  //1: index
  {0x01, 0x30, {0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00, 0x12, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x00, 0x16, 0x00, 0x00, 0x02, 0x00, 0x00, 0x29, 0x00, 0x3f, 0x00, 0x01, 0x03, 0x09, 0x04, 0x10, 0x00, 0x4f, 0x00, 0x09, 0x03, 0x09, 0x04, 0x56, 0x00, 0xa5, 0x00, 0x00, 0x22, 0x00, 0x00, 0xa0, 0x00,}},
  //#i:ready descriptors
  //#i:ready descriptors
  //#i.DAT@0000: 0x02, 0x06, 0x03, 0x03, 0x40, 0x84, 0x03, 0x40,
  //2: endpoints
  {0x02, 0x06, {0x03, 0x03, 0x40, 0x84, 0x03, 0x40,}},
  //3:reset to end the chain
  {E_TYPE_RESET, 0x00, {0x00}},
  //Logitech Driving Force - PS2
  //#i:using device: VID 0x046d PID 0xc294 PATH 01:01:01:02
  //opt: --spoof 046D:C294
  //VID&PID debug message
  {E_TYPE_DEBUG, 0x06, {0x04, 0x6D, 0xC2, 0x94, 0x81, 0x03}},
  //#i:sending descriptors
  //#i.DAT@0000: 0x00, 0x103, 0x12, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x08, 0x6d, 0x04, 0x94, 0xc2, 0x00, 0x00, 0x03, 0x01, 0x00, 0x01, 0x04, 0x03, 0x09, 0x04, 0x09, 0x02, 0x29, 0x00, 0x01, 0x01, 0x00, 0x80, 0x28, 0x09, 0x04, 0x00, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00, 0x09, 0x21, 0x00, 0x01, 0x21, 0x01, 0x22, 0x84, 0x00, 0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x0a, 0x07, 0x05, 0x02, 0x03, 0x08, 0x00, 0x0a, 0x12, 0x03, 0x4c, 0x00, 0x6f, 0x00, 0x67, 0x00, 0x69, 0x00, 0x74, 0x00, 0x65, 0x00, 0x63, 0x00, 0x68, 0x00, 0x2e, 0x03, 0x4c, 0x00, 0x6f, 0x00, 0x67, 0x00, 0x69, 0x00, 0x74, 0x00, 0x65, 0x00, 0x63, 0x00, 0x68, 0x00, 0x20, 0x00, 0x44, 0x00, 0x72, 0x00, 0x69, 0x00, 0x76, 0x00, 0x69, 0x00, 0x6e, 0x00, 0x67, 0x00, 0x20, 0x00, 0x46, 0x00, 0x6f, 0x00, 0x72, 0x00, 0x63, 0x00, 0x65, 0x00, 0x05, 0x01, 0x09, 0x04, 0xa1, 0x01, 0xa1, 0x02, 0x95, 0x01, 0x75, 0x0a, 0x14, 0x26, 0xff, 0x03, 0x34, 0x46, 0xff, 0x03, 0x09, 0x30, 0x81, 0x02, 0x95, 0x0c, 0x75, 0x01, 0x25, 0x01, 0x45, 0x01, 0x05, 0x09, 0x19, 0x01, 0x29, 0x0c, 0x81, 0x02, 0x95, 0x02, 0x06, 0x00, 0xff, 0x09, 0x01, 0x81, 0x02, 0x05, 0x01, 0x26, 0xff, 0x00, 0x46, 0xff, 0x00, 0x95, 0x01, 0x75, 0x08, 0x81, 0x02, 0x25, 0x07, 0x46, 0x3b, 0x01, 0x75, 0x04, 0x65, 0x14, 0x09, 0x39, 0x81, 0x42, 0x75, 0x01, 0x95, 0x04, 0x65, 0x00, 0x06, 0x00, 0xff, 0x09, 0x01, 0x25, 0x01, 0x45, 0x01, 0x81, 0x02, 0x05, 0x01, 0x95, 0x01, 0x75, 0x08, 0x26, 0xff, 0x00, 0x46, 0xff, 0x00, 0x09, 0x31, 0x81, 0x02, 0x09, 0x35, 0x81, 0x02, 0xc0, 0xa1, 0x02, 0x26, 0xff, 0x00, 0x46, 0xff, 0x00, 0x95, 0x07, 0x75, 0x08, 0x09, 0x03, 0x91, 0x02, 0xc0, 0xc0,
  //0: descriptors
  {0x00, 0x103, {0x12, 0x01, 0x00, 0x01, 0x00, 0x00, 0x00, 0x08, 0x6d, 0x04, 0x94, 0xc2, 0x00, 0x00, 0x03, 0x01, 0x00, 0x01, 0x04, 0x03, 0x09, 0x04, 0x09, 0x02, 0x29, 0x00, 0x01, 0x01, 0x00, 0x80, 0x28, 0x09, 0x04, 0x00, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00, 0x09, 0x21, 0x00, 0x01, 0x21, 0x01, 0x22, 0x84, 0x00, 0x07, 0x05, 0x81, 0x03, 0x08, 0x00, 0x0a, 0x07, 0x05, 0x02, 0x03, 0x08, 0x00, 0x0a, 0x12, 0x03, 0x4c, 0x00, 0x6f, 0x00, 0x67, 0x00, 0x69, 0x00, 0x74, 0x00, 0x65, 0x00, 0x63, 0x00, 0x68, 0x00, 0x2e, 0x03, 0x4c, 0x00, 0x6f, 0x00, 0x67, 0x00, 0x69, 0x00, 0x74, 0x00, 0x65, 0x00, 0x63, 0x00, 0x68, 0x00, 0x20, 0x00, 0x44, 0x00, 0x72, 0x00, 0x69, 0x00, 0x76, 0x00, 0x69, 0x00, 0x6e, 0x00, 0x67, 0x00, 0x20, 0x00, 0x46, 0x00, 0x6f, 0x00, 0x72, 0x00, 0x63, 0x00, 0x65, 0x00, 0x05, 0x01, 0x09, 0x04, 0xa1, 0x01, 0xa1, 0x02, 0x95, 0x01, 0x75, 0x0a, 0x14, 0x26, 0xff, 0x03, 0x34, 0x46, 0xff, 0x03, 0x09, 0x30, 0x81, 0x02, 0x95, 0x0c, 0x75, 0x01, 0x25, 0x01, 0x45, 0x01, 0x05, 0x09, 0x19, 0x01, 0x29, 0x0c, 0x81, 0x02, 0x95, 0x02, 0x06, 0x00, 0xff, 0x09, 0x01, 0x81, 0x02, 0x05, 0x01, 0x26, 0xff, 0x00, 0x46, 0xff, 0x00, 0x95, 0x01, 0x75, 0x08, 0x81, 0x02, 0x25, 0x07, 0x46, 0x3b, 0x01, 0x75, 0x04, 0x65, 0x14, 0x09, 0x39, 0x81, 0x42, 0x75, 0x01, 0x95, 0x04, 0x65, 0x00, 0x06, 0x00, 0xff, 0x09, 0x01, 0x25, 0x01, 0x45, 0x01, 0x81, 0x02, 0x05, 0x01, 0x95, 0x01, 0x75, 0x08, 0x26, 0xff, 0x00, 0x46, 0xff, 0x00, 0x09, 0x31, 0x81, 0x02, 0x09, 0x35, 0x81, 0x02, 0xc0, 0xa1, 0x02, 0x26, 0xff, 0x00, 0x46, 0xff, 0x00, 0x95, 0x07, 0x75, 0x08, 0x09, 0x03, 0x91, 0x02, 0xc0, 0xc0,}},
  //#i.DAT@0000: 0x01, 0x30, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00, 0x12, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x00, 0x16, 0x00, 0x00, 0x02, 0x00, 0x00, 0x29, 0x00, 0x3f, 0x00, 0x03, 0x03, 0x09, 0x04, 0x12, 0x00, 0x51, 0x00, 0x01, 0x03, 0x09, 0x04, 0x2e, 0x00, 0x7f, 0x00, 0x00, 0x22, 0x00, 0x00, 0x84, 0x00,
  //1: index
  {0x01, 0x30, {0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00, 0x12, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x00, 0x16, 0x00, 0x00, 0x02, 0x00, 0x00, 0x29, 0x00, 0x3f, 0x00, 0x03, 0x03, 0x09, 0x04, 0x12, 0x00, 0x51, 0x00, 0x01, 0x03, 0x09, 0x04, 0x2e, 0x00, 0x7f, 0x00, 0x00, 0x22, 0x00, 0x00, 0x84, 0x00,}},
  //#i:ready descriptors
  //#i:ready descriptors
  //#i.DAT@0000: 0x02, 0x06, 0x81, 0x03, 0x08, 0x02, 0x03, 0x08,
  //2: endpoints
  {0x02, 0x06, {0x81, 0x03, 0x08, 0x02, 0x03, 0x08,}},
  //3:reset to end the chain
  {E_TYPE_RESET, 0x00, {0x00}},
  //Logitech DFGT - PS3 mode
#if 0
/*                                                                                                                                                                                                                                                                              
 * The reference report data.                                                                                                                                                                                                                                                   
 *                                                                                                                                                                                                                                                                              
 * DFGT Report is composed of 8B data in game mode                                                                                                                                                                                                                              
 *                                                                                                                                                                                                                                                                              
radix: hexadecimal                                                                                                                                                                                                                                                              
08 00 00 7E C9 1F FF FF                                                                                                                                                                                                                                                         
                                                                                                                                                                                                                                                                                
button map                                                                                                                                                                                                                                                                      
                                                                                                                                                                                                                                                                                
 B0:                                                                                                                                                                                                                                                                            
 0x08 hat no button                                                                                                                                                                                                                                                             
 0x00 hat up                                                                                                                                                                                                                                                                    
 0x01 hat right-up                                                                                                                                                                                                                                                              
 0x02 hat right                                                                                                                                                                                                                                                                 
 0x03 hat down-right                                                                                                                                                                                                                                                            
 0x04 hat down                                                                                                                                                                                                                                                                  
 0x05 hat down-left                                                                                                                                                                                                                                                             
 0x06 hat left                                                                                                                                                                                                                                                                  
 0x07 hat up-left                                                                                                                                                                                                                                                               
 0x10 cross                                                                                                                                                                                                                                                                     
 0x20 square                                                                                                                                                                                                                                                                    
 0x40 circle                                                                                                                                                                                                                                                                    
 0x80 triangle                                                                                                                                                                                                                                                                  
                                                                                                                                                                                                                                                                                
 B1:                                                                                                                                                                                                                                                                            
 0x01 R1 / right shift                                                                                                                                                                                                                                                          
 0x02 L1 / left shift                                                                                                                                                                                                                                                           
 0x10 select                                                                                                                                                                                                                                                                    
 0x20 start                                                                                                                                                                                                                                                                     
 0x40 R3                                                                                                                                                                                                                                                                        
 0x80 L3                                                                                                                                                                                                                                                                        
 0x04 R2                                                                                                                                                                                                                                                                        
 0x08 L2                                                                                                                                                                                                                                                                        
                                                                                                                                                                                                                                                                                
 B2:                                                                                                                                                                                                                                                                            
 0x01 shift down                                                                                                                                                                                                                                                                
 0x02 shift up                                                                                                                                                                                                                                                                  
 0x04 dial return / OK                                                                                                                                                                                                                                                          
 0x10 dial right                                                                                                                                                                                                                                                                
 0x20 dial left                                                                                                                                                                                                                                                                 
 0x40 dial minus                                                                                                                                                                                                                                                                
 0x08 dial plus                                                                                                                                                                                                                                                                 
                                                                                                                                                                                                                                                                                
 B3:                                                                                                                                                                                                                                                                            
 0x08 PS                                                                                                                                                                                                                                                                        
                                                                                                                                                                                                                                                                                
 B4 + B5: X axis - wheel                                                                                                                                                                                                                                                        
 B6: accel - FF to 0 (down)                                                                                                                                                                                                                                                     
 B7 : brake - FF to 0 (down                                                                                                                                                                                                                                                     
                                                                                                                                                                                                                                                                                
 *                                                                                                                                                                                                                                                                              
 */                                                                                                                                                                                                                                                                             
                                                                                                                                                                                                                                                                                
#define REPORT_LEN  8                                                                                                                                                                                                                                                           
0x08, 0x00, 0x00, 0x7E, 0xC9, 0x1F, 0xFF, 0xFF };
#endif
  //#i:using device: VID 0x046d PID 0xc29a PATH 01:01:01:02
  //opt: --spoof 046D:C29A
  //endpoint: IN INTERRUPT 1
  //endpoint: OUT INTERRUPT 2
  //#s2u[0]: 00 02 00 00 00 00 00 00 00 00 00 00 00 00 00
  //#u2s[0]: 00 02 00 00 00 00 00 00 00 00 00 00 00 00 00
  //#s2u[1]: 81 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  //#u2s[1]: 81 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  //VID&PID debug message
  {E_TYPE_DEBUG, 0x06, {0x04, 0x6D, 0xC2, 0x9A, 0x81, 0x03}},
  //#i:sending descriptors
  //#i.WHL@0000: 0x00, 0xE6, 0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x6d, 0x04, 0x9a, 0xc2, 0x26, 0x13, 0x00, 0x02, 0x00, 0x01, 0x04, 0x03, 0x09, 0x04, 0x09, 0x02, 0x29, 0x00, 0x01, 0x01, 0x00, 0xc0, 0x32, 0x09, 0x04, 0x00, 0x00, 0x02, 0x03, 0x00, 0x00, 0xfe, 0x09, 0x21, 0x11, 0x01, 0x21, 0x01, 0x22, 0x85, 0x00, 0x07, 0x05, 0x81, 0x03, 0x10, 0x00, 0x02, 0x07, 0x05, 0x02, 0x03, 0x10, 0x00, 0x02, 0x22, 0x03, 0x47, 0x00, 0x32, 0x00, 0x37, 0x00, 0x20, 0x00, 0x52, 0x00, 0x61, 0x00, 0x63, 0x00, 0x69, 0x00, 0x6e, 0x00, 0x67, 0x00, 0x20, 0x00, 0x57, 0x00, 0x68, 0x00, 0x65, 0x00, 0x65, 0x00, 0x6c, 0x00, 0x05, 0x01, 0x09, 0x04, 0xa1, 0x01, 0xa1, 0x02, 0x95, 0x01, 0x75, 0x0a, 0x15, 0x00, 0x26, 0xff, 0x03, 0x35, 0x00, 0x46, 0xff, 0x03, 0x09, 0x30, 0x81, 0x02, 0x95, 0x0c, 0x75, 0x01, 0x25, 0x01, 0x45, 0x01, 0x05, 0x09, 0x19, 0x01, 0x29, 0x0c, 0x81, 0x02, 0x95, 0x02, 0x06, 0x00, 0xff, 0x09, 0x01, 0x81, 0x02, 0x05, 0x01, 0x09, 0x31, 0x26, 0xff, 0x00, 0x46, 0xff, 0x00, 0x95, 0x01, 0x75, 0x08, 0x81, 0x02, 0x25, 0x07, 0x46, 0x3b, 0x01, 0x75, 0x04, 0x65, 0x14, 0x09, 0x39, 0x81, 0x42, 0x75, 0x01, 0x95, 0x04, 0x65, 0x00, 0x06, 0x00, 0xff, 0x09, 0x01, 0x25, 0x01, 0x45, 0x01, 0x81, 0x02, 0x95, 0x02, 0x75, 0x08, 0x26, 0xff, 0x00, 0x46, 0xff, 0x00, 0x09, 0x02, 0x81, 0x02, 0xc0, 0xa1, 0x02, 0x26, 0xff, 0x00, 0x46, 0xff, 0x00, 0x95, 0x07, 0x75, 0x08, 0x09, 0x03, 0x91, 0x02, 0x00, 0x00, 0x00, 0xc0, 0xc0,
  {0x00, 0xE6, {0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x6d, 0x04, 0x9a, 0xc2, 0x26, 0x13, 0x00, 0x02, 0x00, 0x01, 0x04, 0x03, 0x09, 0x04, 0x09, 0x02, 0x29, 0x00, 0x01, 0x01, 0x00, 0xc0, 0x32, 0x09, 0x04, 0x00, 0x00, 0x02, 0x03, 0x00, 0x00, 0xfe, 0x09, 0x21, 0x11, 0x01, 0x21, 0x01, 0x22, 0x85, 0x00, 0x07, 0x05, 0x81, 0x03, 0x10, 0x00, 0x02, 0x07, 0x05, 0x02, 0x03, 0x10, 0x00, 0x02, 0x22, 0x03, 0x47, 0x00, 0x32, 0x00, 0x37, 0x00, 0x20, 0x00, 0x52, 0x00, 0x61, 0x00, 0x63, 0x00, 0x69, 0x00, 0x6e, 0x00, 0x67, 0x00, 0x20, 0x00, 0x57, 0x00, 0x68, 0x00, 0x65, 0x00, 0x65, 0x00, 0x6c, 0x00, 0x05, 0x01, 0x09, 0x04, 0xa1, 0x01, 0xa1, 0x02, 0x95, 0x01, 0x75, 0x0a, 0x15, 0x00, 0x26, 0xff, 0x03, 0x35, 0x00, 0x46, 0xff, 0x03, 0x09, 0x30, 0x81, 0x02, 0x95, 0x0c, 0x75, 0x01, 0x25, 0x01, 0x45, 0x01, 0x05, 0x09, 0x19, 0x01, 0x29, 0x0c, 0x81, 0x02, 0x95, 0x02, 0x06, 0x00, 0xff, 0x09, 0x01, 0x81, 0x02, 0x05, 0x01, 0x09, 0x31, 0x26, 0xff, 0x00, 0x46, 0xff, 0x00, 0x95, 0x01, 0x75, 0x08, 0x81, 0x02, 0x25, 0x07, 0x46, 0x3b, 0x01, 0x75, 0x04, 0x65, 0x14, 0x09, 0x39, 0x81, 0x42, 0x75, 0x01, 0x95, 0x04, 0x65, 0x00, 0x06, 0x00, 0xff, 0x09, 0x01, 0x25, 0x01, 0x45, 0x01, 0x81, 0x02, 0x95, 0x02, 0x75, 0x08, 0x26, 0xff, 0x00, 0x46, 0xff, 0x00, 0x09, 0x02, 0x81, 0x02, 0xc0, 0xa1, 0x02, 0x26, 0xff, 0x00, 0x46, 0xff, 0x00, 0x95, 0x07, 0x75, 0x08, 0x09, 0x03, 0x91, 0x02, 0x00, 0x00, 0x00, 0xc0, 0xc0,}},
  //#i.WHL@0000: 0x01, 0x28, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00, 0x12, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x00, 0x16, 0x00, 0x00, 0x02, 0x00, 0x00, 0x29, 0x00, 0x3f, 0x00, 0x02, 0x03, 0x09, 0x04, 0x22, 0x00, 0x61, 0x00, 0x00, 0x22, 0x00, 0x00, 0x85, 0x00,
  {0x01, 0x28, {0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00, 0x12, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x00, 0x16, 0x00, 0x00, 0x02, 0x00, 0x00, 0x29, 0x00, 0x3f, 0x00, 0x02, 0x03, 0x09, 0x04, 0x22, 0x00, 0x61, 0x00, 0x00, 0x22, 0x00, 0x00, 0x85, 0x00,}},
  //#i:ready descriptors
  //#i.WHL@0000: 0x02, 0x06, 0x81, 0x03, 0x10, 0x02, 0x03, 0x10,
  {0x02, 0x06, {0x81, 0x03, 0x10, 0x02, 0x03, 0x10,}},
  //3:reset to end the chain
  {E_TYPE_RESET, 0x00, {0x00}},
  //Logitech G27 - PS3 mode
#if 0
/*                                                                                                                                                                                                                                                                              
 * The reference report data.                                                                                                                                                                                                                                                   
 *                                                                                                                                                                                                                                                                              
 * G27 Report is composed of 11B data in game mode                                                                                                                                                                                                                              
 *                                                                                                                                                                                                                                                                              
 radix: hexadecimal                                                                                                                                                                                                                                                             
 08 00 00 AC 87 FF FF FF 7B 8B 1C                                                                                                                                                                                                                                               
                                                                                                                                                                                                                                                                                
 DIRT3 + GT6 map                                                                                                                                                                                                                                                                
                                                                                                                                                                                                                                                                                
 B0:                                                                                                                                                                                                                                                                            
 0001 0000: cross                                                                                                                                                                                                                                                               
 0010 0000: rectangle                                                                                                                                                                                                                                                           
 0100 0000: circle                                                                                                                                                                                                                                                              
 1000 0000: triangle                                                                                                                                                                                                                                                            
 0000 1111: h: no button                                                                                                                                                                                                                                                        
 0000 0000: h:0 up                                                                                                                                                                                                                                                              
 0000 0010: h:2 right                                                                                                                                                                                                                                                           
 0000 0100: h:4 down                                                                                                                                                                                                                                                            
 0000 0110: h:6 left                                                                                                                                                                                                                                                            
 0000 0111: h:7 up-left                                                                                                                                                                                                                                                         
 0000 0001: h:1 up-right                                                                                                                                                                                                                                                        
 0000 0011: h:2 down-right                                                                                                                                                                                                                                                      
 0000 0101: h:5 down-left                                                                                                                                                                                                                                                       
                                                                                                                                                                                                                                                                                
 B1:                                                                                                                                                                                                                                                                            
 01: r shifter / R1                                                                                                                                                                                                                                                             
 02: l shifter / L1                                                                                                                                                                                                                                                             
 04: wrt - wheel right top / R2                                                                                                                                                                                                                                                 
 08: wlt - wheel left top / L2                                                                                                                                                                                                                                                  
 80: x--- / L3                                                                                                                                                                                                                                                                  
 10: -x-- / select                                                                                                                                                                                                                                                              
 20: --x- / start                                                                                                                                                                                                                                                               
 40: ---x / R3                                                                                                                                                                                                                                                                  
                                                                                                                                                                                                                                                                                
 B2:                                                                                                                                                                                                                                                                            
 01: gear 1                                                                                                                                                                                                                                                                     
 02: gear 2                                                                                                                                                                                                                                                                     
 04: gear 3                                                                                                                                                                                                                                                                     
 08: gear 4                                                                                                                                                                                                                                                                     
 10: gear 5                                                                                                                                                                                                                                                                     
 20: gear 6                                                                                                                                                                                                                                                                     
 40: wrm - wheel right middle                                                                                                                                                                                                                                                   
 80: wrb - wheel right bottom                                                                                                                                                                                                                                                   
                                                                                                                                                                                                                                                                                
 B3.01: wlm - wheel left middle                                                                                                                                                                                                                                                 
 B3.02: wlb - wheel left bottom                                                                                                                                                                                                                                                 
 B3+B4: X axis                                                                                                                                                                                                                                                                  
 B3+B4: X axis: >  max: 00 00 - 00 80 - max:  FF FF <                                                                                                                                                                                                                           
 B3+B4: X axis: >  90g: 00 65 - 00 80 - 90g:  00 95 <                                                                                                                                                                                                                           
 B3+B4: X axis: > 180g: 00 50 - 00 80 - 180g: 00 A0 <                                                                                                                                                                                                                           
                                                                                                                                                                                                                                                                                
 B5: accel                                                                                                                                                                                                                                                                      
 B6: brake                                                                                                                                                                                                                                                                      
 B7: clutch
  B8+B9+B10: gear R?? from ~ 7D 8A 1C (idle) to ~ B0 2A 5D (engaged)                                                                                                                                                                                                             -H shifter-                                                                                                                                                                                                                                                                     [B8 B9 B10] values:                                                                                                                                                                                                                                                              7C 8A 9C neutral                                                                                                                                                                                                                                                               
 3F 94 9C left pull                                                                                                                                                                                                                                                             
 BA 8A 9C right pull                                                                                                                                                                                                                                                            
 7B 90 DC center push                                                                                                                                                                                                                                                           
 BD 92 DC right push                                                                                                                                                                                                                                                            
 C4 C1 DC right push up                                                                                                                                                                                                                                                         
 B3 2B DD right push down / Reverse                                                                                                                                                                                                                                             
 *                                                                                                                                                                                                                                                                              
 */                                                                                                                                                                                                                                                                             
#define REPORT_LEN  11                                                                                                                                                                                                                                                          
                                                                                                                                                                                                                                                                                
static uint8_t report[REPORT_LEN] =                                                                                                                                                                                                                                             
  {                                                                                                                                                                                                                                                                             
      0x08, 0x00, 0x00, 0x7E, 0xC9, 0x1F, 0xFF, 0xFF };                                                                                                                                                                                                                         
                                                                                                                                                                                                                                                                                
#if 0                                                                                                                                                                                                                                                                           
static struct                                                                                                                                                                                                                                                                   
  __attribute__ ((packed))                                                                                                                                                                                                                                                      
  {                                                                                                                                                                                                                                                                             
    unsigned char hatAndButtons; //4 LSB = hat, 4 MSB = buttons                                                                                                                                                                                                                 
    unsigned short buttons;                                                                                                                                                                                                                                                     
    unsigned short wheel; //big-endian, 14 MSB = axis                                                                                                                                                                                                                           
    unsigned char gasPedal;                                                                                                                                                                                                                                                     
    unsigned char brakePedal;                                                                                                                                                                                                                                                   
    unsigned char clutchPedal;                                                                                                                                                                                                                                                  
    unsigned char unknown[3];                                                                                                                                                                                                                                                   
  } report;                                                                                                                                                                                                                                                                     
#endif                           
#endif
  //#i:using device: VID 0x046d PID 0xc29b PATH 01:01:01:02
  //opt: --spoof 046D:C29B
  //VID&PID debug message
  {E_TYPE_DEBUG, 0x06, {0x04, 0x6D, 0xC2, 0x9B, 0x81, 0x03}},
  //endpoint: IN INTERRUPT 1
  //endpoint: OUT INTERRUPT 2
  //#s2u[0]: 00 02 00 00 00 00 00 00 00 00 00 00 00 00 00
  //#u2s[0]: 00 02 00 00 00 00 00 00 00 00 00 00 00 00 00
  //#s2u[1]: 81 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  //#u2s[1]: 81 00 00 00 00 00 00 00 00 00 00 00 00 00 00
  //#i:sending descriptors
  //#i.WHL@0000: 0x00, 0xF8, 0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x6d, 0x04, 0x9b, 0xc2, 0x38, 0x12, 0x01, 0x02, 0x00, 0x01, 0x04, 0x03, 0x09, 0x04, 0x09, 0x02, 0x29, 0x00, 0x01, 0x01, 0x00, 0xc0, 0x32, 0x09, 0x04, 0x00, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00, 0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, 0x85, 0x00, 0x07, 0x05, 0x81, 0x03, 0x10, 0x00, 0x02, 0x07, 0x05, 0x02, 0x03, 0x10, 0x00, 0x02, 0x12, 0x03, 0x4c, 0x00, 0x6f, 0x00, 0x67, 0x00, 0x69, 0x00, 0x74, 0x00, 0x65, 0x00, 0x63, 0x00, 0x68, 0x00, 0x22, 0x03, 0x47, 0x00, 0x32, 0x00, 0x37, 0x00, 0x20, 0x00, 0x52, 0x00, 0x61, 0x00, 0x63, 0x00, 0x69, 0x00, 0x6e, 0x00, 0x67, 0x00, 0x20, 0x00, 0x57, 0x00, 0x68, 0x00, 0x65, 0x00, 0x65, 0x00, 0x6c, 0x00, 0x05, 0x01, 0x09, 0x04, 0xa1, 0x01, 0xa1, 0x02, 0x95, 0x01, 0x75, 0x0a, 0x15, 0x00, 0x26, 0xff, 0x03, 0x35, 0x00, 0x46, 0xff, 0x03, 0x09, 0x30, 0x81, 0x02, 0x95, 0x0c, 0x75, 0x01, 0x25, 0x01, 0x45, 0x01, 0x05, 0x09, 0x19, 0x01, 0x29, 0x0c, 0x81, 0x02, 0x95, 0x02, 0x06, 0x00, 0xff, 0x09, 0x01, 0x81, 0x02, 0x05, 0x01, 0x09, 0x31, 0x26, 0xff, 0x00, 0x46, 0xff, 0x00, 0x95, 0x01, 0x75, 0x08, 0x81, 0x02, 0x25, 0x07, 0x46, 0x3b, 0x01, 0x75, 0x04, 0x65, 0x14, 0x09, 0x39, 0x81, 0x42, 0x75, 0x01, 0x95, 0x04, 0x65, 0x00, 0x06, 0x00, 0xff, 0x09, 0x01, 0x25, 0x01, 0x45, 0x01, 0x81, 0x02, 0x95, 0x02, 0x75, 0x08, 0x26, 0xff, 0x00, 0x46, 0xff, 0x00, 0x09, 0x02, 0x81, 0x02, 0xc0, 0xa1, 0x02, 0x26, 0xff, 0x00, 0x46, 0xff, 0x00, 0x95, 0x07, 0x75, 0x08, 0x09, 0x03, 0x91, 0x02, 0x00, 0x00, 0x00, 0xc0, 0xc0,
  {0x00, 0xF8, {0x12, 0x01, 0x00, 0x02, 0x00, 0x00, 0x00, 0x40, 0x6d, 0x04, 0x9b, 0xc2, 0x38, 0x12, 0x01, 0x02, 0x00, 0x01, 0x04, 0x03, 0x09, 0x04, 0x09, 0x02, 0x29, 0x00, 0x01, 0x01, 0x00, 0xc0, 0x32, 0x09, 0x04, 0x00, 0x00, 0x02, 0x03, 0x00, 0x00, 0x00, 0x09, 0x21, 0x11, 0x01, 0x00, 0x01, 0x22, 0x85, 0x00, 0x07, 0x05, 0x81, 0x03, 0x10, 0x00, 0x02, 0x07, 0x05, 0x02, 0x03, 0x10, 0x00, 0x02, 0x12, 0x03, 0x4c, 0x00, 0x6f, 0x00, 0x67, 0x00, 0x69, 0x00, 0x74, 0x00, 0x65, 0x00, 0x63, 0x00, 0x68, 0x00, 0x22, 0x03, 0x47, 0x00, 0x32, 0x00, 0x37, 0x00, 0x20, 0x00, 0x52, 0x00, 0x61, 0x00, 0x63, 0x00, 0x69, 0x00, 0x6e, 0x00, 0x67, 0x00, 0x20, 0x00, 0x57, 0x00, 0x68, 0x00, 0x65, 0x00, 0x65, 0x00, 0x6c, 0x00, 0x05, 0x01, 0x09, 0x04, 0xa1, 0x01, 0xa1, 0x02, 0x95, 0x01, 0x75, 0x0a, 0x15, 0x00, 0x26, 0xff, 0x03, 0x35, 0x00, 0x46, 0xff, 0x03, 0x09, 0x30, 0x81, 0x02, 0x95, 0x0c, 0x75, 0x01, 0x25, 0x01, 0x45, 0x01, 0x05, 0x09, 0x19, 0x01, 0x29, 0x0c, 0x81, 0x02, 0x95, 0x02, 0x06, 0x00, 0xff, 0x09, 0x01, 0x81, 0x02, 0x05, 0x01, 0x09, 0x31, 0x26, 0xff, 0x00, 0x46, 0xff, 0x00, 0x95, 0x01, 0x75, 0x08, 0x81, 0x02, 0x25, 0x07, 0x46, 0x3b, 0x01, 0x75, 0x04, 0x65, 0x14, 0x09, 0x39, 0x81, 0x42, 0x75, 0x01, 0x95, 0x04, 0x65, 0x00, 0x06, 0x00, 0xff, 0x09, 0x01, 0x25, 0x01, 0x45, 0x01, 0x81, 0x02, 0x95, 0x02, 0x75, 0x08, 0x26, 0xff, 0x00, 0x46, 0xff, 0x00, 0x09, 0x02, 0x81, 0x02, 0xc0, 0xa1, 0x02, 0x26, 0xff, 0x00, 0x46, 0xff, 0x00, 0x95, 0x07, 0x75, 0x08, 0x09, 0x03, 0x91, 0x02, 0x00, 0x00, 0x00, 0xc0, 0xc0,}},
  //#i.WHL@0000: 0x01, 0x30, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00, 0x12, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x00, 0x16, 0x00, 0x00, 0x02, 0x00, 0x00, 0x29, 0x00, 0x3f, 0x00, 0x01, 0x03, 0x09, 0x04, 0x12, 0x00, 0x51, 0x00, 0x02, 0x03, 0x09, 0x04, 0x22, 0x00, 0x73, 0x00, 0x00, 0x22, 0x00, 0x00, 0x85, 0x00,
  {0x01, 0x30, {0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x12, 0x00, 0x12, 0x00, 0x00, 0x03, 0x00, 0x00, 0x04, 0x00, 0x16, 0x00, 0x00, 0x02, 0x00, 0x00, 0x29, 0x00, 0x3f, 0x00, 0x01, 0x03, 0x09, 0x04, 0x12, 0x00, 0x51, 0x00, 0x02, 0x03, 0x09, 0x04, 0x22, 0x00, 0x73, 0x00, 0x00, 0x22, 0x00, 0x00, 0x85, 0x00,}},
  //#i:ready descriptors
  //#i.WHL@0000: 0x02, 0x06, 0x81, 0x03, 0x10, 0x02, 0x03, 0x10,
  {0x02, 0x06, {0x81, 0x03, 0x10, 0x02, 0x03, 0x10,}},
  //#i:ready indexes
  //3:reset to end the chain
  {E_TYPE_RESET, 0x00, {0x00}},
};

int get_spoof_idx (int vid, int pid)
{
  int lk = 0, ret = -1;
  int spdl = sizeof(devs_spoof)/sizeof(SPOOF_PKT);
  printf("\n#scanning %d spoof pkts", spdl);
  while (lk < spdl)
  {
    if (devs_spoof[lk].type == E_TYPE_DEBUG)
    {
      printf("\n#spoof dev %d %02X%02X:%02X%02X", lk, devs_spoof[lk].pkt[0], devs_spoof[lk].pkt[1], devs_spoof[lk].pkt[2], devs_spoof[lk].pkt[3]);
      if (((vid>>8) & 0xff) == devs_spoof[lk].pkt[0] && (vid & 0xff) == devs_spoof[lk].pkt[1])
      {
        if (((pid>>8) & 0xff) == devs_spoof[lk].pkt[2] && (pid & 0xff) == devs_spoof[lk].pkt[3])
        {
          printf ("\n#found spoof device at %d", lk);
          ret = lk + 1;
          break;
        }
      }
    }
    lk++;
  }
  return ret;
}

int set_spoof_device (int vid, int pid)
{
  spoof_device_index = get_spoof_idx (vid, pid);
  if (-1 == spoof_device_index)
    return -1;
  //
  int lk = 0, ret = -1;
  int spdl = sizeof(spoof_handlers)/sizeof(proc_list);
  printf("\n#scanning %d spoof handlers", spdl);
  while (lk < spdl)
  {
    if (spoof_handlers[lk].vid == vid && spoof_handlers[lk].pid == pid)
    {
      printf ("\n#found spoof handler at %d", lk);
      ret = lk;
      break;
    }
    lk++;
  }
  //
  spoof_handlers_index = ret;
  if (-1 != spoof_handlers_index)
  {
    whl_report = spoof_handlers[spoof_handlers_index].whl_report;
    printf ("\n#spoof handler %s", spoof_handlers[spoof_handlers_index].pdv);
  }
  //
  return ret;
}

/*
 * the atmega32u4 supports up to 6 non-control endpoints
 * that can be IN or OUT (not BIDIR),
 * and only the INTERRUPT type is supported.
 */
static uint8_t targetProperties[ENDPOINT_MAX_NUMBER] = {
  EP_PROP_IN | EP_PROP_OUT | EP_PROP_INT,
  EP_PROP_IN | EP_PROP_OUT | EP_PROP_INT,
  EP_PROP_IN | EP_PROP_OUT | EP_PROP_INT,
  EP_PROP_IN | EP_PROP_OUT | EP_PROP_INT,
  EP_PROP_IN | EP_PROP_OUT | EP_PROP_INT,
  EP_PROP_IN | EP_PROP_OUT | EP_PROP_INT,
};
// --

long get_map (long x, long in_min, long in_max, long out_min, long out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

long get_cmap (long x, long in_min, long in_max, long out_min, long out_max)
{
  long rv = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  if (rv > out_max)
    rv = out_max;
  if (rv < out_min)
    rv = out_min;
  return rv;
}

float get_map_f (float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

//[b3 b2 b1 b0]
float get_float (char *buf, int off)
{
  long lv = (long)((buf[off+3]&0xff)<<24 | (buf[off+2]&0xff)<<16 | (buf[off+1]&0xff)<<8 | (buf[off]&0xff));
  //printf ("\n#lv n %ld", lv);
  float rv;
  //lv = ntohl (lv);
  //printf ("\n#lv h %ld", lv);
  memcpy (&rv, &lv, 4);
  return rv;
}

unsigned int get_uint (char *buf, int off)
{
  long lv = (long)((buf[off+3]&0xff)<<24 | (buf[off+2]&0xff)<<16 | (buf[off+1]&0xff)<<8 | (buf[off]&0xff));
  //printf ("\n#lv n %ld", lv);
  unsigned int rv;
  //lv = ntohl (lv);
  //printf ("\n#lv h %ld", lv);
  memcpy (&rv, &lv, 4);
  return rv;
}

int get_int (char *buf, int off)
{
  long lv = (long)((buf[off+3]&0xff)<<24 | (buf[off+2]&0xff)<<16 | (buf[off+1]&0xff)<<8 | (buf[off]&0xff));
  //printf ("\n#lv n %ld", lv);
  int rv;
  //lv = ntohl (lv);
  //printf ("\n#lv h %ld", lv);
  memcpy (&rv, &lv, 4);
  return rv;
}

unsigned short get_ushort (char *buf, int off)
{
  return (unsigned short)(buf[off+1]<<8|buf[off]);
}

short get_short (unsigned char *buf, int off)
{
  return (short)(buf[off+1]<<8|buf[off]);
}

//--PS2: Logitech DFP
int whl_ps2_df_convert (char *rep, int rl)
{
  /*
        pw_roll = normal_axis (get_short (report, 46), 0x0ffff); //from wheel turn
        lpacc   = normal_accel (get_short (report, 48), 0x0ffff);
        lpbrk   = normal_brake (get_short (report, 50), 0x0ffff);
  */
  //memcpy ((void *)&whl_report, (void *)&default_report, sizeof(s_report_dfPs2));
  //  wheel: 10B LSB
  long whl = get_cmap (get_ushort (rep, 44), 0, MAX_AXIS_VALUE_16BITS, 0, MAX_AXIS_VALUE_10BITS);
  long acc = get_cmap (get_ushort (rep, 46), 0, MAX_AXIS_VALUE_16BITS, 0, MAX_AXIS_VALUE_8BITS);
  long brk = get_cmap (get_ushort (rep, 48), 0, MAX_AXIS_VALUE_16BITS, 0, MAX_AXIS_VALUE_8BITS);
  #if 0
  long ped = get_cmap ((acc - brk) / 2 + CENTER_AXIS_VALUE_8BITS, 0, 255, 0, 255);
  printf("\nwhl %d [%02x %02x], acc %d, brk %d, ped %d", whl, whl>>8, whl&0xff, acc, brk, ped);
  //report_gtf_out[0] = (CENTER_AXIS_VALUE_10BITS >> 8);
  //report_gtf_out[1] = CENTER_AXIS_VALUE_10BITS & 0xff;
  memcpy (report_gtf_out, &whl, 2);
  //report_gtf_out[0] = whl;
  //report_gtf_out[1] = ;
  //buttons: 6B MSB
  if (rep[G29_LB_IDX] & G29_L1_MASK)
    report_gtf_out[GTF_LB_IDX] |= GTF_L1_MASK;
  if (rep[G29_LB_IDX] & G29_R1_MASK)
    report_gtf_out[GTF_LB_IDX] |= GTF_R1_MASK;
  if (rep[G29_PB_IDX] & G29_SQUARE_MASK)
    report_gtf_out[GTF_PB_IDX] |= GTF_SQUARE_MASK;
  if (rep[G29_PB_IDX] & G29_CROSS_MASK)
    report_gtf_out[GTF_PB_IDX] |= GTF_CROSS_MASK;
  if (rep[G29_PB_IDX] & G29_CIRCLE_MASK)
    report_gtf_out[GTF_PB_IDX] |= GTF_CIRCLE_MASK;
  if (rep[G29_PB_IDX] & G29_TRIANGLE_MASK)
    report_gtf_out[GTF_PB_IDX] |= G29_TRIANGLE_MASK;
  // pedals: 8B - combined
  report_gtf_out[3] = ped & 0xff;
  //    gas: 8B
  report_gtf_out[4] = acc & 0xff;
  //  brake: 8B
  report_gtf_out[5] = brk & 0xff;
  #endif
  //
  whl_ps2_df_report.buttonsAndWheel = whl & 0xffff;
  //
  if (rep[G29_LB_IDX] & G29_L1_MASK)
    whl_ps2_df_report.buttonsAndWheel |= DF_L1_MASK;
  if (rep[G29_LB_IDX] & G29_R1_MASK)
    whl_ps2_df_report.buttonsAndWheel |= DF_R1_MASK;
  if (rep[G29_PB_IDX] & G29_SQUARE_MASK)
    whl_ps2_df_report.buttonsAndWheel |= DF_SQUARE_MASK;
  if (rep[G29_PB_IDX] & G29_CROSS_MASK)
    whl_ps2_df_report.buttonsAndWheel |= DF_CROSS_MASK;
  if (rep[G29_PB_IDX] & G29_CIRCLE_MASK)
    whl_ps2_df_report.buttonsAndWheel |= DF_CIRCLE_MASK;
  if (rep[G29_PB_IDX] & G29_TRIANGLE_MASK)
    whl_ps2_df_report.buttonsAndWheel |= DF_TRIANGLE_MASK;
  //hat
  whl_ps2_df_report.hat = rep[G29_HB_IDX] & 0x0f;
  //buttons
  //whl_ps2_df_report.buttons = rep[G29_LB_IDX];
  whl_ps2_df_report.buttons = 0x00;
  if (rep[G29_LB_IDX] & G29_L2_MASK)
    whl_ps2_df_report.buttons |= DF_L2_MASK;
  if (rep[G29_LB_IDX] & G29_R2_MASK)
    whl_ps2_df_report.buttons |= DF_R2_MASK;
  if (rep[G29_LB_IDX] & G29_L3_MASK)
    whl_ps2_df_report.buttons |= DF_L3_MASK;
  if (rep[G29_LB_IDX] & G29_R3_MASK)
    whl_ps2_df_report.buttons |= DF_R3_MASK;
  if (rep[G29_LB_IDX] & G29_SHARE_MASK)
    whl_ps2_df_report.buttons |= DF_SELECT_MASK;
  if (rep[G29_LB_IDX] & G29_OPTIONS_MASK)
    whl_ps2_df_report.buttons |= DF_START_MASK;
  //whl_report.pedals = ped & 0xff;
  whl_ps2_df_report.gasPedal = acc & 0xff;
  whl_ps2_df_report.brakePedal = brk & 0xff;
  //
  return WHL_PS2_DF_REPORT_LEN;
}

/*
#i:initializing USB proxy with device 0eb7:0e04
#i:using device: VID 0x0eb7 PID 0x0e04 PATH 01:01:01:02                                                           
endpoint: OUT INTERRUPT 3               
endpoint: IN INTERRUPT 4                                                                                          
#s2u[0]: 00 00 03 00 00 00 00 00 00 00 00 00 00 00 00
#u2s[0]: 00 00 03 00 00 00 00 00 00 00 00 00 00 00 00                                                             
#s2u[1]: 00 00 00 84 00 00 00 00 00 00 00 00 00 00 00
#u2s[1]: 00 00 00 84 00 00 00 00 00 00 00 00 00 00 00                                                             
#i:starting                             
#i:sending descriptors                                                                                            
#skip sending descriptors               
#spoof dev data 4 type 00                                                                                         
#spoof dev data 5 type 01               
#spoof dev data 6 type 02                                                                                         
#i:ready descriptors                    
#i:ready descriptors                                                                                              
#i:ready indexes                        
#i:ready                                                                                                          
#ffb in 8 bytes: f3 cb 01 00 28 15 80 bf
#ffb out 32 bytes: 30 f3 cb 01 00 28 15 80 bf 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#ffb in 8 bytes: f4 cb 01 00 28 15 80 bf
#ffb out 32 bytes: 30 f4 cb 01 00 28 15 80 bf 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#ffb in 8 bytes: 09 03 00 00 00 00 00 00
#ffb out 32 bytes: 30 09 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#ffb in 8 bytes: 13 03 00 00 00 00 00 00
#ffb out 32 bytes: 30 13 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#ffb in 8 bytes: f5 00 00 00 00 00 00 00
#ffb out 32 bytes: 30 f5 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#ffb in 8 bytes: 11 08 82 80 00 00 00 00
#ffb out 32 bytes: 30 11 08 82 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#ffb in 8 bytes: f4 00 00 00 00 00 00 00
#ffb out 32 bytes: 30 f4 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#ffb in 8 bytes: 13 00 00 00 00 00 00 00
#ffb out 32 bytes: 30 13 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#ffb in 8 bytes: f5 00 00 00 00 00 00 00
#ffb out 32 bytes: 30 f5 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#ffb in 8 bytes: 11 08 82 80 00 00 00 00
#ffb out 32 bytes: 30 11 08 82 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#ffb in 8 bytes: 11 08 7f 80 00 00 00 00
#ffb out 32 bytes: 30 11 08 7f 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
*/
#define FFB_PS2_DF_REPORT_LEN  0x20
#define FFB_ps2_df_out_ep 0x03
int ffb_ps2_df_convert (unsigned char *ffbin, unsigned char *ffbot, int inlen)
{
  int i;
  if (0)
  {
    printf ("\n#ffb in %d bytes: ", inlen);
    for (i = 0; i < inlen; i++)
      printf ("%02X ", ffbin[i]);
    //ff_lg_decode_command (ffbin + 2);
    fflush (stdout);
  }
  //ff_lg_decode_command (ffbin + 2);
  //fflush (stdout);
  /*
  char cmd = ffbin[2] & 0x0f;
  char slt = ffbin[2] >> 4;
  char ftp = ffbin[3];
  printf ("\ncmd %02X slt %02X ftp %02X pars(1-5): %02X %02X %02X %02X %02X ", cmd, slt, ftp, ffbin[4], ffbin[5], ffbin[6], ffbin[7], ffbin[8]);
  fflush (stdout);
  */
  memset (ffbot, 0x00, FFB_PS2_DF_REPORT_LEN); //out/ffb report has 33 bytes so we pad with 0
  ffbot[0] = 0x30;
  memcpy (ffbot + 1, ffbin, inlen);
  //
  if (0)
  {
    printf ("\n#ffb out %d bytes: ", FFB_PS2_DF_REPORT_LEN);
    for (i = 0; i < FFB_PS2_DF_REPORT_LEN; i++)
      printf ("%02X ", ffbot[i]);
    //ff_lg_decode_command (ffbot + 2);
    fflush (stdout);
  }
  //
  return FFB_PS2_DF_REPORT_LEN;
}

//--PS3: Logitech DFGT/G25
/*
whl btn gt
|            -- -- -- -- -- -- -- -- x- -- --
#i.WHL@0080: 06 09 81 08 00 80 7e 30 a0 ff ff
#i.WHL@0180: 06 09 81 08 00 00 7e 30 20 ff ff
whl left-right
|            -- -- -- -- -- -- -- xx xx -- --
#i.WHL@0004: 06 09 81 08 00 00 7e 14 20 ff ff
#i.WHL@0006: 06 09 81 08 00 00 7e 12 20 ff ff
#i.WHL@0004: 06 09 81 08 00 00 7e 10 20 ff ff
whl accel
|            -- -- -- -- -- -- -- -- -- xx --
#i.WHL@0001: 06 09 81 08 00 00 7e 09 20 ee ff
#i.WHL@0003: 06 09 81 08 00 00 7e 09 20 f0 ff
#i.WHL@0001: 06 09 81 08 00 00 7e 09 20 f5 ff
#i.WHL@0003: 06 09 81 08 00 00 7e 09 20 f8 ff
#i.WHL@0001: 06 09 81 08 00 00 7e 09 20 ff ff
whl brake
|            -- -- -- -- -- -- -- -- -- -- xx
#i.WHL@0001: 06 09 81 08 00 00 7e 09 20 ff e0
#i.WHL@0003: 06 09 81 08 00 00 7e 09 20 ff e9
#i.WHL@0001: 06 09 81 08 00 00 7e 09 20 ff f4
#i.WHL@0003: 06 09 81 08 00 00 7e 09 20 ff f7
#i.WHL@0001: 06 09 81 08 00 00 7e 09 20 ff ff
whl btn up
|            -- -- -- -x -- -- -- -- -- -- --
#i.WHL@0001: 06 09 81 00 00 00 7e 0e 20 ff ff
#i.WHL@0245: 06 09 81 08 00 00 7e 0b 20 ff ff
whl btn dn
|            -- -- -- -x -- -- -- -- -- -- --
#i.WHL@0844: 06 09 81 04 00 00 7e 0b 20 ff ff
#i.WHL@0090: 06 09 81 08 00 00 7e 0e 20 ff ff
whl btn left
|            -- -- -- -x -- -- -- -- -- -- --
#i.WHL@0306: 06 09 81 06 00 00 7e 0e 20 ff ff
#i.WHL@0068: 06 09 81 08 00 00 7e 0b 20 ff ff
whl btn right
|            -- -- -- -x -- -- -- -- -- -- --
#i.WHL@0524: 06 09 81 02 00 00 7e 0b 20 ff ff
#i.WHL@0068: 06 09 81 08 00 00 7e 0b 20 ff ff
whl btn cross
|            -- -- -- x- -- -- -- -- -- -- --
#i.WHL@0462: 06 09 81 18 00 00 7e 0e 20 ff ff
#i.WHL@0110: 06 09 81 08 00 00 7e 0e 20 ff ff
whl btn square
|            -- -- -- x- -- -- -- -- -- -- --
#i.WHL@0310: 06 09 81 28 00 00 7e 0e 20 ff ff
#i.WHL@0092: 06 09 81 08 00 00 7e 0e 20 ff ff
whl btn triangle
|            -- -- -- x- -- -- -- -- -- -- --
#i.WHL@0446: 06 09 81 88 00 00 7e 0e 20 ff ff
#i.WHL@0098: 06 09 81 08 00 00 7e 0e 20 ff ff
whl btn circle
|            -- -- -- x- -- -- -- -- -- -- --
#i.WHL@0414: 06 09 81 48 00 00 7e 0e 20 ff ff
#i.WHL@0120: 06 09 81 08 00 00 7e 0e 20 ff ff
whl btn start
|            -- -- -- -- x- -- -- -- -- -- --
#i.WHL@0358: 06 09 81 08 20 00 7e 0b 20 ff ff
#i.WHL@0118: 06 09 81 08 00 00 7e 0b 20 ff ff
whl btn select
|            -- -- -- -- x- -- -- -- -- -- --
#i.WHL@3128: 06 09 81 08 10 00 7e 0b 20 ff ff
#i.WHL@0142: 06 09 81 08 00 00 7e 0b 20 ff ff
whl btn r1
|            -- -- -- -- -x -- -- -- -- -- --
#i.WHL@0884: 06 09 81 08 01 00 7e 1c 20 ff ff
#i.WHL@0071: 06 09 81 08 00 00 7e 1c 20 ff ff
whl btn r2
|            -- -- -- -- -x -- -- -- -- -- --
#i.WHL@0028: 06 09 81 08 04 00 7e 18 20 ff ff
#i.WHL@0091: 06 09 81 08 00 00 7e 18 20 ff ff
whl btn r3
|            -- -- -- -- x- -- -- -- -- -- --
#i.WHL@0656: 06 09 81 08 40 00 7e 18 20 ff ff
#i.WHL@0138: 06 09 81 08 00 00 7e 18 20 ff ff
whl btn l1
|            -- -- -- -- -x -- -- -- -- -- --
#i.WHL@0042: 06 09 81 08 02 00 7e 0e 20 ff ff
#i.WHL@0080: 06 09 81 08 00 00 7e 0e 20 ff ff
whl btn l2
|            -- -- -- -- -x -- -- -- -- -- --
#i.WHL@0500: 06 09 81 08 08 00 7e 0e 20 ff ff
#i.WHL@0136: 06 09 81 08 00 00 7e 0e 20 ff ff
whl btn l3
|            -- -- -- -- x- -- -- -- -- -- --
#i.WHL@0496: 06 09 81 08 80 00 7e 0e 20 ff ff
#i.WHL@0088: 06 09 81 08 00 00 7e 0e 20 ff ff
whl btn PS
|            -- -- -- -- -- -- -x -- -- -- --
#i.WHL@0752: 06 09 81 08 00 00 7f 0b 20 ff ff
#i.WHL@0162: 06 09 81 08 00 00 7e 0b 20 ff ff
whl btn red/dial push
|            -- -- -- -- -- -x -- -- -- -- --
#i.WHL@0490: 06 09 81 08 00 04 7e 0e 20 ff ff
#i.WHL@0138: 06 09 81 08 00 00 7e 0e 20 ff ff
whl btn red/dial left
|            -- -- -- -- -- x- -- -- -- -- --
#i.WHL@0044: 06 09 81 08 00 20 7e 0b 20 ff ff
#i.WHL@0004: 06 09 81 08 00 00 7e 0b 20 ff ff
whl btn red/dial right
|            -- -- -- -- -- x- -- -- -- -- --
#i.WHL@0044: 06 09 81 08 00 10 7e 0b 20 ff ff
#i.WHL@0004: 06 09 81 08 00 00 7e 0b 20 ff ff
whl btn plus
|            -- -- -- -- -- -x -- -- -- -- --
#i.WHL@0465: 06 09 81 08 00 08 7e 01 20 ff ff
#i.WHL@0183: 06 09 81 08 00 00 7e 01 20 ff ff
whl btn minus
|            -- -- -- -- -- x- -- -- -- -- --
#i.WHL@0652: 06 09 81 08 00 40 7e 01 20 ff ff
#i.WHL@0203: 06 09 81 08 00 00 7e 01 20 ff ff--
whl left-right
|            -- -- -- -- -- -- -- xx xx -- --
#i.WHL@0004: 06 09 81 08 00 00 7e 14 20 ff ff
--
typedef struct __attribute__((packed))
{
  unsigned char endpoint;
  unsigned char hatAndButtons;
  unsigned char buttons;  //start, select, l1, l2, l3, r1, r2, r3
  unsigned char buttons2; //red/dial:button, left, right, plus, minus
  unsigned char buttons3; //MSB(B0)PS button
  unsigned short wheel;   // 14bit = wheel axis + LSB(B7) GT button
  unsigned char gasPedal;
  unsigned char brakePedal;
} s_report_dfgtPs3;
--
static s_report_dfgtPs3 whl_ps3_dfgt_report =
{
  .endpoint = 0x81,
  .hatAndButtons = 0x08,
  .buttons = 0x00,
  .buttons2 = 0x00,
  .buttons3 = 0x7e,
  .wheel = CENTER_AXIS_VALUE_14BITS, // 16bit wheel
  .gasPedal = MAX_AXIS_VALUE_8BITS,
  .brakePedal = MAX_AXIS_VALUE_8BITS,
};

*/
int whl_ps3_dfgt_convert (char *rep, int rl)
{
  if (0)
  {
    printf ("\n#whl in %d bytes: ", rl);
    for (int i = 0; i < rl; i++)
      printf ("%02X ", rep[i]);
    //ff_lg_decode_command (ffbin + 2);
    fflush (stdout);
  }
  //wheel and pedals
  long whl = get_cmap (get_ushort (rep, 44), 0, MAX_AXIS_VALUE_16BITS, 0, MAX_AXIS_VALUE_14BITS);
  long acc = get_cmap (get_ushort (rep, 46), 0, MAX_AXIS_VALUE_16BITS, 0, MAX_AXIS_VALUE_8BITS);
  long brk = get_cmap (get_ushort (rep, 48), 0, MAX_AXIS_VALUE_16BITS, 0, MAX_AXIS_VALUE_8BITS);
  //hat and buttons
  whl_ps3_dfgt_report.hatAndButtons = rep[G29_HB_IDX] & 0x0f;
  //
  #define DFGT_SQUARE_MASK    0x20
  #define DFGT_CROSS_MASK     0x10
  #define DFGT_CIRCLE_MASK    0x40
  #define DFGT_TRIANGLE_MASK  0x80
  //
  if (rep[G29_PB_IDX] & G29_SQUARE_MASK)
    whl_ps3_dfgt_report.hatAndButtons |= DFGT_SQUARE_MASK;
  if (rep[G29_PB_IDX] & G29_CROSS_MASK)
    whl_ps3_dfgt_report.hatAndButtons |= DFGT_CROSS_MASK;
  if (rep[G29_PB_IDX] & G29_CIRCLE_MASK)
    whl_ps3_dfgt_report.hatAndButtons |= DFGT_CIRCLE_MASK;
  if (rep[G29_PB_IDX] & G29_TRIANGLE_MASK)
    whl_ps3_dfgt_report.hatAndButtons |= DFGT_TRIANGLE_MASK;
  //buttons
  whl_ps3_dfgt_report.buttons = 0x00;
  if (rep[G29_LB_IDX] & G29_L1_MASK)
    whl_ps3_dfgt_report.buttons |= G27_L1_MASK;
  if (rep[G29_LB_IDX] & G29_R1_MASK)
    whl_ps3_dfgt_report.buttons |= G27_R1_MASK;
  if (rep[G29_LB_IDX] & G29_L2_MASK)
    whl_ps3_dfgt_report.buttons |= G27_L2_MASK;
  if (rep[G29_LB_IDX] & G29_R2_MASK)
    whl_ps3_dfgt_report.buttons |= G27_R2_MASK;
  if (rep[G29_LB_IDX] & G29_L3_MASK)
    whl_ps3_dfgt_report.buttons |= G27_L3_MASK;
  if (rep[G29_LB_IDX] & G29_R3_MASK)
    whl_ps3_dfgt_report.buttons |= G27_R3_MASK;
  if (rep[G29_LB_IDX] & G29_SHARE_MASK)
    whl_ps3_dfgt_report.buttons |= G27_SELECT_MASK;
  if (rep[G29_LB_IDX] & G29_OPTIONS_MASK)
    whl_ps3_dfgt_report.buttons |= G27_START_MASK;
  //
  if (rep[G29_PS_IDX] & G29_PS_MASK)
    whl_ps3_dfgt_report.buttons3 = 0x7f;
  else
    whl_ps3_dfgt_report.buttons3 = 0x7e;
  //
  whl_ps3_dfgt_report.wheel = (whl & 0xffff);//<<2;//((whl>>8) & 0xff) | ((whl & 0xff)<<8);
  whl_ps3_dfgt_report.gasPedal = acc & 0xff;
  whl_ps3_dfgt_report.brakePedal = brk & 0xff;
  //
  if (0)
  {
    extern char *whl_report;
    printf ("\n#whl out %d bytes: ", WHL_PS3_DFGT_REPORT_LEN);
    for (int i = 0; i < WHL_PS3_DFGT_REPORT_LEN; i++)
      printf ("%02X ", whl_report[i]);
    //ff_lg_decode_command (ffbin + 2);
    fflush (stdout);
  }
  return WHL_PS3_DFGT_REPORT_LEN;
}

#define FFB_PS3_DFGT_REPORT_LEN  0x20
#define FFB_ps3_dfgt_out_ep 0x03
int ffb_ps3_dfgt_convert (unsigned char *ffbin, unsigned char *ffbot, int inlen)
{
  int i;
  if (0)
  {
    printf ("\n#ffb in %d bytes: ", inlen);
    for (i = 0; i < inlen; i++)
      printf ("%02X ", ffbin[i]);
    //ff_lg_decode_command (ffbin + 2);
    fflush (stdout);
  }
  //ff_lg_decode_command (ffbin + 2);
  //fflush (stdout);
  /*
  char cmd = ffbin[2] & 0x0f;
  char slt = ffbin[2] >> 4;
  char ftp = ffbin[3];
  printf ("\ncmd %02X slt %02X ftp %02X pars(1-5): %02X %02X %02X %02X %02X ", cmd, slt, ftp, ffbin[4], ffbin[5], ffbin[6], ffbin[7], ffbin[8]);
  fflush (stdout);
  */
  memset (ffbot, 0x00, FFB_PS3_DFGT_REPORT_LEN); //out/ffb report has 33 bytes so we pad with 0
  ffbot[0] = 0x30;
  memcpy (ffbot + 1, ffbin, inlen);
  //
  if (0)
  {
    printf ("\n#ffb out %d bytes: ", FFB_PS3_DFGT_REPORT_LEN);
    for (i = 0; i < FFB_PS3_DFGT_REPORT_LEN; i++)
      printf ("%02X ", ffbot[i]);
    //ff_lg_decode_command (ffbot + 2);
    fflush (stdout);
  }
  //
  return FFB_PS3_DFGT_REPORT_LEN;
}

//--PS3: Logitech G27
int whl_ps3_g27_convert (char *rep, int rl)
{
  if (0)
  {
    printf ("\n#whl in %d bytes: ", rl);
    for (int i = 0; i < rl; i++)
      printf ("%02X ", rep[i]);
    //ff_lg_decode_command (ffbin + 2);
    fflush (stdout);
  }
  //wheel and pedals
  long whl = get_cmap (get_ushort (rep, 44), 0, MAX_AXIS_VALUE_16BITS, 0, MAX_AXIS_VALUE_14BITS);
  long acc = get_cmap (get_ushort (rep, 46), 0, MAX_AXIS_VALUE_16BITS, 0, MAX_AXIS_VALUE_8BITS);
  long brk = get_cmap (get_ushort (rep, 48), 0, MAX_AXIS_VALUE_16BITS, 0, MAX_AXIS_VALUE_8BITS);
  //hat and buttons
  whl_ps3_g27_report.hatAndButtons = rep[G29_HB_IDX] & 0x0f;
  //
  #define DFGT_SQUARE_MASK    0x20
  #define DFGT_CROSS_MASK     0x10
  #define DFGT_CIRCLE_MASK    0x40
  #define DFGT_TRIANGLE_MASK  0x80
  //
  if (rep[G29_PB_IDX] & G29_SQUARE_MASK)
    whl_ps3_g27_report.hatAndButtons |= DFGT_SQUARE_MASK;
  if (rep[G29_PB_IDX] & G29_CROSS_MASK)
    whl_ps3_g27_report.hatAndButtons |= DFGT_CROSS_MASK;
  if (rep[G29_PB_IDX] & G29_CIRCLE_MASK)
    whl_ps3_g27_report.hatAndButtons |= DFGT_CIRCLE_MASK;
  if (rep[G29_PB_IDX] & G29_TRIANGLE_MASK)
    whl_ps3_g27_report.hatAndButtons |= DFGT_TRIANGLE_MASK;
  //buttons
  whl_ps3_g27_report.buttons = 0x00;
  if (rep[G29_LB_IDX] & G29_L1_MASK)
    whl_ps3_g27_report.buttons |= G27_L1_MASK;
  if (rep[G29_LB_IDX] & G29_R1_MASK)
    whl_ps3_g27_report.buttons |= G27_R1_MASK;
  if (rep[G29_LB_IDX] & G29_L2_MASK)
    whl_ps3_g27_report.buttons |= G27_L2_MASK;
  if (rep[G29_LB_IDX] & G29_R2_MASK)
    whl_ps3_g27_report.buttons |= G27_R2_MASK;
  if (rep[G29_LB_IDX] & G29_L3_MASK)
    whl_ps3_g27_report.buttons |= G27_L3_MASK;
  if (rep[G29_LB_IDX] & G29_R3_MASK)
    whl_ps3_g27_report.buttons |= G27_R3_MASK;
  if (rep[G29_LB_IDX] & G29_SHARE_MASK)
    whl_ps3_g27_report.buttons |= G27_SELECT_MASK;
  if (rep[G29_LB_IDX] & G29_OPTIONS_MASK)
    whl_ps3_g27_report.buttons |= G27_START_MASK;
  //
  //whl_ps3_dfgt_report.psButton = rep[G29_PS_IDX];
  //
  whl_ps3_g27_report.buttonsAndWheel = (whl & 0xffff)<<2;//((whl>>8) & 0xff) | ((whl & 0xff)<<8);
  if (rep[G29_LB_IDX] & G29_OPTIONS_MASK) // for NFS Shift 2
    whl_ps3_g27_report.buttonsAndWheel |= G27_L5_MASK;
  if (rep[G29_LB_IDX] & G29_SHARE_MASK)
    whl_ps3_g27_report.buttonsAndWheel |= G27_L4_MASK;
  //
  whl_ps3_g27_report.gasPedal = acc & 0xff;
  whl_ps3_g27_report.brakePedal = brk & 0xff;
  //
  if (0)
  {
    extern char *whl_report;
    printf ("\n#whl in %d bytes: ", WHL_PS3_G27_REPORT_LEN);
    for (int i = 0; i < WHL_PS3_G27_REPORT_LEN; i++)
      printf ("%02X ", whl_report[i]);
    //ff_lg_decode_command (ffbin + 2);
    fflush (stdout);
  }  //
  return WHL_PS3_G27_REPORT_LEN;
}

#define FFB_PS3_G27_REPORT_LEN  0x20
int ffb_ps3_g27_convert (unsigned char *ffbin, unsigned char *ffbot, int inlen)
{
  int i;
  if (0)
  {
    printf ("\n#ffb in %d bytes: ", inlen);
    for (i = 0; i < inlen; i++)
      printf ("%02X ", ffbin[i]);
    //ff_lg_decode_command (ffbin + 2);
    fflush (stdout);
  }
  //ff_lg_decode_command (ffbin + 2);
  //fflush (stdout);
  /*
  char cmd = ffbin[2] & 0x0f;
  char slt = ffbin[2] >> 4;
  char ftp = ffbin[3];
  printf ("\ncmd %02X slt %02X ftp %02X pars(1-5): %02X %02X %02X %02X %02X ", cmd, slt, ftp, ffbin[4], ffbin[5], ffbin[6], ffbin[7], ffbin[8]);
  fflush (stdout);
  */
  memset (ffbot, 0x00, FFB_PS3_G27_REPORT_LEN); //out/ffb report has 33 bytes so we pad with 0
  ffbot[0] = 0x30;
  memcpy (ffbot + 1, ffbin, inlen);
  //
  if (0)
  {
    printf ("\n#ffb out %d bytes: ", FFB_PS3_G27_REPORT_LEN);
    for (i = 0; i < FFB_PS3_G27_REPORT_LEN; i++)
      printf ("%02X ", ffbot[i]);
    //ff_lg_decode_command (ffbot + 2);
    fflush (stdout);
  }
  //
  return FFB_PS3_G27_REPORT_LEN;
}
//
//--end spoof
//*****************************************************************************
//
static int send_next_in_packet()
{

  if (inPending)
  {
    return 0;
  }

  if (nbInEpFifo > 0)
  {
    uint8_t inPacketIndex = ENDPOINT_ADDR_TO_INDEX(inEpFifo[0]);
    int ret = 0;
    if (spoof_device_index != -1)
    {
      char *buf = (char *)&inPackets[inPacketIndex].packet;
      int bsz = inPackets[inPacketIndex].length;
      ret = adapter_send(adapter, E_TYPE_IN, (const unsigned char *)spoof_handlers[spoof_handlers_index].whl_report, spoof_handlers[spoof_handlers_index].whl (buf, bsz));
    }
    else
      ret = adapter_send(adapter, E_TYPE_IN, (const void *)&inPackets[inPacketIndex].packet, inPackets[inPacketIndex].length);
    if(ret < 0)
    {
      return -1;
    }
    inPending = inEpFifo[0];
    //printf ("\n#send_next_in_packet %d inPending", inPending);
    //fflush (stdout);
    --nbInEpFifo;
    memmove(inEpFifo, inEpFifo + 1, nbInEpFifo * sizeof(*inEpFifo));
  }

  return 0;
}

static int queue_in_packet(unsigned char endpoint, const void * buf, int transfered)
{

  if (nbInEpFifo == sizeof(inEpFifo) / sizeof(*inEpFifo))
  {
    PRINT_ERROR_OTHER("no more space in inEpFifo")
    return -1;
  }
  //printf("\n#queue_in_packet ep %02X vs %02X idx %02X", endpoint, U2S_ENDPOINT(endpoint), ENDPOINT_ADDR_TO_INDEX(endpoint));
  uint8_t inPacketIndex = ENDPOINT_ADDR_TO_INDEX(endpoint);
  inPackets[inPacketIndex].packet.endpoint = U2S_ENDPOINT(endpoint);
  memcpy(inPackets[inPacketIndex].packet.data, buf, transfered);
  inPackets[inPacketIndex].length = transfered + 1;
  inEpFifo[nbInEpFifo] = endpoint;
  ++nbInEpFifo;

  /*
   * TODO MLA: Poll the endpoint after registering the packet?
   */

  return 0;
}

int usb_read_callback(int user, unsigned char endpoint, const void * buf, int status)
{
  switch (status)
  {
  case E_TRANSFER_TIMED_OUT:
    PRINT_TRANSFER_READ_ERROR (endpoint, "TIMEOUT")
    break;
  case E_TRANSFER_STALL:
    break;
  case E_TRANSFER_ERROR:
    PRINT_TRANSFER_WRITE_ERROR (endpoint, "OTHER ERROR")
    return -1;
  default:
    break;
  }

  if (endpoint == 0)
  {
    if (status > (int)MAX_PACKET_VALUE_SIZE)
    {
      PRINT_ERROR_OTHER ("too many bytes transfered")
      done = 1;
      return -1;
    }

    int ret;
    if (status >= 0)
    {
      ret = adapter_send (adapter, E_TYPE_CONTROL, buf, status);
    }
    else
    {
      ret = adapter_send (adapter, E_TYPE_CONTROL_STALL, NULL, 0);
    }
    if(ret < 0)
    {
      return -1;
    }
  }
  else
  {
    if (status > MAX_PAYLOAD_SIZE_EP)
    {
      PRINT_ERROR_OTHER ("too many bytes transfered")
      done = 1;
      return -1;
    }

    if (status >= 0)
    {
      int ret = queue_in_packet (endpoint, buf, status);
      if (ret < 0)
      {
        done = 1;
        return -1;
      }

      ret = send_next_in_packet ();
      if (ret < 0)
      {
        done = 1;
        return -1;
      }
    }
  }

  return 0;
}

int usb_write_callback (int user, unsigned char endpoint, int status)
{

  switch (status)
  {
  case E_TRANSFER_TIMED_OUT:
    PRINT_TRANSFER_WRITE_ERROR (endpoint, "TIMEOUT")
    break;
  case E_TRANSFER_STALL:
    if (endpoint == 0)
    {
      int ret = adapter_send (adapter, E_TYPE_CONTROL_STALL, NULL, 0);
      if (ret < 0)
      {
        done = 1;
        return -1;
      }
    }
    break;
  case E_TRANSFER_ERROR:
    PRINT_TRANSFER_WRITE_ERROR (endpoint, "OTHER ERROR")
    return -1;
  default:
    if (endpoint == 0)
    {
      int ret = adapter_send (adapter, E_TYPE_CONTROL, NULL, 0);
      if (ret < 0)
      {
        done = 1;
        return -1;
      }
    }
    break;
  }

  return 0;
}

int usb_close_callback(int user)
{
  done = 1;
  return 1;
}

int adapter_send_callback (int user, int transfered)
{
  if (transfered < 0)
  {
    done = 1;
    return 1;
  }

  return 0;
}

int adapter_close_callback(int user)
{
  done = 1;
  return 1;
}

static char * usb_select (int vid, int pid) 
{
  char * path = NULL;
  //
  s_usb_dev * usb_devs = gusb_enumerate(0x0000, 0x0000);
  if (usb_devs == NULL) 
  {
//	    fprintf (stderr, "\n#e:no USB device detected!");
    fprintf (stdout, "\n#e:no USB device detected!");
    return NULL;
  }
  //printf ("Available USB devices:\n");
  unsigned int index = 0;
  //char vendor[128], product[128];
  s_usb_dev * current;
  unsigned int choice = UINT_MAX;
  for (current = usb_devs; current != NULL; ++current) 
  {
    //get_vendor_string(vendor, sizeof(vendor), current->vendor_id);
    //get_product_string(product, sizeof(product), current->vendor_id, current->product_id);
    //printf("%2d", index);
    index++;
    //printf(" VID 0x%04x (%s)", current->vendor_id, strlen(vendor) ? vendor : "unknown vendor");
    //printf(" PID 0x%04x (%s)", current->product_id, strlen(product) ? product : "unknown product");
    //printf(" PATH %s\n", current->path);
    //fflush (stdout);
    //auto select T300RS
    if (current->product_id == pid && current->vendor_id == vid)
    {
      choice = index - 1;
    }
    if (current->next == 0) 
    {
      break;
    }
  }
  //
  //printf("Selected the USB device number: %d\n", choice);
  //fflush (stdout);
  if (choice < index) //scanf("%d", &choice) == 1 && choice < index) 
  {
    path = strdup (usb_devs[choice].path);
    if(path == NULL) 
    {
//        fprintf (stderr, "\n#e:USB device: can't duplicate path!");
        fprintf (stdout, "\n#e:USB device: can't duplicate path!");
    }
  } else 
  {
//	    fprintf (stderr, "\n#e:USB device not found!");
    fprintf (stdout, "\n#e:USB device not found!");
  }
  //
  gusb_free_enumeration (usb_devs);
  //
  return path;
}

void print_endpoint_properties(uint8_t epProps[ENDPOINT_MAX_NUMBER])
{
  unsigned char i;
  for (i = 0; i < ENDPOINT_MAX_NUMBER; ++i)
  {
    if (epProps[i] != 0)
    {
      printf("%hhu", i + 1);
      if (epProps[i] & EP_PROP_IN)
      {
        printf(" IN");
      }
      if (epProps[i] & EP_PROP_OUT)
      {
        printf(" OUT");
      }
      if (epProps[i] & EP_PROP_BIDIR)
      {
        printf(" BIDIR");
      }
      if (epProps[i] & EP_PROP_INT)
      {
        printf(" INTERRUPT");
      }
      if (epProps[i] & EP_PROP_BLK)
      {
        printf(" BULK");
      }
      if (epProps[i] & EP_PROP_ISO)
      {
        printf(" ISOCHRONOUS");
      }
      printf("\n");
    }
  }
}

void get_endpoint_properties (unsigned char configurationIndex, uint8_t epProps[ENDPOINT_MAX_NUMBER])
{
  struct p_configuration * pConfiguration = descriptors->configurations + configurationIndex;
  unsigned char interfaceIndex;
  for (interfaceIndex = 0; interfaceIndex < pConfiguration->descriptor->bNumInterfaces; ++interfaceIndex) 
  {
    struct p_interface * pInterface = pConfiguration->interfaces + interfaceIndex;
    unsigned char altInterfaceIndex;
    for (altInterfaceIndex = 0; altInterfaceIndex < pInterface->bNumAltInterfaces; ++altInterfaceIndex) 
    {
      struct p_altInterface * pAltInterface = pInterface->altInterfaces + altInterfaceIndex;
      unsigned char endpointIndex;
      for (endpointIndex = 0; endpointIndex < pAltInterface->bNumEndpoints; ++endpointIndex) 
      {
        struct usb_endpoint_descriptor * endpoint =
            descriptors->configurations[configurationIndex].interfaces[interfaceIndex].altInterfaces[altInterfaceIndex].endpoints[endpointIndex];
        uint8_t epIndex = ENDPOINT_ADDR_TO_INDEX(endpoint->bEndpointAddress);
        switch (endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) 
        {
        case USB_ENDPOINT_XFER_INT:
          epProps[epIndex] |= EP_PROP_INT;
          break;
        case USB_ENDPOINT_XFER_BULK:
          epProps[epIndex] |= EP_PROP_BLK;
          break;
        case USB_ENDPOINT_XFER_ISOC:
          epProps[epIndex] |= EP_PROP_ISO;
          break;
        }
        if ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) 
        {
          epProps[epIndex] |= EP_PROP_IN;
        }
        else 
        {
          epProps[epIndex] |= EP_PROP_OUT;
        }
        if ((epProps[epIndex] & (EP_PROP_IN | EP_PROP_OUT)) == (EP_PROP_IN | EP_PROP_OUT)) 
        {
          epProps[epIndex] |= EP_PROP_BIDIR;
        }
      }
    }
  }
}

int compare_endpoint_properties (uint8_t epPropsSource[ENDPOINT_MAX_NUMBER], uint8_t epPropsTarget[ENDPOINT_MAX_NUMBER])
{

  unsigned char i;
  for (i = 0; i < ENDPOINT_MAX_NUMBER; ++i) 
  {
    if (epPropsSource[i] != 0) 
    {
      if ((epPropsTarget[i] & epPropsSource[i]) != epPropsSource[i]) 
      {
        return 1;
      }
    }
  }

  return 0;
}

void fix_endpoints() 
{

  pEndpoints = endpoints;

  unsigned char configurationIndex;
  for (configurationIndex = 0; configurationIndex < descriptors->device.bNumConfigurations; ++configurationIndex) 
  {
    uint8_t sourceProperties[ENDPOINT_MAX_NUMBER] = {};
    get_endpoint_properties(configurationIndex, sourceProperties);
    /*print_endpoint_properties(usedEndpoints);
    print_endpoint_properties(endpointProperties);*/
    int renumber = compare_endpoint_properties(sourceProperties, targetProperties);
    unsigned char endpointNumber = 0;
    struct p_configuration * pConfiguration = descriptors->configurations + configurationIndex;
    //printf("configuration: %hhu\n", pConfiguration->descriptor->bConfigurationValue);
    unsigned char interfaceIndex;
    for (interfaceIndex = 0; interfaceIndex < pConfiguration->descriptor->bNumInterfaces; ++interfaceIndex) 
    {
      struct p_interface * pInterface = pConfiguration->interfaces + interfaceIndex;
      unsigned char altInterfaceIndex;
      for (altInterfaceIndex = 0; altInterfaceIndex < pInterface->bNumAltInterfaces; ++altInterfaceIndex) 
      {
        struct p_altInterface * pAltInterface = pInterface->altInterfaces + altInterfaceIndex;
        //printf("  interface: %hhu:%hhu\n", pAltInterface->descriptor->bInterfaceNumber, pAltInterface->descriptor->bAlternateSetting);
        unsigned char endpointIndex;
        for (endpointIndex = 0; endpointIndex < pAltInterface->bNumEndpoints; ++endpointIndex) 
        {
          struct usb_endpoint_descriptor * endpoint =
              descriptors->configurations[configurationIndex].interfaces[interfaceIndex].altInterfaces[altInterfaceIndex].endpoints[endpointIndex];
          uint8_t originalEndpoint = endpoint->bEndpointAddress;
          if (renumber) 
          {
            ++endpointNumber;
            endpoint->bEndpointAddress = (endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) | endpointNumber;
          }
          if (1)
          {
            printf("\nendpoint:");
            printf(" %s", ((endpoint->bEndpointAddress & USB_ENDPOINT_DIR_MASK) == USB_DIR_IN) ? "IN" : "OUT");
            printf(" %s",
                (endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_INT ? "INTERRUPT" :
                (endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_BULK ? "BULK" :
                (endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) == USB_ENDPOINT_XFER_ISOC ?
                    "ISOCHRONOUS" : "UNKNOWN");
            printf(" %hu", originalEndpoint & USB_ENDPOINT_NUMBER_MASK);
            
            if (originalEndpoint != endpoint->bEndpointAddress) 
            {
              printf(KRED" -> %hu"KNRM, endpointNumber);
            }
            //printf("\n");
          }
          if ((originalEndpoint & USB_ENDPOINT_NUMBER_MASK) == 0) 
          {
            PRINT_ERROR_OTHER("invalid endpoint number")
            continue;
          }
          if (configurationIndex > 0) 
          {
            continue;
          }
          if ((endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK) != USB_ENDPOINT_XFER_INT) 
          {
            //printf("      endpoint %hu won't be configured (not an INTERRUPT endpoint)\n", endpoint->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK);
            continue;
          }
          if (endpoint->wMaxPacketSize > MAX_PAYLOAD_SIZE_EP) 
          {
            //printf("      endpoint %hu won't be configured (max packet size %hu > %hu)\n", endpoint->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK, endpoint->wMaxPacketSize, MAX_PAYLOAD_SIZE_EP);
            continue;
          }
          if (endpointNumber > MAX_ENDPOINTS) 
          {
            //printf("      endpoint %hu won't be configured (endpoint number %hhu > %hhu)\n", endpoint->bEndpointAddress & USB_ENDPOINT_NUMBER_MASK, endpointNumber, MAX_ENDPOINTS);
            continue;
          }
          U2S_ENDPOINT(originalEndpoint) = endpoint->bEndpointAddress;
          S2U_ENDPOINT(endpoint->bEndpointAddress) = originalEndpoint;
          pEndpoints->number = endpoint->bEndpointAddress;
          pEndpoints->type = endpoint->bmAttributes & USB_ENDPOINT_XFERTYPE_MASK;
          pEndpoints->size = endpoint->wMaxPacketSize;
          ++pEndpoints;
        }
      }
    }
  }
}

static int add_descriptor(uint16_t wValue, uint16_t wIndex, uint16_t wLength, void * data) 
{

  if (pDesc + wLength > desc + MAX_DESCRIPTORS_SIZE || pDescIndex >= descIndex + MAX_DESCRIPTORS) 
  {
    fprintf(stderr, "%s:%d %s: unable to add descriptor wValue=0x%04x wIndex=0x%04x wLength=%u (available=%u)\n",
        __FILE__, __LINE__, __func__, wValue, wIndex, wLength, (unsigned int)(MAX_DESCRIPTORS_SIZE - (pDesc - desc)));
    return -1;
  }

  pDescIndex->offset = pDesc - desc;
  pDescIndex->wValue = wValue;
  pDescIndex->wIndex = wIndex;
  pDescIndex->wLength = wLength;
  memcpy(pDesc, data, wLength);
  pDesc += wLength;
  ++pDescIndex;

  return 0;
}

int send_descriptors() 
{

  int ret;

  ret = add_descriptor ((USB_DT_DEVICE << 8), 0, sizeof(descriptors->device), &descriptors->device);
  if (ret < 0)
  {
    return -1;
  }

  ret = add_descriptor ((USB_DT_STRING << 8), 0, sizeof(descriptors->langId0), &descriptors->langId0);
  if (ret < 0)
  {
    return -1;
  }

  unsigned int descNumber;
  for(descNumber = 0; descNumber < descriptors->device.bNumConfigurations; ++descNumber) 
  {

    ret = add_descriptor ((USB_DT_CONFIG << 8) | descNumber, 0, descriptors->configurations[descNumber].descriptor->wTotalLength, descriptors->configurations[descNumber].raw);
    if (ret < 0)
    {
      return -1;
    }
  }

  for(descNumber = 0; descNumber < descriptors->nbOthers; ++descNumber) 
  {

    ret = add_descriptor (descriptors->others[descNumber].wValue, descriptors->others[descNumber].wIndex, descriptors->others[descNumber].wLength, descriptors->others[descNumber].data);
    if (ret < 0)
    {
      return -1;
    }
  }

  if (spoof_device_index == -1)
    ret = adapter_send (adapter, E_TYPE_DESCRIPTORS, desc, pDesc - desc);
  else
  {
    printf ("\n#skip sending descriptors");
    ret = 0;
  }
  if (ret < 0)
  {
    return -1;
  }

  return 0;
}

static int send_index ()
{

  if (descIndexSent)
  {
    return 0;
  }

  descIndexSent = 1;

  return adapter_send (adapter, E_TYPE_INDEX, (unsigned char *)&descIndex, (pDescIndex - descIndex) * sizeof(*descIndex));
}

static int send_endpoints() 
{

  if (endpointsSent)
  {
    return 0;
  }

  endpointsSent = 1;

  return adapter_send (adapter, E_TYPE_ENDPOINTS, (unsigned char *)&endpoints, (pEndpoints - endpoints) * sizeof(*endpoints));
}

static int poll_all_endpoints ()
{

  int ret = 0;
  unsigned char i;
  for (i = 0; i < sizeof(*serialToUsbEndpoint) / sizeof(**serialToUsbEndpoint) && ret >= 0; ++i)
  {
    uint8_t endpoint = S2U_ENDPOINT (USB_DIR_IN | i);
    if (endpoint)
    {
      ret = gusb_poll (usb, endpoint);
      //printf ("\n#polling EP %d vs %d ret %d", endpoint, i, ret);
    }
  }
  return ret;
}

/*
#ffb 8 bytes: F3 CB 01 00 28 15 80 BF
#ffb in 8 bytes: f3 cb 01 00 28 15 80 bf
#ffb out 64 bytes: 30 f3 cb 01 00 28 15 80 bf 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#ffb 64 bytes: 30 F3 CB 01 00 28 15 80 BF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 lib/gasync/src/usb/gusb.c:256 submit_transfer: libusb_submit_transfer failed with error: Invalid parameter

#i:ready                                                                                                                                                                                         
#ffb 32 bytes: 05 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#ffb 64 bytes: 30 F8 09 01 A0 03 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#ffb 64 bytes: 30 F8 09 01 06 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00                                            
#ffb 64 bytes: 30 F8 09 01 03 00 00 01 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#ffb 64 bytes: 30 F8 81 FF FF 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#ffb 64 bytes: 30 F5 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
*/
unsigned char ffb_packet[256];
static int send_out_packet(s_packet * packet) 
{
  s_endpointPacket * epPacket = (s_endpointPacket *)packet->value;
  unsigned char *buf = (unsigned char *)epPacket->data;
  int  bsz = packet->header.length - 1;
  //#define S2U_ENDPOINT(ENDPOINT) serialToUsbEndpoint[ENDPOINT_DIR_TO_INDEX(ENDPOINT)][ENDPOINT_ADDR_TO_INDEX(ENDPOINT)]
  epPacket->endpoint = spoof_handlers[spoof_handlers_index].ffb_out_ep;
  if (0)
  {
    //printf ("\n#ffb s2u (%d) = s2u [%d][%d] ", epPacket->endpoint, ENDPOINT_DIR_TO_INDEX(epPacket->endpoint), ENDPOINT_ADDR_TO_INDEX(epPacket->endpoint));
    printf ("\n#ffb %d bytes on ep %02X vs %02X: ", bsz, S2U_ENDPOINT(epPacket->endpoint), epPacket->endpoint);
    for (int i = 0; i < bsz; i++)
      printf ("%02X ", buf[i]);
    fflush (stdout);
  }
  //
  if (spoof_device_index != -1)
  {
    
    unsigned char *buf = (unsigned char *)epPacket->data;
    int  bsz = packet->header.length - 1;
    bsz = spoof_handlers[spoof_handlers_index].ffb (buf, ffb_packet, bsz);
    buf = ffb_packet;
    if (0)
    {
      printf ("\n#ffb %d bytes for ep %02X: ", bsz, S2U_ENDPOINT(epPacket->endpoint));
      for (int i = 0; i < bsz; i++)
        printf ("%02X ", buf[i]);
      fflush (stdout);
    }
    //PS2: GT4 is doing something weird on Fanatec sending 06 and F6 positions, so avoid it
    if (buf[1] == 0x11 && (buf[3] > 0xF5 || buf[3] < 0x07))
    {
      printf ("\n#ffb drop: range out of bounds %02X", buf[3]);
      return 0;
    }
    //
    //return gusb_write (usb, S2U_ENDPOINT(epPacket->endpoint), buf, bsz);
    //return gusb_write (usb, 0x03, buf, bsz);
    return gusb_write (usb, S2U_ENDPOINT(epPacket->endpoint), buf, bsz);
  }
  else
    return gusb_write (usb, S2U_ENDPOINT(epPacket->endpoint), epPacket->data, packet->header.length - 1);
}

static int send_control_packet(s_packet * packet) 
{

  struct usb_ctrlrequest * setup = (struct usb_ctrlrequest *)packet->value;
  if ((setup->bRequestType & USB_RECIP_MASK) == USB_RECIP_ENDPOINT) 
  {
    if (setup->wIndex != 0) {
      setup->wIndex = S2U_ENDPOINT(setup->wIndex);
    }
  }

  return gusb_write (usb, 0, packet->value, packet->header.length);
}

static void dump(unsigned char * data, unsigned char length)
{
  int i;
  for (i = 0; i < length; ++i)
  {
    if(i && !(i % 8))
    {
      printf("\n");
    }
    printf("0x%02x ", data[i]);
  }
  printf("\n");
}

static int process_packet(int user, s_packet * packet)
{
  unsigned char type = packet->header.type;
  if (adapter_debug (0xff) & 0x0f)
    fprintf (stdout, "\n#i:process pkt type 0x%x", type);
  int ret = 0;

  switch (packet->header.type)
  {
  case E_TYPE_DESCRIPTORS:
    if (spoof_device_index == -1)
      ret = send_index ();
    if (1 || adapter_debug (0xff) & 0x0f)
    {
      fprintf (stdout, "\n#i:ready descriptors");
      fflush (stdout);
    }
    break;
  case E_TYPE_INDEX:
    if (spoof_device_index == -1)
      ret = send_endpoints ();
    if (1 || adapter_debug (0xff) & 0x0f)
    {
      fprintf (stdout, "\n#i:ready indexes");
      fflush (stdout);
    }
    break;
  case E_TYPE_ENDPOINTS:
    if (spoof_device_index == -1)
      gtimer_close (init_timer);
    init_timer = -1;
    printf ("\n#i:ready");
    fflush (stdout);
    ret = poll_all_endpoints ();
    break;
  case E_TYPE_IN:
    if (inPending > 0) 
    {
      ret = gusb_poll (usb, inPending);
      inPending = 0;
      if (ret != -1) 
      {
        ret = send_next_in_packet ();
        if (adapter_debug (0xff) & 0x0f)
        {
          fprintf (stdout, "\n#i:next IN packet");
          fflush (stdout);
        }
      }
    }
    break;
  case E_TYPE_OUT:
    ret = send_out_packet (packet);
    if (adapter_debug (0xff) & 0x0f)
    {
      fprintf (stdout, "\n#i:ready out pkt");
      fflush (stdout);
    }
    break;
  case E_TYPE_CONTROL:
    ret = send_control_packet (packet);
    if (adapter_debug (0xff) & 0x0f)
    {
      fprintf (stdout, "\n#i:ready ctrl");
      fflush (stdout);
    }
    break;
  case E_TYPE_DEBUG:
    {
      struct timeval tv;
      gettimeofday (&tv, NULL);
      printf ("\n#w:%ld.%06ld debug packet received (size = %d bytes)\n", tv.tv_sec, tv.tv_usec, packet->header.length);
      dump (packet->value, packet->header.length);
    }
    break;
  case E_TYPE_RESET:
    ret = -1;
    break;
  default:
    {
      struct timeval tv;
      gettimeofday (&tv, NULL);
          fprintf (stdout, "%ld.%06ld ", tv.tv_sec, tv.tv_usec);
      fprintf (stdout, "unhandled packet (type=0x%02x)\n", type);
    }
    break;
  }

  if (ret < 0)
  {
    done = 1;
  }
  return ret;
}

void print_endpoints()
{
  //static uint8_t serialToUsbEndpoint[2][ENDPOINT_MAX_NUMBER] = {};
  //static uint8_t usbToSerialEndpoint[2][ENDPOINT_MAX_NUMBER] = {};
  for (int i = 0; i < 2; i++)
  {
    printf ("\n#s2u[%d]:", i);
    for (int j = 0; j < ENDPOINT_MAX_NUMBER; j++)
      printf (" %02X", serialToUsbEndpoint[i][j]);
    //
    printf ("\n#u2s[%d]:", i);
    for (int j = 0; j < ENDPOINT_MAX_NUMBER; j++)
      printf (" %02X", usbToSerialEndpoint[i][j]);
  }
}

int proxy_init (int vid, int pid) 
{

  char * path = usb_select (vid, pid);

  if (path == NULL) 
  {
    return -1;
  }
  //
  usb = gusb_open_path (path);

  if (usb < 0) 
  {
    free (path);
    return -1;
  }

  descriptors = gusb_get_usb_descriptors (usb);
  if (descriptors == NULL) 
  {
    free (path);
    return -1;
  }

  printf("\n#i:using device: VID 0x%04x PID 0x%04x PATH %s", descriptors->device.idVendor, descriptors->device.idProduct, path);

  free(path);

  if (descriptors->device.bNumConfigurations == 0) {
    PRINT_ERROR_OTHER ("missing configuration")
    return -1;
  }

  if (descriptors->configurations[0].descriptor->bNumInterfaces == 0) {
    PRINT_ERROR_OTHER ("missing interface")
    return -1;
  }

  if (descriptors->configurations[0].interfaces[0].bNumAltInterfaces == 0) {
    PRINT_ERROR_OTHER ("missing altInterface")
    return -1;
  }

  fix_endpoints ();
  print_endpoints ();
  //
  return 0;
}

static int timer_close (int user) 
{
  done = 1;
  return 1;
}

static int timer_read (int user) 
{
  /*
   * Returning a non-zero value will make gpoll return,
   * this allows to check the 'done' variable.
   */
  return 1;
}

int proxy_start (char * port) 
{

  int ret = set_prio ();
  if (ret < 0)
  {
    PRINT_ERROR_OTHER ("\n#e:failed to set process priority!")
    return -1;
  }

  adapter = adapter_open (port, process_packet, adapter_send_callback, adapter_close_callback);

  //adapter_send (adapter, E_TYPE_RESET, NULL, 0);

  if(adapter < 0) 
  {
    return -1;
  }
  if (1|| adapter_debug (0xff) & 0x0f)
    printf ("\n#i:sending descriptors");
  if (send_descriptors () < 0)
  {
    return -1;
  }
  //
  if (spoof_device_index != -1)
  {
    //send spoof device
    //spoof_len = sizeof (ft_0eb7_0e04);
    //printf("\n#spoof dev len %d", spoof_len);
    int lk = spoof_device_index;
    while (devs_spoof[lk].type != E_TYPE_RESET)
    {
      printf("\n#spoof dev data %d type %02X", lk, devs_spoof[lk].type);
      if (adapter_send (adapter, devs_spoof[lk].type, devs_spoof[lk].pkt, devs_spoof[lk].len) < 0)
      {
        printf("\n#!ERR:spoof dev data %d type %02X", lk, devs_spoof[lk].type);
        return -1;
      }
      lk++;
    }
  }

  if (spoof_device_index == -1)
  {
    init_timer = gtimer_start (0, 1000000, timer_close, timer_close, gpoll_register_fd);
    if (init_timer < 0) 
    {
      return -1;
    }
  }

  if (adapter_debug (0xff) & 0x0f)
  {
    printf ("\n#i:started init timer");
  }
  ret = gusb_register (usb, 0, usb_read_callback, usb_write_callback, usb_close_callback, gpoll_register_fd);
  if (ret < 0)
  {
    return -1;
  }

  if (adapter_debug (0xff) & 0x0f)
  {
    printf ("\n#i:started polling timer");
  }
  int timer = gtimer_start (0, 10000, timer_read, timer_close, gpoll_register_fd);
  if (timer < 0) 
  {
    return -1;
  }

  while (!done) 
  {
    gpoll ();
  }

  printf ("\n#i:cleaning up");
  gtimer_close (timer);
  adapter_send (adapter, E_TYPE_RESET, NULL, 0);
  gusb_close (usb);
  adapter_close ();
  
  if (init_timer >= 0) 
  {
    //PRINT_ERROR_OTHER("Failed to start the proxy: initialization timeout expired!")
    printf ("\n#e:failed to start the job, closing");
    gtimer_close (init_timer);
    return -1;
  }
  //

  return 0;
}

void proxy_stop ()
{
  done = 1;
}
