/* Linux
 *
 * build:
apt install libhidapi-dev libhidapi-hidraw0 libhidapi-libusb0

gcc -o fanatec_play fanatec_play.c -lusb-1.0

 *
 *  */
#include <linux/types.h>
#include <linux/input.h>
#include <linux/hidraw.h>

/*
 * Ugly hack to work around failing compilation on systems that don't
 * yet populate new version of hidraw.h to userspace.
 */
#ifndef HIDIOCSFEATURE
#warning Please have your distro update the userspace kernel headers
#define HIDIOCSFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x06, len)
#define HIDIOCGFEATURE(len)    _IOC(_IOC_WRITE|_IOC_READ, 'H', 0x07, len)
#endif

/* Unix */
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>

/* C */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <errno.h>
#include <signal.h>

//#include "../../emu/adapter_protocol.h"

#define uint8_t unsigned char

#define BYTE_OUT_REPORT  7
#define BYTE_SEND_REPORT 6

#define NAME_LENGTH 128

//#include "../shared/emu_adapter.h"

#define FF_DEBUG 1
#define dprintf(...) \
    do { \
        if(FF_DEBUG) { \
            printf(__VA_ARGS__); \
            fflush(stdout); \
        } \
    } while (0)

static void hdump(const unsigned char* packet, unsigned char length) {
    int i;
    for (i = 0; i < length; ++i) {
        printf("0x%02x ", packet[i]);
    }
}

#define USB_VENDOR_ID_LOGITECH                  0x046d

#define USB_PRODUCT_ID_LOGITECH_FORMULA_YELLOW   0xc202 // no force feedback
#define USB_PRODUCT_ID_LOGITECH_FORMULA_GP       0xc20e // no force feedback
#define USB_PRODUCT_ID_LOGITECH_FORMULA_FORCE    0xc291 // i-force protocol
#define USB_PRODUCT_ID_LOGITECH_FORMULA_FORCE_GP 0xc293 // classic protocol
#define USB_PRODUCT_ID_LOGITECH_DRIVING_FORCE    0xc294 // classic protocol
#define USB_PRODUCT_ID_LOGITECH_MOMO_WHEEL       0xc295 // classic protocol
#define USB_PRODUCT_ID_LOGITECH_DFP_WHEEL        0xc298 // classic protocol
#define USB_PRODUCT_ID_LOGITECH_G25_WHEEL        0xc299 // classic protocol
#define USB_PRODUCT_ID_LOGITECH_DFGT_WHEEL       0xc29a // classic protocol
#define USB_PRODUCT_ID_LOGITECH_G27_WHEEL        0xc29b // classic protocol
#define USB_PRODUCT_ID_LOGITECH_WII_WHEEL        0xc29c // rumble only
#define USB_PRODUCT_ID_LOGITECH_MOMO_WHEEL2      0xca03 // classic protocol
#define USB_PRODUCT_ID_LOGITECH_VIBRATION_WHEEL  0xca04 // rumble only
#define USB_PRODUCT_ID_LOGITECH_G920_WHEEL       0xc262 // hid++ protocol only
#define USB_PRODUCT_ID_LOGITECH_G29_PC_WHEEL     0xc24f // classic protocol
#define USB_PRODUCT_ID_LOGITECH_G29_PS4_WHEEL    0xc260 // classic protocol with 1 byte offset

#define FF_LG_OUTPUT_REPORT_SIZE 7

#define FF_LG_FSLOTS_NB 4
#define FF_LG_FSLOTS_OFFSET 4

#define FF_LG_FSLOT_MASK 0xf0

#define FF_LG_FSLOT_1 0x10
#define FF_LG_FSLOT_2 0x20
#define FF_LG_FSLOT_3 0x40
#define FF_LG_FSLOT_4 0x80

#define FF_LG_CMD_MASK 0x0f

#define FF_LG_CMD_DOWNLOAD           0x00
#define FF_LG_CMD_DOWNLOAD_AND_PLAY  0x01
#define FF_LG_CMD_PLAY               0x02
#define FF_LG_CMD_STOP               0x03
#define FF_LG_CMD_DEFAULT_SPRING_ON  0x04
#define FF_LG_CMD_DEFAULT_SPRING_OFF 0x05
#define FF_LG_CMD_RESERVED_1         0x06
#define FF_LG_CMD_RESERVED_2         0x07
#define FF_LG_CMD_NORMAL_MODE        0x08
#define FF_LG_CMD_EXTENDED_COMMAND   0xF8
#define FF_LG_CMD_SET_LED            0x09
#define FF_LG_CMD_SET_WATCHDOG       0x0A
#define FF_LG_CMD_RAW_MODE           0x0B
#define FF_LG_CMD_REFRESH_FORCE      0x0C
#define FF_LG_CMD_FIXED_TIME_LOOP    0x0D
#define FF_LG_CMD_SET_DEFAULT_SPRING 0x0E
#define FF_LG_CMD_SET_DEAD_BAND      0x0F

#define FF_LG_EXT_CMD_NB 16

#define FF_LG_EXT_CMD_CHANGE_MODE_DFP           0x01
#define FF_LG_EXT_CMD_WHEEL_RANGE_200_DEGREES   0x02
#define FF_LG_EXT_CMD_WHEEL_RANGE_900_DEGREES   0x03
#define FF_FT_EXT_CMD_CHANGE_MODE               0x09
#define FF_LG_EXT_CMD_REVERT_IDENTITY           0x0a
#define FF_LG_EXT_CMD_CHANGE_MODE_G25           0x10
#define FF_LG_EXT_CMD_CHANGE_MODE_G25_NO_DETACH 0x11
#define FF_LG_EXT_CMD_SET_RPM_LEDS              0x12
#define FF_FT_EXT_CMD_SET_RPM_LEDS              0x13
#define FF_LG_EXT_CMD_CHANGE_WHEEL_RANGE        0x81

#define FF_LG_FTYPE_CONSTANT                           0x00
#define FF_LG_FTYPE_SPRING                             0x01
#define FF_LG_FTYPE_DAMPER                             0x02
#define FF_LG_FTYPE_AUTO_CENTER_SPRING                 0x03
#define FF_LG_FTYPE_SAWTOOTH_UP                        0x04
#define FF_LG_FTYPE_SAWTOOTH_DOWN                      0x05
#define FF_LG_FTYPE_TRAPEZOID                          0x06
#define FF_LG_FTYPE_RECTANGLE                          0x07
#define FF_LG_FTYPE_VARIABLE                           0x08
#define FF_LG_FTYPE_RAMP                               0x09
#define FF_LG_FTYPE_SQUARE_WAVE                        0x0A
#define FF_LG_FTYPE_HIGH_RESOLUTION_SPRING             0x0B
#define FF_LG_FTYPE_HIGH_RESOLUTION_DAMPER             0x0C
#define FF_LG_FTYPE_HIGH_RESOLUTION_AUTO_CENTER_SPRING 0x0D
#define FF_LG_FTYPE_FRICTION                           0x0E

static const char * cmd_names[] = {
    [FF_LG_CMD_DOWNLOAD]           = "DOWNLOAD",
    [FF_LG_CMD_DOWNLOAD_AND_PLAY]  = "DOWNLOAD_AND_PLAY",
    [FF_LG_CMD_PLAY]               = "PLAY",
    [FF_LG_CMD_STOP]               = "STOP",
    [FF_LG_CMD_DEFAULT_SPRING_ON]  = "DEFAULT_SPRING_ON",
    [FF_LG_CMD_DEFAULT_SPRING_OFF] = "DEFAULT_SPRING_OFF",
    [FF_LG_CMD_RESERVED_1]         = "RESERVED_1",
    [FF_LG_CMD_RESERVED_2]         = "RESERVED_2",
    [FF_LG_CMD_NORMAL_MODE]        = "NORMAL_MODE",
    [FF_LG_CMD_SET_LED]            = "SET_LED",
    [FF_LG_CMD_SET_WATCHDOG]       = "SET_WATCHDOG",
    [FF_LG_CMD_RAW_MODE]           = "RAW_MODE",
    [FF_LG_CMD_REFRESH_FORCE]      = "REFRESH_FORCE",
    [FF_LG_CMD_FIXED_TIME_LOOP]    = "FIXED_TIME_LOOP",
    [FF_LG_CMD_SET_DEFAULT_SPRING] = "SET_DEFAULT_SPRING",
    [FF_LG_CMD_SET_DEAD_BAND]      = "SET_DEAD_BAND",
};

const char * ff_lg_get_cmd_name(unsigned char header) {
    if (header == FF_LG_CMD_EXTENDED_COMMAND) {
        return "EXTENDED_COMMAND";
    } else {
        unsigned char cmd = header & FF_LG_CMD_MASK;
        if (cmd < sizeof(cmd_names) / sizeof(*cmd_names)) {
            return cmd_names[cmd];
        } else {
            return "UNKNOWN";
        }
    }
}

char * slot_names[] = {
        [0b0000] = "",
        [0b0001] = "1",
        [0b0010] = "2",
        [0b0011] = "1,2",
        [0b0100] = "3",
        [0b0101] = "1,3",
        [0b0110] = "2,3",
        [0b0111] = "1,2,3",
        [0b1000] = "4",
        [0b1001] = "1,4",
        [0b1010] = "2,4",
        [0b1011] = "1,2,4",
        [0b1100] = "3,4",
        [0b1101] = "1,3,4",
        [0b1110] = "2,3,4",
        [0b1111] = "1,2,3,4",
};

const char * ff_lg_get_slot_names(unsigned char header) {
    if (header == FF_LG_CMD_EXTENDED_COMMAND) {
        return "";
    } else {
        return slot_names[header >> 4];
    }
}

static struct {
    unsigned char value;
    const char * name;
} ext_cmd_names[] = {
    { FF_LG_EXT_CMD_CHANGE_MODE_DFP,           "CHANGE_MODE_DFP" },
    { FF_LG_EXT_CMD_WHEEL_RANGE_200_DEGREES,   "WHEEL_RANGE_200_DEGREES" },
    { FF_LG_EXT_CMD_WHEEL_RANGE_900_DEGREES,   "WHEEL_RANGE_900_DEGREES" },
    { FF_FT_EXT_CMD_CHANGE_MODE,               "WHL_LEDS_NR" },
    { FF_LG_EXT_CMD_REVERT_IDENTITY,           "REVERT_IDENTITY" },
    { FF_LG_EXT_CMD_CHANGE_MODE_G25,           "CHANGE_MODE_G25" },
    { FF_LG_EXT_CMD_CHANGE_MODE_G25_NO_DETACH, "CHANGE_MODE_G25_NO_DETACH" },
    { FF_LG_EXT_CMD_SET_RPM_LEDS,              "SET_RPM_LEDS" },
    { FF_FT_EXT_CMD_SET_RPM_LEDS,              "RPM_LEDS" },
    { FF_LG_EXT_CMD_CHANGE_WHEEL_RANGE,        "CHANGE_WHEEL_RANGE" },
};

const char * ff_lg_get_ext_cmd_name(unsigned char ext) {
    unsigned int i;
    for (i = 0; i < sizeof(ext_cmd_names) / sizeof(*ext_cmd_names); ++i) {
        if(ext_cmd_names[i].value == ext) {
            return ext_cmd_names[i].name;
        }
    }
    static char unknown[] = "UNKNOWN (255) ";
    snprintf(unknown, sizeof(unknown), "UNKNOWN (0x%x)", ext);
    return unknown;
}

static const char * ftype_names [] = {
    [FF_LG_FTYPE_CONSTANT]                           = "CONSTANT",
    [FF_LG_FTYPE_SPRING]                             = "SPRING",
    [FF_LG_FTYPE_DAMPER]                             = "DAMPER",
    [FF_LG_FTYPE_AUTO_CENTER_SPRING]                 = "AUTO_CENTER_SPRING",
    [FF_LG_FTYPE_SAWTOOTH_UP]                        = "SAWTOOTH_UP",
    [FF_LG_FTYPE_SAWTOOTH_DOWN]                      = "SAWTOOTH_DOWN",
    [FF_LG_FTYPE_TRAPEZOID]                          = "TRAPEZOID",
    [FF_LG_FTYPE_RECTANGLE]                          = "RECTANGLE",
    [FF_LG_FTYPE_VARIABLE]                           = "VARIABLE",
    [FF_LG_FTYPE_RAMP]                               = "RAMP",
    [FF_LG_FTYPE_SQUARE_WAVE]                        = "SQUARE_WAVE",
    [FF_LG_FTYPE_HIGH_RESOLUTION_SPRING]             = "HIGH_RESOLUTION_SPRING",
    [FF_LG_FTYPE_HIGH_RESOLUTION_DAMPER]             = "HIGH_RESOLUTION_DAMPER",
    [FF_LG_FTYPE_HIGH_RESOLUTION_AUTO_CENTER_SPRING] = "HIGH_RESOLUTION_AUTO_CENTER_SPRING",
    [FF_LG_FTYPE_FRICTION]                           = "FRICTION",
};

const char * ff_lg_get_ftype_name(unsigned char ftype) {
    if (ftype < sizeof(ftype_names) / sizeof(*ftype_names)) {
        return ftype_names[ftype];
    } else {
        return "UNKNOWN";
    }
}

void ff_lg_decode_extended(const unsigned char data[FF_LG_OUTPUT_REPORT_SIZE]) {

    dprintf("%s %s", ff_lg_get_cmd_name(data[0]), ff_lg_get_ext_cmd_name(data[1]));

    switch(data[1]) {
    case FF_LG_EXT_CMD_CHANGE_MODE_DFP:
    case FF_LG_EXT_CMD_WHEEL_RANGE_200_DEGREES:
    case FF_LG_EXT_CMD_WHEEL_RANGE_900_DEGREES:
    case FF_LG_EXT_CMD_CHANGE_MODE_G25:
    case FF_LG_EXT_CMD_CHANGE_MODE_G25_NO_DETACH:
      break;
#if 0
    case FF_FT_EXT_CMD_CHANGE_MODE:
    {
      const char * mode = NULL;
      switch(data[2]) {
      case 0x00:
        mode = "Logitech Driving Force EX";
        break;
      case 0x01:
        mode = "Logitech Driving Force Pro";
        break;
      case 0x02:
        mode = "Logitech G25 Racing Wheel";
        break;
      case 0x03:
        mode = "Logitech Driving Force GT";
        break;
      case 0x04:
        mode = "Logitech G27 Racing Wheel";
        break;
      case 0x08:
        mode = "WHL LED";
        break;
      case 0x13:
        mode = "RPM LED";
        break;
      }
      if(mode == NULL) {
          dprintf(" - unknown mode (0x%02x)", data[2]);
      }
      else {
          dprintf(" - %s", mode);
      }
      dprintf(" - %s", data[3] ? "DETACH" : "NO DETACH");
    }
      break;
#endif
    case FF_LG_EXT_CMD_REVERT_IDENTITY:
      dprintf(" - %s", data[2] ? "REVERT" : "DO NOT REVERT");
      break;
    case FF_LG_EXT_CMD_SET_RPM_LEDS:
      dprintf(" - 0x%02x", data[2]);
      break;
    case FF_LG_EXT_CMD_CHANGE_WHEEL_RANGE:
      dprintf(" - %hu", (data[3] << 8) | data[2]);
      break;
    default:
      dprintf(" - ");
      hdump(data + 2, FF_LG_OUTPUT_REPORT_SIZE - 2);
      break;
    }
    //dprintf("\n");
}

void ff_lg_decode_command(const unsigned char data[FF_LG_OUTPUT_REPORT_SIZE])
{
    if(data[0] == FF_LG_CMD_EXTENDED_COMMAND) {
        ff_lg_decode_extended(data);
        return;
    }

    dprintf("%s ", ff_lg_get_cmd_name(data[0]));
    const char * slots = ff_lg_get_slot_names(data[0]);
    if (*slots != '\0') {
        dprintf("- %s", slots);
    }

    switch(data[0] & FF_LG_CMD_MASK) {
    case FF_LG_CMD_PLAY:
    case FF_LG_CMD_STOP:
        break;
    case FF_LG_CMD_DOWNLOAD:
    case FF_LG_CMD_DOWNLOAD_AND_PLAY:
    case FF_LG_CMD_REFRESH_FORCE:
        dprintf(" - %s", ff_lg_get_ftype_name(data[1]));
        dprintf(" - ");
        hdump(data + 2, FF_LG_OUTPUT_REPORT_SIZE - 2);
        break;
    case FF_LG_CMD_DEFAULT_SPRING_ON:
    case FF_LG_CMD_DEFAULT_SPRING_OFF:
    case FF_LG_CMD_NORMAL_MODE:
    case FF_LG_CMD_RAW_MODE:
        break;
    case FF_LG_CMD_SET_LED:
        dprintf(" - 0x%02x", data[1]);
        break;
    case FF_LG_CMD_SET_WATCHDOG:
        dprintf(" - 0x%02x", data[1]);
        break;
    case FF_LG_CMD_FIXED_TIME_LOOP:
        dprintf(" - %s", data[1] ? "ON" : "OFF");
        break;
    case FF_LG_CMD_SET_DEFAULT_SPRING:
        dprintf(" - ");
        hdump(data + 1, FF_LG_OUTPUT_REPORT_SIZE - 1);
        break;
    case FF_LG_CMD_SET_DEAD_BAND:
        dprintf(" - %s", data[1] ? "ON" : "OFF");
        break;
    default:
        dprintf("!UNK");
        break;
    }
    //dprintf("\n");
}

/*
 * The reference report data.
 *
 * T300RS Report is composed of 15B data in game mode
 *
 radix: hexadecimal
 07 18 80 FF 03 FF 03 FF 03 00 00 00 00 00 0F

 DIRT3 + GT6 map

 B0:

 B1 + B2: X axis - wheel

 B3+B4: brake - FF to 0 (down)

 B5+B6: accel - FF to 0 (down)

 B7+B8: clutch?

 B9:  ??
 B10: ??

 B11:
 0x01 l shifter
 0x02 r shifter
 0x20 cross
 0x04 triangle
 0x08 rectangle
 0x10 circle
 0x80 start / option
 0x40 select / share

 B12:
 0x01 R2
 0x02 L2
 0x08 R3
 0x04 L3
 0x10 PS

 B13:

 B14:
 0x0f hat no button
 0x00 hat up
 0x01 hat right-up
 0x02 hat right
 0x03 hat down-right
 0x04 hat down
 0x05 hat down-left
 0x06 hat left
 0x07 hat up-left

 *
 */
#define REPORT_LEN  15

static uint8_t report[] =
  {
//ref: 08 00 00 AC 87 FF FF FF 7B 8B 1C
      0x08,//buttons+hat
      0x00, //extra buttons + L/R shift
      0x00, //gears H shifter
      0x00, 0x80, //wheel axis
      0xFF, //accel
      0xFF, //brake
      0xFF, 0x83, 0x8A, 0x18 //gear R?
    };

#if 0
int
adapter_send_report (int id)
{
  struct
    __attribute__ ((packed))
    {
      struct
        __attribute__ ((packed))
        {
          unsigned char type;
          unsigned char length;
        } header;
        unsigned char value[REPORT_LEN];
      } packet =
        { .header =
          { .type = BYTE_SEND_REPORT, .length = REPORT_LEN } };
      memcpy (packet.value, report, REPORT_LEN);
      if (serial_send (id, &packet, sizeof(packet)) != sizeof(packet))
      {
        return -1;
      }

      return 0;
    }
#endif

/*
 *
 * lights test
001.skip > #
002.FFB <250> [064]: 30 f8 81 2f 02 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
003.FFB <250> [064]: 30 f8 09 08 01 ff 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
004.FFB <250> [064]: 30 13 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
005.skip > #lights
#uses f8 09 08 with bitfield 01 ff: 9 bits correspond to the 9 leds (3 yellow, 3 red, 3 blue)
#rim light follows the pattern: yellow leds = yellow light, red leds = red light, blue leds = white light
006.FFB <250> [064]: 30 f8 09 08 01 ff 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
007.FFB <250> [064]: 30 f8 09 08 00 ff 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
008.FFB <250> [064]: 30 f8 09 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
009.FFB <250> [064]: 30 f8 09 08 01 55 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
010.skip > #no lights
011.FFB <250> [064]: 30 f8 09 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
012.skip > #stop
013.FFB <250> [064]: 30 03 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
014.FFB <250> [064]: 30 11 0b 80 80 ff 00 1a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
Done 20sec, 14lines!
 *
 *
 * ffb test: GTS model
001.skip > #
002.FFB <250> [064]: 30 f8 81 2f 02 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
003.FFB <250> [064]: 30 f8 09 08 01 ff 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#start?
004.FFB <250> [064]: 30 13 0b 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
005.skip > #ffb
# uses 01 08 with <pos>: L<ff..80..01>R
006.FFB <250> [064]: 30 01 08 80 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
007.FFB <250> [064]: 30 01 08 70 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
008.FFB <250> [064]: 30 01 08 60 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
009.FFB <250> [064]: 30 01 08 70 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
010.FFB <250> [064]: 30 01 08 90 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
011.FFB <250> [064]: 30 01 08 a0 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
012.FFB <250> [064]: 30 01 08 90 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
013.skip > #stop
#reset wheel to center
014.FFB <250> [064]: 30 03 08 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
#stop: center spring?!
015.FFB <250> [064]: 30 11 0b 80 80 ff 00 1a 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 *
 */
    const char *
    bus_str (int bus);

    int
    main (int argc, char **argv)
    {
      int fd;
      int i, res, desc_size = 0, _intv = 0, _nomot = 0;
      char buf[256], *_mflt = NULL, _dwhl[] = "/dev/hidraw0";;
      struct hidraw_report_descriptor rpt_desc;
      struct hidraw_devinfo info;
      /*
       *
       */
      //roll and pitch percentage
      for (int i = 1; i < argc; i++)
      {
        if (argv[i][0] == '-')
          switch (argv[i][1])
          {
            case 'd': //hidraw device to use
              _dwhl[11] = *(argv[i]+2);
              printf ("\nuse device '%s'\n", _dwhl);
              break;
            case 'i': //interval between msgs
              _intv = atoi (argv[i]+2);
              printf ("\nuse delay of %dms\n", _intv);
              break;
            case 'm': //interval between msgs
              _nomot++;
              printf ("\ndon't send data to wheel\n");
              break;
            case 'f': //use only lines containing _mflt string
              _mflt = argv[i] + 2;
              printf ("\nuse filter '%s'\n", _mflt);
              break;
          }
      }
      /* Open the Device with non-blocking reads. In real life,
       don't use a hard coded path; use libudev instead. */
      fd = open (_dwhl, O_RDWR | O_NONBLOCK);

      if (fd < 0)
      {
        printf ("open input device failed 0x%x, err 0x%x\r\n", errno, fd);
        perror ("Unable to open device");
        return 1;
      }

      memset (&rpt_desc, 0x0, sizeof(rpt_desc));
      memset (&info, 0x0, sizeof(info));
      memset (buf, 0x0, sizeof(buf));

      /* Get Report Descriptor Size */
      res = ioctl (fd, HIDIOCGRDESCSIZE, &desc_size);
      if (res < 0)
        perror ("HIDIOCGRDESCSIZE");
      else
        printf ("Report Descriptor Size: %d\n", desc_size);

      /* Get Report Descriptor */
      rpt_desc.size = desc_size;
      res = ioctl (fd, HIDIOCGRDESC, &rpt_desc);
      if (res < 0)
      {
        perror ("HIDIOCGRDESC");
      }
      else
      {
        printf ("Report Descriptor:\n");
        for (i = 0; i < rpt_desc.size; i++)
          printf ("%hhx ", rpt_desc.value[i]);
        puts ("\n");
      }

      /* Get Raw Name */
      res = ioctl (fd, HIDIOCGRAWNAME(256), buf);
      if (res < 0)
        perror ("HIDIOCGRAWNAME");
      else
        printf ("Raw Name: %s\n", buf);

      /* Get Physical Location */
      res = ioctl (fd, HIDIOCGRAWPHYS(256), buf);
      if (res < 0)
        perror ("HIDIOCGRAWPHYS");
      else
        printf ("Raw Phys: %s\n", buf);

      /* Get Raw Info */
      res = ioctl (fd, HIDIOCGRAWINFO, &info);
      if (res < 0)
      {
        perror ("HIDIOCGRAWINFO");
      }
      else
      {
        printf ("Raw Info:\n");
        printf ("\tbustype: %d (%s)\n", info.bustype, bus_str (info.bustype));
        printf ("\tvendor: 0x%04hx\n", info.vendor);
        printf ("\tproduct: 0x%04hx\n", info.product);
      }
      /*
       * main loop
       *
       */
      printf ("#i:ready, using device '%s'", _dwhl);
      int rkntr = 0, kl = 0;
      int ret = 0;
#define LINE_MAX  255
      char line[LINE_MAX];
      FILE* fp;
      fp = fopen (argv[1], "r");
      //fp = stdin;
      if (!fp)
      {
        printf ("Can not open '%s'\n", argv[1]);
        return 1;
      }
      //else
        //printf ("Using '%s'\n", argv[1]);
      char dbuf[LINE_MAX];
      //ffb line example
      //<meta>: <pkt type> <pkt len> <data..>
      //dat: 07 41 03 30 f8 81 2f 02 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00
      while (fgets (line, LINE_MAX, fp) && !ret)
      {
        kl++;
        printf ("\n%03d.", kl);
        if (line[0] != '/' && line[0] != '\n')
        {
          int value, length, i, pktype;
          int delay = 20;
          int index = 0;
          //remove line ending
          char *stc = strchr (line, '\n');
          if (stc)
            *stc = '\0';
          stc = strchr (line, '\r');
          if (stc)
            *stc = '\0';
          //apply filter
          if (_mflt && strstr (line, _mflt) == NULL)
          {
            printf ("filter skip > %s", line);
            continue;
          }
          //look for valid data start
          stc = strchr (line, ':');
          if (!stc)
          {
            printf ("\nInvalid line: %s", line);
            //ret = -1;
            continue;
          }
          index += stc - line + 1;
          //read delay
          stc = strchr (line, '@');
          //printf ("\nprocessing data: %s", line + index);
          if (stc)
          {
            if (sscanf (stc + 1, "%04d", &value) == 1)
            {
              delay = value;
              //printf ("delay %dms\n", delay);
            }
          }
          //read pkt type
          if (sscanf (line + index, "%02x", &value) < 1)
          {
            printf ("\nInvalid pkt type line: %s", line);
            //ret = -1;
            continue;
          }
          index += 3;
          pktype = value;
          //read length
          if (sscanf (line + index, "%02x", &value) < 1)
          {
            printf ("\nInvalid length line: %s", line);
            //ret = -1;
            continue;
          }
          index += 3;
          length = value;
          //pktype = BYTE_OUT_REPORT;
          //printf ("type %d ", pktype);
          //printf ("len %db\n", length);
          //read data buffer
          for (i = 0; i < length; i++)
          {
            if (sscanf (line + index, "%02x", &value) < 1)
            {
              printf ("invalid data nr %d index %d, line: %s", i, index, line);
              //ret = -1;
              break;
            }
            index += 3;
            dbuf[i] = (char)value;
          }
          if (i != length)
            continue; //don't have enough data as advertised
          //read all data
          int retval;
          switch (pktype)
          {
            case BYTE_OUT_REPORT:
              {
                //if (!(dbuf[0] == (char)0x03 && dbuf[3] > (char)0x70))
                {
                  if (_intv)
                    delay = _intv;
                  printf ("FFB@%04d [%03d]: ", delay, length);
                  for (i = 0; i < length; i++)
                    printf ("%02x ", (unsigned char)dbuf[i]);
                  fflush (stdout);
                  //send data to device
                  usleep (delay * 1000);
                  rkntr += delay;
                  //decode command
                  ff_lg_decode_command (dbuf + 2);
                  //
                  if (_nomot)
                    retval = length-1;
                  else
                    retval = write (fd, dbuf + 1, length - 1);
                  if (retval != length - 1)
                    printf ("\n!!!FFB data write error %db\n", retval);
                }
              }
              break;
            case BYTE_SEND_REPORT:
              {
                if (1)
                {
                  printf ("WHL@%04d [%03d]: ", delay, length);
                  for (i = 0; i < length; i++)
                    printf ("%02x ", (unsigned char)dbuf[i]);
                  fflush (stdout);
                  //print data from device
                  if (_intv)
                    delay = _intv;
                  usleep (delay * 1000);
                  rkntr += delay;
                  //
                }
              }
              break;
            default:
              ;
          }
        }
        else
        {
          printf ("skip > %s", line);
        }
      }
      printf ("\nDone %dsec, %dlines!\r\n", rkntr / 1000, kl);
      //
      close (fd);
      return 0;
    }

    const char *
    bus_str (int bus)
    {
      switch (bus)
        {
        case BUS_USB:
          return "USB";
          break;
        case BUS_HIL:
          return "HIL";
          break;
        case BUS_BLUETOOTH:
          return "Bluetooth";
          break;
        case BUS_VIRTUAL:
          return "Virtual";
          break;
        default:
          return "Other";
          break;
        }
    }
