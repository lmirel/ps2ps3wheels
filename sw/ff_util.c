//ff_util.c

#include <ff_lg.h>

static void dump(const unsigned char* packet, unsigned char length) {
    int i;
    for (i = 0; i < length; ++i) {
        printf("0x%02x ", packet[i]);
    }
}


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
        [0b0001] = "slot 1",
        [0b0010] = "slot 2",
        [0b0011] = "slots 1,2",
        [0b0100] = "slot 3",
        [0b0101] = "slots 1,3",
        [0b0110] = "slots 2,3",
        [0b0111] = "slots 1,2,3",
        [0b1000] = "slot 4",
        [0b1001] = "slots 1,4",
        [0b1010] = "slots 2,4",
        [0b1011] = "slots 1,2,4",
        [0b1100] = "slots 3,4",
        [0b1101] = "slots 1,3,4",
        [0b1110] = "slots 2,3,4",
        [0b1111] = "slots 1,2,3,4",
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
    { FF_LG_EXT_CMD_CHANGE_MODE,               "CHANGE_MODE" },
    { FF_LG_EXT_CMD_REVERT_IDENTITY,           "REVERT_IDENTITY" },
    { FF_LG_EXT_CMD_CHANGE_MODE_G25,           "CHANGE_MODE_G25" },
    { FF_LG_EXT_CMD_CHANGE_MODE_G25_NO_DETACH, "CHANGE_MODE_G25_NO_DETACH" },
    { FF_LG_EXT_CMD_SET_RPM_LEDS,              "SET_RPM_LEDS" },
    { FF_LG_EXT_CMD_CHANGE_WHEEL_RANGE,        "CHANGE_WHEEL_RANGE" },
};

const char * ff_lg_get_ext_cmd_name(unsigned char ext) {
    unsigned int i;
    for (i = 0; i < sizeof(ext_cmd_names) / sizeof(*ext_cmd_names); ++i) {
        if(ext_cmd_names[i].value == ext) {
            return ext_cmd_names[i].name;
        }
    }
    static char unknown[] = "UNKNOWN (255)";
    snprintf(unknown, sizeof(unknown), "UNKNOWN %hu", ext);
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
    case FF_LG_EXT_CMD_CHANGE_MODE:
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
      dump(data + 2, FF_LG_OUTPUT_REPORT_SIZE - 2);
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
        dump(data + 2, FF_LG_OUTPUT_REPORT_SIZE - 2);
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
        dump(data + 1, FF_LG_OUTPUT_REPORT_SIZE - 1);
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
//--
/*
void ff_lg_convert_slot(const s_haptic_core_data * from, int slot, s_ff_lg_report * to, uint8_t caps) {

    switch (from->type) {
    case E_DATA_TYPE_CONSTANT:
        command->force_type = FF_LG_FTYPE_VARIABLE;
        command->parameters[0] = ff_lg_s16_to_u8(from->constant.level);
        command->parameters[1] = 0x80;
        break;
    case E_DATA_TYPE_SPRING:
    {
        uint16_t d1;
        uint16_t d2;
        uint8_t k1;
        uint8_t k2;
        uint8_t s1 = 0x00;
        int16_t left = from->spring.coefficient.left;
        int16_t right = from->spring.coefficient.right;
        if (left < 0) {
            s1 = 0x01;
            left *= -1;
        }
        uint8_t s2 = 0x00;
        if (right < 0) {
            s2 = 0x01;
            right *= -1;
        }
        uint8_t clip = GET_CLIP(from->spring.saturation.left, from->spring.saturation.right);
        uint16_t d = (from->spring.deadband / 2);
        if (caps & FF_LG_CAPS_HIGH_RES_COEF) {
            command->force_type = FF_LG_FTYPE_HIGH_RESOLUTION_SPRING;
            int32_t v = from->spring.center - d;
            d1 = ff_lg_s16_to_u11(CLAMP(-SHRT_MAX, v, SHRT_MAX));
            v = from->spring.center + d;
            d2 = ff_lg_s16_to_u11(CLAMP(-SHRT_MAX, v, SHRT_MAX));
            k1 = left * 0x0f / SHRT_MAX;
            k2 = right * 0x0f / SHRT_MAX;
        } else {
            command->force_type = FF_LG_FTYPE_SPRING;
            int32_t v = from->spring.center - d;
            d1 = ff_lg_s16_to_u8(CLAMP(-SHRT_MAX, v, SHRT_MAX)) << 3;
            v = from->spring.center + d;
            d2 = ff_lg_s16_to_u8(CLAMP(-SHRT_MAX, v, SHRT_MAX)) << 3;
            k1 = left * 0x07 / SHRT_MAX;
            k2 = right * 0x07 / SHRT_MAX;
            if (caps & FF_LG_CAPS_OLD_LOW_RES_COEF) {
                k1 = convert_coef_lr2lr(k1);
                k2 = convert_coef_lr2lr(k2);
            }
        }
        command->parameters[0] = d1 >> 3;
        command->parameters[1] = d2 >> 3;
        command->parameters[2] = (k2 << 4) | k1;
        command->parameters[3] = ((d2 & 0x0f) << 5) | (s2 << 4) | ((d1 & 0x07) << 1) | s1;
        command->parameters[4] = clip;
        break;
    }
    case E_DATA_TYPE_DAMPER:
    {
        uint8_t k1;
        uint8_t s1 = 0x00;
        uint8_t k2;
        uint8_t s2 = 0x00;
        uint8_t clip = 0;
        int16_t left = from->spring.coefficient.left;
        int16_t right = from->spring.coefficient.right;
        if (left < 0) {
            s1 = 0x01;
            left *= -1;
        }
        if (right < 0) {
            s2 = 0x01;
            right *= -1;
        }
        if (caps & FF_LG_CAPS_HIGH_RES_COEF) {
            command->force_type = FF_LG_FTYPE_HIGH_RESOLUTION_DAMPER;
            k1 = left * 0x0f / SHRT_MAX;
            k2 = right * 0x0f / SHRT_MAX;
            if (caps & FF_LG_CAPS_DAMPER_CLIP) {
                clip = GET_CLIP(from->damper.saturation.left, from->damper.saturation.right);
            } else {
                clip = 0xff;
            }
        } else {
            command->force_type = FF_LG_FTYPE_DAMPER;
            k1 = left * 0x07 / SHRT_MAX;
            k2 = right * 0x07 / SHRT_MAX;
            if (caps & FF_LG_CAPS_OLD_LOW_RES_COEF) {
                k1 = convert_coef_lr2lr(k1);
                k2 = convert_coef_lr2lr(k2);
            }
        }
        command->parameters[0] = k1;
        command->parameters[1] = s1;
        command->parameters[2] = k2;
        command->parameters[3] = s2;
        command->parameters[4] = clip;
        break;
    }
    case E_DATA_TYPE_LEDS:
        *cmd = FF_LG_CMD_EXTENDED_COMMAND;
        command->cmd_param = FF_LG_EXT_CMD_SET_RPM_LEDS;
        command->parameters[0] = from->leds.value;
        break;
    case E_DATA_TYPE_RANGE:
        if (caps & FF_LG_CAPS_RANGE_200_900) {
            *cmd = FF_LG_CMD_EXTENDED_COMMAND;
            unsigned short full_range;
            if (from->range.value > 200) {
                command->cmd_param = FF_LG_EXT_CMD_WHEEL_RANGE_900_DEGREES;
                full_range = 900;
            } else {
                command->cmd_param = FF_LG_EXT_CMD_WHEEL_RANGE_200_DEGREES;
                full_range = 200;
            }

            ginfo("wheel range adjusted to %hu degrees\n", full_range);
            if (from->range.value != full_range) {
                static int warn = 1;
                if (warn == 1) {
                    gwarn("Driving Force Pro currently only supports 200 and 900 degree ranges\n");
                    warn = 0;
                }
                // division by 2 is performed when computing high and low order bits
            }
        } else {
            *cmd = FF_LG_CMD_EXTENDED_COMMAND;
            command->cmd_param = FF_LG_EXT_CMD_CHANGE_WHEEL_RANGE;
            command->parameters[0] = from->range.value & 0xFF;
            command->parameters[1] = from->range.value >> 8;

            ginfo("wheel range adjusted to %hu degrees\n", from->range.value);
        }
        break;
    default:
        break;
    }
}
*/
#define GET_CLIP(SL, SR) ((SL > SR ? SL : SR) * 255 / USHRT_MAX)

#define CLAMP(MIN,VALUE,MAX) (((VALUE) < MIN) ? (MIN) : (((VALUE) > MAX) ? (MAX) : (VALUE)))

static inline unsigned char convert_coef_lr2lr(unsigned char k) {

    static const unsigned char map[] = { 0, 1, 2, 3, 4, 6, 5, 7 };
    return map[k];
}


/*
 * \brief Get the spring or damper force coefficient, normalized to [0..1].
 *
 * \param caps the capabilities of the wheel (bitfield of FF_LG_CAPS)
 * \param k    the constant selector
 *
 * \return the force coefficient
 */
typedef struct {
    unsigned char num;
    unsigned char den;
} s_coef;

static s_coef ff_lg_get_force_coefficient(uint8_t caps, unsigned char k) {

    s_coef coef;

    if (caps & FF_LG_CAPS_HIGH_RES_COEF) {
        coef.num = k;
        coef.den = 0x0F;
    } else {
        if (caps & FF_LG_CAPS_OLD_LOW_RES_COEF) {
            static const s_coef old_coefs[] = { { 1, 16 }, { 1, 8 }, { 3, 16 }, { 1, 4 }, { 3, 8 }, { 3, 4 }, { 2, 4 }, { 4, 4 } };
            coef = old_coefs[k];
        } else {
            static const s_coef coefs[] = { { 1, 16 }, { 1, 8 }, { 3, 16 }, { 1, 4 }, { 3, 8 }, { 2, 4 }, { 3, 4 }, { 4, 4 } };
            coef = coefs[k];
        }
    }
    return coef;
}

static int16_t ff_lg_get_condition_coef(uint8_t caps, unsigned char k, unsigned char s) {

    s_coef coef = ff_lg_get_force_coefficient(caps, k);
    int value = (s ? -SHRT_MAX : SHRT_MAX) * coef.num / coef.den;
    return value;
}

static uint16_t ff_lg_get_spring_deadband(uint8_t caps, unsigned char d, unsigned char dL) {

    uint16_t deadband;
    if (caps & FF_LG_CAPS_HIGH_RES_DEADBAND) {
        deadband = ((d << 3) | dL) * USHRT_MAX / 0x7FF;
    } else {
        deadband = d * USHRT_MAX / UCHAR_MAX;
    }
    return deadband;
}

static uint16_t ff_lg_get_damper_clip(uint8_t caps, unsigned char c) {

    uint16_t clip;
    if (caps & FF_LG_CAPS_DAMPER_CLIP) {
        clip = c * USHRT_MAX / UCHAR_MAX;
    } else {
        clip = USHRT_MAX;
    }
    return clip;
}

long get_map (long x, long in_min, long in_max, long out_min, long out_max);

unsigned char uget_cmap (unsigned char x, unsigned char  in_min, unsigned char  in_max, unsigned char  out_min, unsigned char out_max)
{
  unsigned char rv = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
  return rv;
}

int ff_lg_convert_force(unsigned char caps, const unsigned char *data, unsigned char *ffbot)
{
    int ret = FF_LG_OUTPUT_REPORT_SIZE;
    s_ff_lg_command * force = (s_ff_lg_command *)(data + 1);
    uint8_t * cmd = ffbot;
    s_ff_lg_command * command = (s_ff_lg_command *)(ffbot + 1);
    //copy command
    *cmd = data[0] & 0x0f;  //remove slot info, keep only the command
    //
    int slot_index = (data[0] >> 4); //get slot number
    //
    switch (force->force_type)
    {
    case FF_LG_FTYPE_CONSTANT:
    {
        ret = 0;
        //to->type = E_DATA_TYPE_CONSTANT;
        command->force_type = FF_LG_FTYPE_VARIABLE;
        command->parameters[0] = FF_LG_CONSTANT_LEVEL(force, slot_index - 1);//ff_lg_s16_to_u8(from->constant.level);
        command->parameters[1] = 0x00;//0x80;
    }
        break;
    case FF_LG_FTYPE_VARIABLE:
    {
        //ret = 0;
        command->force_type = FF_LG_FTYPE_VARIABLE;
        //to->type = E_DATA_TYPE_CONSTANT;
        if (slot_index == 1)
        {
            if (FF_LG_VARIABLE_T1(force) && FF_LG_VARIABLE_S1(force))
            {
                printf ("\n#w:variable force cannot be converted to constant force (l1=%hu, t1=%hu, s1=%hu, d1=%hu",
                    FF_LG_VARIABLE_L1(force), FF_LG_VARIABLE_T1(force), FF_LG_VARIABLE_S1(force), FF_LG_VARIABLE_D1(force));
            }
            else
            {
                //  long rv = (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
                //command->parameters[0] = get_map (FF_LG_VARIABLE_L1(force), 0xfe, 0x01, 0xa0, 0x50);//FF_LG_VARIABLE_L1(force);//ff_lg_s16_to_u8(from->constant.level);
                command->parameters[0] = FF_LG_VARIABLE_L1(force);//ff_lg_s16_to_u8(from->constant.level);
                command->parameters[1] = 0x00;//0x80;
            }
        }
        else if (slot_index == 3)
        {
            if (FF_LG_VARIABLE_T2(force) && FF_LG_VARIABLE_S2(force))
            {
                printf ("\n#w:variable force cannot be converted to constant force (l2=%hu, t2=%hu, s2=%hu, d2=%hu",
                    FF_LG_VARIABLE_L2(force), FF_LG_VARIABLE_T2(force), FF_LG_VARIABLE_S2(force), FF_LG_VARIABLE_D2(force));
            }
            else
            {
                //to->constant.level = ff_lg_u8_to_s16(FF_LG_VARIABLE_L2(force));
                command->parameters[0] = FF_LG_VARIABLE_L2(force);//ff_lg_s16_to_u8(from->constant.level);
                command->parameters[1] = 0x00;//0x80;
            }
        }
        else
        {
            printf ("\n#w:variable force cannot be converted to constant force for slot 0x%x", slot_index);
            command->parameters[0] = force->parameters[0];//ff_lg_s16_to_u8(from->constant.level);
            command->parameters[1] = 0x00;//0x80;
        }
        //avoid extreme moves
        if (command->parameters[0] > 0xf0 || command->parameters[0] < 0x10)
            ret = 0;
    }
        break;
    case FF_LG_FTYPE_SPRING:
    {
        ret = 0;
    /*
        to->type = E_DATA_TYPE_SPRING;
            to->spring.saturation.left = ff_lg_u8_to_u16(FF_LG_SPRING_CLIP(force));
            to->spring.saturation.right = ff_lg_u8_to_u16(FF_LG_SPRING_CLIP(force));
            to->spring.coefficient.left =
                ff_lg_get_condition_coef(caps & FF_LG_CAPS_OLD_LOW_RES_COEF, FF_LG_SPRING_K1(force), FF_LG_SPRING_S1(force));
            to->spring.coefficient.right =
                ff_lg_get_condition_coef(caps & FF_LG_CAPS_OLD_LOW_RES_COEF, FF_LG_SPRING_K2(force), FF_LG_SPRING_S2(force));
            to->spring.center = ff_lg_u8_to_s16((FF_LG_SPRING_D1(force) + FF_LG_SPRING_D2(force)) / 2);
            to->spring.deadband = ff_lg_u8_to_u16(FF_LG_SPRING_D2(force) - FF_LG_SPRING_D1(force));
        */
        uint16_t d1;
        uint16_t d2;
        uint8_t k1;
        uint8_t k2;
        uint8_t s1 = 0x00;
        int16_t left = ff_lg_get_condition_coef(caps & FF_LG_CAPS_OLD_LOW_RES_COEF, FF_LG_SPRING_K1(force), FF_LG_SPRING_S1(force));//from->spring.coefficient.left;
        int16_t right = ff_lg_get_condition_coef(caps & FF_LG_CAPS_OLD_LOW_RES_COEF, FF_LG_SPRING_K2(force), FF_LG_SPRING_S2(force));//from->spring.coefficient.right;
        if (left < 0) {
            s1 = 0x01;
            left *= -1;
        }
        uint8_t s2 = 0x00;
        if (right < 0) {
            s2 = 0x01;
            right *= -1;
        }
        uint8_t clip = GET_CLIP(FF_LG_SPRING_CLIP(force), FF_LG_SPRING_CLIP(force));
        uint16_t d = ((FF_LG_SPRING_D2(force) - FF_LG_SPRING_D1(force)) / 2);
        if (caps & FF_LG_CAPS_HIGH_RES_COEF) {
            command->force_type = FF_LG_FTYPE_HIGH_RESOLUTION_SPRING;
            int32_t v = ((FF_LG_SPRING_D1(force) + FF_LG_SPRING_D2(force)) / 2) - d;
            d1 = (CLAMP(-SHRT_MAX, v, SHRT_MAX));
            v = ((FF_LG_SPRING_D1(force) + FF_LG_SPRING_D2(force)) / 2) + d;
            d2 = (CLAMP(-SHRT_MAX, v, SHRT_MAX));
            k1 = left * 0x0f / SHRT_MAX;
            k2 = right * 0x0f / SHRT_MAX;
        } else {
            command->force_type = FF_LG_FTYPE_SPRING;
            int32_t v = ((FF_LG_SPRING_D1(force) + FF_LG_SPRING_D2(force)) / 2) - d;
            d1 = (CLAMP(-SHRT_MAX, v, SHRT_MAX)) << 3;
            v = ((FF_LG_SPRING_D1(force) + FF_LG_SPRING_D2(force)) / 2) + d;
            d2 = (CLAMP(-SHRT_MAX, v, SHRT_MAX)) << 3;
            k1 = left * 0x07 / SHRT_MAX;
            k2 = right * 0x07 / SHRT_MAX;
            if (caps & FF_LG_CAPS_OLD_LOW_RES_COEF) {
                k1 = convert_coef_lr2lr(k1);
                k2 = convert_coef_lr2lr(k2);
            }
        }
        command->parameters[0] = d1 >> 3;
        command->parameters[1] = d2 >> 3;
        command->parameters[2] = (k2 << 4) | k1;
        command->parameters[3] = ((d2 & 0x0f) << 5) | (s2 << 4) | ((d1 & 0x07) << 1) | s1;
        command->parameters[4] = clip;
    }
        break;
    case FF_LG_FTYPE_HIGH_RESOLUTION_SPRING:
    {
        //ret = 0;
    /*
        to->type = E_DATA_TYPE_SPRING;
            to->playing = 1;
            to->spring.saturation.left = ff_lg_u8_to_u16(FF_LG_HIGHRES_SPRING_CLIP(force));
            to->spring.saturation.right = ff_lg_u8_to_u16(FF_LG_HIGHRES_SPRING_CLIP(force));
            to->spring.coefficient.left =
                ff_lg_get_condition_coef(FF_LG_CAPS_HIGH_RES_COEF, FF_LG_HIGHRES_SPRING_K1(force), FF_LG_HIGHRES_SPRING_S1(force));
            to->spring.coefficient.right =
                ff_lg_get_condition_coef(FF_LG_CAPS_HIGH_RES_COEF, FF_LG_HIGHRES_SPRING_K2(force), FF_LG_HIGHRES_SPRING_S2(force));
            uint16_t d2 = ff_lg_get_spring_deadband(caps, FF_LG_HIGHRES_SPRING_D2(force), FF_LG_HIGHRES_SPRING_D2L(force));
            uint16_t d1 = ff_lg_get_spring_deadband(caps, FF_LG_HIGHRES_SPRING_D1(force), FF_LG_HIGHRES_SPRING_D1L(force));
            to->spring.center = ff_lg_u16_to_s16((d1 + d2) / 2);
            to->spring.deadband = d2 - d1;
        */
       #if 0
        uint16_t dd2 = ff_lg_get_spring_deadband(caps, FF_LG_HIGHRES_SPRING_D2(force), FF_LG_HIGHRES_SPRING_D2L(force));
        uint16_t dd1 = ff_lg_get_spring_deadband(caps, FF_LG_HIGHRES_SPRING_D1(force), FF_LG_HIGHRES_SPRING_D1L(force));
        uint16_t d1;
        uint16_t d2;
        uint8_t k1;
        uint8_t k2;
        uint8_t s1 = 0x00;
        int16_t left = ff_lg_get_condition_coef(FF_LG_CAPS_HIGH_RES_COEF, FF_LG_HIGHRES_SPRING_K1(force), FF_LG_HIGHRES_SPRING_S1(force));
        int16_t right = ff_lg_get_condition_coef(FF_LG_CAPS_HIGH_RES_COEF, FF_LG_HIGHRES_SPRING_K2(force), FF_LG_HIGHRES_SPRING_S2(force));
        if (left < 0) {
            s1 = 0x01;
            left *= -1;
        }
        uint8_t s2 = 0x00;
        if (right < 0) {
            s2 = 0x01;
            right *= -1;
        }
        uint8_t clip = GET_CLIP(FF_LG_HIGHRES_SPRING_CLIP(force), FF_LG_HIGHRES_SPRING_CLIP(force));
        uint16_t d = ((dd2 - dd1)) / 2;
        if (caps & FF_LG_CAPS_HIGH_RES_COEF) {
            command->force_type = FF_LG_FTYPE_HIGH_RESOLUTION_SPRING;
            int32_t v = ((dd1 + dd2) / 2) - d;
            d1 = (CLAMP(-SHRT_MAX, v, SHRT_MAX));
            v = ((dd1 + dd2) / 2) + d;
            d2 = (CLAMP(-SHRT_MAX, v, SHRT_MAX));
            k1 = left * 0x0f / SHRT_MAX;
            k2 = right * 0x0f / SHRT_MAX;
        } else {
            command->force_type = FF_LG_FTYPE_SPRING;
            int32_t v = ((dd1 + dd2) / 2) - d;
            d1 = (CLAMP(-SHRT_MAX, v, SHRT_MAX)) << 3;
            v = ((dd1 + dd2) / 2) + d;
            d2 = (CLAMP(-SHRT_MAX, v, SHRT_MAX)) << 3;
            k1 = left * 0x07 / SHRT_MAX;
            k2 = right * 0x07 / SHRT_MAX;
            if (caps & FF_LG_CAPS_OLD_LOW_RES_COEF) {
                k1 = convert_coef_lr2lr(k1);
                k2 = convert_coef_lr2lr(k2);
            }
        }
        #endif
        command->parameters[0] = FF_LG_HIGHRES_SPRING_D1(force);//d1 >> 3;
        command->parameters[1] = FF_LG_HIGHRES_SPRING_D2(force);//d2 >> 3;
        command->parameters[2] = force->parameters[2];//(k2 << 4) | k1;
        command->parameters[3] = force->parameters[3];//((d2 & 0x0f) << 5) | (s2 << 4) | ((d1 & 0x07) << 1) | s1;
        command->parameters[4] = force->parameters[4];//clip;
        *cmd |= 0x10;
    }
        break;
    case FF_LG_FTYPE_DAMPER:
    {
        ret = 0;
    /*
        to->type = E_DATA_TYPE_DAMPER;
            to->damper.saturation.left = USHRT_MAX;
            to->damper.saturation.right = USHRT_MAX;
            to->damper.coefficient.left =
                ff_lg_get_condition_coef(caps & FF_LG_CAPS_OLD_LOW_RES_COEF, FF_LG_DAMPER_K1(force), FF_LG_DAMPER_S1(force));
            to->damper.coefficient.right =
                ff_lg_get_condition_coef(caps & FF_LG_CAPS_OLD_LOW_RES_COEF, FF_LG_DAMPER_K2(force), FF_LG_DAMPER_S2(force));
                */
        uint8_t k1;
        uint8_t s1 = 0x00;
        uint8_t k2;
        uint8_t s2 = 0x00;
        uint8_t clip = 0;
        int16_t left = ff_lg_get_condition_coef(caps & FF_LG_CAPS_OLD_LOW_RES_COEF, FF_LG_DAMPER_K1(force), FF_LG_DAMPER_S1(force));//from->spring.coefficient.left;
        int16_t right = ff_lg_get_condition_coef(caps & FF_LG_CAPS_OLD_LOW_RES_COEF, FF_LG_DAMPER_K2(force), FF_LG_DAMPER_S2(force));//from->spring.coefficient.right;
        if (left < 0) {
            s1 = 0x01;
            left *= -1;
        }
        if (right < 0) {
            s2 = 0x01;
            right *= -1;
        }
        if (caps & FF_LG_CAPS_HIGH_RES_COEF) {
            command->force_type = FF_LG_FTYPE_HIGH_RESOLUTION_DAMPER;
            k1 = left * 0x0f / SHRT_MAX;
            k2 = right * 0x0f / SHRT_MAX;
            if (caps & FF_LG_CAPS_DAMPER_CLIP) {
                clip = GET_CLIP(USHRT_MAX, USHRT_MAX);
            } else {
                clip = 0xff;
            }
        } else {
            command->force_type = FF_LG_FTYPE_DAMPER;
            k1 = left * 0x07 / SHRT_MAX;
            k2 = right * 0x07 / SHRT_MAX;
            if (caps & FF_LG_CAPS_OLD_LOW_RES_COEF) {
                k1 = convert_coef_lr2lr(k1);
                k2 = convert_coef_lr2lr(k2);
            }
        }
        command->parameters[0] = k1;
        command->parameters[1] = s1;
        command->parameters[2] = k2;
        command->parameters[3] = s2;
        command->parameters[4] = clip;
    }
    break;
    case FF_LG_FTYPE_HIGH_RESOLUTION_DAMPER:
    {
        //ret = 0;
    /*
        to->type = E_DATA_TYPE_DAMPER;
            to->damper.saturation.left = ff_lg_get_damper_clip(caps, FF_LG_HIGHRES_DAMPER_CLIP(force));
            to->damper.saturation.right = ff_lg_get_damper_clip(caps, FF_LG_HIGHRES_DAMPER_CLIP(force));
            to->damper.coefficient.left =
                ff_lg_get_condition_coef(FF_LG_CAPS_HIGH_RES_COEF, FF_LG_HIGHRES_DAMPER_K1(force), FF_LG_HIGHRES_DAMPER_S1(force));
            to->damper.coefficient.right =
                ff_lg_get_condition_coef(FF_LG_CAPS_HIGH_RES_COEF, FF_LG_HIGHRES_DAMPER_K2(force), FF_LG_HIGHRES_DAMPER_S2(force));
            to->damper.center = 0;
            to->damper.deadband = 0;
                */
#if 0
        uint8_t k1;
        uint8_t s1 = 0x00;
        uint8_t k2;
        uint8_t s2 = 0x00;
        uint8_t clip = 0;
        int16_t left = ff_lg_get_condition_coef(FF_LG_CAPS_HIGH_RES_COEF, FF_LG_HIGHRES_DAMPER_K1(force), FF_LG_HIGHRES_DAMPER_S1(force));
        int16_t right = ff_lg_get_condition_coef(FF_LG_CAPS_HIGH_RES_COEF, FF_LG_HIGHRES_DAMPER_K2(force), FF_LG_HIGHRES_DAMPER_S2(force));
        if (left < 0) {
            s1 = 0x01;
            left *= -1;
        }
        if (right < 0) {
            s2 = 0x01;
            right *= -1;
        }
        if (caps & FF_LG_CAPS_HIGH_RES_COEF) {
            *cmd |= 0x20;
            command->force_type = FF_LG_FTYPE_HIGH_RESOLUTION_DAMPER;
            k1 = left * 0x0f / SHRT_MAX;
            k2 = right * 0x0f / SHRT_MAX;
            if (caps & FF_LG_CAPS_DAMPER_CLIP) {
                clip = GET_CLIP(ff_lg_get_damper_clip(caps, FF_LG_HIGHRES_DAMPER_CLIP(force)), ff_lg_get_damper_clip(caps, FF_LG_HIGHRES_DAMPER_CLIP(force)));
            } else {
                clip = 0xff;
            }
        } else {
            command->force_type = FF_LG_FTYPE_DAMPER;
            k1 = left * 0x07 / SHRT_MAX;
            k2 = right * 0x07 / SHRT_MAX;
            if (caps & FF_LG_CAPS_OLD_LOW_RES_COEF) {
                k1 = convert_coef_lr2lr(k1);
                k2 = convert_coef_lr2lr(k2);
            }
        }
#endif
        //clip = (FF_LG_HIGHRES_DAMPER_CLIP(force) & 0xf0) >> 4;
        uint8_t clip = get_map (FF_LG_HIGHRES_DAMPER_CLIP(force), 0x00, 0xff, 0x00, 0x05);
        //clip = (FF_LG_HIGHRES_DAMPER_CLIP(force) & 0x0f);
        command->parameters[0] = clip;//k1;
        command->parameters[1] = 0x00;//s1;
        command->parameters[2] = clip;//k2;
        command->parameters[3] = 0x00;//s2;
        command->parameters[4] = 0xFF;//clip;
    }
        break;
    default:
        //TODO MLA: other force types
        {
            printf ("\n#w:unsupported force type: %s", ff_lg_get_ftype_name(force->force_type));
            ret = 0;
        }
        break;
    }

    return ret;
}
/*
int ff_lg_convert_extended(const s_ff_lg_command * ext, s_haptic_core_data * to) {

    int ret = 0;

    memset(to, 0x00, sizeof(*to));

    switch (ext->cmd_param) {
    case FF_LG_EXT_CMD_WHEEL_RANGE_200_DEGREES:
        to->type = E_DATA_TYPE_RANGE;
        to->range.value = 200;
        ret = 1;
        break;
    case FF_LG_EXT_CMD_WHEEL_RANGE_900_DEGREES:
        to->type = E_DATA_TYPE_RANGE;
        to->range.value = 900;
        ret = 1;
        break;
    case FF_LG_EXT_CMD_CHANGE_WHEEL_RANGE:
        to->type = E_DATA_TYPE_RANGE;
        to->range.value = (ext->parameters[1] << 8) | ext->parameters[0];
        ret = 1;
        break;
    case FF_LG_EXT_CMD_SET_RPM_LEDS:
        to->type = E_DATA_TYPE_LEDS;
        to->leds.value = ext->parameters[0];
        ret = 1;
        break;
    default:
        break;
    }

    return ret;
}
*/
//--
typedef struct {
    unsigned char mask;
    unsigned char playing;
    unsigned char updated;
    unsigned char parameters[FF_LG_OUTPUT_REPORT_SIZE - 1];
} s_force;

typedef struct {
    unsigned char updated;
    unsigned char cmd[FF_LG_OUTPUT_REPORT_SIZE];
} s_ext_cmd;

int ff_lg_convert_extended(const unsigned char *data, unsigned char *ffbot)
{
    memcpy (ffbot, data, FF_LG_OUTPUT_REPORT_SIZE);
    if (0)
    switch(data[1])
    {
        case FF_LG_EXT_CMD_CHANGE_MODE_DFP:
        case FF_LG_EXT_CMD_WHEEL_RANGE_200_DEGREES:
        case FF_LG_EXT_CMD_WHEEL_RANGE_900_DEGREES:
        case FF_LG_EXT_CMD_CHANGE_MODE_G25:
        case FF_LG_EXT_CMD_CHANGE_MODE_G25_NO_DETACH:
        break;
        case FF_LG_EXT_CMD_CHANGE_MODE:
        {
            const char * mode = NULL;
            switch(data[2])
            {
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
            }
            if(mode == NULL)
            {
                dprintf(" - unknown mode (0x%02x)", data[2]);
            }
            else
            {
                dprintf(" - %s", mode);
            }
            dprintf(" - %s", data[3] ? "DETACH" : "NO DETACH");
        }
        break;
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
        dump(data + 2, FF_LG_OUTPUT_REPORT_SIZE - 2);
        break;
    }
    return 0;//FF_LG_OUTPUT_REPORT_SIZE;
}

static char cmd_stop[] = { 0x01, 0x08, 0x80, 0x00, 0x00, 0x00, 0x00 };
int ff_lg_convert(const unsigned char *data, unsigned char *ffbot)
{
    int ret = FF_LG_OUTPUT_REPORT_SIZE;
    static unsigned char caps = FF_LG_CAPS_HIGH_RES_COEF | FF_LG_CAPS_HIGH_RES_DEADBAND | FF_LG_CAPS_LEDS | FF_LG_CAPS_RANGE;
    uint8_t * ocmd = ffbot;
    s_ff_lg_command * command = (s_ff_lg_command *)(ffbot + 1);
    //copy command
    if(data[0] != FF_LG_CMD_EXTENDED_COMMAND)
    {

        //unsigned char slots = data[0] & FF_LG_FSLOT_MASK;
        unsigned char cmd = data[0] & FF_LG_CMD_MASK;

        //int i = 0;
        //s_force forces[1];
        switch(cmd)
        {
            case FF_LG_CMD_DOWNLOAD:
            case FF_LG_CMD_DOWNLOAD_AND_PLAY:
            case FF_LG_CMD_PLAY:
            case FF_LG_CMD_STOP:
            case FF_LG_CMD_REFRESH_FORCE:
            {
                //printf("\n#i:convert command %s", ff_lg_get_cmd_name(cmd));
                if (cmd == FF_LG_CMD_STOP)
                {
                    //memset(forces[i].parameters + 1, 0x00, sizeof(forces[i].parameters) - 1); // keep force type
                    memcpy (ffbot, cmd_stop, FF_LG_OUTPUT_REPORT_SIZE);
                }
                else
                {
                    ret = ff_lg_convert_force (caps, data, ffbot);
                    //memcpy(forces[i].parameters, data + 1, sizeof(forces[i].parameters));
                }
                //s_cmd cmd = { forces[i].mask, 0x00 };
                //ff_lg_fifo_push(state->fifo, cmd, 1);
            }
            break;
            case FF_LG_CMD_DEFAULT_SPRING_ON:
            case FF_LG_CMD_DEFAULT_SPRING_OFF:
            {
                memcpy (ffbot, data, FF_LG_OUTPUT_REPORT_SIZE);
            }
            break;
            default:
            {
                printf("\n#w:skipping unsupported command %s", ff_lg_get_cmd_name(cmd));
                memcpy (ffbot, data, FF_LG_OUTPUT_REPORT_SIZE);
                ret = 0;
            }
            break;
        }
    }
    else    //extended command
    {
        unsigned short range = 0;
        switch(data[1])
        {
            case FF_LG_EXT_CMD_WHEEL_RANGE_200_DEGREES:
                range = 380;
                break;
            case FF_LG_EXT_CMD_WHEEL_RANGE_900_DEGREES:
                range = 900;
                break;
            case FF_LG_EXT_CMD_CHANGE_WHEEL_RANGE:
                range = (data[3] << 8) | data[2];
                break;
        }
        if (range != 0)
        {
            printf("\n#i:set wheel range %d", range);
            *ocmd = FF_LG_CMD_EXTENDED_COMMAND;
            command->cmd_param = FF_LG_EXT_CMD_CHANGE_WHEEL_RANGE;
            command->parameters[0] = range & 0xFF;
            command->parameters[1] = range >> 8;
            //memcpy (ffbot, data, FF_LG_OUTPUT_REPORT_SIZE);
            //ret = FF_LG_OUTPUT_REPORT_SIZE;
        }

        switch(data[1])
        {
            case FF_LG_EXT_CMD_CHANGE_MODE_DFP:
            case FF_LG_EXT_CMD_CHANGE_MODE:
            case FF_LG_EXT_CMD_REVERT_IDENTITY:
            case FF_LG_EXT_CMD_CHANGE_MODE_G25:
            case FF_LG_EXT_CMD_CHANGE_MODE_G25_NO_DETACH:
            {
                printf("\n#w:skipping unsupported change wheel mode commands 0x%02x", data[1]);
                ret = 0;
            }
            break;
            default:
            break;
        }
        //return ff_lg_convert_extended (data, ffbot);
    }
    //
    return ret;
}
