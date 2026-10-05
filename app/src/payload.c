#include "payload.h"

#include <math.h>
#include <zephyr/sys/byteorder.h>

static int16_t clamp_i16(double v) {
  if (v > (double)INT16_MAX) {
    return INT16_MAX;
  }
  if (v < (double)INT16_MIN) {
    return INT16_MIN;
  }
  return (int16_t)lrint(v);
}

static uint16_t clamp_u16(double v) {
  if (v > (double)UINT16_MAX) {
    return UINT16_MAX;
  }
  if (v < 0.0) {
    return 0;
  }
  return (uint16_t)lrint(v);
}

size_t pack_fix(uint8_t *buf,
                const struct nrf_modem_gnss_pvt_data_frame *fix) {
  sys_put_le32((uint32_t)(int32_t)lrint(fix->latitude * 1e7), &buf[0]);
  sys_put_le32((uint32_t)(int32_t)lrint(fix->longitude * 1e7), &buf[4]);
  sys_put_le16((uint16_t)clamp_i16(fix->altitude * 10.0), &buf[8]);
  sys_put_le16(clamp_u16(fix->accuracy * 10.0), &buf[10]);
  buf[12] = (uint8_t)(fix->datetime.year - 2000);
  buf[13] = fix->datetime.month;
  buf[14] = fix->datetime.day;
  buf[15] = fix->datetime.hour;
  buf[16] = fix->datetime.minute;
  buf[17] = fix->datetime.seconds;
  return FIX_PAYLOAD_LEN;
}
