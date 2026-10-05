/*
 * Binary wire format for a fix report: 18 bytes, little-endian, no padding.
 *
 *   offset  size  field
 *   0       4     int32   latitude  in degrees * 1e7
 *   4       4     int32   longitude in degrees * 1e7
 *   8       2     int16   altitude  in decimetres (clamped to +/-3276.7 m)
 *   10      2     uint16  accuracy  in decimetres (clamped to 6553.5 m)
 *   12      1     uint8   year - 2000
 *   13      1     uint8   month
 *   14      1     uint8   day
 *   15      1     uint8   hour
 *   16      1     uint8   minute
 *   17      1     uint8   second
 *
 * All timestamp fields are UTC as reported by the modem.
 */

#ifndef APP_PAYLOAD_H
#define APP_PAYLOAD_H

#include <stddef.h>
#include <stdint.h>

#include <nrf_modem_gnss.h>

#define FIX_PAYLOAD_LEN 18

/* Serialize `fix` into `buf` (must hold at least FIX_PAYLOAD_LEN bytes).
 * Returns the number of bytes written. */
size_t pack_fix(uint8_t *buf, const struct nrf_modem_gnss_pvt_data_frame *fix);

#endif /* APP_PAYLOAD_H */
