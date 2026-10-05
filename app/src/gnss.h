/*
 * GNSS fix acquisition for the nRF9160 modem.
 */

#ifndef APP_GNSS_H
#define APP_GNSS_H

#include <nrf_modem_gnss.h>

/* Register the GNSS event handler. Call once after nrf_modem_lib_init(). */
int gnss_init(void);

/*
 * Blocks until a valid fix is written to `out` or the fix timeout expires.
 * Handles the LTE/GNSS radio arbitration (LTE off, GNSS on, then back off).
 * Returns 0 on success, negative errno otherwise.
 */
int gnss_get_fix(struct nrf_modem_gnss_pvt_data_frame *out);

#endif /* APP_GNSS_H */
