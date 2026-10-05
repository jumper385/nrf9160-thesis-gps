#include "gnss.h"

#include <modem/lte_lc.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(gnss, LOG_LEVEL_INF);

/* ---- Edit these ---------------------------------------------------------- */
#define FIX_TIMEOUT_S 300 /* give up on a fix after this long */
/* -------------------------------------------------------------------------- */

static struct nrf_modem_gnss_pvt_data_frame pvt;
static K_SEM_DEFINE(pvt_sem, 0, 1);

static void gnss_event_handler(int event) {
  if (event == NRF_MODEM_GNSS_EVT_PVT) {
    if (nrf_modem_gnss_read(&pvt, sizeof(pvt), NRF_MODEM_GNSS_DATA_PVT) == 0) {
      k_sem_give(&pvt_sem);
    }
  }
}

int gnss_init(void) {
  int err = nrf_modem_gnss_event_handler_set(gnss_event_handler);

  if (err) {
    LOG_ERR("Failed to set GNSS handler: %d", err);
  }
  return err;
}

int gnss_get_fix(struct nrf_modem_gnss_pvt_data_frame *out) {
  int err;
  int64_t deadline = k_uptime_get() + (int64_t)FIX_TIMEOUT_S * 1000;

  /* GNSS can only be activated while LTE is off. */
  (void)lte_lc_func_mode_set(LTE_LC_FUNC_MODE_DEACTIVATE_LTE);

  err = lte_lc_func_mode_set(LTE_LC_FUNC_MODE_ACTIVATE_GNSS);
  if (err) {
    LOG_ERR("Activate GNSS failed: %d", err);
    return err;
  }

  /* Single-fix mode: fix_interval 0 means "one fix, then stop". */
  nrf_modem_gnss_fix_retry_set(0);
  nrf_modem_gnss_fix_interval_set(0);

  k_sem_reset(&pvt_sem);

  err = nrf_modem_gnss_start();
  if (err) {
    LOG_ERR("GNSS start failed: %d", err);
    return err;
  }

  LOG_INF("Searching for fix (timeout %d s)...", FIX_TIMEOUT_S);

  err = -ETIMEDOUT;
  while (k_uptime_get() < deadline) {
    if (k_sem_take(&pvt_sem, K_SECONDS(5)) != 0) {
      continue;
    }

    if (pvt.flags & NRF_MODEM_GNSS_PVT_FLAG_FIX_VALID) {
      *out = pvt;
      err = 0;
      break;
    }

    int tracked = 0, used = 0;

    for (int i = 0; i < NRF_MODEM_GNSS_MAX_SATELLITES; i++) {
      if (pvt.sv[i].sv > 0) {
        tracked++;
        if (pvt.sv[i].flags & NRF_MODEM_GNSS_SV_FLAG_USED_IN_FIX) {
          used++;
        }
      }
    }
    LOG_INF("No fix yet (tracking %d, using %d)", tracked, used);
  }

  nrf_modem_gnss_stop();
  (void)lte_lc_func_mode_set(LTE_LC_FUNC_MODE_DEACTIVATE_GNSS);

  return err;
}
