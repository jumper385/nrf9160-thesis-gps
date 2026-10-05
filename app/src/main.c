/*
 * GNSS fix -> CoAP POST, nRF9160 DK (nrf9160dk/nrf9160/ns)
 *
 * A periodic timer drives the reporting cycle:
 *   1. LTE off, GNSS on, wait for a valid fix        (gnss.c)
 *   2. Pack the fix into an 18-byte binary record     (payload.c)
 *   3. GNSS off, LTE on, POST the record to a CoAP
 *      server and wait for the ACK                    (coap_client.c)
 *   4. LTE off, wait for the next timer tick
 */

#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include "coap_client.h"
#include "gnss.h"
#include "payload.h"
#include <zephyr/drivers/gpio.h>

#define LED_PORT DT_LABEL(DT_NODELABEL(led0))
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(DT_NODELABEL(led0), gpios);

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/* ---- Edit these ---------------------------------------------------------- */
#define REPORT_PERIOD_S 10 /* time between the start of reporting cycles */
/* -------------------------------------------------------------------------- */

/*
 * The cycle itself can block for minutes (waiting for a fix), so the timer
 * handler only wakes the main thread instead of doing the work in ISR
 * context. The semaphore has a count limit of 1: if a cycle runs longer
 * than the period, the next cycle starts immediately rather than piling up.
 */
static K_SEM_DEFINE(report_sem, 0, 1);

static void report_timer_handler(struct k_timer *timer) {
  ARG_UNUSED(timer);
  k_sem_give(&report_sem);
}

K_TIMER_DEFINE(report_timer, report_timer_handler, NULL);

/* One reporting cycle: get a fix and POST it. */
static void report_cycle(void) {

  int err;
  struct nrf_modem_gnss_pvt_data_frame fix;

  gpio_pin_set_dt(&led, 1);

  err = gnss_get_fix(&fix);
  if (err) {
    LOG_WRN("No fix this cycle (%d)", err);
    return;
  }

  uint8_t payload[FIX_PAYLOAD_LEN];
  size_t len = pack_fix(payload, &fix);

  LOG_INF("Fix: lat %.6f lon %.6f acc %.1f m alt %.1f m "
          "%04u-%02u-%02uT%02u:%02u:%02uZ (%u-byte binary payload)",
          fix.latitude, fix.longitude, (double)fix.accuracy,
          (double)fix.altitude, fix.datetime.year, fix.datetime.month,
          fix.datetime.day, fix.datetime.hour, fix.datetime.minute,
          fix.datetime.seconds, (unsigned)len);

  LOG_INF("Connecting to LTE...");
  err = lte_lc_connect();
  if (err) {
    LOG_ERR("LTE connect failed: %d", err);
    return;
  }
  LOG_INF("LTE connected");

  err = coap_client_post(payload, len);
  if (err) {
    LOG_ERR("CoAP send failed: %d", err);
  }
  gpio_pin_set_dt(&led, 0);
}

int main(void) {
  int err;

  if (!device_is_ready(led.port)) {
    LOG_ERR("LED device not ready");
    return -1;
  }
  gpio_pin_configure_dt(&led, GPIO_OUTPUT_INACTIVE);

  LOG_INF("GNSS -> CoAP app starting");

  err = nrf_modem_lib_init();
  if (err) {
    LOG_ERR("Modem library init failed: %d", err);
    return err;
  }

  err = gnss_init();
  if (err) {
    return err;
  }

  /* Fire immediately, then every REPORT_PERIOD_S. */
  k_timer_start(&report_timer, K_NO_WAIT, K_SECONDS(REPORT_PERIOD_S));

  for (;;) {
    k_sem_take(&report_sem, K_FOREVER);

    report_cycle();

    /* LTE off so GNSS can use the radio next cycle. */
    (void)lte_lc_func_mode_set(LTE_LC_FUNC_MODE_DEACTIVATE_LTE);
  }

  return 0;
}
