/*
 * GNSS fix -> CoAP POST, nRF9160 DK (nrf9160dk/nrf9160/ns)
 *
 * Loop:
 *   1. LTE off, GNSS on, wait for a valid fix
 *   2. GNSS off, LTE on
 *   3. POST the position as a packed binary record to a CoAP server and wait
 *      for the ACK
 *   4. LTE off, sleep, repeat
 */

#include <math.h>
#include <modem/lte_lc.h>
#include <modem/nrf_modem_lib.h>
#include <nrf_modem_gnss.h>
#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/coap.h>
#include <zephyr/net/socket.h>
#include <zephyr/sys/byteorder.h>

LOG_MODULE_REGISTER(app, LOG_LEVEL_INF);

/* ---- Edit these ---------------------------------------------------------- */
#define COAP_SERVER_HOST "172.105.254.131"
#define COAP_SERVER_PORT "1183"
#define COAP_URI_PATH "location"
#define FIX_TIMEOUT_S 300  /* give up on a fix after this long */
#define REPORT_PERIOD_S 10 /* pause between cycles */
#define COAP_ACK_TIMEOUT_MS 10000
/* -------------------------------------------------------------------------- */

static struct nrf_modem_gnss_pvt_data_frame pvt;
static K_SEM_DEFINE(pvt_sem, 0, 1);

/* ---------------------------------------------------------------- GNSS --- */

static void gnss_event_handler(int event) {
  if (event == NRF_MODEM_GNSS_EVT_PVT) {
    if (nrf_modem_gnss_read(&pvt, sizeof(pvt), NRF_MODEM_GNSS_DATA_PVT) == 0) {
      k_sem_give(&pvt_sem);
    }
  }
}

/* Blocks until a valid fix is in `pvt` or the timeout expires. */
static int get_fix(struct nrf_modem_gnss_pvt_data_frame *out) {
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

/* ------------------------------------------------------------ payload --- */

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
#define FIX_PAYLOAD_LEN 18

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

static size_t pack_fix(uint8_t *buf,
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

/* ---------------------------------------------------------------- CoAP --- */

static int coap_post(const uint8_t *payload, size_t payload_len) {
  int err;
  int fd = -1;
  struct zsock_addrinfo *res = NULL;
  struct zsock_addrinfo hints = {
      .ai_family = AF_INET,
      .ai_socktype = SOCK_DGRAM,
  };

  err = zsock_getaddrinfo(COAP_SERVER_HOST, COAP_SERVER_PORT, &hints, &res);
  if (err || res == NULL) {
    LOG_ERR("DNS lookup for %s failed: %d", COAP_SERVER_HOST, err);
    return -EHOSTUNREACH;
  }

  fd = zsock_socket(res->ai_family, SOCK_DGRAM, IPPROTO_UDP);
  if (fd < 0) {
    LOG_ERR("socket() failed: %d", errno);
    err = -errno;
    goto out;
  }

  if (zsock_connect(fd, res->ai_addr, res->ai_addrlen) < 0) {
    LOG_ERR("connect() failed: %d", errno);
    err = -errno;
    goto out;
  }

  /* Build the request. */
  uint8_t tx[256];
  struct coap_packet req;
  uint16_t msg_id = coap_next_id();
  uint8_t *token = coap_next_token();

  err = coap_packet_init(&req, tx, sizeof(tx), COAP_VERSION_1, COAP_TYPE_CON,
                         COAP_TOKEN_MAX_LEN, token, COAP_METHOD_POST, msg_id);
  if (err < 0) {
    LOG_ERR("coap_packet_init failed: %d", err);
    goto out;
  }

  err = coap_packet_append_option(&req, COAP_OPTION_URI_PATH, COAP_URI_PATH,
                                  strlen(COAP_URI_PATH));
  if (err < 0) {
    goto out;
  }

  err = coap_append_option_int(&req, COAP_OPTION_CONTENT_FORMAT,
                               COAP_CONTENT_FORMAT_APP_OCTET_STREAM);
  if (err < 0) {
    goto out;
  }

  err = coap_packet_append_payload_marker(&req);
  if (err < 0) {
    goto out;
  }

  err = coap_packet_append_payload(&req, payload, payload_len);
  if (err < 0) {
    LOG_ERR("Payload does not fit in buffer: %d", err);
    goto out;
  }

  if (zsock_send(fd, req.data, req.offset, 0) < 0) {
    LOG_ERR("send() failed: %d", errno);
    err = -errno;
    goto out;
  }
  LOG_INF("Sent CoAP POST /%s (%u bytes payload)", COAP_URI_PATH,
          (unsigned)payload_len);

  /* Wait for the ACK / piggybacked response. */
  struct zsock_pollfd pfd = {.fd = fd, .events = ZSOCK_POLLIN};

  if (zsock_poll(&pfd, 1, COAP_ACK_TIMEOUT_MS) <= 0) {
    LOG_WRN("No response from server within %d ms", COAP_ACK_TIMEOUT_MS);
    err = -ETIMEDOUT;
    goto out;
  }

  uint8_t rx[256];
  int n = zsock_recv(fd, rx, sizeof(rx), 0);

  if (n < 0) {
    LOG_ERR("recv() failed: %d", errno);
    err = -errno;
    goto out;
  }

  struct coap_packet reply;

  err = coap_packet_parse(&reply, rx, n, NULL, 0);
  if (err < 0) {
    LOG_ERR("Could not parse reply: %d", err);
    goto out;
  }

  uint8_t code = coap_header_get_code(&reply);

  LOG_INF("Server replied with CoAP code %u.%02u", code >> 5, code & 0x1f);
  err = 0;

out:
  if (fd >= 0) {
    zsock_close(fd);
  }
  if (res) {
    zsock_freeaddrinfo(res);
  }
  return err;
}

/* ---------------------------------------------------------------- main --- */

int main(void) {
  int err;

  LOG_INF("GNSS -> CoAP app starting");

  err = nrf_modem_lib_init();
  if (err) {
    LOG_ERR("Modem library init failed: %d", err);
    return err;
  }

  err = nrf_modem_gnss_event_handler_set(gnss_event_handler);
  if (err) {
    LOG_ERR("Failed to set GNSS handler: %d", err);
    return err;
  }

  for (;;) {
    struct nrf_modem_gnss_pvt_data_frame fix;

    err = get_fix(&fix);
    if (err) {
      LOG_WRN("No fix this cycle (%d), retrying in %d s", err, REPORT_PERIOD_S);
      goto wait;
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
      goto wait;
    }
    LOG_INF("LTE connected");

    err = coap_post(payload, len);
    if (err) {
      LOG_ERR("CoAP send failed: %d", err);
    }

  wait:
    /* LTE off so GNSS can use the radio next cycle. */
    (void)lte_lc_func_mode_set(LTE_LC_FUNC_MODE_DEACTIVATE_LTE);
    k_sleep(K_SECONDS(REPORT_PERIOD_S));
  }

  return 0;
}
