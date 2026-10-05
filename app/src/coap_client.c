#include "coap_client.h"

#include <errno.h>
#include <string.h>
#include <zephyr/logging/log.h>
#include <zephyr/net/coap.h>
#include <zephyr/net/socket.h>

LOG_MODULE_REGISTER(coap_client, LOG_LEVEL_INF);

/* ---- Edit these ---------------------------------------------------------- */
#define COAP_SERVER_HOST "172.105.254.131"
#define COAP_SERVER_PORT "1183"
#define COAP_URI_PATH "location"
#define COAP_ACK_TIMEOUT_MS 10000
/* -------------------------------------------------------------------------- */

int coap_client_post(const uint8_t *payload, size_t payload_len) {
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
