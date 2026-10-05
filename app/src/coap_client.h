/*
 * Minimal CoAP client: POST a binary payload to the configured server and
 * wait for the ACK / piggybacked response.
 */

#ifndef APP_COAP_CLIENT_H
#define APP_COAP_CLIENT_H

#include <stddef.h>
#include <stdint.h>

/*
 * POST `payload` to the configured CoAP server. Requires an active LTE
 * connection. Returns 0 if the server answered, negative errno otherwise.
 */
int coap_client_post(const uint8_t *payload, size_t payload_len);

#endif /* APP_COAP_CLIENT_H */
