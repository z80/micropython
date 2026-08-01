#ifndef MODULE_NRF24_H
#define MODULE_NRF24_H

#include "py/obj.h"
#include "nrf24/nrf24.h"

/* Transitional ownership boundary used by the current Core binding. It will
 * disappear when Core owns the supplied SPI and Pin objects directly. */
nrf24_t *mp_nrf24_get_native(mp_obj_t radio_obj);
bool mp_nrf24_claim(mp_obj_t radio_obj, void *owner);
bool mp_nrf24_release(mp_obj_t radio_obj, void *owner);
bool mp_nrf24_is_claimed(mp_obj_t radio_obj);

#endif
