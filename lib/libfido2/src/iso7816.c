/*
 * Copyright (c) 2018 Yubico AB. All rights reserved.
 * Use of this source code is governed by a BSD-style
 * license that can be found in the LICENSE file.
 */

#include <string.h>
#include "fido.h"

iso7816_apdu_t *
iso7816_new(uint8_t ins, uint8_t p1, uint16_t payload_len)
{
	iso7816_apdu_t	*apdu;
	size_t		 alloc_len;

	alloc_len = sizeof(iso7816_apdu_t) + payload_len;

	if ((apdu = calloc(1, alloc_len)) == NULL)
		return (NULL);

	apdu->alloc_len = alloc_len;
	apdu->payload_len = payload_len;
	apdu->payload_ptr = apdu->payload;
	apdu->header.ins = ins;
	apdu->header.p1 = p1;
	apdu->header.lc2 = (payload_len >> 8) & 0xff;
	apdu->header.lc3 = payload_len & 0xff;

	return (apdu);
}

void
iso7816_free(iso7816_apdu_t **apdu_p)
{
	iso7816_apdu_t *apdu;

	if (apdu_p == NULL || (apdu = *apdu_p) == NULL)
		return;

	explicit_bzero(apdu, apdu->alloc_len);
	free(apdu);

	*apdu_p = NULL;
}

int
iso7816_add(iso7816_apdu_t *apdu, const void *buf, size_t cnt)
{
	if (cnt > apdu->payload_len || cnt > UINT16_MAX)
		return (-1);

	memcpy(apdu->payload_ptr, buf, cnt);
	apdu->payload_ptr += cnt;
	apdu->payload_len -= (uint16_t)cnt;

	return (0);
}

const unsigned char *
iso7816_ptr(const iso7816_apdu_t *apdu)
{
	return ((const unsigned char *)&apdu->header);
}

size_t
iso7816_len(const iso7816_apdu_t *apdu)
{
	return (apdu->alloc_len - sizeof(apdu->alloc_len) -
	    sizeof(apdu->payload_len) - sizeof(apdu->payload_ptr));
}
