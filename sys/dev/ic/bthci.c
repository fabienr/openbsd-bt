/*
 * Copyright (c) 2020 Fabien ROMANO <fabien@openbsd.org>
 *
 * Permission to use, copy, modify, and distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

#include <sys/types.h>
#include <sys/systm.h>

#include "bthci.h"

struct cfdriver bthci_cd = {
    NULL, "bthci", DV_DULL
};

#define HCI_OGF_LM		0x01
#define HCI_OCF_INQUIRY		0x0001

#define HCI_LM_INQUIRY		(HCI_OCF_INQUIRY|(HCI_OGF_LM<<10))

/* hci link manager inquiry
 * LAP : https://www.bluetooth.com/specifications/assigned-numbers/baseband/
 */
#define HCI_INQUIRY_LAP_2	0x9E
#define HCI_INQUIRY_LAP_1	0x8B
#define HCI_INQUIRY_LAP_GIAC_0	0x33
#define HCI_INQUIRY_LAP_LIAC_0	0x00
struct hci_lm_inquiry {
	u_int8_t lap[3];
	u_int8_t timeout;
	u_int8_t limit;
} hci_lm_inquiry;

int
bthci_attach(struct bthci_softc *sc, void *priv)
{
	struct bt_cmd pkt;
	struct hci_lm_inquiry *inquiry;

	sc->priv = priv;

	pkt.head.op = htole16(HCI_LM_INQUIRY);
	pkt.head.len = htole16(sizeof(struct hci_lm_inquiry));
	inquiry = (struct hci_lm_inquiry *)&pkt.data;
	inquiry->lap[0] = HCI_INQUIRY_LAP_GIAC_0;
	inquiry->lap[1] = HCI_INQUIRY_LAP_1;
	inquiry->lap[2] = HCI_INQUIRY_LAP_2;
	inquiry->timeout = 0x0F; /* 15 * 1,28s */
	inquiry->limit = 0x01; /* no limit */
	sc->ops.cmd(sc, &pkt);

	return (0);
}
