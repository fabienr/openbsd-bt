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

/* XXX debug */
#define BTHCI_DEBUG

#include <sys/types.h>
#include <sys/malloc.h>
#include <sys/systm.h>
#include <sys/errno.h>

#include "bthci.h"

struct cfdriver bthci_cd = {
    NULL, "bthci", DV_DULL
};

#ifdef BTHCI_DEBUG
#define DPRINTF(x)	do { printf x; } while (0)
#else
#define DPRINTF(x)
#endif
#define DEVNAME(sc) ((sc)->sc_dev.dv_xname)

#define HCI_OGF_LM			0x01
#define HCI_OCF_INQUIRY			0x0001

#define HCI_OGF_CB			0x03
#define HCI_OCF_RESET			0x0003

#define HCI_OGF_INFO			0x04
#define HCI_OCF_READ_LOCAL_VERSION	0x0001
#define HCI_OCF_READ_BUFFER_SIZE	0x0005
#define HCI_OCF_READ_BDADDR		0x0009

/* XXX_DOMAIN_BLUETOOTH */
/* #define XXX_DOMAIN_BLUETOOTH */
#ifdef XXX_DOMAIN_BLUETOOTH
#define BLUETOOTH_BDADDR_SIZE	6
typedef struct {
	uint8_t b[BLUETOOTH_BDADDR_SIZE];
} bdaddr_t;

static inline int
bdaddr_equ(const bdaddr_t *a, const bdaddr_t *b)
{
	return (a->b[0] == b->b[0] && a->b[1] == b->b[1]
	    && a->b[2] == b->b[2] && a->b[3] == b->b[3]
	    && a->b[4] == b->b[4] && a->b[5] == b->b[5]);
}

static inline int
bdaddr_any(const bdaddr_t *a)
{
	return (a->b[0] == 0 && a->b[1] == 0 && a->b[2] == 0
	    && a->b[3] == 0 && a->b[4] == 0 && a->b[5] == 0);
}

static inline void
bdaddr_cpy(bdaddr_t *d, const bdaddr_t *s)
{
	d->b[0] = s->b[0];
	d->b[1] = s->b[1];
	d->b[2] = s->b[2];
	d->b[3] = s->b[3];
	d->b[4] = s->b[4];
	d->b[5] = s->b[5];
}
#endif /* XXX_DOMAIN_BLUETOOTH */

int
bthci_attach(struct bthci_softc *sc, void *priv)
{
	sc->priv = priv;

	/*sc->cmd = malloc(sizeof(struct bt_cmd), M_BTHCI, M_NOWAIT);
	if (sc->cmd == NULL) {
		printf("%s: could not allocate bt cmd buffer\n",
		    DEVNAME(sc));
		return (ENOMEM);
	}*/

	if (bthci_info_version(sc)) {
		printf("%s: bthci_attach cannot get version\n",
		    DEVNAME(sc));
		return (EIO);
	}

	if (bthci_info_bdaddr(sc)) {
		printf("%s: bthci_attach cannot get bdaddr\n",
		    DEVNAME(sc));
		return (EIO);
	}

	if (bthci_info_buffer(sc)) {
		printf("%s: bthci_attach cannot get buffer info\n",
		    DEVNAME(sc));
		return (EIO);
	}

	if (bthci_cb_reset(sc)) {
		printf("%s: bthci_attach cannot reset\n",
		    DEVNAME(sc));
		return (EIO);
	}

	if (bthci_lm_inquiry(sc, 30, 100)) {
		printf("%s: bthci_attach cannot inquiry\n",
		    DEVNAME(sc));
		return (EIO);
	}

	return (0);
}

void
bthci_detach(struct bthci_softc *sc)
{
	/*if (sc->cmd != NULL)
		free(sc->cmd, M_BTHCI, sizeof(struct bt_cmd));*/
	return;
}

/* hci link manager inquiry
 * 3 bytes LAP : see bluetooth specifications/assigned-numbers/baseband/
 * 1 byte timeout : max 0x30 * 1,28s
 * 1 byte limit : 0x00 mean no limit
 */
#define HCI_LM_INQUIRY		(HCI_OCF_INQUIRY|(HCI_OGF_LM<<10))
#define HCI_INQUIRY_LAP_2	0x9E
#define HCI_INQUIRY_LAP_1	0x8B
#define HCI_INQUIRY_LAP_GIAC_0	0x33
#define HCI_INQUIRY_LAP_LIAC_0	0x00
int
bthci_lm_inquiry(struct bthci_softc *sc, int timeout, int limit)
{
	struct hci_lm_inquiry {
		uint8_t lap[3];
		uint8_t timeout;
		uint8_t limit;
	} * inquiry;

	if (timeout * 100 / 128 > 0x30 || timeout < 0) {
		printf("%s: bthci_inquiry invalid timeout 0 > %d > 0x30\n",
		    DEVNAME(sc), timeout);
		return (EINVAL);
	}
	if (limit > 0xFF || limit < 0) {
		printf("%s: bthci_inquiry invalid limit 0 > %d > 0xFF\n",
		    DEVNAME(sc), timeout);
		return (EINVAL);
	}

	sc->cmd.head.op = htole16(HCI_LM_INQUIRY);
	sc->cmd.head.len = sizeof(struct hci_lm_inquiry);
	inquiry = (struct hci_lm_inquiry *)&sc->cmd.data;
	inquiry->lap[0] = HCI_INQUIRY_LAP_GIAC_0;
	inquiry->lap[1] = HCI_INQUIRY_LAP_1;
	inquiry->lap[2] = HCI_INQUIRY_LAP_2;
	inquiry->timeout = timeout * 100 / 128;
	inquiry->limit = limit;

	return (sc->ops.cmd(sc, &sc->cmd));
}

/* hci controller and baseband reset */
#define HCI_CB_RESET		(HCI_OCF_RESET|(HCI_OGF_CB<<10))
int
bthci_cb_reset(struct bthci_softc *sc)
{
	sc->cmd.head.op = htole16(HCI_CB_RESET);
	sc->cmd.head.len = 0;
	return (sc->ops.cmd(sc, &sc->cmd));
}

/* */
#define HCI_INFO_VERSION	(HCI_OCF_READ_LOCAL_VERSION|(HCI_OGF_INFO<<10))
int
bthci_info_version(struct bthci_softc *sc)
{
	sc->cmd.head.op = htole16(HCI_INFO_VERSION);
	sc->cmd.head.len = 0;
	return (sc->ops.cmd(sc, &sc->cmd));
}

#define HCI_INFO_BUFFER		(HCI_OCF_READ_BUFFER_SIZE|(HCI_OGF_INFO<<10))
int
bthci_info_buffer(struct bthci_softc *sc)
{
	sc->cmd.head.op = htole16(HCI_INFO_BUFFER);
	sc->cmd.head.len = 0;
	return (sc->ops.cmd(sc, &sc->cmd));
}

/* */
#define HCI_INFO_BDADDR		(HCI_OCF_READ_BDADDR|(HCI_OGF_INFO<<10))
int
bthci_info_bdaddr(struct bthci_softc *sc)
{
	sc->cmd.head.op = htole16(HCI_INFO_BDADDR);
	sc->cmd.head.len = 0;
	return (sc->ops.cmd(sc, &sc->cmd));
}
