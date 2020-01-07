/*	$OpenBSD$	*/
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
#include <sys/param.h>
#include <sys/errno.h>
#include <machine/intr.h>
#include <sys/time.h>
#include <sys/pool.h>
#include <sys/mutex.h>
#include <sys/device.h>
#include <sys/systm.h>
#include <sys/mutex.h>

#include <dev/bluetooth/bluetoothreg.h>
#include <dev/bluetooth/bluetoothvar.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/bthci.h>

#ifdef BLUETOOTH_DEBUG
#define BTHCI_DEBUG
#endif

#ifdef BTHCI_DEBUG
#define DPRINTF(x)	do { printf x; } while (0)
#else
#define DPRINTF(x)
#endif
#define DEVNAME(hci) ((hci)->sc->dv_xname)

struct bthci_cmd_complete {
	uint8_t			buff_sz;
	uint16_t		op;
} __packed;
struct bthci_evt_complete {
	struct bt_evt_head		head;
	struct bthci_cmd_complete	event;
	/* evt_head + cmd_complete = 5 */
	uint8_t				data[sizeof(struct bt_evt) - 5];
} __packed;
#ifdef BTHCI_DEBUG
#define DUMP_BT_EVT_COMPLETE(hci, evt) do {					\
	DPRINTF(("%s: command complete head(op=%02X, len=%d), "			\
	    "event(buff_sz=%d, op=%04x), data",					\
	    DEVNAME(hci), (evt)->head.op, (evt)->head.len,			\
	    (evt)->event.buff_sz, (evt)->event.op));				\
	for (int i = 0;								\
	    i < (evt)->head.len - sizeof(struct bthci_cmd_complete); i++)	\
		DPRINTF((" %02x", (evt)->data[i]));				\
	DPRINTF(("\n"));							\
} while(0)
#else
#define DUMP_BT_EVT_COMPLETE(hci, evt)
#endif /* BTHCI_DEBUG */

struct bthci_cmd_state {
	uint8_t			state;
	uint8_t			buff_sz;
	uint16_t		op;
} __packed;
struct bthci_evt_state {
	struct bt_evt_head	head;
	struct bthci_cmd_state	event;
} __packed;
#ifdef BTHCI_DEBUG
#define DUMP_BT_EVT_STATE(hci, evt) do {					\
	DPRINTF(("%s: command state head(op=%02X, len=%d), "			\
	    "event(state=%02X, buff_sz=%d, op=%04X)\n",				\
	    DEVNAME(hci), (evt)->head.op, (evt)->head.len,			\
	    (evt)->event.state, (evt)->event.buff_sz, (evt)->event.op));	\
} while(0)
#else
#define DUMP_BT_EVT_STATE(hci, evt)
#endif /* BTHCI_DEBUG */

int bthci_enter(struct bthci *);
int bthci_cmd(struct bthci *, uint8_t);
void bthci_leave(struct bthci *);

void
bthci_init(struct bthci *hci, struct device *sc, struct btbus *bus, int ipl)
{
	hci->sc = sc;
	hci->bus = bus;
	mtx_init(&hci->mtx, ipl);
	pool_init(&hci->evts, sizeof(struct bthci_evt), 0, ipl, 0,
	    "bthci", NULL);
	if (pool_prime(&hci->evts, BT_EVTS_POOLSIZE))
		printf("%s: pool_prime fail, err=ENOMEM",
		    DEVNAME(hci));
	SIMPLEQ_INIT(&hci->fifo);
}

void
bthci_destroy(struct bthci *hci)
{
	pool_destroy(&hci->evts);
}

struct bt_evt *
bthci_pool_get(struct bthci *hci)
{
	return ((struct bt_evt*)pool_get(&hci->evts, PR_NOWAIT|PR_ZERO));
}

void
bthci_pool_put(struct bthci *hci, struct bt_evt *evt)
{
	pool_put(&hci->evts, evt);
}

void
bthci_write_evt(struct bthci *hci, struct bt_evt *evt)
{
	struct bthci_evt_complete *cmd_complete;
	struct bthci_evt_state *cmd_state;

	if (evt->head.op == BT_EVT_CMD_COMPLETE) {
		if (evt->head.len < sizeof(struct bthci_cmd_complete)) {
			DUMP_BT_EVT(DEVNAME(hci), evt);
			printf("%s: invalid command complete event, short len\n",
			    DEVNAME(hci));
			return;
		}
		cmd_complete = (struct bthci_evt_complete *) evt;
		if (cmd_complete->event.op != hci->cmd.head.op) {
			DUMP_BT_EVT_COMPLETE(hci, cmd_complete);
			printf("%s: unexpected command complete event, drop\n",
			    DEVNAME(hci));
			pool_put(&hci->evts, evt);
			return;
		}
	} else if (evt->head.op == BT_EVT_CMD_STATE) {
		if (evt->head.len < sizeof(struct bthci_cmd_state)) {
			DUMP_BT_EVT(DEVNAME(hci), evt);
			printf("%s: invalid command state event, short len\n",
			    DEVNAME(hci));
			return;
		}
		cmd_state = (struct bthci_evt_state *) &evt;
		if (cmd_state->event.op != hci->cmd.head.op) {
			DUMP_BT_EVT_STATE(hci, cmd_state);
			printf("%s: unexpected command state event, drop\n",
			    DEVNAME(hci));
			pool_put(&hci->evts, evt);
			return;
		}
	}
	if (hci->evt_filter == evt->head.op) {
		if (hci->evt) {
			DUMP_BT_EVT(DEVNAME(hci), evt);
			printf("%s: pending command filtered event, drop\n",
			    DEVNAME(hci));
			wakeup(hci);
			pool_put(&hci->evts, evt);
			return;
		}
		/* XXX debug */
		DUMP_BT_EVT(DEVNAME(hci), evt);
		hci->evt = evt;
		wakeup(hci);
		return;
	} else if (evt->head.op == BT_EVT_CMD_COMPLETE ||
	    evt->head.op == BT_EVT_CMD_STATE) {
		DUMP_BT_EVT(DEVNAME(hci), evt);
		printf("%s: unexpected command event, drop\n",
		    DEVNAME(hci));
		wakeup(hci);
		pool_put(&hci->evts, evt);
		return;
	}
	/* XXX debug */
	DUMP_BT_EVT(DEVNAME(hci), evt);
	/* XXX SIMPLEQ_INSERT_TAIL(&hci->fifo, (struct bthci_evt *)evt, fifo); */
	pool_put(&hci->evts, evt);
}

struct bt_evt *
bthci_read_evt(struct bthci *hci)
{
	struct bthci_evt *evt;

	mtx_enter(&hci->mtx);
	evt = SIMPLEQ_FIRST(&hci->fifo);
	if (evt)
		SIMPLEQ_REMOVE_HEAD(&hci->fifo, fifo);
	mtx_leave(&hci->mtx);
	return ((struct bt_evt *)evt);
}

/* hci link control inquiry
 * 3 bytes LAP : see bluetooth specifications/assigned-numbers/baseband/
 * 1 byte timeout : max 0x30 * 1,28s
 * 1 byte limit : 0x00 mean no limit
 */
#define BT_HCI_LC_INQUIRY		(BT_HCI_OCF_INQUIRY|(BT_HCI_OGF_LC<<10))
#define BT_HCI_INQUIRY_LAP_2		0x9E
#define BT_HCI_INQUIRY_LAP_1		0x8B
#define BT_HCI_INQUIRY_LAP_GIAC_0	0x33
#define BT_HCI_INQUIRY_LAP_LIAC_0	0x00
int
bthci_lc_inquiry(struct bthci *hci, int timeout, int limit)
{
	int err;
	struct bthci_evt_state *cmd_state;
	struct hci_lc_inquiry {
		uint8_t lap[3];
		uint8_t timeout;
		uint8_t limit;
	} * inquiry;
	bthci_enter(hci);
	if (timeout * 100 / 128 > 0x30 || timeout < 0) {
		printf("%s: bthci_inquiry invalid timeout 0 > %d > 0x30\n",
		    DEVNAME(hci), timeout);
		return (EINVAL);
	}
	if (limit > 0xFF || limit < 0) {
		printf("%s: bthci_inquiry invalid limit 0 > %d > 0xFF\n",
		    DEVNAME(hci), timeout);
		return (EINVAL);
	}
	if (hci->cmd.head.op != 0) {
		printf("%s: bthci command pending, return\n",
		    DEVNAME(hci));
		return (EBUSY);
	}
	hci->cmd.head.op = htole16(BT_HCI_LC_INQUIRY);
	hci->cmd.head.len = sizeof(struct hci_lc_inquiry);
	inquiry = (struct hci_lc_inquiry *)&hci->cmd.data;
	inquiry->lap[0] = BT_HCI_INQUIRY_LAP_GIAC_0;
	inquiry->lap[1] = BT_HCI_INQUIRY_LAP_1;
	inquiry->lap[2] = BT_HCI_INQUIRY_LAP_2;
	inquiry->timeout = timeout * 100 / 128;
	inquiry->limit = limit;
	err = bthci_cmd(hci, BT_EVT_CMD_STATE);
	bthci_leave(hci);
	if (hci->evt) {
		cmd_state = (struct bthci_evt_state *)hci->evt;
		err = cmd_state->event.state;
		pool_put(&hci->evts, hci->evt);
		hci->evt = NULL;
	}
	return (err);
}

/*
 * hci controller and baseband
 */
#define BT_HCI_CB_RESET		(BT_HCI_OCF_RESET|(BT_HCI_OGF_CB<<10))
int
bthci_cb_reset(struct bthci *hci)
{
	int err;
	struct bthci_evt_complete *cmd_complete;
	bthci_enter(hci);
	hci->cmd.head.op = htole16(BT_HCI_CB_RESET);
	hci->cmd.head.len = 0;
	if ((err = bthci_cmd(hci, BT_EVT_CMD_COMPLETE)))
		goto fail;
	if (hci->evt) {
		cmd_complete = (struct bthci_evt_complete *)hci->evt;
		if (cmd_complete->head.len !=
		    sizeof(struct bthci_cmd_complete) + 1) {
			printf("%s: invalid event parameter len %d != %d\n",
			    DEVNAME(hci), cmd_complete->head.len,
			    (int)(sizeof(struct bthci_cmd_complete) + 1));
			err = BT_ERR_UNKNOW; /* XXX proper error code */
		} else
			err = cmd_complete->data[0];
		pool_put(&hci->evts, hci->evt);
		hci->evt = NULL;
	}
 fail:
	bthci_leave(hci);
	return (err);
}

#define BT_HCI_INFO_VERSION	(BT_HCI_OCF_READ_VERSION|(BT_HCI_OGF_INFO<<10))
int
bthci_info_version(struct bthci *hci)
{
	int err;
	bthci_enter(hci);
	hci->cmd.head.op = htole16(BT_HCI_INFO_VERSION);
	hci->cmd.head.len = 0;
	err = bthci_cmd(hci, BT_EVT_CMD_COMPLETE);
	bthci_leave(hci);
	return (err);
}

#define BT_HCI_INFO_COMMANDS	(BT_HCI_OCF_READ_COMMANDS|(BT_HCI_OGF_INFO<<10))
int
bthci_info_commands(struct bthci *hci)
{
	int err;
	bthci_enter(hci);
	hci->cmd.head.op = htole16(BT_HCI_INFO_COMMANDS);
	hci->cmd.head.len = 0;
	err = bthci_cmd(hci, BT_EVT_CMD_COMPLETE);
	bthci_leave(hci);
	return (err);
}

#define BT_HCI_INFO_FEATURES	(BT_HCI_OCF_READ_FEATURES|(BT_HCI_OGF_INFO<<10))
int
bthci_info_features(struct bthci *hci)
{
	int err;
	bthci_enter(hci);
	hci->cmd.head.op = htole16(BT_HCI_INFO_FEATURES);
	hci->cmd.head.len = 0;
	err = bthci_cmd(hci, BT_EVT_CMD_COMPLETE);
	bthci_leave(hci);
	return (err);
}

#define BT_HCI_INFO_FEATURES_E	(BT_HCI_OCF_READ_FEATURES_E|(BT_HCI_OGF_INFO<<10))
int
bthci_info_extended_features(struct bthci *hci)
{
	int err;
	bthci_enter(hci);
	hci->cmd.head.op = htole16(BT_HCI_INFO_FEATURES_E);
	hci->cmd.head.len = 0;
	err = bthci_cmd(hci, BT_EVT_CMD_COMPLETE);
	bthci_leave(hci);
	return (err);
}

#define BT_HCI_INFO_BUFFER	(BT_HCI_OCF_READ_BUFFER_SIZE|(BT_HCI_OGF_INFO<<10))
int
bthci_info_buffer(struct bthci *hci)
{
	int err;
	mtx_enter(&hci->mtx);
	hci->cmd.head.op = htole16(BT_HCI_INFO_BUFFER);
	hci->cmd.head.len = 0;
	err = bthci_cmd(hci, BT_EVT_CMD_COMPLETE);
	bthci_leave(hci);
	return (err);
}

#define BT_HCI_INFO_BDADDR	(BT_HCI_OCF_READ_BDADDR|(BT_HCI_OGF_INFO<<10))
int
bthci_info_bdaddr(struct bthci *hci)
{
	int err;
	bthci_enter(hci);
	hci->cmd.head.op = htole16(BT_HCI_INFO_BDADDR);
	hci->cmd.head.len = 0;
	err = bthci_cmd(hci, BT_EVT_CMD_COMPLETE);
	bthci_leave(hci);
	return (err);
}

int
bthci_enter(struct bthci *hci)
{
	mtx_enter(&hci->mtx);
	if (hci->cmd.head.op != 0) {
		printf("%s: bthci locked, err=EBUSY\n",
		    DEVNAME(hci));
		mtx_leave(&hci->mtx);
		return (EBUSY);
	}
	return (0);
}

int
bthci_cmd(struct bthci *hci, uint8_t evt_filter)
{
	int err;
	hci->evt_filter = evt_filter;
	/* XXX debug */
	DUMP_BT_CMD(DEVNAME(hci), &hci->cmd);
	mtx_leave(&hci->mtx);
	err = hci->bus->cmd(hci->sc, &hci->cmd);
	mtx_enter(&hci->mtx);
	if (err)
		return (err);
	if (hci->evt == NULL)
		err = msleep_nsec(hci, &hci->mtx, MAXPRI, "bthci", BT_TIMEOUT);
	return (err);
}

void
bthci_leave(struct bthci *hci)
{
	if (hci->evt) {
		pool_put(&hci->evts, hci->evt);
		hci->evt = NULL;
	}
	hci->evt_filter = 0;
	hci->cmd.head.op = 0;
	mtx_leave(&hci->mtx);
}
