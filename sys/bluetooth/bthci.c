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
#include <sys/errno.h>
#include <machine/intr.h>
#include <sys/time.h>
#include <sys/pool.h>
#include <sys/mutex.h>
#include <sys/device.h>
#include <sys/systm.h>
#include <sys/mutex.h>
#include <dev/bluetooth/bluetooth.h>

#include "bthci.h"

#ifdef BTHCI_DEBUG
#define DPRINTF(x)	do { printf x; } while (0)
#else
#define DPRINTF(x)
#endif
#define DEVNAME(hci) ((hci)->sc->dv_xname)

#define HCI_EVTS_POOLSIZE		32
#ifdef BTHCI_DEBUG
#define DUMP_BTHCI_EVT(hci, evt) do {						\
	DPRINTF(("%s: event head(op=%02x, len=%d)",				\
	    DEVNAME(hci), evt->head.op, evt->head.len));			\
	for (int i = 0; i < evt->head.len; i++)					\
		DPRINTF((" %02x", evt->data[i]));				\
	DPRINTF(("]\n"));							\
} while(0)
#else
#define DUMP_BTHCI_EVT(hci, evt)
#endif

#define HCI_EVENT_INQUIRY_COMPL			0x01
#define HCI_EVENT_INQUIRY_RESULT		0x02
#define HCI_EVENT_CON_COMPL			0x03
#define HCI_EVENT_CON_REQ			0x04
#define HCI_EVENT_DISCON_COMPL			0x05
#define HCI_EVENT_AUTH_COMPL			0x06
#define HCI_EVENT_REMOTE_NAME_REQ_COMPL		0x07
#define HCI_EVENT_ENCRYPTION_CHANGE		0x08
#define HCI_EVENT_CHANGE_CON_LINK_KEY_COMPL	0x09
#define HCI_EVENT_MASTER_LINK_KEY_COMPL		0x0a
#define HCI_EVENT_READ_REMOTE_FEATURES_COMPL	0x0b
#define HCI_EVENT_READ_REMOTE_VER_INFO_COMPL	0x0c
#define HCI_EVENT_QOS_SETUP_COMPL		0x0d
#define HCI_EVENT_COMMAND_COMPL			0x0e
#define HCI_EVENT_COMMAND_STATE			0x0f
#define HCI_EVENT_HARDWARE_ERROR		0x10
#define HCI_EVENT_FLUSH_OCCUR			0x11
#define HCI_EVENT_ROLE_CHANGE			0x12
#define HCI_EVENT_NUM_COMPL_PKTS		0x13
#define HCI_EVENT_MODE_CHANGE			0x14
#define HCI_EVENT_RETURN_LINK_KEYS		0x15
#define HCI_EVENT_PIN_CODE_REQ			0x16
#define HCI_EVENT_LINK_KEY_REQ			0x17
#define HCI_EVENT_LINK_KEY_NOTIFICATION		0x18
#define HCI_EVENT_LOOPBACK_COMMAND		0x19
#define HCI_EVENT_DATA_BUFFER_OVERFLOW		0x1a
#define HCI_EVENT_MAX_SLOT_CHANGE		0x1b
#define HCI_EVENT_READ_CLOCK_OFFSET_COMPL	0x1c
#define HCI_EVENT_CON_PKT_TYPE_CHANGED		0x1d
#define HCI_EVENT_QOS_VIOLATION			0x1e
#define HCI_EVENT_PAGE_SCAN_MODE_CHANGE		0x1f
#define HCI_EVENT_PAGE_SCAN_REP_MODE_CHANGE	0x20
#define HCI_EVENT_FLOW_SPECIFICATION_COMPL	0x21
#define HCI_EVENT_RSSI_RESULT			0x22
#define HCI_EVENT_READ_REMOTE_EXTENDED_FEATURES	0x23
#define HCI_EVENT_SCO_CON_COMPL			0x2c
#define HCI_EVENT_SCO_CON_CHANGED		0x2d
#define HCI_EVENT_SNIFF_SUBRATING		0x2e
#define HCI_EVENT_EXTENDED_RESULT		0x2f
#define HCI_EVENT_ENCRYPTION_KEY_REFRESH	0x30
#define HCI_EVENT_IO_CAPABILITY_REQ		0x31
#define HCI_EVENT_IO_CAPABILITY_RSP		0x32
#define HCI_EVENT_USER_CONFIRM_REQ		0x33
#define HCI_EVENT_USER_PASSKEY_REQ		0x34
#define HCI_EVENT_REMOTE_OOB_DATA_REQ		0x35
#define HCI_EVENT_SIMPLE_PAIRING_COMPL		0x36
#define HCI_EVENT_LINK_SUPERVISION_TO_CHANGED	0x38
#define HCI_EVENT_ENHANCED_FLUSH_COMPL		0x39
#define HCI_EVENT_USER_PASSKEY_NOTIFICATION	0x3b
#define HCI_EVENT_KEYPRESS_NOTIFICATION		0x3c
#define HCI_EVENT_REMOTE_FEATURES_NOTIFICATION	0x3d
#define HCI_EVENT_BT_LOGO			0xfe
#define HCI_EVENT_VENDOR			0xff

#define HCI_OGF_LC			0x01
#define HCI_OCF_INQUIRY			0x0001

#define HCI_OGF_CB			0x03
#define HCI_OCF_RESET			0x0003

#define HCI_OGF_INFO			0x04
#define HCI_OCF_READ_VERSION		0x0001
#define HCI_OCF_READ_COMMANDS		0x0002
#define HCI_OCF_READ_FEATURES		0x0003
#define HCI_OCF_READ_FEATURES_E		0x0004
#define HCI_OCF_READ_BUFFER_SIZE	0x0005
#define HCI_OCF_READ_BDADDR		0x0009

struct hci_cmd_complete {
	uint8_t			buff_sz;
	uint16_t		op;
};
struct bthci_evt_complete {
	struct bt_evt_head	head;
	struct hci_cmd_complete	event;
	/* evt_head + cmd_complete = 5 */
	uint8_t			data[sizeof(struct bt_evt)-5];
};
#ifdef BTHCI_DEBUG
#define DUMP_BTHCI_EVT_COMPLETE(hci, evt) do {					\
	DPRINTF(("%s: event head(op=%02X, len=%d), event(buff_sz=%d, op=%04X)",	\
	    DEVNAME(hci), (evt)->head.op, (evt)->head.len,			\
	    (evt)->event.buff_sz, (evt)->event.op));				\
	for (int i = 0;								\
	    i < (evt)->head.len - sizeof(struct hci_cmd_complete); i++)		\
		DPRINTF((" %02x", (evt)->data[i]));				\
	DPRINTF(("\n"));							\
} while(0)
#else
#define DUMP_BTHCI_EVT_COMPLETE(hci, evt)
#endif /* BTHCI_DEBUG */

struct hci_cmd_state {
	uint8_t			state;
	uint8_t			buff_sz;
	uint16_t		op;
};
struct bthci_evt_state {
	struct bt_evt_head	head;
	struct hci_cmd_state	event;
};
#ifdef BTHCI_DEBUG
#define DUMP_BTHCI_EVT_STATE(hci, evt) do {					\
	DPRINTF(("%s: event head(op=%02X, len=%d), "				\
	    "event(state=%02X, buff_sz=%d, op=%04X)\n",				\
	    DEVNAME(hci), (evt)->head.op, (evt)->head.len,			\
	    (evt)->event.state, (evt)->event.buff_sz, (evt)->event.op));	\
} while(0)
#else
#define DUMP_BTHCI_EVT_STATE(hci, evt)
#endif /* BTHCI_DEBUG */

int bthci_enter(struct bthci *);
int bthci_cmd(struct bthci *);
void bthci_leave(struct bthci *);

void
bthci_init(struct bthci *hci, struct device *sc, struct btbus *bus, int ipl)
{
	hci->sc = sc;
	hci->bus = bus;
	mtx_init(&hci->mtx, ipl);
	pool_init(&hci->evts, sizeof(struct bthci_evt), 0, ipl, 0,
	    "bthci", NULL);
	if (pool_prime(&hci->evts, HCI_EVTS_POOLSIZE))
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

	if (evt->head.op == HCI_EVENT_COMMAND_COMPL) {
		if (evt->head.len < sizeof(struct hci_cmd_complete)) {
			printf("%s: invalid command complete event, short len\n",
			    DEVNAME(hci));
			DUMP_BTHCI_EVT(hci, evt);
			return;
		}
		cmd_complete = (struct bthci_evt_complete *) evt;
		DUMP_BTHCI_EVT_COMPLETE(hci, cmd_complete);
		if (hci->cmd_complete) {
			printf("%s: pending command complete event, abort\n",
			    DEVNAME(hci));
			wakeup(hci);
			pool_put(&hci->evts, evt);
			return;
		}
		if (cmd_complete->event.op == hci->cmd.head.op) {
			hci->cmd_complete = cmd_complete;
			wakeup(hci);
		}
		printf("%s: unexpected command complete event, drop\n",
		    DEVNAME(hci));
		pool_put(&hci->evts, evt);
		return;
	} else if (evt->head.op == HCI_EVENT_COMMAND_STATE) {
		if (evt->head.len < sizeof(struct hci_cmd_state)) {
			printf("%s: invalid command state event, short len\n",
			    DEVNAME(hci));
			DUMP_BTHCI_EVT(hci, evt);
			return;
		}
		cmd_state = (struct bthci_evt_state *) &evt;
		DUMP_BTHCI_EVT_STATE(hci, cmd_state);
		if (hci->cmd_state) {
			printf("%s: pending command state event, abort\n",
			    DEVNAME(hci));
			wakeup(hci);
			pool_put(&hci->evts, evt);
			return;
		}
		if (cmd_state->event.op == hci->cmd.head.op) {
			hci->cmd_state = cmd_state;
			wakeup(hci);
			return;
		}
		printf("%s: unexpected command state event, drop\n",
		    DEVNAME(hci));
		pool_put(&hci->evts, evt);
		return;
	}
	DUMP_BTHCI_EVT(hci, evt);
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
#define HCI_LC_INQUIRY		(HCI_OCF_INQUIRY|(HCI_OGF_LC<<10))
#define HCI_INQUIRY_LAP_2	0x9E
#define HCI_INQUIRY_LAP_1	0x8B
#define HCI_INQUIRY_LAP_GIAC_0	0x33
#define HCI_INQUIRY_LAP_LIAC_0	0x00
int
bthci_lc_inquiry(struct bthci *hci, int timeout, int limit)
{
	int err;
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
	hci->cmd.head.op = htole16(HCI_LC_INQUIRY);
	hci->cmd.head.len = sizeof(struct hci_lc_inquiry);
	inquiry = (struct hci_lc_inquiry *)&hci->cmd.data;
	inquiry->lap[0] = HCI_INQUIRY_LAP_GIAC_0;
	inquiry->lap[1] = HCI_INQUIRY_LAP_1;
	inquiry->lap[2] = HCI_INQUIRY_LAP_2;
	inquiry->timeout = timeout * 100 / 128;
	inquiry->limit = limit;
	err = bthci_cmd(hci);
	bthci_leave(hci);
	return (err);
}

/*
 * hci controller and baseband
 */
#define HCI_CB_RESET		(HCI_OCF_RESET|(HCI_OGF_CB<<10))
int
bthci_cb_reset(struct bthci *hci)
{
	int err;
	bthci_enter(hci);
	hci->cmd.head.op = htole16(HCI_CB_RESET);
	hci->cmd.head.len = 0;
	if ((err = bthci_cmd(hci)))
		goto fail;
	if (hci->cmd_state) {
		if (hci->cmd_state->event.state)
			err = hci->cmd_state->event.state;
		pool_put(&hci->evts, hci->cmd_state);
		hci->cmd_state = NULL;
		goto fail;
	}
	if (hci->cmd_complete) {
		if (hci->cmd_complete->head.len !=
		    sizeof(struct hci_cmd_complete) + 1) {
			printf("%s: invalid event parameter len\n",
			    DEVNAME(hci));
			err = BTERR_UNKNOW; /* XXX proper error code */
		} else
			err = hci->cmd_complete->data[0];
		pool_put(&hci->evts, hci->cmd_complete);
		hci->cmd_complete = NULL;
	}
 fail:
	bthci_leave(hci);
	return (err);
}

/*
 * hci information
 */
#define HCI_OCF_READ_VERSION		0x0001
#define HCI_OCF_READ_COMMANDS		0x0002
#define HCI_OCF_READ_FEATURES		0x0003
#define HCI_OCF_READ_FEATURES_E		0x0004
#define HCI_OCF_READ_BUFFER_SIZE	0x0005
#define HCI_OCF_READ_BDADDR		0x0009

#define HCI_INFO_VERSION	(HCI_OCF_READ_VERSION|(HCI_OGF_INFO<<10))
int
bthci_info_version(struct bthci *hci)
{
	int err;
	bthci_enter(hci);
	hci->cmd.head.op = htole16(HCI_INFO_VERSION);
	hci->cmd.head.len = 0;
	err = bthci_cmd(hci);
	bthci_leave(hci);
	return (err);
}

#define HCI_INFO_COMMANDS	(HCI_OCF_READ_COMMANDS|(HCI_OGF_INFO<<10))
int
bthci_info_commands(struct bthci *hci)
{
	int err;
	bthci_enter(hci);
	hci->cmd.head.op = htole16(HCI_INFO_COMMANDS);
	hci->cmd.head.len = 0;
	err = bthci_cmd(hci);
	bthci_leave(hci);
	return (err);
}

#define HCI_INFO_FEATURES	(HCI_OCF_READ_FEATURES|(HCI_OGF_INFO<<10))
int
bthci_info_features(struct bthci *hci)
{
	int err;
	bthci_enter(hci);
	hci->cmd.head.op = htole16(HCI_INFO_FEATURES);
	hci->cmd.head.len = 0;
	err = bthci_cmd(hci);
	bthci_leave(hci);
	return (err);
}

#define HCI_INFO_FEATURES_E	(HCI_OCF_READ_FEATURES_E|(HCI_OGF_INFO<<10))
int
bthci_info_extended_features(struct bthci *hci)
{
	int err;
	bthci_enter(hci);
	hci->cmd.head.op = htole16(HCI_INFO_FEATURES_E);
	hci->cmd.head.len = 0;
	err = bthci_cmd(hci);
	bthci_leave(hci);
	return (err);
}

#define HCI_INFO_BUFFER		(HCI_OCF_READ_BUFFER_SIZE|(HCI_OGF_INFO<<10))
int
bthci_info_buffer(struct bthci *hci)
{
	int err;
	mtx_enter(&hci->mtx);
	hci->cmd.head.op = htole16(HCI_INFO_BUFFER);
	hci->cmd.head.len = 0;
	err = bthci_cmd(hci);
	bthci_leave(hci);
	return (err);
}

#define HCI_INFO_BDADDR		(HCI_OCF_READ_BDADDR|(HCI_OGF_INFO<<10))
int
bthci_info_bdaddr(struct bthci *hci)
{
	int err;
	bthci_enter(hci);
	hci->cmd.head.op = htole16(HCI_INFO_BDADDR);
	hci->cmd.head.len = 0;
	err = bthci_cmd(hci);
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
bthci_cmd(struct bthci *hci)
{
	int err;
	mtx_leave(&hci->mtx);
	err = hci->bus->cmd(hci->sc, &hci->cmd);
	mtx_enter(&hci->mtx);
	if (err)
		return (err);
	err = msleep_nsec(hci, &hci->mtx, 0, "bthci", BT_TIMEOUT);
	return (err);
}

void
bthci_leave(struct bthci *hci)
{
	hci->cmd.head.op = 0;
	mtx_leave(&hci->mtx);
}
