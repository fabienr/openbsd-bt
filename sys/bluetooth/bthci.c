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
#include <sys/malloc.h>

#include <bluetooth/bluetooth.h>
#include <dev/bluetooth/bluetoothreg.h>
#include <dev/bluetooth/bluetoothvar.h>
#include <bluetooth/bthci.h>

/* XXX debug */
#ifdef BLUETOOTH_DEBUG
#define BTHCI_DEBUG
#endif

#ifdef BTHCI_DEBUG
#define DPRINTF(x)	do { printf x; } while (0)
#define IF_DPRINTF(c,x)	do { if (c) { printf x; } } while(0) /* XXX keep ? */
#else
#define DPRINTF(x)
#define IF_DPRINTF(c,x)
#endif

#define DEVNAME(hci) ((hci)->sc->dv_xname)

/* private registers */
struct bthci_cmd_complete {
	uint8_t			buff_sz;
	uint16_t		op;
	uint8_t			state;
} __packed;
struct bthci_evt_complete {
	struct bt_evt_head		head;
	struct bthci_cmd_complete	event;
	/* evt_head + cmd_complete = 6 */
	uint8_t				data[sizeof(struct bt_evt) - 6];
} __packed;
#ifdef BTHCI_DEBUG
#define DUMP_BT_EVT_COMPLETE(hci, evt) do {					\
	DPRINTF(("%s: command complete head(op=%02X, len=%d), "			\
	    "event(buff_sz=%d, op=%04x, state=%0x), data",			\
	    DEVNAME(hci), (evt)->head.op, (evt)->head.len,			\
	    (evt)->event.buff_sz, (evt)->event.op, (evt)->event.state));	\
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

/* private functions */
int bthci_enter(struct bthci *);
int bthci_cmd(struct bthci *, uint8_t);
int bthci_cmd_state(struct bthci *);
int bthci_cmd_complete(struct bthci *, void *, int);
int bthci_void(struct bthci *, void *, int, int);
void bthci_leave(struct bthci *);

/* public api */
void
bthci_init(struct bthci *hci, struct device *sc, struct btbus *bus, int ipl)
{
	hci->sc = sc;
	DPRINTF(("%s: bthci_init\n", DEVNAME(hci)));
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
bthci_register(struct bthci *hci, void *ident)
{
	mtx_enter(&hci->mtx);
	hci->ident = ident;
	mtx_leave(&hci->mtx);
}

void
bthci_destroy(struct bthci *hci)
{
	DPRINTF(("%s: bthci_destroy\n", DEVNAME(hci)));
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
	/* from interrupt context */
	struct bthci_evt_complete *cmd_complete;
	struct bthci_evt_state *cmd_state;

	/* XXX debug */
	/* DUMP_BT_EVT(DEVNAME(hci), evt); */

	/* check BT_EVT_CMD_COMPLETE|_STATE event consistency */
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
#ifdef BTHCI_DEBUG
		if (hci->evt_filter == evt->head.op)
			DUMP_BT_EVT_COMPLETE(hci, cmd_complete);
#endif /* BTHCI_DEBUG */
	} else if (evt->head.op == BT_EVT_CMD_STATE) {
		if (evt->head.len < sizeof(struct bthci_cmd_state)) {
			DUMP_BT_EVT(DEVNAME(hci), evt);
			printf("%s: invalid command state event, short len\n",
			    DEVNAME(hci));
			return;
		}
		cmd_state = (struct bthci_evt_state *) evt;
		if (cmd_state->event.op != hci->cmd.head.op) {
			DUMP_BT_EVT_STATE(hci, cmd_state);
			printf("%s: unexpected command state event, drop\n",
			    DEVNAME(hci));
			pool_put(&hci->evts, evt);
			return;
		}
#ifdef BTHCI_DEBUG
		if (hci->evt_filter == evt->head.op)
			DUMP_BT_EVT_STATE(hci, cmd_state);
#endif /* BTHCI_DEBUG */
	}
	/* catch event related to bthci_cmd and wake it up */
	if (hci->evt_filter == evt->head.op) {
		if (hci->evt) {
			DUMP_BT_EVT(DEVNAME(hci), evt);
			printf("%s: previous filtered event pending, drop\n",
			    DEVNAME(hci));
			wakeup(hci);
			pool_put(&hci->evts, evt);
			return;
		}
		/* pass evt to bthci_cmd */
		hci->evt = evt;
		wakeup(hci);
		return;
	} else if (evt->head.op == BT_EVT_CMD_COMPLETE ||
	    evt->head.op == BT_EVT_CMD_STATE) {
		if (evt->head.op == BT_EVT_CMD_COMPLETE) {
			cmd_complete = (struct bthci_evt_complete *) evt;
			DUMP_BT_EVT_COMPLETE(hci, cmd_complete);
			printf("%s: unrelated command complete event, drop\n",
			    DEVNAME(hci));
		} else {
			cmd_state = (struct bthci_evt_state *) evt;
			DUMP_BT_EVT_STATE(hci, cmd_state);
			printf("%s: unrelated command state event, drop\n",
			    DEVNAME(hci));
		}
		/* throw error to bthci_cmd */
		wakeup(hci);
		pool_put(&hci->evts, evt);
		return;
	}
	/* push event on the queue for later processing through bthci_read_evt */
	SIMPLEQ_INSERT_TAIL(&hci->fifo, (struct bthci_evt *)evt, fifo);
	/* wakeup blutooth_kthread */
	if (hci->ident)
		wakeup(hci->ident);
}

struct bt_evt *
bthci_read_evt(struct bthci *hci)
{
	struct bthci_evt *evt;
	mtx_enter(&hci->mtx);
	evt = SIMPLEQ_FIRST(&hci->fifo);
	if (evt) {
		DPRINTF(("%s: bthci_read_evt\n", DEVNAME(hci)));
		SIMPLEQ_REMOVE_HEAD(&hci->fifo, fifo);
	}
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
bthci_lc_inquiry(struct bthci *hci, uint64_t timeout, int limit)
{
	struct lc_inquiry {
		uint8_t lap[3];
		uint8_t timeout;
		uint8_t limit;
	} __packed * inquiry;
	int err;
	DPRINTF(("%s: bthci_lc_inquiry(%llu second, %d limit)\n",
	    DEVNAME(hci), timeout / 1000000000ULL, limit));
	if ((err = bthci_enter(hci)) != 0)
		return (err);
	if ((timeout / 1000000000ULL) * 100 / 128 > 0x30 || timeout < 0) {
		printf("%s: bthci_inquiry invalid timeout 0 > %llu > 0x30\n",
		    DEVNAME(hci), (timeout / 1000000000ULL));
		err = EINVAL;
		goto fail;
	}
	if (limit > 0xFF || limit < 0) {
		printf("%s: bthci_inquiry invalid limit 0 > %d > 0xFF\n",
		    DEVNAME(hci), limit);
		err = EINVAL;
		goto fail;
	}
	hci->cmd.head.op = htole16(BT_HCI_LC_INQUIRY);
	hci->cmd.head.len = sizeof(*inquiry);
	inquiry = (struct lc_inquiry *)&hci->cmd.data;
	inquiry->lap[0] = BT_HCI_INQUIRY_LAP_GIAC_0;
	inquiry->lap[1] = BT_HCI_INQUIRY_LAP_1;
	inquiry->lap[2] = BT_HCI_INQUIRY_LAP_2;
	inquiry->timeout = (timeout / 1000000000ULL) * 100 / 128;
	inquiry->limit = limit;
	if ((err = bthci_cmd(hci, BT_EVT_CMD_STATE)))
		goto fail;
	if ((err = bthci_cmd_state(hci)))
		goto fail;
 fail:
	bthci_leave(hci);
	return (err);
}

#define BT_HCI_LC_CONNECT	(BT_HCI_OCF_CONNECTION|(BT_HCI_OGF_LC<<10))
int
bthci_lc_connect(struct bthci *hci, uint16_t acl_type,
    struct bluetooth_device *dev)
{
	struct lc_connection {
		struct bluetooth_bdaddr	bdaddr;
		uint16_t		acl_type;
		uint8_t			scan_mode;
		uint8_t			reserved;
		uint16_t		clock;
		uint8_t			role_switch;
	} __packed *connection;
	int err = 0, i;
#ifdef BTHCI_DEBUG
	DPRINTF(("%s: bthci_lc_connect(%000X, [#%X, ",
	    DEVNAME(hci), acl_type, dev->unit));
	for (i = BT_ADDR_LEN; --i >= 0;)
		DPRINTF(("%0X%c", dev->bt_addr.b[i], (i)?':':','));
	DPRINTF((" %u, %000x])\n", dev->bt_scan_mode, dev->bt_clock));
#endif /* BTHCI_DEBUG */
	if ((err = bthci_enter(hci)) != 0)
		return (err);
	hci->cmd.head.op = htole16(BT_HCI_LC_CONNECT);
	hci->cmd.head.len = sizeof(*connection);
	connection = (struct lc_connection *)&hci->cmd.data;
	for (i = BT_ADDR_LEN; --i >= 0;)
		connection->bdaddr.b[i] = dev->bt_addr.b[i];
	connection->acl_type = acl_type;
	connection->scan_mode = dev->bt_scan_mode;
	connection->clock = dev->bt_clock;
	connection->role_switch = 0; /* XXX depends on controller ? */
	if ((err = bthci_cmd(hci, BT_EVT_CMD_STATE)))
		goto fail;
	if ((err = bthci_cmd_state(hci)))
		goto fail;
 fail:
	bthci_leave(hci);
	return (err);
}

#define BT_HCI_LC_DISCONNECT	(BT_HCI_OCF_DISCONNECT|(BT_HCI_OGF_LC<<10))
int
bthci_lc_disconnect(struct bthci *hci, uint16_t handle)
{
	/* XXX not implemented yet */
	return (0);
}

#define BT_HCI_LC_REMOTE_NAME	(BT_HCI_OCF_REMOTE_NAME|(BT_HCI_OGF_LC<<10))
int
bthci_lc_remote_name(struct bthci *hci, struct bluetooth_bdaddr *bdaddr,
    uint8_t mode, uint16_t clock)
{
	struct lc_remote_name {
		struct bluetooth_bdaddr	addr;
		uint8_t			mode;
		uint8_t			reserved;
		uint16_t		clock;
	} __packed *remote_name;
	int err;
#ifdef BTHCI_DEBUG
	int i;
	DPRINTF(("%s: bthci_lc_remote_name(", DEVNAME(hci)));
	for (i = BT_ADDR_LEN; --i >= 0;)
		DPRINTF(("%0X%c", bdaddr->b[i], (i)?':':' '));
	DPRINTF((", %u, %x)\n", mode, clock));
#endif /* BTHCI_DEBUG */
	if ((err = bthci_enter(hci)) != 0)
		return (err);
	hci->cmd.head.op = htole16(BT_HCI_LC_REMOTE_NAME);
	hci->cmd.head.len = sizeof(*remote_name);
	remote_name = (struct lc_remote_name *)&hci->cmd.data;
	memcpy(&remote_name->addr, bdaddr, sizeof(*bdaddr));
	remote_name->mode = mode;
	remote_name->clock = clock;
	if ((err = bthci_cmd(hci, BT_EVT_CMD_STATE)))
		goto fail;
	if ((err = bthci_cmd_state(hci)))
		goto fail;
 fail:
	bthci_leave(hci);
	return (err);
}

#define BT_HCI_CB_RESET		(BT_HCI_OCF_RESET|(BT_HCI_OGF_CB<<10))
int
bthci_cb_reset(struct bthci *hci)
{
	DPRINTF(("%s: bthci_cb_reset\n", DEVNAME(hci)));
	return (bthci_void(hci, NULL, 0, BT_HCI_CB_RESET));
}

#define BT_HCI_CB_NAME		(BT_HCI_OCF_WRITE_NAME|(BT_HCI_OGF_CB<<10))
int
bthci_cb_name(struct bthci *hci, char *local_name)
{
	struct cb_name {
		char	local[248];
	} * name;
	int err;
	DPRINTF(("%s: bthci_cb_name(%s)\n", DEVNAME(hci), local_name));
	if ((err = bthci_enter(hci)) != 0)
		return (err);
	if (strlen(local_name) >= sizeof(*name)) {
		printf("%s: bthci_cb_name invalid local name lenght %zu > %zu\n",
		    DEVNAME(hci), strlen(local_name), sizeof(*name));
		err = EINVAL;
		goto fail;
	}
	hci->cmd.head.op = htole16(BT_HCI_CB_NAME);
	hci->cmd.head.len = sizeof(*name);
	name = (struct cb_name *)&hci->cmd.data;
	strlcpy((char *)&name->local, local_name, sizeof(name->local));
	if ((err = bthci_cmd(hci, BT_EVT_CMD_COMPLETE)))
		goto fail;
	if ((err = bthci_cmd_complete(hci, NULL, 0)))
		goto fail;
 fail:
	bthci_leave(hci);
	return (err);
}


#define BT_HCI_INFO_VERSION	(BT_HCI_OCF_READ_VERSION|(BT_HCI_OGF_INFO<<10))
int
bthci_info_version(struct bthci *hci, struct bt_hci_info_version *info)
{
	DPRINTF(("%s: bthci_info_version\n", DEVNAME(hci)));
	return (bthci_void(hci, info, sizeof(*info), BT_HCI_INFO_VERSION));
}

#define BT_HCI_INFO_COMMANDS	(BT_HCI_OCF_READ_COMMANDS|(BT_HCI_OGF_INFO<<10))
int
bthci_info_commands(struct bthci *hci, struct bt_hci_info_commands *info)
{
	DPRINTF(("%s: bthci_info_commands\n", DEVNAME(hci)));
	return (bthci_void(hci, info, sizeof(*info), BT_HCI_INFO_COMMANDS));
}

#define BT_HCI_INFO_FEATURES	(BT_HCI_OCF_READ_FEATURES|(BT_HCI_OGF_INFO<<10))
int
bthci_info_features(struct bthci *hci, struct bt_hci_info_features *info)
{
	DPRINTF(("%s: bthci_info_features\n", DEVNAME(hci)));
	return (bthci_void(hci, info, sizeof(*info), BT_HCI_INFO_FEATURES));
}

#define BT_HCI_INFO_EXTENDED	(BT_HCI_OCF_READ_EXTENDED|(BT_HCI_OGF_INFO<<10))
int
bthci_info_extended(struct bthci *hci, int p, struct bt_hci_info_extended *info)
{
	struct hci_info_extended {
		uint8_t page;
	} * command;
	int err;
	DPRINTF(("%s: bthci_info_extended\n", DEVNAME(hci)));
	if ((err = bthci_enter(hci)) != 0)
		return (err);
	if (p > 2) {
		printf("%s: bthci_info_extended invalid page %d > 2\n",
		    DEVNAME(hci), p);
		err = (EINVAL);
		goto fail;
	}
	hci->cmd.head.op = htole16(BT_HCI_INFO_EXTENDED);
	hci->cmd.head.len = sizeof(struct hci_info_extended);
	command = (struct hci_info_extended *)&hci->cmd.data;
	command->page = (uint8_t)p;
	if ((err = bthci_cmd(hci, BT_EVT_CMD_COMPLETE)))
		goto fail;
	if ((err = bthci_cmd_complete(hci, info, sizeof(*info))))
		goto fail;
	if (info->page != command->page) {
		printf("%s : bthci_info_extended invalid answer page %d != %d\n",
		    DEVNAME(hci), info->page, command->page);
		err = EPROTO;
	}
 fail:
	bthci_leave(hci);
	return (err);
}

#define BT_HCI_INFO_BUFFER	(BT_HCI_OCF_READ_BUFFER|(BT_HCI_OGF_INFO<<10))
int
bthci_info_buffer(struct bthci *hci, struct bt_hci_info_buffer *info)
{
	int err;
	DPRINTF(("%s: bthci_info_buffer\n", DEVNAME(hci)));
	err = bthci_void(hci, info, sizeof(*info), BT_HCI_INFO_BUFFER);
	if (err == 0) {
		info->acl_size = le16toh(info->acl_size);
		info->acl_bufferlen = le16toh(info->acl_bufferlen);
		info->sco_bufferlen = le16toh(info->sco_bufferlen);
	}
	return (err);
}

#define BT_HCI_INFO_BDADDR	(BT_HCI_OCF_READ_BDADDR|(BT_HCI_OGF_INFO<<10))
int
bthci_info_bdaddr(struct bthci *hci, struct bt_hci_info_bdaddr *info)
{
	DPRINTF(("%s: bthci_info_bdaddr\n", DEVNAME(hci)));
	return (bthci_void(hci, info, sizeof(*info), BT_HCI_INFO_BDADDR));
}

/* private functions */
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
	/* DUMP_BT_CMD(DEVNAME(hci), &hci->cmd); */
	mtx_leave(&hci->mtx);
	err = hci->bus->cmd(hci->sc, &hci->cmd);
	mtx_enter(&hci->mtx);
	if (err)
		goto fail;
	if (hci->evt == NULL)
		err = msleep_nsec(hci, &hci->mtx, BTPRI,
		    DEVNAME(hci), BT_TIMEOUT);
	if (hci->evt == NULL) {
		printf("%s: bthci_cmd, no event, err=%d\n",
		    DEVNAME(hci), err);
		goto fail;
	}
	DPRINTF(("%s: bthci_cmd event\n", DEVNAME(hci)));
 fail:
	return (err);
}

int
bthci_cmd_state(struct bthci *hci)
{
	int err = 0;
	struct bthci_evt_state *cmd_state;
	if (hci->evt == NULL) {
		printf("%s: cmd_complete call with no event\n",
		    DEVNAME(hci));
		return (EPROTO);
	}
	cmd_state = (struct bthci_evt_state *)hci->evt;
	err = BT_ERR_TOH(cmd_state->event.state);
	pool_put(&hci->evts, hci->evt);
	hci->evt = NULL;
	return (err);
}

int
bthci_cmd_complete(struct bthci *hci, void *dest, int len)
{
	int err = 0;
	struct bthci_evt_complete *cmd_complete;
	if (hci->evt == NULL) {
		printf("%s: cmd_complete call with no event\n",
		    DEVNAME(hci));
		return (EPROTO);
	}
	cmd_complete = (struct bthci_evt_complete *)hci->evt;
	if (cmd_complete->head.len - sizeof(struct bthci_cmd_complete) != len) {
		if (dest && len) 
			memset(dest, 0, len); /* XXX not required, safer ? */
		printf("%s: cmd_complete event len mismatch, %u - %zu != %d\n",
		    DEVNAME(hci),
		    cmd_complete->head.len,
		    sizeof(struct bthci_cmd_complete),
		    len);
		return (EPROTO);
	}
	if (dest && len)
		memcpy(dest, cmd_complete->data, len);
	err = BT_ERR_TOH(cmd_complete->event.state);
	pool_put(&hci->evts, hci->evt);
	hci->evt = NULL;
	return (err);
}

int
bthci_void(struct bthci *hci, void *info, int len, int op)
{
	int err;
	if ((err = bthci_enter(hci)) != 0)
		return (err);
	hci->cmd.head.op = htole16(op);
	hci->cmd.head.len = 0;
	if ((err = bthci_cmd(hci, BT_EVT_CMD_COMPLETE)))
		goto fail;
	if ((err = bthci_cmd_complete(hci, info, len)))
		goto fail;
 fail:
	bthci_leave(hci);
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
