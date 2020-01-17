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
#include <sys/device.h>

struct bthci_evt {
	struct bt_evt			evt;
	SIMPLEQ_ENTRY(bthci_evt)	fifo;
};
SIMPLEQ_HEAD(bthci_evts, bthci_evt);

struct bthci_filter {
	uint8_t			 filter;
	struct bt_evt		*evt;
	LIST_ENTRY(bthci_filter) l;
};
LIST_HEAD(bthci_filters, bthci_filter);

struct bthci {
	struct device		*sc;
	struct btbus		*bus;
	struct mutex		 mtx;
	int			 count;
	void			*ident;
	struct bt_cmd		 cmd;
	uint8_t			 evt_filter;
	struct bt_evt		*evt;
	struct bthci_filters	 filters;
	struct pool		 evts;
	struct bthci_evts	 fifo;
};

void bthci_init(struct bthci *, struct device *, struct btbus *, int);
void bthci_register(struct bthci *, void *);
void bthci_die(struct bthci *);
void bthci_free(struct bthci *);
struct bt_evt *bthci_pool_get(struct bthci *);
void bthci_pool_put(struct bthci *, struct bt_evt *);
void bthci_write_evt(struct bthci *, struct bt_evt *);
struct bt_evt *bthci_read_evt(struct bthci *);

int bthci_lc_inquiry(struct bthci *, uint64_t, int);

struct bt_hci_lc_connect {
	uint8_t			state;
	uint16_t		handle;
	struct bluetooth_bdaddr	bdaddr;
	uint8_t			link_type;
	uint8_t			encryption;
} __packed;
int bthci_lc_connect(struct bthci *, uint16_t, struct bluetooth_device *,
    struct bt_hci_lc_connect *);
struct bt_hci_lc_disconnect {
	uint8_t			state;
	uint16_t		handle;
	uint8_t			reason;
} __packed;
int bthci_lc_disconnect(struct bthci *, uint16_t, struct bt_hci_lc_disconnect *);
struct bt_hci_lc_remote_name {
	uint8_t			state;
	struct bluetooth_bdaddr	bdaddr;
	char			name[248]; /* XXX see BT_EVT_MAX_PAYLOAD */
} __packed;
int bthci_lc_remote_name(struct bthci *, struct bluetooth_bdaddr *,
    uint8_t, uint16_t, struct bt_hci_lc_remote_name *);

int bthci_cb_reset_event_mask(struct bthci *);
int bthci_cb_reset(struct bthci *);
int bthci_cb_name(struct bthci *, char *);

struct bt_hci_info_version {
	uint8_t		hci_version;
	uint16_t	hci_revision;
	uint8_t		lmp_version;
	uint16_t	bt_manufacturer;
	uint16_t	lmp_revision;
} __packed;
int bthci_info_version(struct bthci *, struct bt_hci_info_version *);
struct bt_hci_info_commands {
	uint8_t		bitmask[BT_COMMANDS_BITMASK_LEN];
} __packed;
int bthci_info_commands(struct bthci *, struct bt_hci_info_commands *);
struct bt_hci_info_features {
	uint8_t		bitmask[BT_FEATURES_BITMASK_LEN];
} __packed;
int bthci_info_features(struct bthci *, struct bt_hci_info_features *);
struct bt_hci_info_extended {
	uint8_t		page;
	uint8_t		max_page;
	uint8_t		bitmask[BT_FEATURES_BITMASK_LEN];
} __packed;
int bthci_info_extended(struct bthci *, int, struct bt_hci_info_extended *);
struct bt_hci_info_buffer {
	uint16_t	acl_size;
	uint8_t		sco_size;
	uint16_t	acl_bufferlen;
	uint16_t	sco_bufferlen;
} __packed;
int bthci_info_buffer(struct bthci *, struct bt_hci_info_buffer *);
struct bt_hci_info_bdaddr {
	struct bluetooth_bdaddr	bdaddr;
} __packed;
int bthci_info_bdaddr(struct bthci *, struct bt_hci_info_bdaddr *);
