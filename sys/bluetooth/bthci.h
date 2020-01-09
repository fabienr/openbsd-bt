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

struct bthci_evt_complete;
struct bthci_evt_state;
struct bthci_evt {
	struct bt_evt			evt;
	SIMPLEQ_ENTRY(bthci_evt)	fifo;
};
SIMPLEQ_HEAD(bthci_evts, bthci_evt);

struct bthci {
	struct device			*sc;
	struct btbus			*bus;
	struct mutex			 mtx;
	struct bt_cmd			 cmd;
	uint8_t				 evt_filter;
	struct bt_evt			*evt;
	struct pool			 evts;
	struct bthci_evts		 fifo;
};

void bthci_init(struct bthci *, struct device *, struct btbus *, int);
void bthci_destroy(struct bthci *);
struct bt_evt *bthci_pool_get(struct bthci *);
void bthci_pool_put(struct bthci *, struct bt_evt *);
void bthci_write_evt(struct bthci *, struct bt_evt *);
struct bt_evt *bthci_read_evt(struct bthci *);

int bthci_lc_inquiry(struct bthci *, int, int);

int bthci_cb_reset(struct bthci *);

int bthci_info_version(struct bthci *, struct bt_hci_info *);
int bthci_info_commands(struct bthci *);
int bthci_info_features(struct bthci *);
int bthci_info_extended_features(struct bthci *);
int bthci_info_buffer(struct bthci *);
int bthci_info_bdaddr(struct bthci *);
