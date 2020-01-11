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

#define BTPRI			MAXPRI /* XXX to change for something else */
#define BT_TIMEOUT		SEC_TO_NSEC(5)
#define BT_STATE_INIT		0
#define BT_STATE_DYING		1
#define BT_STATE_WAITING	2
#define BT_STATE_DEVOPEN	3
#define BT_STATE_INQUIRY	4

#define BT_EVTS_POOLSIZE	32

struct bt_cmd;
struct bt_evt;
struct bt_acl;
struct bt_sco;
struct bthci;
struct btbus {
	int (*cmd)(struct device *, const struct bt_cmd *);
	int (*acl)(struct device *, const struct bt_acl *);
	int (*sco)(struct device *, const struct bt_sco *);
};

struct bluetooth_dev_io {
	size_t				 size;
	void				*buf;
	SIMPLEQ_ENTRY(bluetooth_dev_io)	 fifo;
};
SIMPLEQ_HEAD(bluetooth_dev_ios, bluetooth_dev_io);

struct bluetooth_softc {
	struct device			 sc_dev;
	struct bthci			*hci;
	struct rwlock			 lock; /* XXX actually, a mutex feet */
	int				 state;
	struct bluetooth_dev_ios	 fifo_tx;
	struct bluetooth_dev_ios	 fifo_rx; /* XXX not used */
};

void bluetooth_attach(struct bluetooth_softc *,  struct bthci *);
void bluetooth_detach(struct bluetooth_softc *);
