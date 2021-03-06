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

#define BTPRI			PRIBIO /* XXX to change for something else ? */

#define BT_TIMEOUT		SEC_TO_NSEC(1)
#define BT_TIMEOUT_INQUIRY	SEC_TO_NSEC(5)
#define BT_TIMEOUT_CONNECT	SEC_TO_NSEC(5) /* XXX to check */
#define BT_TIMEOUT_DISCONNECT	SEC_TO_NSEC(5) /* XXX to check */
#define BT_TIMEOUT_REMOTE	SEC_TO_NSEC(5) /* XXX to check */
#define BT_STATE_INIT		0
#define BT_STATE_DYING		1
#define BT_STATE_WAITING	2
#define BT_STATE_DEVOPEN	3
#define BT_STATE_INQUIRY	4
#define BT_STATE_CONNECT	5

#define BT_EVTS_POOLSIZE	32 /* XXX move into bthci.h/c ? */

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

struct bluetooth_io {
	size_t				 size;
	int				 err;
	SIMPLEQ_ENTRY(bluetooth_io)	 fifo;
	uint8_t				 buf; /* first bytes of buf */
};
SIMPLEQ_HEAD(bluetooth_ios, bluetooth_io);

struct bluetooth_device_unit {
	struct bluetooth_device			unit;
	SLIST_ENTRY(bluetooth_device_unit)	sl;
	LIST_ENTRY(bluetooth_device_unit)	handleh;
};
SLIST_HEAD(bluetooth_device_units, bluetooth_device_unit);
LIST_HEAD(bluetooth_device_handles, bluetooth_device_unit);

struct bluetooth_softc {
	struct device			 sc_dev;
	struct bthci			*hci;
	struct rwlock			 lock; /* XXX actually, a mutex feet */
	int				 state;
	int				 count;

	/* HCI dynamic parameters */
	uint16_t			 acl_type;
	uint16_t			 sco_type;

	/* Userland IO */
	struct bluetooth_ios		 rxfifo;

	/* Devices unit database */
	int				 ndevices;
	struct bluetooth_device_units	 devices;
};

void bluetooth_attach(struct bluetooth_softc *, struct bthci *);
void bluetooth_detach(struct bluetooth_softc *);

int bluetoothopen(dev_t, int, int, struct proc *);
int bluetoothclose(dev_t, int, int, struct proc *);
int bluetoothread(dev_t, struct uio *, int);
int bluetoothioctl(dev_t, u_long, caddr_t, int, struct proc *);

