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
#include <sys/systm.h>
#include <sys/time.h>
#include <sys/device.h>
#include <sys/pool.h>
#include <sys/mutex.h>
#include <sys/param.h>
#include <sys/kthread.h>
#include <sys/rwlock.h>

#include <dev/bluetooth/bluetoothreg.h>
#include <dev/bluetooth/bluetoothvar.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/bthci.h>


#ifdef BLUETOOTH_DEBUG
#define BT_DEBUG
#endif

#ifdef BT_DEBUG
#define DPRINTF(x)	do { printf x; } while (0)
#else
#define DPRINTF(x)
#endif

#define DEVNAME(sc)		((sc)->sc_dev.dv_xname)
#define BLUETOOTHUNIT(n)	(minor(n))

struct cfdriver bluetooth_cd = {
    NULL, "bluetooth", DV_DULL
};

void bluetooth_attach(struct bluetooth_softc *, struct bthci *);
void bluetooth_detach(struct bluetooth_softc *);

int bluetoothopen(dev_t, int, int, struct proc *);
int bluetoothclose(dev_t, int, int, struct proc *);
int bluetoothioctl(dev_t, u_long, caddr_t, int, struct proc *);

void bt_kthread_deferred(void *);
void bt_kthread(void *);
int bt_init(struct bluetooth_softc *);

void
bluetooth_attach(struct bluetooth_softc *sc, struct bthci *hci)
{
	sc->hci = hci;
	rw_init(&sc->lock, DEVNAME(sc));
	kthread_create_deferred(bt_kthread_deferred, (void*)sc);
}

void
bluetooth_detach(struct bluetooth_softc *sc)
{
	rw_enter_write(&sc->lock);
	sc->state = BT_STATE_DYING;
	rw_exit_write(&sc->lock);
}

int
bluetoothopen(dev_t dev, int flags, int fmt, struct proc *p)
{
	int unit;
	unit = BLUETOOTHUNIT(dev);
	if (unit >= bluetooth_cd.cd_ndevs ||    /* make sure it was attached */
	    bluetooth_cd.cd_devs[unit] == NULL)
		return (ENXIO);
	return (0);
}

int bluetoothclose(dev_t dev, int flags, int fmt, struct proc *p)
{
	return (0);
}

int
bluetoothioctl(dev_t dev, u_long cmd, caddr_t addr, int flags, struct proc *p)
{
	struct bluetooth_softc *sc;
	union {
		struct bt_hci_info_version version;
		struct bt_hci_info_bdaddr bdaddr;
	} hci_info;
	struct bluetooth_info *info;
	int err = 0;

	sc = bluetooth_cd.cd_devs[BLUETOOTHUNIT(dev)];
	switch (cmd) {
	case DIOCBTINFO:
		info = (struct bluetooth_info *)addr;
		memset(addr, 0, sizeof(*info)); /* XXX not required, safer ? */

		err = bthci_info_version(sc->hci, &hci_info.version);
		if (err)
			goto fail;
		info->bt_manufacturer = hci_info.version.bt_manufacturer;
		info->hci_version = hci_info.version.hci_version;
		info->hci_revision = hci_info.version.hci_revision;
		info->lmp_version = hci_info.version.lmp_version;
		info->lmp_revision = hci_info.version.lmp_revision;

		err = bthci_info_bdaddr(sc->hci, &hci_info.bdaddr);
		if (err)
			goto fail;
		memcpy(&info->bt_addr, &hci_info.bdaddr.bdaddr,
		    sizeof(info->bt_addr));
		break;
	default:
		err = EINVAL;
		break;
	}
 fail:
	return (err);
}

void
bt_kthread_deferred(void *priv)
{
	struct bluetooth_softc *sc;
	int err;
	sc = (struct bluetooth_softc *)priv;
	if ((err = kthread_create(bt_kthread, priv, NULL, DEVNAME(sc)))) {
		printf("%s: kthread_create, err=%d",
		    DEVNAME(sc), err);
		sc->state = BT_STATE_DYING;
	}
}

void
bt_kthread(void *priv)
{
	struct bluetooth_softc *sc;
	int err;
	struct bt_evt *evt;

	sc = (struct bluetooth_softc *)priv;
	rw_enter_write(&sc->lock);
	if ((err = bt_init(sc)))
		sc->state = BT_STATE_DYING;
	else
		sc->state = BT_STATE_WAITING;
	if ((err = rw_enter(&sc->lock, RW_DOWNGRADE))) {
		printf("%s: rw_enter, err=%d\n",
		    DEVNAME(sc), err);
		sc->state = BT_STATE_DYING;
	}
	while(sc->state != BT_STATE_DYING) {
		while((evt = bthci_read_evt(sc->hci)) != NULL) {
			DUMP_BT_EVT(DEVNAME(sc), evt);
		}
		rwsleep_nsec(sc, &sc->lock, MAXPRI, DEVNAME(sc), 100);
	}
	err = (sc->state != BT_STATE_DYING);
	rw_exit_read(&sc->lock);
	kthread_exit(err);
}

int
bt_init(struct bluetooth_softc *sc)
{
	int err;
	if ((err = bthci_cb_reset(sc->hci))) {
		printf("%s: bthci_attach reset fail, err=%d\n",
		    DEVNAME(sc), err);
		goto fail;
	}
 fail:
	return (err);
}
