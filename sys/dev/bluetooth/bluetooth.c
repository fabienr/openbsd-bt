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

/* XXX */
#define BT_DEBUG

#include <sys/types.h>
#include <sys/systm.h>
#include <sys/time.h>
#include <sys/device.h>
#include <sys/pool.h>
#include <sys/mutex.h>
#include <sys/param.h>
#include <sys/kthread.h>
#include <sys/rwlock.h>

#include <dev/bluetooth/bluetooth.h>
#include <bluetooth/bthci.h>

#ifdef BLUETOOTH_DEBUG
#define BT_DEBUG
#endif

#ifdef BT_DEBUG
#define DPRINTF(x)	do { printf x; } while (0)
#else
#define DPRINTF(x)
#endif
#define DEVNAME(sc) ((sc)->sc_dev.dv_xname)

struct cfdriver bluetooth_cd = {
    NULL, "bluetooth", DV_DULL
};

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
		sc->state = BT_STATE_WAITING;
	}
	while(sc->state != BT_STATE_DYING) {
		while((evt = bthci_read_evt(sc->hci)) != NULL) {
			DUMP_BT_EVT(DEVNAME(sc), evt);
		}
		rw_exit(&sc->lock);
		tsleep_nsec(sc, MAXPRI, DEVNAME(sc), 100);
		rw_enter_read(&sc->lock);
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
	/*
	if (bthci_info_version(hci)) {
		printf("%s: bthci_attach get version fail\n",
		    DEVNAME(hci));
		return (EIO);
	}
	if (bthci_info_commands(hci)) {
		printf("%s: bthci_attach get commands supported fail\n",
		    DEVNAME(hci));
		return (EIO);
	}

	if (bthci_info_features(hci)) {
		printf("%s: bthci_attach get features supported fail\n",
		    DEVNAME(hci));
		return (EIO);
	}

	if (bthci_info_extended_features(hci)) {
		printf("%s: bthci_attach get extended features supported fail\n",
		    DEVNAME(hci));
		return (EIO);
	}

	if (bthci_info_buffer(hci)) {
		printf("%s: bthci_attach get buffer info fail\n",
		    DEVNAME(hci));
		return (EIO);
	}

	if (bthci_info_bdaddr(hci)) {
		printf("%s: bthci_attach get bdaddr fail\n",
		    DEVNAME(hci));
		return (EIO);
	}

	if (bthci_lm_inquiry(hci, 30, 100)) {
		printf("%s: bthci_attach inquiry fail\n",
		    DEVNAME(hci));
		return (EIO);
	}
	*/
 fail:
	return (err);
}
