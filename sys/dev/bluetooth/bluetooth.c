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
#include <sys/malloc.h>
#include <sys/vnode.h>

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
int bluetoothread(dev_t, struct uio *, int);
int bluetoothioctl(dev_t, u_long, caddr_t, int, struct proc *);
int bluetooth_dev_write(struct bluetooth_softc *, size_t, void*);

void bluetooth_kthread_deferred(void *);
void bluetooth_kthread(void *);
int bluetooth_init(struct bluetooth_softc *);
int bluetooth_inquiry(struct bluetooth_softc *, struct bt_evt *);

void
bluetooth_attach(struct bluetooth_softc *sc, struct bthci *hci)
{
	sc->hci = hci;
	rw_init(&sc->lock, DEVNAME(sc));
	SIMPLEQ_INIT(&sc->fifo_tx);
	SIMPLEQ_INIT(&sc->fifo_rx);
	kthread_create_deferred(bluetooth_kthread_deferred, (void*)sc);
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
	struct bluetooth_softc *sc;
	int unit, err = 0;
	unit = BLUETOOTHUNIT(dev);
	if (unit >= bluetooth_cd.cd_ndevs ||    /* make sure it was attached */
	    bluetooth_cd.cd_devs[unit] == NULL)
		return (ENXIO);
	sc = bluetooth_cd.cd_devs[BLUETOOTHUNIT(dev)];
	rw_enter_write(&sc->lock);
	if (sc->state != BT_STATE_WAITING) {
		err = EBUSY;
		goto fail;
	}
	sc->state = BT_STATE_DEVOPEN;
 fail:
	rw_exit_write(&sc->lock);
	return (err);
}

int bluetoothclose(dev_t dev, int flags, int fmt, struct proc *p)
{
	struct bluetooth_softc *sc;
	struct bluetooth_dev_io *io;

	sc = bluetooth_cd.cd_devs[BLUETOOTHUNIT(dev)];
	rw_enter_write(&sc->lock);
	sc->state = BT_STATE_WAITING;
	while ((io = SIMPLEQ_FIRST(&sc->fifo_tx)) != NULL) {
		if (io->buf)
			free(io->buf, M_BLUETOOTH, io->size);
		free(io, M_BLUETOOTH, sizeof(*io));
		SIMPLEQ_REMOVE_HEAD(&sc->fifo_tx, fifo);
	}
	while ((io = SIMPLEQ_FIRST(&sc->fifo_rx)) != NULL) {
		if (io->buf)
			free(io->buf, M_BLUETOOTH, io->size);
		free(io, M_BLUETOOTH, sizeof(*io));
		SIMPLEQ_REMOVE_HEAD(&sc->fifo_rx, fifo);
	}
	rw_exit_write(&sc->lock);
	return (0);
}

int
bluetoothread(dev_t dev, struct uio *uio, int flag)
{
	struct bluetooth_softc *sc;
	struct bluetooth_dev_io *io;
	int err;

	sc = bluetooth_cd.cd_devs[BLUETOOTHUNIT(dev)];
	rw_enter_write(&sc->lock);
	io = SIMPLEQ_FIRST(&sc->fifo_tx);
	if (flag & IO_NDELAY && io == NULL) {
		err = EWOULDBLOCK;
		goto fail;
	}
	if (io == NULL) {
		err = rwsleep_nsec(&sc->fifo_tx, &sc->lock, BTPRI|PCATCH,
		    DEVNAME(sc), BT_TIMEOUT*3); /* XXX synchro with kthread */
		if (err)
			goto fail;
		io = SIMPLEQ_FIRST(&sc->fifo_tx);
	}
	if (io)
		SIMPLEQ_REMOVE_HEAD(&sc->fifo_tx, fifo);
	else {
		printf("%s: bluetoothread wakeup but there is no io pending\n",
		    DEVNAME(sc));
		err = EPROTO;
		goto fail;
	}
	if (uio->uio_resid != io->size) {
		DPRINTF(("%s: invalid uio size %zu != %zu\n",
		    DEVNAME(sc), uio->uio_resid, io->size));
		err = EPROTO;
		goto fail;
	}
	err = uiomove(io->buf, io->size, uio);
 fail:
	rw_exit_write(&sc->lock);
	if (io && io->buf)
		free(io->buf, M_BLUETOOTH, io->size);
	if (io)
		free(io, M_BLUETOOTH, sizeof(*io));
	return (err);
}

int
bluetoothioctl(dev_t dev, u_long cmd, caddr_t addr, int flags, struct proc *p)
{
	struct bluetooth_softc *sc;
	union {
		struct bt_hci_info_version version;
		struct bt_hci_info_bdaddr bdaddr;
		struct bt_hci_info_commands commands;
		struct bt_hci_info_features features;
		struct bt_hci_info_extended extended;
		struct bt_hci_info_buffer buffer;
	} hci;
	struct bluetooth_info *info;
	struct bluetooth_info_extended *info_ext;
	int err = 0, i;

	sc = bluetooth_cd.cd_devs[BLUETOOTHUNIT(dev)];
	switch (cmd) {
	case DIOCBTINFO:
		info = (struct bluetooth_info *)addr;
		memset(addr, 0, sizeof(*info)); /* XXX not required, safer ? */

		err = bthci_info_version(sc->hci, &hci.version);
		if (err)
			goto fail;
		info->bt_manufacturer = hci.version.bt_manufacturer;
		info->hci_version = hci.version.hci_version;
		info->hci_revision = hci.version.hci_revision;
		info->lmp_version = hci.version.lmp_version;
		info->lmp_revision = hci.version.lmp_revision;

		err = bthci_info_bdaddr(sc->hci, &hci.bdaddr);
		if (err)
			goto fail;
		memcpy(&info->bt_addr, &hci.bdaddr.bdaddr,
		    sizeof(info->bt_addr));

		err = bthci_info_buffer(sc->hci, &hci.buffer);
		if (err)
			goto fail;
		info->acl_size = hci.buffer.acl_size;
		info->acl_bufferlen = hci.buffer.acl_bufferlen;
		info->sco_size = hci.buffer.sco_size;
		info->sco_bufferlen = hci.buffer.sco_bufferlen;
		break;

	case DIOCBTINFOEXT:
		info_ext = (struct bluetooth_info_extended *)addr;
		memset(addr, 0, sizeof(*info_ext)); /* XXX not required, safer ? */

		err = bthci_info_features(sc->hci, &hci.features);
		if (err)
			goto fail;
		memcpy(&info_ext->features[0][0], &hci.commands.bitmask,
		    sizeof(info_ext->features));
		info_ext->flow_control_lag = 0;
		info_ext->flow_control_lag |= (info_ext->features[0][2] & (1<<6));
		info_ext->flow_control_lag <<= 1;
		info_ext->flow_control_lag |= (info_ext->features[0][2] & (1<<5));
		info_ext->flow_control_lag <<= 1;
		info_ext->flow_control_lag |= (info_ext->features[0][2] & (1<<4));
		info_ext->flow_control_lag <<= 8; /* in unit of 256 bytes */

		err = bthci_info_commands(sc->hci, &hci.commands);
		if (err)
			goto fail;
		memcpy(&info_ext->commands, &hci.commands.bitmask,
		    sizeof(*info_ext->commands));

		err = bthci_info_extended(sc->hci, 0, &hci.extended);
		if (err)
			goto fail;
		i = hci.extended.max_page;
		if (i > BT_EXTENDED_PAGE_MAX) {
			printf("%s : invalid maximum page number %d > %d\n",
			    DEVNAME(sc), i, BT_EXTENDED_PAGE_MAX);
			i = BT_EXTENDED_PAGE_MAX;
		}
		while(i) {
			err = bthci_info_extended(sc->hci, i, &hci.extended);
			if (err)
				break;
			memcpy(&info_ext->features[i][0], &hci.extended.bitmask,
			    sizeof(*info_ext->features));
			i--;
		}
		break;

	case DIOCBTINQUIRY:
		sc->state = BT_STATE_INQUIRY;
		err = bthci_lc_inquiry(sc->hci, BT_TIMEOUT*3, 0);/* XXX timeout */
		if (err)
			goto fail;
		break;

	default:
		err = EINVAL;
		break;
	}
 fail:
	return (err);
}

int
bluetooth_dev_write(struct bluetooth_softc *sc, size_t size, void *buf)
{
	struct bluetooth_dev_io *io;
	io = malloc(sizeof(*io), M_BLUETOOTH, M_WAITOK|M_CANFAIL);
	if (io == NULL)
		return (ENOMEM);
	io->size = size;
	io->buf = buf;
	SIMPLEQ_INSERT_TAIL(&sc->fifo_tx, io, fifo);
	wakeup(&sc->fifo_tx);
	return (0);
}

void
bluetooth_kthread_deferred(void *priv)
{
	struct bluetooth_softc *sc;
	int err;
	sc = (struct bluetooth_softc *)priv;
	if ((err = kthread_create(bluetooth_kthread, priv, NULL, DEVNAME(sc)))) {
		printf("%s: kthread_create, err=%d",
		    DEVNAME(sc), err);
		sc->state = BT_STATE_DYING;
	}
}

void
bluetooth_kthread(void *priv)
{
	struct bluetooth_softc *sc;
	int err;
	struct bt_evt *evt;

	sc = (struct bluetooth_softc *)priv;
	rw_enter_write(&sc->lock);
	if ((err = bluetooth_init(sc)))
		sc->state = BT_STATE_DYING;
	else
		sc->state = BT_STATE_WAITING;
	/* XXX keep exclusive access
	 * if ((err = rw_enter(&sc->lock, RW_DOWNGRADE))) {
	 * 	printf("%s: rw_enter, err=%d\n",
	 * 	    DEVNAME(sc), err);
	 * 	sc->state = BT_STATE_DYING;
	 * }
	 */
	while(sc->state != BT_STATE_DYING) {
		err = 0;
		while((evt = bthci_read_evt(sc->hci)) != NULL) {
			switch (sc->state) {
			case BT_STATE_INQUIRY:
				err = bluetooth_inquiry(sc, evt);
				break;
			default:
				err = EPROTO;
			}
			if (err) {
				DUMP_BT_EVT(DEVNAME(sc), evt);
				err = 0;
			}
			bthci_pool_put(sc->hci, evt);
		}
		/* XXX implement bthci_handle() and change for infinite sleep */
		rwsleep_nsec(sc, &sc->lock, BTPRI, DEVNAME(sc), 100);
	}
	err = (sc->state != BT_STATE_DYING);
	rw_exit_read(&sc->lock);
	kthread_exit(err);
}

int
bluetooth_init(struct bluetooth_softc *sc)
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

int
bluetooth_inquiry(struct bluetooth_softc *sc, struct bt_evt *evt)
{
	struct _inquiry_result {
		int			 ndevices;
		struct bluetooth_bdaddr	*bdaddrs;
		uint8_t			*scan_mode;
		struct bluetooth_class	*classes;
		uint16_t		*clocks;
	} inquiry;
	struct _name_result {
		uint8_t			state;
		struct bluetooth_bdaddr	bdaddr;
	} * name;
	struct bluetooth_device *dev;
	int err = 0, len, i;

	switch(evt->head.op) {
	case BT_EVT_INQUIRY_COMPL:
		/* XXX debug */
		printf("%s: inquiry complete\n",
		    DEVNAME(sc));
		break;
	case BT_EVT_INQUIRY_RESULT:
		if (evt->head.len < 1) {
			printf("%s: inquiry result minimum len %d < 1\n",
			    DEVNAME(sc), evt->head.len);
			goto fail;
		}
		inquiry.ndevices = evt->data[0];
		if (inquiry.ndevices < 1) {
			printf("%s: inquiry result minimum devices %d < 1\n",
			    DEVNAME(sc), inquiry.ndevices);
			goto fail;
		}
		len = inquiry.ndevices * (sizeof(*inquiry.bdaddrs)
		    + sizeof(*inquiry.scan_mode) + sizeof(*inquiry.classes)
		    + sizeof(*inquiry.clocks) + 2); /* +2 reserved field */
		if (evt->head.len - 1 != len) {
			printf("%s: inquiry result len mismatch %d != %d\n",
			    DEVNAME(sc), evt->head.len - 1, len);
			goto fail;
		}
		i = 1;
		inquiry.bdaddrs = (struct bluetooth_bdaddr *)&evt->data[i];
		i += inquiry.ndevices * sizeof(*inquiry.bdaddrs);
		inquiry.scan_mode = &evt->data[i];
		i += inquiry.ndevices * sizeof(*inquiry.scan_mode);
		i += inquiry.ndevices * 2; /* +2 reserved field */
		inquiry.classes = (struct bluetooth_class *)&evt->data[i];
		i += inquiry.ndevices * sizeof(*inquiry.classes);
		inquiry.clocks = (uint16_t *)&evt->data[i];
		for (i = 0; i < inquiry.ndevices; i++) {
			if (bthci_lc_remote_name(sc->hci, &inquiry.bdaddrs[i],
			    inquiry.scan_mode[i], inquiry.clocks[i])) {
				printf("%s: inquiry bthci_lc_remote_name fail\n",
				    DEVNAME(sc));
			}
		}
		break;
	case BT_EVT_REMOTE_NAME_REQ_COMPL:
		if (evt->head.len <= sizeof(*name)) {
			printf("%s: inquiry name min len %d <= %zu\n",
			    DEVNAME(sc), evt->head.len, sizeof(*name));
			goto fail;
		}
		len = evt->head.len - sizeof(*name);
		if (len > sizeof(dev->name)) {
			printf("%s: inquiry name max len %d > %zu\n",
			    DEVNAME(sc), len, sizeof(dev->name));
			goto fail;
		}
		name = (void *)evt->data;
		if (name->state != 0) {
			printf("%s: inquiry name max len %d > %zu\n",
			    DEVNAME(sc), len, sizeof(dev->name));
			goto fail;
		}
		for (i = 0; i < len && evt->data[sizeof(*name)+i] != '\0'; i++);
		if (i == len) {
			printf("%s: inquiry name, add null character\n",
			    DEVNAME(sc));
			evt->data[sizeof(*name) + i - 1] = '\0';
		}
		dev = malloc(sizeof(*dev), M_BLUETOOTH, M_WAITOK|M_CANFAIL);
		if (dev == NULL) {
			printf("%s: inquiry name malloc ENOMEM\n",
			    DEVNAME(sc));
			goto fail;
		}
		memcpy(&dev->bt_addr, &name->bdaddr, sizeof(dev->bt_addr));
		memcpy(&dev->name, &evt->data[sizeof(*name)], len);
		if (bluetooth_dev_write(sc, sizeof(*dev), dev)) {
			printf("%s: inquiry name, bluetooth_dev_write ENOMEM\n",
			    DEVNAME(sc));
			goto fail;
		}
	default:
 fail:
		err = -1;
	}
	return (err);
}
