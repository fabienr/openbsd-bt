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

#include <bluetooth/bluetooth.h>
#include <dev/bluetooth/bluetoothreg.h>
#include <dev/bluetooth/bluetoothvar.h>
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

int bluetooth_dev_write(struct bluetooth_softc *, void *, size_t);

void bluetooth_kthread_deferred(void *);
void bluetooth_kthread(void *);
int bluetooth_init(struct bluetooth_softc *);
int bluetooth_inquiry(struct bluetooth_softc *, struct bt_evt *);

struct bluetooth_device *bluetooth_device_loockup(struct bluetooth_softc *,
    struct bluetooth_bdaddr *);
struct bluetooth_device *bluetooth_device_new(struct bluetooth_softc *,
    struct bluetooth_bdaddr *, uint8_t, struct bluetooth_class *, uint16_t);

void
bluetooth_attach(struct bluetooth_softc *sc, struct bthci *hci)
{
	DPRINTF(("%s: bluetooth_attach\n", DEVNAME(sc)));
	sc->hci = hci;
	bthci_register(sc->hci, sc);
	rw_init(&sc->lock, DEVNAME(sc));
	SIMPLEQ_INIT(&sc->fifo);
	kthread_create_deferred(bluetooth_kthread_deferred, (void*)sc);
}

void
bluetooth_detach(struct bluetooth_softc *sc)
{
	struct bluetooth_device_unit *dev = NULL;
	DPRINTF(("%s: bluetooth_detach\n", DEVNAME(sc)));
	wakeup(sc);
	wakeup(&sc->fifo);
	rw_enter_write(&sc->lock);
	sc->state = BT_STATE_DYING;
	while (!SLIST_EMPTY(&sc->devices)) {
		dev = SLIST_FIRST(&sc->devices);
		SLIST_REMOVE_HEAD(&sc->devices, sl);
		free(dev, M_BLUETOOTH, sizeof(*dev));
	}
	/* XXX assert fifo is empty, ensure memory is clean */
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
	DPRINTF(("%s: bluetoothopen\n", DEVNAME(sc)));

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

int
bluetoothclose(dev_t dev, int flags, int fmt, struct proc *p)
{
	struct bluetooth_softc *sc;
	struct bluetooth_dev_io *io;

	sc = bluetooth_cd.cd_devs[BLUETOOTHUNIT(dev)];
	DPRINTF(("%s: bluetoothclose\n", DEVNAME(sc)));

	rw_enter_write(&sc->lock);
	sc->state = BT_STATE_WAITING;
	sc->count = 0;
	while ((io = SIMPLEQ_FIRST(&sc->fifo)) != NULL) {
		free(io, M_BLUETOOTH, sizeof(*io)+io->size);
		SIMPLEQ_REMOVE_HEAD(&sc->fifo, fifo);
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
	io = SIMPLEQ_FIRST(&sc->fifo);
	if (flag & IO_NDELAY && io == NULL) {
		err = EWOULDBLOCK;
		goto fail;
	}
	if (io == NULL) {
		err = rwsleep_nsec(&sc->fifo, &sc->lock, BTPRI|PCATCH,
		    DEVNAME(sc), INFSLP);
		if (err)
			goto fail;
		io = SIMPLEQ_FIRST(&sc->fifo);
	}
	if (io)
		SIMPLEQ_REMOVE_HEAD(&sc->fifo, fifo);
	else {
		printf("%s: bluetoothread wakeup but there is no io pending\n",
		    DEVNAME(sc));
		err = EPROTO;
		goto fail;
	}
	if (io->size == 0) {
		err = 0;
		goto fail;
	}
	if (uio->uio_resid != io->size) {
		DPRINTF(("%s: invalid uio size %zu != %zu\n",
		    DEVNAME(sc), uio->uio_resid, io->size));
		err = EPROTO;
		goto fail;
	}
	err = uiomove(&io->buf, io->size, uio);
 fail:
	rw_exit_write(&sc->lock);
	if (io)
		free(io, M_BLUETOOTH, sizeof(*io)+io->size);
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
	/* XXX bthci_* can sleep, avoid locking ? */
	rw_enter_write(&sc->lock);

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
		sc->count++;
		err = bthci_lc_inquiry(sc->hci, BT_INQUIRY_INTERVAL, 0);
		if (err)
			goto fail;
		if (sc->count) {
			getnanotime(&sc->start);
			sc->timeout = BT_INQUIRY_TIMEOUT;
		} else
			printf("%s: inquiry count reset\n", DEVNAME(sc));
		/* wakeup blutooth_kthread */
		wakeup(sc);
		break;

	default:
		err = EINVAL;
		break;
	}
 fail:
	if (err) {
		sc->state = BT_STATE_DEVOPEN;
		sc->count = 0;
	}
	rw_exit_write(&sc->lock);
	return (err);
}

int
bluetooth_dev_write(struct bluetooth_softc *sc, void *buf, size_t size)
{
	struct bluetooth_dev_io *io;
	DPRINTF(("%s: dev_write %zu bytes\n",
	    DEVNAME(sc), size));
	if (sc->state < BT_STATE_DEVOPEN)
		return (EINTR);
	io = malloc(sizeof(*io)+size, M_BLUETOOTH, M_WAITOK|M_CANFAIL);
	if (io == NULL)
		return (ENOMEM);
	io->size = size;
	if (size)
		memcpy(&io->buf, buf, size);
	SIMPLEQ_INSERT_TAIL(&sc->fifo, io, fifo);
	wakeup(&sc->fifo);
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
	struct bt_evt *evt;
	struct timespec now;
	int err = 0;

	sc = (struct bluetooth_softc *)priv;
	DPRINTF(("%s: bluetooth_kthread\n", DEVNAME(sc)));

	/* XXX bthci_* can sleep, avoid locking ? */
	rw_enter_write(&sc->lock);
	do {
		err = bluetooth_init(sc);
		if (err)
			printf("%s: bluetooth_init, err %d\n", DEVNAME(sc), err);
	} while(err == EAGAIN);
	if (err)
		sc->state = BT_STATE_DYING;
	else
		sc->state = BT_STATE_WAITING;
	while(sc->state != BT_STATE_DYING) {
		err = 0;
		while((evt = bthci_read_evt(sc->hci)) != NULL) {
			DPRINTF(("%s: kthread state=%d, count=%d\n",
			    DEVNAME(sc), sc->state, sc->count));
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
		rwsleep_nsec(sc, &sc->lock, BTPRI, DEVNAME(sc), (sc->timeout)?
		    BT_TIMEOUT:INFSLP);
		if (sc->timeout) {
			getnanotime(&now);
			timespecsub(&now, &sc->start, &now);
			if (now.tv_sec >= sc->timeout) {
				DPRINTF(("%s: watchdog state %d, count %d\n",
				    DEVNAME(sc), sc->state, sc->count));
				sc->count = 0;
			}
		}
		if (sc->count == 0)
			goto state_done;
		continue;
  state_done:
		/* reset watchdog, notify even if already closed */
		sc->timeout = 0;
		if (bluetooth_dev_write(sc, NULL, 0) == ENOMEM)
			printf("%s: bluetooth_dev_write ENOMEM\n",
			    DEVNAME(sc));
	}
	err = (sc->state != BT_STATE_DYING);
	DPRINTF(("%s: kthread exit state=%d, count=%d, err=%d\n",
	    DEVNAME(sc), sc->state, sc->count, err));
	rw_exit_write(&sc->lock);
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
	if ((err = bthci_cb_name(sc->hci, DEVNAME(sc)))) {
		printf("%s: bthci_cb_name fail, err=%d\n",
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
		sc->count--;
		if (sc->count) {
			err = bthci_lc_inquiry(sc->hci, BT_INQUIRY_INTERVAL, 0);
			if (err)
				goto fail;
			sc->count++;
		}
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
			dev = bluetooth_device_loockup(sc, &inquiry.bdaddrs[i]);
			if (dev == NULL)
				dev = bluetooth_device_new(sc,
				    &inquiry.bdaddrs[i],
				    inquiry.scan_mode[i],
				    &inquiry.classes[i],
				    inquiry.clocks[i]);
			if (dev == NULL) {
				printf("%s: inquiry result device_new err\n",
				    DEVNAME(sc));
				goto fail;
			}
			if (timespeccmp(&dev->nanotime, &sc->start, >))
				continue;
			getnanotime(&dev->nanotime);
			if (dev->name[0] != '\0') {
				err = bluetooth_dev_write(sc, dev, sizeof(*dev));
				if (err) {
					printf("%s: inquiry result, write %d\n",
					    DEVNAME(sc), err);
					goto fail;
				}
				continue;
			}
			err = bthci_lc_remote_name(sc->hci, &dev->bt_addr,
			    dev->bt_scan_mode, dev->bt_clock);
			if (err) {
				printf("%s: inquiry result lc_remote_name %d\n",
				    DEVNAME(sc), err);
			} else
				sc->count++;
		}
		break;
	case BT_EVT_REMOTE_NAME_REQ_COMPL:
		sc->count--;
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
		dev = bluetooth_device_loockup(sc, &name->bdaddr);
		if (dev == NULL) {
			printf("%s: inquiry name device_loockup err\n",
			    DEVNAME(sc));
			goto fail;
		}
		memcpy(&dev->name, &evt->data[sizeof(*name)], len);
		err = bluetooth_dev_write(sc, dev, sizeof(*dev));
		if (err) {
			printf("%s: inquiry name, write %d\n",
			    DEVNAME(sc), err);
			goto fail;
		}
		break;
	default:
 fail:
		err = -1;
	}
	if (sc->count == 0) {
		sc->timeout = 0;
		err = bluetooth_dev_write(sc, NULL, 0);
		if (err)
			printf("%s: inquiry name, bluetooth_dev_write(NULL) %d\n",
			    DEVNAME(sc), err);
	}
	return (err);
}

struct bluetooth_device *
bluetooth_device_loockup(struct bluetooth_softc *sc,
    struct bluetooth_bdaddr *bdaddr)
{
	struct bluetooth_device_unit *dev = NULL;
	int i;
#ifdef BT_DEBUG
	printf("%s: loockup ", DEVNAME(sc));
	for (i = BT_ADDR_LEN; --i >= 0;)
		printf("%0X%c", bdaddr->b[i], (i)?':':' ');
	printf("\n");
#endif
	SLIST_FOREACH(dev, &sc->devices, sl) {
		for (i = BT_ADDR_LEN; --i >= 0;)
			if (dev->unit.bt_addr.b[i] != bdaddr->b[i])
				break;
		if (i < 0)
			break;
	}
	return ((struct bluetooth_device *)dev);
}

struct bluetooth_device *
bluetooth_device_new(struct bluetooth_softc *sc, struct bluetooth_bdaddr *bdaddr,
    uint8_t scan_mode, struct bluetooth_class *class, uint16_t clock)
{
	struct bluetooth_device_unit *dev = NULL;
	dev = malloc(sizeof(*dev), M_BLUETOOTH, M_WAITOK|M_CANFAIL|M_ZERO);
	if (dev == NULL)
		return (NULL);
	dev->unit.unit = ++sc->ndevices;
	memcpy(&dev->unit.bt_addr, bdaddr, sizeof(dev->unit.bt_addr));
	dev->unit.bt_scan_mode = scan_mode;
	memcpy(&dev->unit.bt_class, class, sizeof(dev->unit.bt_class));
	dev->unit.bt_clock = clock;
	SLIST_INSERT_HEAD(&sc->devices, dev, sl);
	return ((struct bluetooth_device *)dev);
}

