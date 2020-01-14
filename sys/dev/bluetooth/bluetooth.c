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

void bluetooth_kthread_deferred(void *);
void bluetooth_kthread(void *);
int bluetooth_init(struct bluetooth_softc *);
int bluetooth_inquiry(struct bluetooth_softc *, struct bt_evt *);
int bluetooth_connect(struct bluetooth_softc *, struct bt_evt *);

struct bluetooth_device *bluetooth_device_loockup(struct bluetooth_softc *,
    struct bluetooth_bdaddr *);
struct bluetooth_device *bluetooth_device_new(struct bluetooth_softc *,
    struct bluetooth_bdaddr *, uint8_t, struct bluetooth_class *, uint16_t);
struct bluetooth_device *bluetooth_device_unit(struct bluetooth_softc *,
    uint8_t);

int bluetooth_rxfifo_write(struct bluetooth_softc *, void *, size_t);
int bluetooth_rxfifo_read(struct bluetooth_softc *, struct bluetooth_dev_io **);
void bluetooth_io_free(struct bluetooth_dev_io *);

void
bluetooth_attach(struct bluetooth_softc *sc, struct bthci *hci)
{
	DPRINTF(("%s: bluetooth_attach\n", DEVNAME(sc)));
	sc->hci = hci;
	bthci_register(sc->hci, sc);
	rw_init(&sc->lock, DEVNAME(sc));
	SIMPLEQ_INIT(&sc->rxfifo);
	kthread_create_deferred(bluetooth_kthread_deferred, (void*)sc);
}

void
bluetooth_detach(struct bluetooth_softc *sc)
{
	struct bluetooth_device_unit *dev = NULL;
	DPRINTF(("%s: bluetooth_detach\n", DEVNAME(sc)));
	rw_enter_write(&sc->lock);
	wakeup(sc);
	wakeup(&sc->rxfifo);
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
	while (!SIMPLEQ_EMPTY(&sc->rxfifo)) {
		io = SIMPLEQ_FIRST(&sc->rxfifo);
		SIMPLEQ_REMOVE_HEAD(&sc->rxfifo, fifo);
		bluetooth_io_free(io);
	}
	rw_exit_write(&sc->lock);
	return (0);
}

int
bluetoothread(dev_t dev, struct uio *uio, int flag)
{
	struct bluetooth_softc *sc;
	struct bluetooth_dev_io *io = NULL;
	int err;

	sc = bluetooth_cd.cd_devs[BLUETOOTHUNIT(dev)];
	rw_enter_write(&sc->lock);

	err = bluetooth_rxfifo_read(sc, &io);
	if (err)
		goto fail;
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
		bluetooth_io_free(io);
	return (err);
}

int
bluetoothioctl(dev_t dev, u_long cmd, caddr_t addr, int flags, struct proc *p)
{
	struct bluetooth_softc *sc;
	union {
		struct bt_hci_info_bdaddr bdaddr;
		struct bt_hci_info_version version;
		struct bt_hci_info_buffer buffer;
		struct bt_hci_info_features features;
		struct bt_hci_info_extended extended;
		struct bt_hci_info_commands commands;
		uint16_t handle;
	} hci;
	union {
		struct bluetooth_info *info;
		struct bluetooth_info_extended *info_ext;
		struct bluetooth_device_match *match;
	} ctl;
	struct bluetooth_device *device;
	struct bluetooth_dev_io *io = NULL;
	int err = 0, i;

	sc = bluetooth_cd.cd_devs[BLUETOOTHUNIT(dev)];
	/* XXX bthci_* can sleep, avoid locking ? */
	rw_enter_write(&sc->lock);

	switch (cmd) {
	case DIOCBTINFO:
		ctl.info = (struct bluetooth_info *)addr;
		memset(addr, 0, sizeof(*ctl.info)); /* XXX not required, safer ? */

		err = bthci_info_bdaddr(sc->hci, &hci.bdaddr);
		if (err)
			goto fail;
		memcpy(&ctl.info->bt_addr, &hci.bdaddr.bdaddr,
		    sizeof(ctl.info->bt_addr));

		err = bthci_info_version(sc->hci, &hci.version);
		if (err)
			goto fail;
		ctl.info->bt_manufacturer = hci.version.bt_manufacturer;
		ctl.info->hci_version = hci.version.hci_version;
		ctl.info->hci_revision = hci.version.hci_revision;
		ctl.info->lmp_version = hci.version.lmp_version;
		ctl.info->lmp_revision = hci.version.lmp_revision;

		err = bthci_info_buffer(sc->hci, &hci.buffer);
		if (err)
			goto fail;
		ctl.info->acl_size = hci.buffer.acl_size;
		ctl.info->acl_bufferlen = hci.buffer.acl_bufferlen;
		ctl.info->sco_size = hci.buffer.sco_size;
		ctl.info->sco_bufferlen = hci.buffer.sco_bufferlen;
		break;

	case DIOCBTINFOEXT:
		ctl.info_ext = (struct bluetooth_info_extended *)addr;
		memset(addr, 0, sizeof(*ctl.info_ext)); /* XXX not required, safer ? */

		err = bthci_info_features(sc->hci, &hci.features);
		if (err)
			goto fail;
		memcpy(&ctl.info_ext->features[0][0], &hci.features.bitmask,
		    sizeof(ctl.info_ext->features[0]));
		ctl.info_ext->flow_control_lag = 0;
		ctl.info_ext->flow_control_lag |= (
		    ctl.info_ext->features[0][2] & (1<<6));
		ctl.info_ext->flow_control_lag <<= 1;
		ctl.info_ext->flow_control_lag |= (
		    ctl.info_ext->features[0][2] & (1<<5));
		ctl.info_ext->flow_control_lag <<= 1;
		ctl.info_ext->flow_control_lag |= (
		    ctl.info_ext->features[0][2] & (1<<4));
		ctl.info_ext->flow_control_lag <<= 8; /* in unit of 256 bytes */

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
			memcpy(&ctl.info_ext->features[i][0],
			    &hci.extended.bitmask,
			    sizeof(ctl.info_ext->features[i]));
			i--;
		}

		err = bthci_info_commands(sc->hci, &hci.commands);
		if (err)
			goto fail;
		memcpy(&ctl.info_ext->commands, &hci.commands.bitmask,
		    sizeof(ctl.info_ext->commands));
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

	case DIOCBTMATCH:
		ctl.match = (struct bluetooth_device_match *)addr;
		if (ctl.match->unit > 0)
			device = bluetooth_device_unit(sc, ctl.match->unit);
		else
			device = bluetooth_device_loockup(sc, &ctl.match->bt_addr);
		if (device == NULL) {
			err = EINVAL;
			goto fail;
		}

		sc->state = BT_STATE_CONNECT;
		sc->count++;
		err = bthci_lc_connect(sc->hci, sc->acl_type, device);
		if (err)
			goto fail;
		if (sc->count) {
			getnanotime(&sc->start);
			sc->timeout = BT_CONNECT_TIMEOUT;
		} else
			printf("%s: match, connect count reset\n", DEVNAME(sc));
		err = bluetooth_rxfifo_read(sc, &io);
		if (err)
			goto fail;
		if (io->size != sizeof(hci.handle)) {
			printf("%s: match, connect handle size mismatch,"
			    " %zu != %zu\n",
			    DEVNAME(sc), io->size, sizeof(hci.handle));
			err = EPROTO;
			goto fail;
		}
		hci.handle = *((uint16_t*)&io->buf);
		bluetooth_io_free(io);
		io = NULL;

		sc->state = BT_STATE_CONNECT;
		sc->count++;
		err = bthci_lc_disconnect(sc->hci, hci.handle);
		if (sc->count) {
			getnanotime(&sc->start);
			sc->timeout = BT_CONNECT_TIMEOUT;
		} else
			printf("%s: match, disconect count reset\n", DEVNAME(sc));
		err = bluetooth_rxfifo_read(sc, &io);
		if (err)
			goto fail;
		if (io->size != 0) {
			printf("%s: match, disconnect not null\n", DEVNAME(sc));
			err = EPROTO;
			goto fail;
		}
		break;

	default:
		err = EINVAL;
		break;
	}
 fail:
	if (io)
		bluetooth_io_free(io);
	if (err) {
		sc->state = BT_STATE_DEVOPEN;
		sc->count = 0;
	}
	rw_exit_write(&sc->lock);
	return (err);
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
	DPRINTF(("%s: kthread\n", DEVNAME(sc)));

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
	DPRINTF(("%s: kthread init done, state=%d, count=%d\n",
	    DEVNAME(sc), sc->state, sc->count));

	while(sc->state != BT_STATE_DYING) {
		err = 0;
		while((evt = bthci_read_evt(sc->hci)) != NULL) {
			DPRINTF(("%s: kthread event, state=%d, count=%d\n",
			    DEVNAME(sc), sc->state, sc->count));
			switch (sc->state) {
			case BT_STATE_INQUIRY:
				err = bluetooth_inquiry(sc, evt);
				break;
			case BT_STATE_CONNECT:
				err = bluetooth_connect(sc, evt);
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
		if (bluetooth_rxfifo_write(sc, NULL, 0) == ENOMEM)
			printf("%s: kthread rxfifo_write ENOMEM\n",
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
	struct bt_hci_info_features features;
	int err;
	if ((err = bthci_cb_reset(sc->hci))) {
		printf("%s: init, bthci_attach %d\n",
		    DEVNAME(sc), err);
		goto fail;
	}
	if ((err = bthci_cb_name(sc->hci, DEVNAME(sc)))) {
		printf("%s: init, bthci_cb_name %d\n",
		    DEVNAME(sc), err);
		goto fail;
	}
	if ((err = bthci_info_features(sc->hci, &features))) {
		printf("%s: init, bthci_info_features %d\n",
		    DEVNAME(sc), err);
		goto fail;
	}
	/* ACL packet type */
	sc->acl_type = BT_ACL_DM1 | BT_ACL_DH1;
	if ((features.bitmask[0] & BT_HCI_FEATURE_0_3SLOT) != 0)
		sc->acl_type |= BT_ACL_DM3 | BT_ACL_DH3;
	if ((features.bitmask[0] & BT_HCI_FEATURE_0_5SLOT) != 0)
		sc->acl_type |= BT_ACL_DM5 | BT_ACL_DH5;
	if ((features.bitmask[3] & BT_HCI_FEATURE_3_EDRACL2MBPS) == 0)
		sc->acl_type |= BT_ACL_NO_2MBPS_DH1
		    | BT_ACL_NO_2MBPS_DH3
		    | BT_ACL_NO_2MBPS_DH5;
	if ((features.bitmask[3] & BT_HCI_FEATURE_3_EDRACL3MBPS) == 0)
		sc->acl_type |= BT_ACL_NO_3MBPS_DH1
		    | BT_ACL_NO_3MBPS_DH3
		    | BT_ACL_NO_3MBPS_DH5;
	if ((features.bitmask[4] & BT_HCI_FEATURE_4_3SLOTSEDRACL) == 0)
		sc->acl_type |= BT_ACL_NO_2MBPS_DH3 | BT_ACL_NO_3MBPS_DH3;
	if ((features.bitmask[5] & BT_HCI_FEATURE_5_5SLOTSEDRACL) == 0)
		sc->acl_type |= BT_ACL_NO_2MBPS_DH5 | BT_ACL_NO_3MBPS_DH5;
	/* SCO packet mask */
	sc->sco_type = 0;
	if ((features.bitmask[1] & BT_HCI_FEATURE_1_SC0) != 0 )
		sc->sco_type |= BT_SCO_HV1;
	if ((features.bitmask[1] & BT_HCI_FEATURE_1_HV2) != 0 )
		sc->sco_type |= BT_SCO_HV2;
	if ((features.bitmask[1] & BT_HCI_FEATURE_1_HV3) != 0 )
		sc->sco_type |= BT_SCO_HV3;
	if ((features.bitmask[3] & BT_HCI_FEATURE_3_EV3) != 0 )
		sc->sco_type |= BT_SCO_EV3;
	if ((features.bitmask[4] & BT_HCI_FEATURE_4_EV4) != 0 )
		sc->sco_type |= BT_SCO_EV4;
	if ((features.bitmask[4] & BT_HCI_FEATURE_4_EV5) != 0 )
		sc->sco_type |= BT_SCO_EV5;
	if ((features.bitmask[5] & BT_HCI_FEATURE_5_EDRSCO2MBPS) == 0)
		sc->sco_type |= BT_SCO_NO_2MBPS_EV3 | BT_SCO_NO_2MBPS_EV5;
	if ((features.bitmask[5] & BT_HCI_FEATURE_5_EDRSCO3MBPS) == 0)
		sc->sco_type |= BT_SCO_NO_3MBPS_EV3 | BT_SCO_NO_3MBPS_EV5;
	if ((features.bitmask[5] & BT_HCI_FEATURE_5_3SLOTSEDRSCO) == 0) {
		sc->sco_type &= ~BT_SCO_EV4;
		sc->sco_type |= BT_SCO_NO_2MBPS_EV5 | BT_SCO_NO_3MBPS_EV5;
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
	} inquiries;
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
			printf("%s: inquiry result, minimum len %d < 1\n",
			    DEVNAME(sc), evt->head.len);
			goto fail;
		}
		inquiries.ndevices = evt->data[0];
		if (inquiries.ndevices < 1) {
			printf("%s: inquiry result, minimum devices %d < 1\n",
			    DEVNAME(sc), inquiries.ndevices);
			goto fail;
		}
		/* XXX ndevices > 1 untested */
		if (inquiries.ndevices < 1)
			printf("%s: pls report to tech@, inquiry ndevices %d\n",
			    DEVNAME(sc), inquiries.ndevices);
		len = inquiries.ndevices * (sizeof(*inquiries.bdaddrs)
		    + sizeof(*inquiries.scan_mode) + sizeof(*inquiries.classes)
		    + sizeof(*inquiries.clocks) + 2); /* +2 reserved field */
		if (evt->head.len - 1 != len) {
			printf("%s: inquiry result, len mismatch %d != %d\n",
			    DEVNAME(sc), evt->head.len - 1, len);
			goto fail;
		}
		i = 1;
		inquiries.bdaddrs = (struct bluetooth_bdaddr *)&evt->data[i];
		i += inquiries.ndevices * sizeof(*inquiries.bdaddrs);
		inquiries.scan_mode = &evt->data[i];
		i += inquiries.ndevices * sizeof(*inquiries.scan_mode);
		i += inquiries.ndevices * 2; /* +2 reserved field */
		inquiries.classes = (struct bluetooth_class *)&evt->data[i];
		i += inquiries.ndevices * sizeof(*inquiries.classes);
		inquiries.clocks = (uint16_t *)&evt->data[i];
		for (i = 0; i < inquiries.ndevices; i++) {
			dev = bluetooth_device_loockup(sc, &inquiries.bdaddrs[i]);
			if (dev == NULL)
				dev = bluetooth_device_new(sc,
				    &inquiries.bdaddrs[i],
				    inquiries.scan_mode[i],
				    &inquiries.classes[i],
				    inquiries.clocks[i]);
			if (dev == NULL) {
				printf("%s: inquiry result, device_new err\n",
				    DEVNAME(sc));
				goto fail;
			}
			if (timespeccmp(&dev->nanotime, &sc->start, >))
				continue;
			getnanotime(&dev->nanotime);
			if (dev->name[0] != '\0') {
				err = bluetooth_rxfifo_write(sc,dev,sizeof(*dev));
				if (err) {
					printf("%s: inquiry result,"
					    " rxfifo_write %d\n",
					    DEVNAME(sc), err);
					goto fail;
				}
				continue;
			}
			err = bthci_lc_remote_name(sc->hci, &dev->bt_addr,
			    dev->bt_scan_mode, dev->bt_clock);
			if (err) {
				printf("%s: inquiry result, lc_remote_name %d\n",
				    DEVNAME(sc), err);
			} else
				sc->count++;
		}
		break;
	case BT_EVT_REMOTE_NAME_REQ_COMPL:
		sc->count--;
		if (evt->head.len <= sizeof(*name)) {
			printf("%s: inquiry name, min len %d <= %zu\n",
			    DEVNAME(sc), evt->head.len, sizeof(*name));
			goto fail;
		}
		len = evt->head.len - sizeof(*name);
		if (len > sizeof(dev->name)) {
			printf("%s: inquiry name, max len %d > %zu\n",
			    DEVNAME(sc), len, sizeof(dev->name));
			goto fail;
		}
		name = (void *)evt->data;
		if (name->state != 0) {
			printf("%s: inquiry name, max len %d > %zu\n",
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
			printf("%s: inquiry name, device_loockup err\n",
			    DEVNAME(sc));
			goto fail;
		}
		memcpy(&dev->name, &evt->data[sizeof(*name)], len);
		err = bluetooth_rxfifo_write(sc, dev, sizeof(*dev));
		if (err) {
			printf("%s: inquiry name, rxfifo_write %d\n",
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
		err = bluetooth_rxfifo_write(sc, NULL, 0);
		if (err)
			printf("%s: inquiry name, rxfifo_write(NULL) %d\n",
			    DEVNAME(sc), err);
	}
	return (err);
}

int
bluetooth_connect(struct bluetooth_softc *sc, struct bt_evt *evt)
{
	struct _conect_result { /* XXX sanity checks to implement */
		uint8_t			state;
		uint16_t		handle;
		struct bluetooth_bdaddr	bdaddr;
		uint8_t			link_type;
		uint8_t			encryption;
	} __packed *connect;
	struct _disconect_result { /* XXX sanity checks to implement */
		uint8_t			state;
		uint16_t		handle;
		uint8_t			reason;
	} __packed *disconnect;
	int err = 0;

	switch(evt->head.op) {
	case BT_EVT_CON_COMPL:
		sc->count--;
		if (evt->head.len < 1) {
			printf("%s: connect result, minimum len, %d < 1\n",
			    DEVNAME(sc), evt->head.len);
			goto fail;
		}
		connect = (struct _conect_result *) evt->data;
		if (connect->state) {
			err = BT_ERR_TOH(connect->state);
			goto fail;
		}
		if (evt->head.len != sizeof(*connect)) {
			printf("%s: connect result, len mismatch, %u != %zu\n",
			    DEVNAME(sc), evt->head.len, sizeof(*connect));
			goto fail;
		}
		err = bluetooth_rxfifo_write(sc, &connect->handle,
		    sizeof(connect->handle));
		if (err) {
			printf("%s: connect result, rxfifo_write %d\n",
			    DEVNAME(sc), err);
			goto fail;
		}
		break;
	case BT_EVT_DISCON_COMPL:
		sc->count--;
		if (evt->head.len < 1) {
			printf("%s: disconnect result, minimum len, %d < 1\n",
			    DEVNAME(sc), evt->head.len);
			goto fail;
		}
		disconnect = (struct _disconect_result *) evt->data;
		if (disconnect->state) {
			err = BT_ERR_TOH(disconnect->state);
			goto fail;
		}
		if (evt->head.len != sizeof(*disconnect)) {
			printf("%s: disconnect result, len mismatch, %u != %zu\n",
			    DEVNAME(sc), evt->head.len, sizeof(*disconnect));
			goto fail;
		}
		err = bluetooth_rxfifo_write(sc, NULL, 0);
		if (err) {
			printf("%s: inquiry name, rxfifo_write %d\n",
			    DEVNAME(sc), err);
			goto fail;
		}
		break;
	default:
 fail:
		err = -1;
	}
	if (sc->count == 0) {
		/* XXX write NULL ? */
		sc->timeout = 0;
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
#ifdef BT_DEBUG
	int i;
	printf("%s: new ", DEVNAME(sc));
	for (i = BT_ADDR_LEN; --i >= 0;)
		printf("%0X%c", bdaddr->b[i], (i)?':':' ');
	printf("\n");
#endif
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

struct bluetooth_device *
bluetooth_device_unit(struct bluetooth_softc *sc, uint8_t unit)
{
	struct bluetooth_device_unit *dev = NULL;
	DPRINTF(("%s: device unit %d\n", DEVNAME(sc), unit));
	SLIST_FOREACH(dev, &sc->devices, sl) {
		if (dev->unit.unit == unit)
			break;
	}
	return ((struct bluetooth_device *)dev);
}

int
bluetooth_rxfifo_write(struct bluetooth_softc *sc, void *buf, size_t size)
{
	struct bluetooth_dev_io *io;
	DPRINTF(("%s: dev_write %zu bytes\n",
	    DEVNAME(sc), size));
	if (sc->state < BT_STATE_DEVOPEN)
		return (EINTR);
	io = malloc(sizeof(*io) + size, M_BLUETOOTH, M_WAITOK|M_CANFAIL);
	if (io == NULL)
		return (ENOMEM);
	io->size = size;
	if (size)
		memcpy(&io->buf, buf, size);
	*((uint8_t*)(&io->buf + size)) = '\0'; /* XXX canary, null guard ? */
	SIMPLEQ_INSERT_TAIL(&sc->rxfifo, io, fifo);
	wakeup(&sc->rxfifo);
	return (0);
}

int
bluetooth_rxfifo_read(struct bluetooth_softc *sc, struct bluetooth_dev_io **io)
{
	int err = 0;
	*io = SIMPLEQ_FIRST(&sc->rxfifo);
	if (*io == NULL) {
		err = rwsleep_nsec(&sc->rxfifo, &sc->lock, BTPRI|PCATCH,
		    DEVNAME(sc), INFSLP);
		if (err)
			return (err);
		*io = SIMPLEQ_FIRST(&sc->rxfifo);
	}
	if (*io)
		SIMPLEQ_REMOVE_HEAD(&sc->rxfifo, fifo);
	else {
		printf("%s: rxfifo_read wakeup but there is no io pending\n",
		    DEVNAME(sc));
		return (EPROTO);
	}
	return (0);
}

void
bluetooth_io_free(struct bluetooth_dev_io *io)
{
	/* XXX canary, null guard ? */
	if (*((uint8_t*)(&io->buf + io->size)) != '\0')
		printf("bluetooth: dev_io overflow, not null terminated\n");
	free(io, M_BLUETOOTH, sizeof(*io) + io->size);
}
