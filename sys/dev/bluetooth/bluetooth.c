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

/* internal kthread context */
void bluetooth_kthread_deferred(void *);
void bluetooth_kthread(void *);
int bluetooth_init(struct bluetooth_softc *);
int bluetooth_inquiry(struct bluetooth_softc *, struct bt_evt *);
int bluetooth_connect(struct bluetooth_softc *, struct bt_evt *);

/* private locking */
int bluetooth_dying(struct bluetooth_softc *);
int bluetooth_enter(struct bluetooth_softc *);
void bluetooth_out(struct bluetooth_softc *);
int bluetooth_in(struct bluetooth_softc *);
int bluetooth_unsleep(struct bluetooth_softc *);
void bluetooth_leave(struct bluetooth_softc *);

/* private functions */
struct bluetooth_device *bluetooth_device_loockup(struct bluetooth_softc *,
    struct bluetooth_bdaddr *);
struct bluetooth_device *bluetooth_device_new(struct bluetooth_softc *,
    struct bluetooth_bdaddr *, uint8_t, struct bluetooth_class *, uint16_t);
struct bluetooth_device *bluetooth_device_unit(struct bluetooth_softc *,
    uint8_t);

int bluetooth_rxfifo_write(struct bluetooth_softc *, void *, size_t, int);
int bluetooth_rxfifo_read(struct bluetooth_softc *, struct bluetooth_io **);
int bluetooth_rxfifo_readerr(struct bluetooth_softc *);
void bluetooth_io_free(struct bluetooth_io *);

/* autoconf api called from transport layer */
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
	struct bluetooth_device_unit *device = NULL;
	struct bluetooth_io *io = NULL;
	if (bluetooth_in(sc))
		return;
	DPRINTF(("%s: bluetooth_detach\n", DEVNAME(sc)));
	sc->state = BT_STATE_DYING; /* dying */
	wakeup(sc);
	wakeup(&sc->rxfifo);
	if (sc->count) /* XXX INFSLP too risky ? */
		rwsleep_nsec(&sc->count, &sc->lock, BTPRI, DEVNAME(sc), INFSLP);
	if (sc->count)
		printf("%s: die with %d zombies\n", DEVNAME(sc), sc->count);

	while (!SIMPLEQ_EMPTY(&sc->rxfifo)) {
		io = SIMPLEQ_FIRST(&sc->rxfifo);
		SIMPLEQ_REMOVE_HEAD(&sc->rxfifo, fifo);
		bluetooth_io_free(io);
	}
	while (!SLIST_EMPTY(&sc->devices)) {
		device = SLIST_FIRST(&sc->devices);
		SLIST_REMOVE_HEAD(&sc->devices, sl);
		free(device, M_BLUETOOTH, sizeof(*device));
	}
	rw_exit_write(&sc->lock);
}

/* dev/bluetooth api : open, close, read, ioctl */
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

	if ((err = bluetooth_enter(sc)) != 0)
		return (err);
	DPRINTF(("%s: bluetoothopen\n", DEVNAME(sc)));
	if (sc->state != BT_STATE_WAITING)
		err = EBUSY;
	else
		sc->state = BT_STATE_DEVOPEN;
	bluetooth_leave(sc);
	return (err);
}

int
bluetoothclose(dev_t dev, int flags, int fmt, struct proc *p)
{
	struct bluetooth_softc *sc;
	struct bluetooth_io *io;
	int err;

	sc = bluetooth_cd.cd_devs[BLUETOOTHUNIT(dev)];
	DPRINTF(("%s: bluetoothclose\n", DEVNAME(sc)));

	if ((err = bluetooth_enter(sc)) != 0)
		return (err);
	sc->state = BT_STATE_WAITING;
	while (!SIMPLEQ_EMPTY(&sc->rxfifo)) {
		io = SIMPLEQ_FIRST(&sc->rxfifo);
		SIMPLEQ_REMOVE_HEAD(&sc->rxfifo, fifo);
		bluetooth_io_free(io);
	}
	bluetooth_leave(sc);
	return (0);
}

int
bluetoothread(dev_t dev, struct uio *uio, int flag)
{
	struct bluetooth_softc *sc;
	struct bluetooth_io *io = NULL;
	int err;

	sc = bluetooth_cd.cd_devs[BLUETOOTHUNIT(dev)];
	if ((err = bluetooth_enter(sc)) != 0)
		return (err);

	err = bluetooth_rxfifo_read(sc, &io);
	if (err)
		goto fail;
	if (io->size && uio->uio_resid != io->size) {
		DPRINTF(("%s: invalid uio size %zu != %zu\n",
		    DEVNAME(sc), uio->uio_resid, io->size));
		err = EPROTO;
		goto fail;
	}
	err = uiomove(&io->buf, io->size, uio);
 fail:
	if (io)
		bluetooth_io_free(io);
	bluetooth_leave(sc);
	return (err);
}

int
bluetoothioctl(dev_t dev, u_long cmd, caddr_t addr, int flags, struct proc *p)
{
	struct bluetooth_softc *sc;
	struct timespec start;
	union {
		struct bt_hci_version version;
		struct bt_hci_features features;
		struct bt_hci_lc_connect connect;
		struct bt_hci_lc_disconnect disconnect;
		struct bt_hci_lc_remote_name remote_name;
		struct bt_hci_info_bdaddr bdaddr;
		struct bt_hci_info_buffer buffer;
		struct bt_hci_info_commands commands;
	} hci;
	union {
		struct bluetooth_version *version;
		struct bluetooth_device_match *match;
	} ctl;
	struct bluetooth_device *device;
	struct bluetooth_device_unit *unit;
	uint16_t handle;
	int err = 0, i;

	sc = bluetooth_cd.cd_devs[BLUETOOTHUNIT(dev)];
	if ((err = bluetooth_enter(sc)) != 0)
		return (err);

	switch (cmd) {
	  case DIOCBTVERSION: {
		ctl.version = (struct bluetooth_version *)addr;
		/* XXX not required, safer ? */
		memset(addr, 0, sizeof(*ctl.version));

		bluetooth_out(sc);
		err = bthci_info_bdaddr(sc->hci, &hci.bdaddr);
		if (bluetooth_in(sc)) {
			err = EINTR;
			goto fail;
		}
		if (err)
			goto fail;
		memcpy(&ctl.version->bt_addr, &hci.bdaddr.bdaddr,
		    sizeof(ctl.version->bt_addr));

		bluetooth_out(sc);
		err = bthci_info_version(sc->hci, &hci.version);
		if (bluetooth_in(sc)) {
			err = EINTR;
			goto fail;
		}
		if (err)
			goto fail;
		ctl.version->bt_manufacturer = hci.version.bt_manufacturer;
		ctl.version->hci_version = hci.version.hci_version;
		ctl.version->hci_revision = hci.version.hci_revision;
		ctl.version->lmp_version = hci.version.lmp_version;
		ctl.version->lmp_revision = hci.version.lmp_revision;

		bluetooth_out(sc);
		err = bthci_info_buffer(sc->hci, &hci.buffer);
		if (bluetooth_in(sc)) {
			err = EINTR;
			goto fail;
		}
		if (err)
			goto fail;
		ctl.version->acl_size = hci.buffer.acl_size;
		ctl.version->acl_bufferlen = hci.buffer.acl_bufferlen;
		ctl.version->sco_size = hci.buffer.sco_size;
		ctl.version->sco_bufferlen = hci.buffer.sco_bufferlen;

		bluetooth_out(sc);
		err = bthci_info_features(sc->hci, &hci.features);
		if (bluetooth_in(sc)) {
			err = EINTR;
			goto fail;
		}
		if (err)
			goto fail;
		memcpy(&ctl.version->features[0][0], &hci.features.bitmask,
		    sizeof(ctl.version->features[0]));
		/* XXX make a function */
		ctl.version->flow_control_lag = 0;
		ctl.version->flow_control_lag |= (
		    ctl.version->features[0][2] & (1<<6));
		ctl.version->flow_control_lag <<= 1;
		ctl.version->flow_control_lag |= (
		    ctl.version->features[0][2] & (1<<5));
		ctl.version->flow_control_lag <<= 1;
		ctl.version->flow_control_lag |= (
		    ctl.version->features[0][2] & (1<<4));
		ctl.version->flow_control_lag <<= 8; /* in unit of 256 bytes */

		bluetooth_out(sc);
		err = bthci_info_efeatures(sc->hci, 0, &hci.features);
		if (bluetooth_in(sc)) {
			err = EINTR;
			goto fail;
		}
		if (err)
			goto fail;
		i = hci.features.max_page;
		if (i > BT_EXTENDED_PAGE_MAX) {
			printf("%s : invalid maximum page number %d > %d\n",
			    DEVNAME(sc), i, BT_EXTENDED_PAGE_MAX);
			i = BT_EXTENDED_PAGE_MAX;
		}
		while(i) {
			bluetooth_out(sc);
			err = bthci_info_efeatures(sc->hci, i, &hci.features);
			if (bluetooth_in(sc)) {
				err = EINTR;
				goto fail;
			}
			if (err)
				break;
			memcpy(&ctl.version->features[i][0],
			    &hci.features.bitmask,
			    sizeof(ctl.version->features[i]));
			i--;
		}

		bluetooth_out(sc);
		err = bthci_info_commands(sc->hci, &hci.commands);
		if (bluetooth_in(sc)) {
			err = EINTR;
			goto fail;
		}
		if (err)
			goto fail;
		memcpy(&ctl.version->commands, &hci.commands.bitmask,
		    sizeof(ctl.version->commands));
		break;

	} case DIOCBTINQUIRY: {
		/* XXX make a function */
		sc->state = BT_STATE_INQUIRY;
		wakeup(sc);

		bluetooth_out(sc);
		getnanotime(&start);
		err = bthci_lc_inquiry(sc->hci, BT_TIMEOUT_INQUIRY, 0);
		if (bluetooth_in(sc)) {
			err = EINTR;
			goto fail;
		}
		if (err)
			goto fail;

		/* XXX make a function */
		sc->state = BT_STATE_DEVOPEN;
		wakeup(sc);
		err = bluetooth_rxfifo_readerr(sc);
		if (err)
			goto fail;
		SLIST_FOREACH(unit, &sc->devices, sl) {
			device = (struct bluetooth_device *)unit;
			if (timespeccmp(&device->nanotime, &start, >))
				continue;
			getnanotime(&device->nanotime);
			if (device->name[0] == '\0') {
				err = bthci_lc_remote_name(sc->hci,
				    &device->bt_addr, device->bt_scan_mode,
				    device->bt_clock, &hci.remote_name);
				if (err)
					goto fail;
				memcpy(&device->name, hci.remote_name.name,
				    sizeof(device->name));
			}
			err = bluetooth_rxfifo_write(sc,device,sizeof(*device),0);
			if (err)
				goto fail;
		}
		err = bluetooth_rxfifo_write(sc, NULL, 0, 0);
		if (err)
			goto fail;
		break;

	} case DIOCBTMATCH: {
		/* loockup and connect remote device */
		ctl.match = (struct bluetooth_device_match *)addr;
		if (ctl.match->unit > 0)
			device = bluetooth_device_unit(sc, ctl.match->unit);
		else
			device = bluetooth_device_loockup(sc,&ctl.match->bt_addr);
		if (device == NULL) {
			err = EINVAL;
			goto fail;
		}

		/* XXX not required, safer ? */
		memset(addr, 0, sizeof(*ctl.match));
		ctl.match->unit = device->unit;
		memcpy(&ctl.match->bt_addr, &device->bt_addr,
		    sizeof(ctl.match->bt_addr));

		/* enter connect state and establich a link handle */
		/* XXX make a function */
		sc->state = BT_STATE_CONNECT;
		wakeup(sc);
		bluetooth_out(sc);
		err = bthci_lc_connect(sc->hci, sc->acl_type, device,
		    &hci.connect);
		if (bluetooth_in(sc)) {
			err = EINTR;
			goto fail;
		}
		if (err)
			goto fail;
		/* XXX make a function */
		sc->state = BT_STATE_DEVOPEN;
		wakeup(sc);
		err = bluetooth_rxfifo_readerr(sc);
		if (err)
			goto fail;
		handle = hci.connect.handle;

		/* remote device LMP informations : versions and features */
		bluetooth_out(sc);
		err = bthci_lc_remote_version(sc->hci, handle, &hci.version);
		if (bluetooth_in(sc)) {
			err = EINTR;
			goto fail;
		}
		if (err)
			goto fail;
		ctl.match->bt_manufacturer = hci.version.bt_manufacturer;
		ctl.match->lmp_version = hci.version.lmp_version;
		ctl.match->lmp_revision = hci.version.lmp_revision;

		bluetooth_out(sc);
		err = bthci_lc_remote_features(sc->hci, handle, &hci.features);
		if (bluetooth_in(sc)) {
			err = EINTR;
			goto fail;
		}
		if (err)
			goto fail;
		memcpy(&ctl.match->features[0][0], &hci.features.bitmask,
		    sizeof(ctl.match->features[0]));
		/* XXX make a function */
		ctl.match->flow_control_lag = 0;
		ctl.match->flow_control_lag |= (
		    ctl.match->features[0][2] & (1<<6));
		ctl.match->flow_control_lag <<= 1;
		ctl.match->flow_control_lag |= (
		    ctl.match->features[0][2] & (1<<5));
		ctl.match->flow_control_lag <<= 1;
		ctl.match->flow_control_lag |= (
		    ctl.match->features[0][2] & (1<<4));
		ctl.match->flow_control_lag <<= 8; /* in unit of 256 bytes */

		bluetooth_out(sc);
		err = bthci_lc_remote_efeatures(sc->hci, handle, 0,
		    &hci.features);
		if (bluetooth_in(sc)) {
			err = EINTR;
			goto fail;
		}
		if (err)
			goto fail;
		i = hci.features.max_page;
		if (i > BT_EXTENDED_PAGE_MAX) {
			printf("%s : invalid maximum page number %d > %d\n",
			    DEVNAME(sc), i, BT_EXTENDED_PAGE_MAX);
			i = BT_EXTENDED_PAGE_MAX;
		}
		while(i) {
			bluetooth_out(sc);
			err = bthci_lc_remote_efeatures(sc->hci, handle, i,
			    &hci.features);
			if (bluetooth_in(sc)) {
				err = EINTR;
				goto fail;
			}
			if (err)
				break;
			memcpy(&ctl.match->features[i][0],
			    &hci.features.bitmask,
			    sizeof(ctl.match->features[i]));
			i--;
		}

		/* establich a logical link handle and perform SDP */

		/* disconnect remote device */
		bluetooth_out(sc);
		err = bthci_lc_disconnect(sc->hci, handle, &hci.disconnect);
		if (bluetooth_in(sc)) {
			err = EINTR;
			goto fail;
		}
		if (err)
			goto fail;
		break;

	} default: {
		err = EINVAL;
		break;
	}}
 fail:
	if (err && bluetooth_dying(sc) == 0 && sc->state > BT_STATE_DEVOPEN)
		sc->state = BT_STATE_DEVOPEN;
	bluetooth_leave(sc);
	return (err);
}

/* internal kthread context */
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
	int err = 0, state;

	sc = (struct bluetooth_softc *)priv;
	DPRINTF(("%s: kthread\n", DEVNAME(sc)));

	if ((err = bluetooth_enter(sc)) != 0) {
		kthread_exit(err);
		return;
	}

	do {
		err = bluetooth_init(sc);
		if (err)
			printf("%s: bluetooth_init, err %d\n", DEVNAME(sc), err);
	} while(err == EAGAIN);
	if (err)
		sc->state = BT_STATE_DYING;
	else
		sc->state = BT_STATE_WAITING;
	DPRINTF(("%s: kthread init done, state=%d\n",
	    DEVNAME(sc), sc->state));

	state = sc->state;
	while(sc->state != BT_STATE_DYING) {
		err = 0;
		while((evt = bthci_read_evt(sc->hci)) != NULL) {
			DPRINTF(("%s: kthread event, state %d, sc state %d\n",
			    DEVNAME(sc), state, sc->state));
			switch (sc->state) {
			case BT_STATE_INQUIRY:
				err = bluetooth_inquiry(sc, evt);
				break;
			case BT_STATE_CONNECT:
				err = bluetooth_connect(sc, evt);
				break;
			default:
				DUMP_BT_EVT(DEVNAME(sc), evt);
				err = EPROTO;
			}
			if (err < 0) {
				DUMP_BT_EVT(DEVNAME(sc), evt);
				err = 0;
			}
			bthci_pool_put(sc->hci, evt);
			if (err)
				goto state_done; /* abort state, report error */
		}
		rwsleep_nsec(sc, &sc->lock, BTPRI, DEVNAME(sc), INFSLP);
		if (sc->state == BT_STATE_DYING) /* dying */
			break;
		if (state > BT_STATE_DEVOPEN && state > sc->state)
			goto state_done;
		if (state != sc->state) {
			DPRINTF(("%s: kthread state %d -> %d\n",
			    DEVNAME(sc), state, sc->state));
			state = sc->state;
		}
		continue;
 state_done:
		DPRINTF(("%s: kthread state %d done, sc state %d, err %d\n",
		    DEVNAME(sc), state, sc->state, err));
		if (sc->state >= BT_STATE_DEVOPEN)
			state = sc->state = BT_STATE_DEVOPEN;
		if (bluetooth_rxfifo_write(sc, NULL, 0, err) == ENOMEM)
			printf("%s: kthread rxfifo_write ENOMEM\n",
			    DEVNAME(sc));
	}
	err = (sc->state != BT_STATE_DYING);
	DPRINTF(("%s: kthread exit state=%d, err=%d\n",
	    DEVNAME(sc), sc->state, err));
	bluetooth_leave(sc);
	kthread_exit(err);
}

int
bluetooth_init(struct bluetooth_softc *sc)
{
	struct bt_hci_features features;
	int err;

	bluetooth_leave(sc);
	if ((err = bthci_cb_reset(sc->hci))) {
		printf("%s: init, bthci_cb_reset %d\n",
		    DEVNAME(sc), err);
		goto fail;
	}
	if ((err = bthci_cb_reset_event_mask(sc->hci))) {
		printf("%s: init, bthci_cb_reset_event_mask %d\n",
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

	bluetooth_enter(sc);
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
	return (0);
 fail:
	bluetooth_enter(sc);
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
	struct bluetooth_device *dev;
	int err = 0, len, i;

	switch(evt->head.op) {
	case BT_EVT_INQUIRY_RESULT: {
		if (evt->head.len < 1) {
			printf("%s: inquiry result, minimum len %d < 1\n",
			    DEVNAME(sc), evt->head.len);
			err = EPROTO;
			goto fail;
		}
		inquiries.ndevices = evt->data[0];
		if (inquiries.ndevices < 1) {
			printf("%s: inquiry result, minimum devices %d < 1\n",
			    DEVNAME(sc), inquiries.ndevices);
			err = EPROTO;
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
			err = EPROTO;
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
				err = EPROTO;
				goto fail;
			}
		}
		break;
	}
	default:
		err = -1;
	}
 fail:
	return (err);
}

int
bluetooth_connect(struct bluetooth_softc *sc, struct bt_evt *evt)
{
	int err = 0;

	switch(evt->head.op) {
	default: {
		err = -1;
		break;
	}}
	return (err);
}

/* private locking */
int
bluetooth_dying(struct bluetooth_softc *sc)
{
	if (sc->state == BT_STATE_DYING)
		return (EINTR);
	return (0);
}

int
bluetooth_enter(struct bluetooth_softc *sc)
{
	rw_enter_write(&sc->lock);
	if (bluetooth_dying(sc)) {
		rw_exit_write(&sc->lock);
		return (EINTR);
	}
	sc->count++;
	return (0);
}

void
bluetooth_out(struct bluetooth_softc *sc)
{
	rw_exit_write(&sc->lock);
}

int
bluetooth_in(struct bluetooth_softc *sc)
{
	rw_enter_write(&sc->lock);
	return (bluetooth_unsleep(sc));
}

int
bluetooth_unsleep(struct bluetooth_softc *sc)
{
	if (bluetooth_dying(sc))
		return (EINTR);
	return (0);
}

void
bluetooth_leave(struct bluetooth_softc *sc)
{
	sc->count--;
	if (bluetooth_dying(sc) && sc->count == 0)
		wakeup(&sc->count);
	rw_exit_write(&sc->lock);
}

/* private functions */
struct bluetooth_device *
bluetooth_device_loockup(struct bluetooth_softc *sc,
    struct bluetooth_bdaddr *bdaddr)
{
	struct bluetooth_device_unit *dev = NULL;
	int i;
#ifdef BT_DEBUG
	printf("%s: loockup ", DEVNAME(sc));
	for (i = BT_ADDR_LEN; --i >= 0;)
		printf("%02X%c", bdaddr->b[i], (i)?':':' ');
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
		printf("%02X%c", bdaddr->b[i], (i)?':':' ');
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
bluetooth_rxfifo_write(struct bluetooth_softc *sc,void *buf,size_t size,int err)
{
	struct bluetooth_io *io;
	DPRINTF(("%s: dev_write %zu bytes, err %d\n",
	    DEVNAME(sc), size, err));
	if (sc->state < BT_STATE_DEVOPEN)
		return (EINTR);
	bluetooth_out(sc);
	io = malloc(sizeof(*io) + size, M_BLUETOOTH, M_WAITOK|M_CANFAIL);
	if (bluetooth_in(sc)) {
		if (io)
			bluetooth_io_free(io);
		return (EINTR);
	}
	if (io == NULL)
		return (ENOMEM);
	io->size = size;
	io->err = err;
	if (size)
		memcpy(&io->buf, buf, size);
	*((uint8_t*)(&io->buf + size)) = '\0'; /* XXX canary, null guard ? */
	SIMPLEQ_INSERT_TAIL(&sc->rxfifo, io, fifo);
	wakeup(&sc->rxfifo);
	return (0);
}

int
bluetooth_rxfifo_read(struct bluetooth_softc *sc, struct bluetooth_io **io)
{
	int err = 0;
	*io = SIMPLEQ_FIRST(&sc->rxfifo);
	if (*io == NULL) {
		err = rwsleep_nsec(&sc->rxfifo, &sc->lock, BTPRI|PCATCH,
		    DEVNAME(sc), INFSLP);
		if (bluetooth_unsleep(sc))
			return (EINTR);
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
	err = (*io)->err;
	if (err) {
		bluetooth_io_free(*io);
		*io = NULL;
	}
	return (err);
}

int
bluetooth_rxfifo_readerr(struct bluetooth_softc *sc)
{
	struct bluetooth_io *io = NULL;
	int err = 0;

	err = bluetooth_rxfifo_read(sc, &io);
	if (err)
		goto fail;
	if (io->size) {
		printf("%s: rxfifo read void, non empty io size %zu\n",
		    DEVNAME(sc), io->size);
		err = EPROTO;
	}
	bluetooth_io_free(io);
 fail:
	return (err);
}

void
bluetooth_io_free(struct bluetooth_io *io)
{
	/* XXX canary, null guard ? */
	if (*((uint8_t*)(&io->buf + io->size)) != '\0')
		printf("bluetooth: dev_io overflow, not null terminated\n");
	free(io, M_BLUETOOTH, sizeof(*io) + io->size);
}
