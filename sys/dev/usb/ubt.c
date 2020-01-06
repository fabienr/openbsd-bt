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

 /* XXX debug */
#define UBT_DEBUG

#include <sys/types.h>
#include <sys/errno.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <sys/time.h>
#include <sys/pool.h>
#include <machine/bus.h>
#include <machine/intr.h>
#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbdivar.h>
#include <dev/usb/usbdevs.h>
#include <dev/bluetooth/bluetooth.h>
#include <bluetooth/bthci.h>

#include "ubt.h"

struct ubt_softc {
	struct bluetooth_softc	 sc_sc;
	struct bthci		 hci;

	/* USB specific goo. */
	struct usbd_device	*sc_udev;
	struct usbd_interface	*sc_iface0;
	struct usbd_pipe	*tx_cmd_pipe;
	struct usbd_xfer	*tx_cmd_xfer;
	struct usbd_pipe	*rx_evt_pipe;
	struct bt_evt		 rx_evt_buf;
	struct usbd_pipe	*tx_acl_pipe;
	struct usbd_xfer	*tx_acl_xfer;
	struct usbd_pipe	*rx_acl_pipe;
	struct usbd_xfer	*rx_acl_xfer;
	struct usbd_interface	*sc_iface1;
	struct usbd_pipe	*tx_sco_pipe;
	struct usbd_xfer	*tx_sco_xfer;
	struct usbd_pipe	*rx_sco_pipe;
	struct usbd_xfer	*rx_sco_xfer;
};

int	ubt_cmd(struct device *, const struct bt_cmd *);
int	ubt_acl(struct device *, const struct bt_acl *);
int	ubt_sco(struct device *, const struct bt_sco *);
static struct btbus ubt_bus = {
	.cmd = ubt_cmd,
	.acl = ubt_acl,
	.sco = ubt_sco,
};

#ifdef UBT_DEBUG
#define DPRINTF(x)	do { printf x; } while (0)
#else
#define DPRINTF(x)
#endif
#define DEVNAME(sc) ((sc)->sc_sc.sc_dev.dv_xname)

int	ubt_match(struct device *, void *, void *);
void	ubt_attach(struct device *, struct device *, void *);
int	ubt_detach(struct device *, int);
int	ubt_open_pipes(struct ubt_softc *);
void	ubt_close_pipes(struct ubt_softc *);
int	ubt_malloc_xfers(struct ubt_softc *);
void	ubt_free_xfers(struct ubt_softc *);

static void	ubt_evt(struct usbd_xfer *, void *, usbd_status);
static int	ubt_rx_acl_start(struct ubt_softc *);
static void	ubt_rx_acl(struct usbd_xfer *, void *, usbd_status);
static int	ubt_rx_sco_start(struct ubt_softc *);
static void	ubt_rx_sco(struct usbd_xfer *, void *, usbd_status);

struct cfattach ubt_ca = {
    sizeof(struct ubt_softc), ubt_match, ubt_attach, ubt_detach
};

/*
 * To match or ignore, add details to the ubt_dev list.
 * Use value of -1 to indicate a wildcard
 * To override another entry, add details earlier
 */
static const struct ubt_devno {
	int			vendor;
	int			product;
	int			class;
	int			subclass;
	int			proto;
	int			match;
} ubt_dev[] = {
	{    /* ignore Broadcom 2033 without firmware */
	    USB_VENDOR_BROADCOM, USB_PRODUCT_BROADCOM_BCM2033NF,
	    -1, -1, -1, UMATCH_NONE
	}, { /* Apple Bluetooth Host Controller MacbookPro 7,1 */
	    USB_VENDOR_APPLE, USB_PRODUCT_APPLE_BLUETOOTH_HOST_1,
	    -1, -1, -1,
	    UMATCH_VENDOR_PRODUCT
	}, { /* Apple Bluetooth Host Controller iMac 11,1 */
	    USB_VENDOR_APPLE, USB_PRODUCT_APPLE_BLUETOOTH_HOST_2,
	    -1, -1, -1,
	    UMATCH_VENDOR_PRODUCT
	}, { /* Apple Bluetooth Host Controller MacBookPro 8,2 */
	    USB_VENDOR_APPLE, USB_PRODUCT_APPLE_BLUETOOTH_HOST_3,
	    -1, -1, -1,
	    UMATCH_VENDOR_PRODUCT
	}, { /* Apple Bluetooth Host Controller MacBookAir 3,1 3,2*/
	    USB_VENDOR_APPLE, USB_PRODUCT_APPLE_BLUETOOTH_HOST_4,
	    -1, -1, -1,
	    UMATCH_VENDOR_PRODUCT
	}, { /* Apple Bluetooth Host Controller MacBookAir 4,1 */
	    USB_VENDOR_APPLE, USB_PRODUCT_APPLE_BLUETOOTH_HOST_5,
	    -1, -1, -1,
	    UMATCH_VENDOR_PRODUCT
	}, { /* Apple Bluetooth Host Controller MacMini 5,1 */
	    USB_VENDOR_APPLE, USB_PRODUCT_APPLE_BLUETOOTH_HOST_6,
	    -1, -1, -1,
	    UMATCH_VENDOR_PRODUCT
	}, { /* Apple Bluetooth Host Controller MacBookAir 6,1 */
	    USB_VENDOR_APPLE, USB_PRODUCT_APPLE_BLUETOOTH_HOST_7,
	    -1, -1, -1,
	    UMATCH_VENDOR_PRODUCT
	}, { /* Apple Bluetooth Host Controller MacBookPro 9,2 */
	    USB_VENDOR_APPLE, USB_PRODUCT_APPLE_BLUETOOTH_HOST_8,
	    -1, -1, -1,
	    UMATCH_VENDOR_PRODUCT
	}, { /* Broadcom chips with PatchRAM support */
	    USB_VENDOR_BROADCOM, -1,
	    UDCLASS_VENDOR, UDSUBCLASS_RF, UDPROTO_BLUETOOTH,
	    UMATCH_VENDOR_DEVCLASS_DEVPROTO
	}, { /* Broadcom based device with PatchRAM support */
	    USB_VENDOR_FOXCONN, -1,
	    UDCLASS_VENDOR, UDSUBCLASS_RF, UDPROTO_BLUETOOTH,
	    UMATCH_VENDOR_DEVCLASS_DEVPROTO
	}, { /* Broadcom based device with PatchRAM support */
	    USB_VENDOR_LITEON, -1,
	    UDCLASS_VENDOR, UDSUBCLASS_RF, UDPROTO_BLUETOOTH,
	    UMATCH_VENDOR_DEVCLASS_DEVPROTO
	}, { /* Broadcom based device with PatchRAM support */
	    USB_VENDOR_BELKIN, -1,
	    UDCLASS_VENDOR, UDSUBCLASS_RF, UDPROTO_BLUETOOTH,
	    UMATCH_VENDOR_DEVCLASS_DEVPROTO
	}, { /* Broadcom based device with PatchRAM support */
	    USB_VENDOR_TOSHIBA, -1,
	    UDCLASS_VENDOR, UDSUBCLASS_RF, UDPROTO_BLUETOOTH,
	    UMATCH_VENDOR_DEVCLASS_DEVPROTO
	}, { /* Broadcom based device with PatchRAM support */
	    USB_VENDOR_ASUSTEK, -1,
	    UDCLASS_VENDOR, UDSUBCLASS_RF, UDPROTO_BLUETOOTH,
	    UMATCH_VENDOR_DEVCLASS_DEVPROTO
	}, { /* Asus based device with PatchRAM support */
	    USB_VENDOR_ASUS, -1,
	    UDCLASS_VENDOR, UDSUBCLASS_RF, UDPROTO_BLUETOOTH,
	    UMATCH_VENDOR_DEVCLASS_DEVPROTO
	}, { /* Generic Bluetooth SIG compliant devices */
	    -1, -1,
	    UDCLASS_WIRELESS, UDSUBCLASS_RF, UDPROTO_BLUETOOTH,
	    UMATCH_DEVCLASS_DEVSUBCLASS_DEVPROTO
	},
};

int
ubt_match(struct device *parent, void *match, void *aux)
{
	struct usb_attach_arg		*uaa = aux;
	usb_device_descriptor_t		*dd;
	size_t i;

	DPRINTF(("ubt_match\n"));

	dd = usbd_get_device_descriptor(uaa->device);
	for (i = 0; i < sizeof(ubt_dev); i++) {
		if (ubt_dev[i].vendor != -1
		    && ubt_dev[i].vendor != (int)uaa->vendor)
			continue;
		if (ubt_dev[i].product != -1
		    && ubt_dev[i].product != (int)uaa->product)
			continue;
		if (ubt_dev[i].class != -1
		    && ubt_dev[i].class != dd->bDeviceClass)
			continue;
		if (ubt_dev[i].subclass != -1
		    && ubt_dev[i].subclass != dd->bDeviceSubClass)
			continue;
		if (ubt_dev[i].proto != -1
		    && ubt_dev[i].proto != dd->bDeviceProtocol)
			continue;
		if (ubt_dev[i].match == UMATCH_NONE)
			DPRINTF(("ubt_match reject invalid vendor\n"));
		return (ubt_dev[i].match);
	}

	return (UMATCH_NONE);
}

void
ubt_attach(struct device *parent, struct device *self, void *aux)
{
	struct ubt_softc *usc = (struct ubt_softc *)self;
	struct usb_attach_arg *uaa = aux;
	usbd_status err;

	DPRINTF(("%s: ubt_attach\n", DEVNAME(usc)));

	usc->sc_udev = uaa->device;
	err = usbd_set_config_index(usc->sc_udev, 0, 1);
	if (err) {
		printf("%s: setting configuration index 0 failed, err=%s\n",
		    DEVNAME(usc), usbd_errstr(err));
		goto fail;
	}
	err = usbd_device2interface_handle(usc->sc_udev, 0, &usc->sc_iface0);
	if (err || usc->sc_iface0 == NULL) {
		printf("%s: failed to locate control interface, err=%s\n",
		    DEVNAME(usc), usbd_errstr(err));
		goto fail;
	}
	err = usbd_device2interface_handle(usc->sc_udev, 1, &usc->sc_iface1);
	if (err || usc->sc_iface1 == NULL) {
		printf("%s: failed to locate sco interface, err=%s\n",
		    DEVNAME(usc), usbd_errstr(err));
		goto fail;
	}

	if (ubt_open_pipes(usc) != 0)
		goto fail;

	if (ubt_malloc_xfers(usc) != 0)
		goto fail;

	/*
	if (ubt_rx_acl_start(usc) != 0)
		goto fail;

	if (ubt_rx_sco_start(usc) != 0)
		goto fail;
	*/

	bthci_init(&usc->hci, (struct device *)usc, &ubt_bus, IPL_SOFTUSB);
	bluetooth_attach(&usc->sc_sc, &usc->hci);
	return;
 fail:
	usbd_deactivate(usc->sc_udev);
	ubt_close_pipes(usc);
	ubt_free_xfers(usc);
}

int
ubt_detach(struct device *self, int flags)
{
	struct ubt_softc *usc = (struct ubt_softc *)self;
	DPRINTF(("%s: ubt_detach\n", DEVNAME(usc)));

	usbd_ref_wait(usc->sc_udev);
	bthci_destroy(&usc->hci);
	ubt_close_pipes(usc);
	ubt_free_xfers(usc);

	return (0);
}

int
ubt_open_pipes(struct ubt_softc *usc)
{
	usbd_status err = USBD_NORMAL_COMPLETION;

	/* interface 0 :
	 * - ctrl cmd pipe at UBT_CMD_OUT, default_pipe, pipe address 0x00
	 * - intr evt pipe at UBT_EVT_INT
	 * - tx acl pipe at UBT_ACL_OUT
	 * - rx acl pipe at UBT_ACL_IN
	 * interface 1 :
	 * - tx sco pipe at UBT_SCO_OUT
	 * - rx sco pipe at UBT_SCO_IN
	 */
	usc->tx_cmd_pipe = usc->sc_udev->default_pipe;
	if (usc->tx_cmd_pipe == NULL) {
		err = USBD_NOT_STARTED; /* XXX better error code */
		printf("%s: invalid default_pipe\n",
		    DEVNAME(usc));
		goto fail;
	}
	err = usbd_open_pipe_intr(usc->sc_iface0, UBT_EVT_IN,
	    USBD_SHORT_XFER_OK, &usc->rx_evt_pipe, usc,
	    &usc->rx_evt_buf, sizeof(usc->rx_evt_buf),
	    ubt_evt, USBD_DEFAULT_INTERVAL);
	if (err != USBD_NORMAL_COMPLETION) {
		printf("%s: could not open intr evt pipe, err=%s\n",
		    DEVNAME(usc), usbd_errstr(err));
		goto fail;
	}
	/* XXX
	err = usbd_open_pipe(usc->sc_iface0, UBT_ACL_OUT, USBD_EXCLUSIVE_USE,
	    &usc->tx_acl_pipe);
	if (err != USBD_NORMAL_COMPLETION) {
		printf("%s: could not open Tx acl bulk pipe, err=%s\n",
		    DEVNAME(usc), usbd_errstr(err));
		goto fail;
	}
	err = usbd_open_pipe(usc->sc_iface0, UBT_ACL_IN, USBD_EXCLUSIVE_USE,
	    &usc->rx_acl_pipe);
	if (err != USBD_NORMAL_COMPLETION) {
		printf("%s: could not open Rx acl bulk pipe, err=%s\n",
		    DEVNAME(usc), usbd_errstr(err));
		goto fail;
	}
	err = usbd_open_pipe(usc->sc_iface1, UBT_SCO_OUT, USBD_EXCLUSIVE_USE,
	    &usc->tx_sco_pipe);
	if (err != USBD_NORMAL_COMPLETION) {
		printf("%s: could not open Tx sco bulk pipe, err=%s\n",
		    DEVNAME(usc), usbd_errstr(err));
		goto fail;
	}
	err = usbd_open_pipe(usc->sc_iface1, UBT_SCO_IN, USBD_EXCLUSIVE_USE,
	    &usc->rx_sco_pipe);
	if (err != USBD_NORMAL_COMPLETION) {
		printf("%s: could not open Rx sco bulk pipe, err=%s\n",
		    DEVNAME(usc), usbd_errstr(err));
		goto fail;
	}
	*/

 fail:
	if (err != USBD_NORMAL_COMPLETION)
		ubt_close_pipes(usc);
	return (err);
}

void
ubt_close_pipes(struct ubt_softc *usc)
{
	if (usc->tx_cmd_pipe) {
		usbd_abort_pipe(usc->tx_cmd_pipe);
		usc->tx_cmd_pipe = NULL;
	}
	if (usc->rx_evt_pipe != NULL) {
		usbd_close_pipe(usc->rx_evt_pipe);
		usc->rx_evt_pipe = NULL;
	}
	if (usc->tx_acl_pipe != NULL) {
		usbd_close_pipe(usc->tx_acl_pipe);
		usc->tx_acl_pipe = NULL;
	}
	if (usc->rx_acl_pipe != NULL) {
		usbd_close_pipe(usc->rx_acl_pipe);
		usc->rx_acl_pipe = NULL;
	}
	if (usc->tx_sco_pipe != NULL) {
		usbd_close_pipe(usc->tx_sco_pipe);
		usc->tx_sco_pipe = NULL;
	}
	if (usc->rx_sco_pipe != NULL) {
		usbd_close_pipe(usc->rx_sco_pipe);
		usc->rx_sco_pipe = NULL;
	}
}

int
ubt_malloc_xfers(struct ubt_softc *usc)
{
	usbd_status err = USBD_NORMAL_COMPLETION;
	void *buffer;
	usc->tx_cmd_xfer = usbd_alloc_xfer(usc->sc_udev);
	if (usc->tx_cmd_xfer == NULL) {
		err = USBD_NOMEM;
		goto fail;
	}
	buffer = usbd_alloc_buffer(usc->tx_cmd_xfer, sizeof(struct bt_cmd));
	if (buffer == NULL) {
		err = USBD_NOMEM;
		goto fail;
	}
	/* XXX
	usc->tx_acl_xfer = usbd_alloc_xfer(usc->sc_udev);
	if (usc->tx_acl_xfer == NULL) {
		err = USBD_NOMEM;
		goto fail;
	}
	buffer = usbd_alloc_buffer(usc->tx_acl_xfer, sizeof(struct bt_acl));
	if (buffer == NULL) {
		err = USBD_NOMEM;
		goto fail;
	}
	usc->rx_acl_xfer = usbd_alloc_xfer(usc->sc_udev);
	if (usc->rx_acl_xfer == NULL) {
		err = USBD_NOMEM;
		goto fail;
	}
	buffer = usbd_alloc_buffer(usc->rx_acl_xfer, sizeof(struct bt_acl));
	if (buffer == NULL) {
		err = USBD_NOMEM;
		goto fail;
	}
	usc->tx_sco_xfer = usbd_alloc_xfer(usc->sc_udev);
	if (usc->tx_sco_xfer == NULL) {
		err = USBD_NOMEM;
		goto fail;
	}
	buffer = usbd_alloc_buffer(usc->tx_sco_xfer, sizeof(struct bt_sco));
	if (buffer == NULL) {
		err = USBD_NOMEM;
		goto fail;
	}
	usc->rx_sco_xfer = usbd_alloc_xfer(usc->sc_udev);
	if (usc->rx_sco_xfer == NULL) {
		err = USBD_NOMEM;
		goto fail;
	}
	buffer = usbd_alloc_buffer(usc->rx_sco_xfer, sizeof(struct bt_sco));
	if (buffer == NULL) {
		err = USBD_NOMEM;
		goto fail;
	}
	*/
 fail:
	if (err != USBD_NORMAL_COMPLETION)
		ubt_free_xfers(usc);
	return (err);
}

void
ubt_free_xfers(struct ubt_softc *usc)
{
	if (usc->tx_cmd_xfer != NULL) {
		usbd_free_xfer(usc->tx_cmd_xfer);
		usc->tx_cmd_xfer = NULL;
	}
	if (usc->tx_acl_xfer != NULL) {
		usbd_free_xfer(usc->tx_acl_xfer);
		usc->tx_acl_xfer = NULL;
	}
	if (usc->rx_acl_xfer != NULL) {
		usbd_free_xfer(usc->rx_acl_xfer);
		usc->rx_acl_xfer = NULL;
	}
	if (usc->tx_sco_xfer != NULL) {
		usbd_free_xfer(usc->tx_sco_xfer);
		usc->tx_sco_xfer = NULL;
	}
	if (usc->rx_sco_xfer != NULL) {
		usbd_free_xfer(usc->rx_sco_xfer);
		usc->rx_sco_xfer = NULL;
	}
}

int
ubt_cmd(struct device *sc, const struct bt_cmd *pkt)
{
	struct ubt_softc	*usc;
	usb_device_request_t	 req;
	usbd_status		 err;

	usc = (struct ubt_softc *)sc;
	memset(&req, 0, sizeof(req));
	req.bmRequestType = UT_WRITE_CLASS_DEVICE;
	USETW(req.wLength, pkt->head.len + sizeof(pkt->head));

	err = usbd_do_request_flags(usc->sc_udev, &req, (void*)pkt,
	    USBD_FORCE_SHORT_XFER, NULL, USBD_DEFAULT_TIMEOUT);
	if (err != USBD_NORMAL_COMPLETION) {
		printf("%s: ubt_cmd, usbd_transfer, err=%s\n",
		    DEVNAME(usc), usbd_errstr(err));
		return (EIO);
	}

	return (0);
}

int
ubt_acl(struct device *sc, const struct bt_acl *pkt)
{
	return (0);
}

int
ubt_sco(struct device *sc, const struct bt_sco *pkt)
{
	return (0);
}

static void
ubt_evt(struct usbd_xfer *xfer, void *priv, usbd_status status)
{
	struct ubt_softc	*usc = priv;
	struct bt_evt		*evt, *pkt = &usc->rx_evt_buf;
	uint32_t		 len;

	if (status != USBD_NORMAL_COMPLETION)
		return;
	if (usbd_is_dying(usc->sc_udev))
		return;

	usbd_get_xfer_status(xfer, NULL, NULL, &len, NULL);
	if (len < sizeof(struct bt_evt_head)) {
		printf("%s: invalid interrupt, len %d < minimum %zu\n",
		    DEVNAME(usc), len, sizeof(struct bt_evt_head));
		return;
	}
	if (len - sizeof(struct bt_evt_head) != pkt->head.len) {
		printf("%s: invalid interrupt, len mismatch %d != %d\n",
		    DEVNAME(usc), len, pkt->head.len);
		return;
	}
	if ((evt = bthci_pool_get(&usc->hci)) == NULL) {
		printf("%s: bthci pool empty\n",
		    DEVNAME(usc));
		return;
	}
	memcpy(evt, &usc->rx_evt_buf, len);
	bthci_write_evt(&usc->hci, evt);
	return;
}

static int
ubt_rx_acl_start(struct ubt_softc *usc)
{
	usbd_status err;

	DPRINTF(("%s: ubt_rx_acl_start\n", DEVNAME(usc)));
	usbd_setup_xfer(usc->rx_acl_xfer,
	    usc->rx_acl_pipe,
	    usc,
	    NULL,
	    sizeof(struct bt_acl),
	    USBD_SHORT_XFER_OK,
	    USBD_NO_TIMEOUT,
	    ubt_rx_acl);
	err = usbd_transfer(usc->rx_acl_xfer);
	if (err != USBD_IN_PROGRESS) {
		printf("%s: ubt_rx_acl_start, err=%s\n",
		    DEVNAME(usc), usbd_errstr(err));
		return (err);
	}
	return (0);
}

static void
ubt_rx_acl(struct usbd_xfer *xfer, void *self, usbd_status err)
{
	struct ubt_softc	*usc;
	uint32_t		 len;

	usc = (struct ubt_softc *)self;
	if (usbd_is_dying(usc->sc_udev))
		return;
	if (xfer == NULL)
		goto done;
	if (err != USBD_NORMAL_COMPLETION) {
		if (err == USBD_NOT_STARTED || err == USBD_CANCELLED ||
		    err == USBD_IOERROR)
			return;
		if (err == USBD_STALLED)
			usbd_clear_endpoint_stall_async(usc->rx_acl_pipe);
		DPRINTF(("%s: ubt_rx_acl, err=%s\n",
		    DEVNAME(usc), usbd_errstr(err)));
		goto done;
	}
	usbd_get_xfer_status(xfer, NULL, NULL, &len, NULL);
	DPRINTF(("%s: ubt_rx_acl got %d bytes\n",
	    DEVNAME(usc), len));
done:
	/* XXX may fail, how to handle it ? */
	if (ubt_rx_acl_start(usc))
		printf("%s: ubt_rx_acl, start failed\n",
		    DEVNAME(usc));
}

static int
ubt_rx_sco_start(struct ubt_softc *usc)
{
	usbd_status err;

	DPRINTF(("%s: ubt_rx_sco_start\n", DEVNAME(usc)));
	usbd_setup_xfer(usc->rx_sco_xfer,
	    usc->rx_sco_pipe,
	    usc,
	    NULL,
	    sizeof(struct bt_sco),
	    USBD_SHORT_XFER_OK,
	    USBD_NO_TIMEOUT,
	    ubt_rx_sco);
	err = usbd_transfer(usc->rx_sco_xfer);
	if (err != USBD_IN_PROGRESS) {
		printf("%s: ubt_rx_sco_start, err=%s\n",
		    DEVNAME(usc), usbd_errstr(err));
		return (err);
	}
	return (0);
}

static void
ubt_rx_sco(struct usbd_xfer *xfer, void *self, usbd_status err)
{
	struct ubt_softc	*usc;
	uint32_t		 len;

	usc = (struct ubt_softc *)self;
	if (usbd_is_dying(usc->sc_udev))
		return;
	if (xfer == NULL)
		goto done;
	if (err != USBD_NORMAL_COMPLETION) {
		if (err == USBD_NOT_STARTED || err == USBD_CANCELLED ||
		    err == USBD_IOERROR)
			return;
		if (err == USBD_STALLED)
			usbd_clear_endpoint_stall_async(usc->rx_sco_pipe);
		DPRINTF(("%s: ubt_rx_sco, err=%s\n",
		    DEVNAME(usc), usbd_errstr(err)));
		goto done;
	}
	usbd_get_xfer_status(xfer, NULL, NULL, &len, NULL);
	DPRINTF(("%s: ubt_rx_sco got %d bytes\n",
	    DEVNAME(usc), len));
done:
	/* XXX may fail, how to handle it ? */
	if (ubt_rx_sco_start(usc))
		printf("%s: ubt_rx_sco, start failed\n",
		    DEVNAME(usc));
}
