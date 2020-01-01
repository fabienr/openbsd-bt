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
#include <machine/bus.h>
#include <sys/malloc.h>
#include <sys/systm.h>
#include <sys/device.h>
#include <dev/usb/usb.h>
#include <dev/usb/usbdi.h>
#include <dev/usb/usbdi_util.h>
#include <dev/usb/usbdivar.h>
#include <dev/usb/usbdevs.h>
#include <dev/ic/bthci.h>

#include "ubt.h"

struct ubt_softc {
	struct bthci_softc	 sc_sc;

	/* USB specific goo. */
	struct usbd_device	*sc_udev;
	struct usbd_interface	*sc_iface0;
	struct usbd_pipe	*ctrl_cmd_pipe;
	struct usbd_pipe	*intr_evt_pipe;
	uint8_t			*ibuf;
	size_t			 ibuflen;
	struct usbd_pipe	*tx_acl_pipe;
	struct usbd_pipe	*rx_acl_pipe;
	struct usbd_interface	*sc_iface1;
	struct usbd_pipe	*rx_sco_pipe;
	struct usbd_pipe	*tx_sco_pipe;
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

void	ubt_cmd(struct bthci_softc *, struct bt_cmd *);
void	ubt_intr(struct usbd_xfer *, void *, usbd_status);
void	ubt_acl(struct bthci_softc *, struct bt_acl *);
void	ubt_sco(struct bthci_softc *, struct bt_sco *);

struct cfdriver ubt_cd = {
    NULL, "ubt", DV_DULL
};

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
	{   /* ignore Broadcom 2033 without firmware */
	    USB_VENDOR_BROADCOM,
	    USB_PRODUCT_BROADCOM_BCM2033NF,
	    -1,
	    -1,
	    -1,
	    UMATCH_NONE
	},
	{   /* Apple Bluetooth Host Controller MacbookPro 7,1 */
	    USB_VENDOR_APPLE,
	    USB_PRODUCT_APPLE_BLUETOOTH_HOST_1,
	    -1,
	    -1,
	    -1,
	    UMATCH_VENDOR_PRODUCT
	},
	{   /* Apple Bluetooth Host Controller iMac 11,1 */
	    USB_VENDOR_APPLE,
	    USB_PRODUCT_APPLE_BLUETOOTH_HOST_2,
	    -1,
	    -1,
	    -1,
	    UMATCH_VENDOR_PRODUCT
	},
	{   /* Apple Bluetooth Host Controller MacBookPro 8,2 */
	    USB_VENDOR_APPLE,
	    USB_PRODUCT_APPLE_BLUETOOTH_HOST_3,
	    -1,
	    -1,
	    -1,
	    UMATCH_VENDOR_PRODUCT
	},
	{   /* Apple Bluetooth Host Controller MacBookAir 3,1 3,2*/
	    USB_VENDOR_APPLE,
	    USB_PRODUCT_APPLE_BLUETOOTH_HOST_4,
	    -1,
	    -1,
	    -1,
	    UMATCH_VENDOR_PRODUCT
	},
	{   /* Apple Bluetooth Host Controller MacBookAir 4,1 */
	    USB_VENDOR_APPLE,
	    USB_PRODUCT_APPLE_BLUETOOTH_HOST_5,
	    -1,
	    -1,
	    -1,
	    UMATCH_VENDOR_PRODUCT
	},
	{   /* Apple Bluetooth Host Controller MacMini 5,1 */
	    USB_VENDOR_APPLE,
	    USB_PRODUCT_APPLE_BLUETOOTH_HOST_6,
	    -1,
	    -1,
	    -1,
	    UMATCH_VENDOR_PRODUCT
	},
	{   /* Apple Bluetooth Host Controller MacBookAir 6,1 */
	    USB_VENDOR_APPLE,
	    USB_PRODUCT_APPLE_BLUETOOTH_HOST_7,
	    -1,
	    -1,
	    -1,
	    UMATCH_VENDOR_PRODUCT
	},
	{   /* Apple Bluetooth Host Controller MacBookPro 9,2 */
	    USB_VENDOR_APPLE,
	    USB_PRODUCT_APPLE_BLUETOOTH_HOST_8,
	    -1,
	    -1,
	    -1,
	    UMATCH_VENDOR_PRODUCT
	},
	{   /* Broadcom chips with PatchRAM support */
	    USB_VENDOR_BROADCOM,
	    -1,
	    UDCLASS_VENDOR,
	    UDSUBCLASS_RF,
	    UDPROTO_BLUETOOTH,
	    UMATCH_VENDOR_DEVCLASS_DEVPROTO
	},
	{   /* Broadcom based device with PatchRAM support */
	    USB_VENDOR_FOXCONN,
	    -1,
	    UDCLASS_VENDOR,
	    UDSUBCLASS_RF,
	    UDPROTO_BLUETOOTH,
	    UMATCH_VENDOR_DEVCLASS_DEVPROTO
	},
	{   /* Broadcom based device with PatchRAM support */
	    USB_VENDOR_LITEON,
	    -1,
	    UDCLASS_VENDOR,
	    UDSUBCLASS_RF,
	    UDPROTO_BLUETOOTH,
	    UMATCH_VENDOR_DEVCLASS_DEVPROTO
	},
	{   /* Broadcom based device with PatchRAM support */
	    USB_VENDOR_BELKIN,
	    -1,
	    UDCLASS_VENDOR,
	    UDSUBCLASS_RF,
	    UDPROTO_BLUETOOTH,
	    UMATCH_VENDOR_DEVCLASS_DEVPROTO
	},
	{   /* Broadcom based device with PatchRAM support */
	    USB_VENDOR_TOSHIBA,
	    -1,
	    UDCLASS_VENDOR,
	    UDSUBCLASS_RF,
	    UDPROTO_BLUETOOTH,
	    UMATCH_VENDOR_DEVCLASS_DEVPROTO
	},
	{   /* Broadcom based device with PatchRAM support */
	    USB_VENDOR_ASUSTEK,
	    -1,
	    UDCLASS_VENDOR,
	    UDSUBCLASS_RF,
	    UDPROTO_BLUETOOTH,
	    UMATCH_VENDOR_DEVCLASS_DEVPROTO
	},
	{   /* Generic Bluetooth SIG compliant devices */
	    -1,
	    -1,
	    UDCLASS_WIRELESS,
	    UDSUBCLASS_RF,
	    UDPROTO_BLUETOOTH,
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
	struct bthci_softc *sc = &usc->sc_sc;
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

	sc->ops.cmd = ubt_cmd;
	sc->ops.acl = ubt_acl;
	sc->ops.sco = ubt_sco;

	if (ubt_open_pipes(usc) != 0)
		goto fail;

	return;
 fail:
	usbd_deactivate(usc->sc_udev);
}

int
ubt_detach(struct device *self, int flags)
{
	struct ubt_softc *usc = (struct ubt_softc *)self;
	/*struct bthci_softc *sc = &usc->sc_sc;*/
	DPRINTF(("%s: ubt_detach\n", DEVNAME(usc)));
	usbd_ref_wait(usc->sc_udev);

	/* Abort and close Tx/Rx pipes. */
	ubt_close_pipes(usc);

	return (0);
}

int
ubt_open_pipes(struct ubt_softc *usc)
{
	usb_endpoint_descriptor_t *ed;
	usbd_status err;
	int isize;

	/* interface 0 :
	 * - ctrl cmd pipe at UBT_CMD_OUT, default_pipe, pipe address 0x00
	 * - intr evt pipe at UBT_EVT_INT
	 * - tx acl pipe at UBT_ACL_OUT
	 * - rx acl pipe at UBT_ACL_IN
	 * interface 1 :
	 * - tx sco pipe at UBT_SCO_OUT
	 * - rx sco pipe at UBT_SCO_IN
	 */

	usc->ctrl_cmd_pipe = usc->sc_udev->default_pipe;
	if (usc->ctrl_cmd_pipe == NULL) {
		printf("%s: invalid ctrl cmd pipe\n",
		    DEVNAME(usc));
		goto fail;
	}
	DPRINTF(("%s: ctrl_cmd_pipe ok\n", DEVNAME(usc)));

	ed = usbd_get_endpoint_descriptor(usc->sc_iface0, UBT_EVT_IN);
	if (ed == NULL) {
		printf("%s: could not retrieve intr evt pipe descriptor\n",
		    DEVNAME(usc));
		goto fail;
	}
	isize = UGETW(ed->wMaxPacketSize);
	if (isize == 0) {
		printf("%s: invalid intr evt pipe descriptor\n",
		    DEVNAME(usc));
		goto fail;
	}
	usc->ibuf = malloc(isize, M_USBDEV, M_NOWAIT);
	if (usc->ibuf == NULL) {
		printf("%s: could not allocate Rx intr buffer\n",
		    DEVNAME(usc));
		goto fail;
	}
	usc->ibuflen = isize;
	err = usbd_open_pipe_intr(usc->sc_iface0, UBT_EVT_IN,
	    USBD_SHORT_XFER_OK, &usc->intr_evt_pipe, usc, usc->ibuf, isize,
	    ubt_intr, USBD_DEFAULT_INTERVAL);
	if (err != 0) {
		printf("%s: could not open intr evt pipe, err=%s\n",
		    DEVNAME(usc), usbd_errstr(err));
		goto fail;
	}
	DPRINTF(("%s: intr_evt_pipe ok\n", DEVNAME(usc)));

	err = usbd_open_pipe(usc->sc_iface0, UBT_ACL_OUT, 0,
	    &usc->tx_acl_pipe);
	if (err != 0) {
		printf("%s: could not open Tx acl bulk pipe, err=%s\n",
		    DEVNAME(usc), usbd_errstr(err));
		goto fail;
	}
	DPRINTF(("%s: tx_acl_pipe ok\n", DEVNAME(usc)));

	err = usbd_open_pipe(usc->sc_iface0, UBT_ACL_IN, 0,
	    &usc->rx_acl_pipe);
	if (err != 0) {
		printf("%s: could not open Rx acl bulk pipe, err=%s\n",
		    DEVNAME(usc), usbd_errstr(err));
		goto fail;
	}
	DPRINTF(("%s: rx_acl_pipe ok\n", DEVNAME(usc)));

	err = usbd_open_pipe(usc->sc_iface1, UBT_SCO_OUT, 0,
	    &usc->tx_sco_pipe);
	if (err != 0) {
		printf("%s: could not open Tx sco bulk pipe, err=%s\n",
		    DEVNAME(usc), usbd_errstr(err));
		goto fail;
	}
	DPRINTF(("%s: tx_sco_pipe ok\n", DEVNAME(usc)));

	err = usbd_open_pipe(usc->sc_iface1, UBT_SCO_IN, 0,
	    &usc->rx_sco_pipe);
	if (err != 0) {
		printf("%s: could not open Rx sco bulk pipe, err=%s\n",
		    DEVNAME(usc), usbd_errstr(err));
		goto fail;
	}
	DPRINTF(("%s: rx_sco_pipe ok\n", DEVNAME(usc)));

	fail:
	if (err != 0)
		ubt_close_pipes(usc);
	return (err);
}

void
ubt_close_pipes(struct ubt_softc *usc)
{
	if (usc->ctrl_cmd_pipe != NULL) {
		usbd_abort_pipe(usc->ctrl_cmd_pipe);
		usc->ctrl_cmd_pipe = NULL;
	}
	if (usc->intr_evt_pipe != NULL) {
		usbd_close_pipe(usc->intr_evt_pipe);
		usc->intr_evt_pipe = NULL;
	}
	if (usc->ibuf != NULL) {
		free(usc->ibuf, M_USBDEV, usc->ibuflen);
		usc->ibuf = NULL;
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

void
ubt_cmd(struct bthci_softc *sc, struct bt_cmd *pkt)
{
	return;
}

void
ubt_intr(struct usbd_xfer *xfer, void *priv,
    usbd_status status)
{
	/*
	struct ubt_softc *usc = priv;
	uint8_t *buf = usc->ibuf;
	*/
	return;
}

void
ubt_acl(struct bthci_softc *sc, struct bt_acl *pkt)
{
	return;
}

void
ubt_sco(struct bthci_softc *sc, struct bt_sco *pkt)
{
	return;
}
