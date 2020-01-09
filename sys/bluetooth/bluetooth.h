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

#ifndef _NET_BLUETOOTH_H_
#define _NET_BLUETOOTH_H_

#include <sys/ioccom.h>

#ifndef _KERNEL
#include <sys/types.h>
#endif /* _KERNEL */

/* Bluetooth error codes */
#define BT_ERR_UNKNOW_HCI_COMMAND		0x01
#define BT_ERR_UNKNOW_CONN_ID			0x02
#define BT_ERR_HW_FAILURE			0x03
#define BT_ERR_PAGE_TIMEOUT			0x04
#define BT_ERR_AUTH_FAILURE			0x05
#define BT_ERR_PIN_KEY_MISSING			0x06
#define BT_ERR_NOMEM				0x07
#define BT_ERR_CONN_TIMEOUT			0x08
#define BT_ERR_CONN_LIMIT			0x09
#define BT_ERR_SCO_LIMIT			0x0A
#define BT_ERR_CONN_EXISTS			0x0B
#define BT_ERR_COMMAND_DISALLOWED		0x0C
#define BT_ERR_CONN_REJ_NOMEM			0x0D
#define BT_ERR_CONN_REJ_SECU			0x0E
#define BT_ERR_CONN_REJ_BDADDR			0x0F
#define BT_ERR_CONN_ACC_TIMEOUT			0x10
#define BT_ERR_UNSUPPORTED			0x11
#define BT_ERR_INVALID_HCI_COMMAND		0x12
#define BT_ERR_CONN_TERM_REMOTE			0x13
#define BT_ERR_CONN_TERM_NOMEM			0x14
#define BT_ERR_CONN_TERM_PWOFF			0x15
#define BT_ERR_CONN_TERM_HOST			0x16
#define BT_ERR_REPEATED_ATTEMPS			0x17
#define BT_ERR_PAIRING_DISALLOWED		0x18
#define BT_ERR_LMP_UNKNOW_PDU			0x19
#define BT_ERR_LMP_UNSUPPORTED			0x1A
#define BT_ERR_SCO_OFFSET_REJ			0x1B
#define BT_ERR_SCO_INTERVAL_REJ			0x1C
#define BT_ERR_SCO_MODE_REJ			0x1D
#define BT_ERR_LMPLL_INVALID			0x1E
#define BT_ERR_UNKNOW				0x1F
/* XXX ... */

struct bt_hci_info {
	uint8_t		state;
	uint8_t		hci_version;
	uint16_t	hci_revision;
	uint8_t		lmp_version;
	uint16_t	manufacturer;
	uint16_t	lmp_revision;
} __packed;

#define DIOCBTINFO	_IOR('B', 1, struct bt_hci_info)

#endif /* _NET_BLUETOOTH_H_ */
