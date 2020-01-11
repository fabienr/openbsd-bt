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
#define BT_ERR_SHIFT					8
#define BT_ERR_TOH(err)					((err)<<BT_ERR_SHIFT)
#define BT_ERR_SUCCESS						BT_ERR_TOH(0x00)
#define BT_ERR_UNKNOWN_HCI_COMMAND				BT_ERR_TOH(0x01)
#define BT_ERR_UNKNOWN_CONNECTION_IDENTIFIER			BT_ERR_TOH(0x02)
#define BT_ERR_HARDWARE_FAILURE					BT_ERR_TOH(0x03)
#define BT_ERR_PAGE_TIMEOUT					BT_ERR_TOH(0x04)
#define BT_ERR_AUTHENTICATION_FAILURE				BT_ERR_TOH(0x05)
#define BT_ERR_PIN_OR_KEY_MISSING				BT_ERR_TOH(0x06)
#define BT_ERR_MEMORY_CAPACITY_EXCEEDED				BT_ERR_TOH(0x07)
#define BT_ERR_CONNECTION_TIMEOUT				BT_ERR_TOH(0x08)
#define BT_ERR_CONNECTION_LIMIT_EXCEEDED			BT_ERR_TOH(0x09)
#define BT_ERR_SCO_LIMIT_TO_A_DEVICE_EXCEEDED			BT_ERR_TOH(0x0A)
#define BT_ERR_ACL_CONNECTION_ALREADY_EXISTS			BT_ERR_TOH(0x0B)
#define BT_ERR_COMMAND_DISALLOWED				BT_ERR_TOH(0x0C)
#define BT_ERR_CONNECTION_REJECTED_DUE_TO_LIMITED_RESOURCES	BT_ERR_TOH(0x0D)
#define BT_ERR_CONNECTION_REJECTED_DUE_TO_SECURITY_REASONS	BT_ERR_TOH(0x0E)
#define BT_ERR_CONNECTION_REJECTED_DUE_TO_UNACCEPTABLE_BD_ADDR	BT_ERR_TOH(0x0F)
#define BT_ERR_CONNECTION_ACCEPT_TIMEOUT_EXCEEDED		BT_ERR_TOH(0x10)
#define BT_ERR_UNSUPPORTED_FEATURE_OR_PARAMETER_VALUE		BT_ERR_TOH(0x11)
#define BT_ERR_INVALID_HCI_COMMAND_PARAMETERS			BT_ERR_TOH(0x12)
#define BT_ERR_REMOTE_USER_TERMINATED_CONNECTION		BT_ERR_TOH(0x13)
#define BT_ERR_REMOTE_DEV_TERMINATED_CONNECTION_LOW_RESOURCES	BT_ERR_TOH(0x14)
#define BT_ERR_REMOTE_DEV_TERMINATED_CONNECTION_POWER_OFF	BT_ERR_TOH(0x15)
#define BT_ERR_CONNECTION_TERMINATED_BY_LOCAL_HOST		BT_ERR_TOH(0x16)
#define BT_ERR_REPEATED_ATTEMPTS				BT_ERR_TOH(0x17)
#define BT_ERR_PAIRING_NOT_ALLOWED				BT_ERR_TOH(0x18)
#define BT_ERR_UNKNOWN_LMP_PDU					BT_ERR_TOH(0x19)
#define BT_ERR_UNSUPPORTED_REMOTE_LMP_FEATURE			BT_ERR_TOH(0x1A)
#define BT_ERR_SCO_OFFSET_REJECTED				BT_ERR_TOH(0x1B)
#define BT_ERR_SCO_INTERVAL_REJECTED				BT_ERR_TOH(0x1C)
#define BT_ERR_SCO_AIR_MODE_REJECTED				BT_ERR_TOH(0x1D)
#define BT_ERR_INVALID_LMP_PARAMETERS_INVALID_LL_PARAMETERS	BT_ERR_TOH(0x1E)
#define BT_ERR_UNSPECIFIED_ERROR				BT_ERR_TOH(0x1F)
#define BT_ERR_UNSUPPORTED_LMP_LL_PARAMETER_VALUE		BT_ERR_TOH(0x20)
#define BT_ERR_ROLE_CHANGE_NOT_ALLOWED				BT_ERR_TOH(0x21)
#define BT_ERR_LMP_RESPONSE_TIMEOUT_LL_RESPONSE_TIMEOUT		BT_ERR_TOH(0x22)
#define BT_ERR_LMP_ERROR_TRANSACTION_COLLISION			BT_ERR_TOH(0x23)
#define BT_ERR_LMP_PDU_NOT_ALLOWED				BT_ERR_TOH(0x24)
#define BT_ERR_ENCRYPTION_MODE_NOT_ACCEPTABLE			BT_ERR_TOH(0x25)
#define BT_ERR_LINK_KEY_CANNOT_BE_CHANGED			BT_ERR_TOH(0x26)
#define BT_ERR_REQUESTED_QOS_NOT_SUPPORTED			BT_ERR_TOH(0x27)
#define BT_ERR_INSTANT_PASSED					BT_ERR_TOH(0x28)
#define BT_ERR_PAIRING_WITH_UNIT_KEY_NOT_SUPPORTED		BT_ERR_TOH(0x29)
#define BT_ERR_DIFFERENT_TRANSACTION_COLLISION			BT_ERR_TOH(0x2A)
#define BT_ERR_RESERVED						BT_ERR_TOH(0x2B)
#define BT_ERR_QOS_UNACCEPTABLE_PARAMETER			BT_ERR_TOH(0x2C)
#define BT_ERR_QOS_REJECTED					BT_ERR_TOH(0x2D)
#define BT_ERR_CHANNEL_CLASSIFICATION_NOT_SUPPORTED		BT_ERR_TOH(0x2E)
#define BT_ERR_INSUFFICIENT_SECURITY				BT_ERR_TOH(0x2F)
#define BT_ERR_PARAMETER_OUT_OF_MANDATORY_RANGE			BT_ERR_TOH(0x30)
// #define BT_ERR_RESERVED
#define BT_ERR_ROLE_SWITCH_PENDING				BT_ERR_TOH(0x32)
// #define BT_ERR_RESERVED
#define BT_ERR_RESERVED_SLOT_VIOLATION				BT_ERR_TOH(0x34)
#define BT_ERR_ROLE_SWITCH_FAILED				BT_ERR_TOH(0x35)
#define BT_ERR_EXTENDED_INQUIRY_RESPONSE_TOO_LARGE		BT_ERR_TOH(0x36)
#define BT_ERR_SECURE_SIMPLE_PAIRING_NOT_SUPPORTED_BY_HOST	BT_ERR_TOH(0x37)
#define BT_ERR_HOST_BUSY_PAIRING				BT_ERR_TOH(0x38)
#define BT_ERR_CONNECTION_REJECTED_NO_SUITABLE_CHANNEL_FOUND	BT_ERR_TOH(0x39)
#define BT_ERR_CONTROLLER_BUSY					BT_ERR_TOH(0x3A)
#define BT_ERR_UNACCEPTABLE_CONNECTION_PARAMETERS		BT_ERR_TOH(0x3B)
#define BT_ERR_DIRECTED_ADVERTISING_TIMEOUT			BT_ERR_TOH(0x3C)
#define BT_ERR_CONNECTION_TERMINATED_DUE_TO_MIC_FAILURE		BT_ERR_TOH(0x3D)
#define BT_ERR_CONNECTION_FAILED_TO_BE_ESTABLISHED		BT_ERR_TOH(0x3E)
#define BT_ERR_MAC_CONNECTION_FAILED				BT_ERR_TOH(0x3F)
#define BT_ERR_COARSE_CLOCK_ADJ_REJECTED_USING_CLOCK_DRAGGING	BT_ERR_TOH(0x40)

#define BT_ADDR_LEN			6
#define BT_COMMANDS_BITMASK_LEN		64
#define BT_FEATURES_BITMASK_LEN		8
#define BT_EXTENDED_PAGE_MAX		3

/* XXX implement kind of strbdaddr for printing */
struct bluetooth_bdaddr {
	uint8_t		bdaddr[BT_ADDR_LEN];
};

struct bluetooth_class {
	uint8_t		fields[3];
};

struct bluetooth_info {
	struct bluetooth_bdaddr	bt_addr;
	uint16_t	bt_manufacturer; /* see btstrings.c */
	uint8_t		hci_version; /* see btstrings.c */
	int		hci_revision;
	uint8_t		lmp_version; /* see btstrings.c */
	int		lmp_revision;
	int		acl_size;
	int		acl_bufferlen;
	int		sco_size;
	int		sco_bufferlen;
};
#define DIOCBTINFO	_IOR('B', 1, struct bluetooth_info)

struct bluetooth_info_extended {
	int		flow_control_lag; /* in bytes */
	uint8_t		features[BT_EXTENDED_PAGE_MAX][BT_FEATURES_BITMASK_LEN];
	uint8_t		commands[BT_COMMANDS_BITMASK_LEN]; /* see btstrings.c */
};
#define DIOCBTINFOEXT	_IOR('B', 2, struct bluetooth_info_extended)

/* parameter void */
#define DIOCBTINQUIRY	_IO('B', 3)
/* data to be read after */
struct bluetooth_device { /* XXX to consolidate in a database with unit id */
	struct bluetooth_bdaddr	bt_addr;
	struct bluetooth_class	bt_class;
	char			name[248]; /* XXX see BT_EVT_MAX_PAYLOAD */
};

#endif /* _NET_BLUETOOTH_H_ */