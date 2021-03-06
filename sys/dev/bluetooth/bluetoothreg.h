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

#ifdef BLUETOOTH_DUMP
#define DUMP_BT_CMD(devname, cmd) do {						\
	printf("%s: cmd head(op=%04X, len=%d), data",				\
	    (devname), (cmd)->head.op, (cmd)->head.len);			\
	for (int i = 0; i < (cmd)->head.len; i++)				\
		printf(" %02x", (cmd)->data[i]);				\
	printf("\n");								\
} while(0)
#define DUMP_BT_EVT(devname, evt) do {						\
	printf("%s: event head(op=%02X, len=%d), data",				\
	    (devname), (evt)->head.op, (evt)->head.len);			\
	for (int i = 0; i < (evt)->head.len; i++)				\
		printf(" %02x", (evt)->data[i]);				\
	printf("\n");								\
} while(0)
#define DUMP_BT_ACL(devname, acl) do {						\
	printf("%s: acl head(h_f=%04X, len=%d), data",				\
	    (devname), (acl)->head.h_f, (acl)->head.len);			\
	for (int i = 0; i < (acl)->head.len; i++)				\
		printf(" %02x", (acl)->data[i]);				\
	printf("\n");								\
} while(0)
#else
#define DUMP_BT_EVT(devname, evt)
#define DUMP_BT_CMD(devname, evt)
#define DUMP_BT_ACL(devname, acl)
#endif

/* Bluetooth Link Control commands */
#define BT_HCI_OGF_LC			0x01
#define BT_HCI_OCF_INQUIRY		0x0001
#define BT_HCI_OCF_CONNECTION		0x0005
#define BT_HCI_OCF_DISCONNECT		0x0006
#define BT_HCI_OCF_REMOTE_NAME		0x0019
#define BT_HCI_OCF_REMOTE_FEATURES	0x001B
#define BT_HCI_OCF_REMOTE_EFEATURES	0x001C
#define BT_HCI_OCF_REMOTE_VERSION	0x001D
/* Bluetooth Link Policy commands */
#define BT_HCI_OGF_LP			0x02
/* Bluetooth Controller & Baseband commands */
#define BT_HCI_OGF_CB			0x03
#define BT_HCI_OCF_EVENTMASK		0x0001
#define BT_HCI_OCF_RESET		0x0003
#define BT_HCI_OCF_WRITE_NAME		0x0013
/* Bluetooth Informational parameters */
#define BT_HCI_OGF_INFO			0x04
#define BT_HCI_OCF_READ_VERSION		0x0001
#define BT_HCI_OCF_READ_COMMANDS	0x0002
#define BT_HCI_OCF_READ_FEATURES	0x0003
#define BT_HCI_OCF_READ_EFEATURES	0x0004
#define BT_HCI_OCF_READ_BUFFER		0x0005
#define BT_HCI_OCF_READ_BDADDR		0x0009
/* Bluetooth Status parameters */
#define BT_HCI_OGF_STAT			0x05

/* Bluetooth feature bitmask byte description for packet type negociation */
#define BT_HCI_FEATURE_0_3SLOT		(1<<0)
#define BT_HCI_FEATURE_0_5SLOT		(1<<1)
#define BT_HCI_FEATURE_1_SC0		(1<<3)
#define BT_HCI_FEATURE_1_HV2		(1<<4)
#define BT_HCI_FEATURE_1_HV3		(1<<5)
#define BT_HCI_FEATURE_3_EDRACL2MBPS	(1<<1)
#define BT_HCI_FEATURE_3_EDRACL3MBPS	(1<<2)
#define BT_HCI_FEATURE_3_EV3		(1<<7)
#define BT_HCI_FEATURE_4_EV4		(1<<0)
#define BT_HCI_FEATURE_4_EV5		(1<<1)
#define BT_HCI_FEATURE_4_3SLOTSEDRACL	(1<<7)
#define BT_HCI_FEATURE_5_5SLOTSEDRACL	(1<<0)
#define BT_HCI_FEATURE_5_EDRSCO2MBPS	(1<<5)
#define BT_HCI_FEATURE_5_EDRSCO3MBPS	(1<<6)
#define BT_HCI_FEATURE_5_3SLOTSEDRSCO	(1<<7)

/* Bluetooth inquiry LAP code */
#define BT_HCI_INQUIRY_LAP_2		0x9E
#define BT_HCI_INQUIRY_LAP_1		0x8B
#define BT_HCI_INQUIRY_LAP_GIAC_0	0x33
#define BT_HCI_INQUIRY_LAP_LIAC_0	0x00 /* XXX not used */

/* bt hci cmd packet, header 3, data max 255, total 258 bytes */
struct bt_cmd_head {
	uint16_t	op;
	uint8_t		len;
} __packed;
struct bt_cmd {
	struct bt_cmd_head	head;
	uint8_t			data[255];
} __packed;

/* Bluetooth Event codes */
#define BT_EVT_INQUIRY_COMPL			0x01
#define BT_EVT_INQUIRY_RESULT			0x02
#define BT_EVT_CON_COMPL			0x03
#define BT_EVT_CON_REQ				0x04
#define BT_EVT_DISCON_COMPL			0x05
#define BT_EVT_AUTH_COMPL			0x06
#define BT_EVT_REMOTE_NAME_REQ_COMPL		0x07
#define BT_EVT_ENCRYPTION_CHANGE		0x08
#define BT_EVT_CHANGE_CON_LINK_KEY_COMPL	0x09
#define BT_EVT_MASTER_LINK_KEY_COMPL		0x0a
#define BT_EVT_READ_REMOTE_FEATURES_COMPL	0x0b
#define BT_EVT_READ_REMOTE_VER_INFO_COMPL	0x0c
#define BT_EVT_QOS_SETUP_COMPL			0x0d
#define BT_EVT_CMD_COMPLETE			0x0e
#define BT_EVT_CMD_STATE			0x0f
#define BT_EVT_HARDWARE_ERROR			0x10
#define BT_EVT_FLUSH_OCCUR			0x11
#define BT_EVT_ROLE_CHANGE			0x12
#define BT_EVT_NUM_COMPL_PKTS			0x13
#define BT_EVT_MODE_CHANGE			0x14
#define BT_EVT_RETURN_LINK_KEYS			0x15
#define BT_EVT_PIN_CODE_REQ			0x16
#define BT_EVT_LINK_KEY_REQ			0x17
#define BT_EVT_LINK_KEY_NOTIFICATION		0x18
#define BT_EVT_LOOPBACK_COMMAND			0x19
#define BT_EVT_DATA_BUFFER_OVERFLOW		0x1a
#define BT_EVT_MAX_SLOT_CHANGE			0x1b
#define BT_EVT_READ_CLOCK_OFFSET_COMPL		0x1c
#define BT_EVT_CON_PKT_TYPE_CHANGED		0x1d
#define BT_EVT_QOS_VIOLATION			0x1e
#define BT_EVT_PAGE_SCAN_MODE_CHANGE		0x1f
#define BT_EVT_PAGE_SCAN_REP_MODE_CHANGE	0x20
#define BT_EVT_FLOW_SPECIFICATION_COMPL		0x21
#define BT_EVT_RSSI_RESULT			0x22
#define BT_EVT_READ_REMOTE_EXTENDED_FEATURES	0x23
#define BT_EVT_SCO_CON_COMPL			0x2c
#define BT_EVT_SCO_CON_CHANGED			0x2d
#define BT_EVT_SNIFF_SUBRATING			0x2e
#define BT_EVT_EXTENDED_RESULT			0x2f
#define BT_EVT_ENCRYPTION_KEY_REFRESH		0x30
#define BT_EVT_IO_CAPABILITY_REQ		0x31
#define BT_EVT_IO_CAPABILITY_RSP		0x32
#define BT_EVT_USER_CONFIRM_REQ			0x33
#define BT_EVT_USER_PASSKEY_REQ			0x34
#define BT_EVT_REMOTE_OOB_DATA_REQ		0x35
#define BT_EVT_SIMPLE_PAIRING_COMPL		0x36
#define BT_EVT_LINK_SUPERVISION_TO_CHANGED	0x38
#define BT_EVT_ENHANCED_FLUSH_COMPL		0x39
#define BT_EVT_USER_PASSKEY_NOTIFICATION	0x3b
#define BT_EVT_KEYPRESS_NOTIFICATION		0x3c
#define BT_EVT_REMOTE_FEATURES_NOTIFICATION	0x3d
#define BT_EVT_BT_LOGO				0xfe
#define BT_EVT_VENDOR				0xff
/* bt hci event packet, header 2, data max 255, total 257 bytes */
#define BT_EVT_MAX_PAYLOAD	255
struct bt_evt_head {
	uint8_t		op;
	uint8_t		len;
} __packed;
struct bt_evt {
	struct bt_evt_head	head;
	uint8_t			data[BT_EVT_MAX_PAYLOAD];
} __packed;

/* ACL Packet types which may be used for "Create Connection" */
#define BT_ACL_NO_2MBPS_DH1	(1<<1) /* shall not be used */
#define BT_ACL_NO_3MBPS_DH1	(1<<2) /* shall not be used */
#define BT_ACL_DM1		(1<<3)
#define BT_ACL_DH1		(1<<4)
#define BT_ACL_NO_2MBPS_DH3	(1<<8) /* shall not be used */
#define BT_ACL_NO_3MBPS_DH3	(1<<9) /* shall not be used */
#define BT_ACL_DM3		(1<<10)
#define BT_ACL_DH3		(1<<11)
#define BT_ACL_NO_2MBPS_DH5	(1<<12) /* shall not be used */
#define BT_ACL_NO_3MBPS_DH5	(1<<13) /* shall not be used */
#define BT_ACL_DM5		(1<<14)
#define BT_ACL_DH5		(1<<15)
/* bt hci asynchronous packet, header 4, data max 27, total 31 bytes
 * h_f :
 * - 12 bits handle, identification of logical link
 * - 02 bits pb : Packet Boundary flag
 * - 02 bits bc : BroadCast flag
 */
#define BT_ACL_HANDLE(h_f) (h_f >> 4)
#define BT_ACL_PBFLAG(h_f) ((h_f & 0xFF00) >> 2)
#define BT_ACL_BCFLAG(h_f) (h_f & 0xFF)
#define BT_ACL_PB_FIRST		0x0	/* start of non auto-flushable packet */
#define BT_ACL_PB_NEXT		0x1	/* continuing packet fragment */ 
#define BT_ACL_PB_FLUSH		0x2	/* start of auto-flushable packet */
#define BT_ACL_PB_AMP		0x3	/* complete auto-flushable packet(AMP) */
#define BT_ACL_BC_P2P		0x0	/* only point to point */
#define BT_ACL_BC_ASB		0x1	/* active slave broadcast */
struct bt_acl_head {
	uint16_t	h_f;
	uint16_t	len;
} __packed;
struct bt_acl {
	struct bt_acl_head	head;
	uint8_t			data[27]; /* XXX minimum/only lenght in spec */
} __packed;

/* SCO Packet types bitmask which may be used for "Setup Synchronous Connection" */
#define BT_SCO_HV1		(1<<0)
#define BT_SCO_HV2		(1<<1)
#define BT_SCO_HV3		(1<<2)
#define BT_SCO_EV3		(1<<3)
#define BT_SCO_EV4		(1<<4)
#define BT_SCO_EV5		(1<<5)
#define BT_SCO_NO_2MBPS_EV3	(1<<6) /* shall not be used */
#define BT_SCO_NO_3MBPS_EV3	(1<<7) /* shall not be used */
#define BT_SCO_NO_2MBPS_EV5	(1<<8) /* shall not be used */
#define BT_SCO_NO_3MBPS_EV5	(1<<9) /* shall not be used */
/* bt hci sychronous packet, header 3, data max 28, total 31 bytes
 * h_f :
 * - 12 bits handle, identification of logical link
 * - 02 bits ps : Packet Status, from host 0x00, from controller it depends on
 * Erronous Data Reporting parameter
 * - 02 bits rfu : Reserved for Future Use
 */
#define BT_SCO_HANDLE(h_f) (h_f >> 4)
#define BT_SCO_PSFLAG(h_f) ((h_f & 0xFF00) >> 2)
#define BT_SCO_PS_OK		0x0	/* correctly received data */
#define BT_SCO_PS_ERR		0x1	/* possibly invalid data */ 
#define BT_SCO_PS_EMPTY		0x2	/* no data received, length=0 */
#define BT_SCO_PS_LOST		0x3	/* data lost, on missing pkt length=0 */
struct bt_sco_head {
	uint16_t	h_f;
	uint8_t		len;
} __packed;
struct bt_sco {
	struct bt_sco_head	head;
	uint8_t			data[28];
} __packed;
