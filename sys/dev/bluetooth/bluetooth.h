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
#define BT_DEBUG

#define BT_TIMEOUT		SEC_TO_NSEC(5)
#define BT_STATE_INIT		0
#define BT_STATE_DYING		1
#define BT_STATE_WAITING	2

#define BT_EVTS_POOLSIZE	32
#ifdef BT_DEBUG
#define DUMP_BT_CMD(devname, cmd) do {						\
	DPRINTF(("%s: cmd head(op=%04X, len=%d), data",				\
	    (devname), (cmd)->head.op, (cmd)->head.len));			\
	for (int i = 0; i < (cmd)->head.len; i++)				\
		DPRINTF((" %02x ", (cmd)->data[i]));				\
	DPRINTF(("\n"));							\
} while(0)
#define DUMP_BT_EVT(devname, evt) do {						\
	DPRINTF(("%s: event head(op=%02X, len=%d), data",			\
	    (devname), (evt)->head.op, (evt)->head.len));			\
	for (int i = 0; i < (evt)->head.len; i++)				\
		DPRINTF((" %02x", (evt)->data[i]));				\
	DPRINTF(("\n"));							\
} while(0)
#else
#define DUMP_BT_EVT(hci, evt)
#define DUMP_BT_CMD(hci, evt)
#endif

struct bt_cmd;
struct bt_evt;
struct bt_acl;
struct bt_sco;
struct bthci;
struct btbus {
	int (*cmd)(struct device *, const struct bt_cmd *);
	int (*acl)(struct device *, const struct bt_acl *);
	int (*sco)(struct device *, const struct bt_sco *);
};

struct bluetooth_softc {
	struct device		 sc_dev;
	struct bthci		*hci;
	struct rwlock		 lock;
	int			 state;
};

void bluetooth_attach(struct bluetooth_softc *,  struct bthci *);
void bluetooth_detach(struct bluetooth_softc *);

/* bt hci cmd packet, header 3, data max 255, total 258 bytes */
struct bt_cmd_head {
	uint16_t	op;
	uint8_t		len;
} __packed;
struct bt_cmd {
	struct bt_cmd_head	head;
	uint8_t			data[255];
} __packed;

/* bt hci event packet, header 2, data max 255, total 257 bytes */
struct bt_evt_head {
	uint8_t		op;
	uint8_t		len;
} __packed;
struct bt_evt {
	struct bt_evt_head	head;
	uint8_t			data[255];
} __packed;

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
	uint8_t			data[27];
} __packed;

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

/* 
 * Bluetooth event codes
 */
#define BTEVT_INQUIRY_COMPL			0x01
#define BTEVT_INQUIRY_RESULT			0x02
#define BTEVT_CON_COMPL				0x03
#define BTEVT_CON_REQ				0x04
#define BTEVT_DISCON_COMPL			0x05
#define BTEVT_AUTH_COMPL			0x06
#define BTEVT_REMOTE_NAME_REQ_COMPL		0x07
#define BTEVT_ENCRYPTION_CHANGE			0x08
#define BTEVT_CHANGE_CON_LINK_KEY_COMPL		0x09
#define BTEVT_MASTER_LINK_KEY_COMPL		0x0a
#define BTEVT_READ_REMOTE_FEATURES_COMPL	0x0b
#define BTEVT_READ_REMOTE_VER_INFO_COMPL	0x0c
#define BTEVT_QOS_SETUP_COMPL			0x0d
#define BTEVT_CMD_COMPLETE			0x0e
#define BTEVT_CMD_STATE				0x0f
#define BTEVT_HARDWARE_ERROR			0x10
#define BTEVT_FLUSH_OCCUR			0x11
#define BTEVT_ROLE_CHANGE			0x12
#define BTEVT_NUM_COMPL_PKTS			0x13
#define BTEVT_MODE_CHANGE			0x14
#define BTEVT_RETURN_LINK_KEYS			0x15
#define BTEVT_PIN_CODE_REQ			0x16
#define BTEVT_LINK_KEY_REQ			0x17
#define BTEVT_LINK_KEY_NOTIFICATION		0x18
#define BTEVT_LOOPBACK_COMMAND			0x19
#define BTEVT_DATA_BUFFER_OVERFLOW		0x1a
#define BTEVT_MAX_SLOT_CHANGE			0x1b
#define BTEVT_READ_CLOCK_OFFSET_COMPL		0x1c
#define BTEVT_CON_PKT_TYPE_CHANGED		0x1d
#define BTEVT_QOS_VIOLATION			0x1e
#define BTEVT_PAGE_SCAN_MODE_CHANGE		0x1f
#define BTEVT_PAGE_SCAN_REP_MODE_CHANGE		0x20
#define BTEVT_FLOW_SPECIFICATION_COMPL		0x21
#define BTEVT_RSSI_RESULT			0x22
#define BTEVT_READ_REMOTE_EXTENDED_FEATURES	0x23
#define BTEVT_SCO_CON_COMPL			0x2c
#define BTEVT_SCO_CON_CHANGED			0x2d
#define BTEVT_SNIFF_SUBRATING			0x2e
#define BTEVT_EXTENDED_RESULT			0x2f
#define BTEVT_ENCRYPTION_KEY_REFRESH		0x30
#define BTEVT_IO_CAPABILITY_REQ			0x31
#define BTEVT_IO_CAPABILITY_RSP			0x32
#define BTEVT_USER_CONFIRM_REQ			0x33
#define BTEVT_USER_PASSKEY_REQ			0x34
#define BTEVT_REMOTE_OOB_DATA_REQ		0x35
#define BTEVT_SIMPLE_PAIRING_COMPL		0x36
#define BTEVT_LINK_SUPERVISION_TO_CHANGED	0x38
#define BTEVT_ENHANCED_FLUSH_COMPL		0x39
#define BTEVT_USER_PASSKEY_NOTIFICATION		0x3b
#define BTEVT_KEYPRESS_NOTIFICATION		0x3c
#define BTEVT_REMOTE_FEATURES_NOTIFICATION	0x3d
#define BTEVT_BT_LOGO				0xfe
#define BTEVT_VENDOR				0xff

/*
 * Bluetooth error codes
 */
#define BTERR_UNKNOW_HCI_COMMAND		0x01
#define BTERR_UNKNOW_CONN_ID			0x02
#define BTERR_HW_FAILURE			0x03
#define BTERR_PAGE_TIMEOUT			0x04
#define BTERR_AUTH_FAILURE			0x05
#define BTERR_PIN_KEY_MISSING			0x06
#define BTERR_NOMEM				0x07
#define BTERR_CONN_TIMEOUT			0x08
#define BTERR_CONN_LIMIT			0x09
#define BTERR_SCO_LIMIT				0x0A
#define BTERR_CONN_EXISTS			0x0B
#define BTERR_COMMAND_DISALLOWED		0x0C
#define BTERR_CONN_REJ_NOMEM			0x0D
#define BTERR_CONN_REJ_SECU			0x0E
#define BTERR_CONN_REJ_BDADDR			0x0F
#define BTERR_CONN_ACC_TIMEOUT			0x10
#define BTERR_UNSUPPORTED			0x11
#define BTERR_INVALID_HCI_COMMAND		0x12
#define BTERR_CONN_TERM_REMOTE			0x13
#define BTERR_CONN_TERM_NOMEM			0x14
#define BTERR_CONN_TERM_PWOFF			0x15
#define BTERR_CONN_TERM_HOST			0x16
#define BTERR_REPEATED_ATTEMPS			0x17
#define BTERR_PAIRING_DISALLOWED		0x18
#define BTERR_LMP_UNKNOW_PDU			0x19
#define BTERR_LMP_UNSUPPORTED			0x1A
#define BTERR_SCO_OFFSET_REJ			0x1B
#define BTERR_SCO_INTERVAL_REJ			0x1C
#define BTERR_SCO_MODE_REJ			0x1D
#define BTERR_LMPLL_INVALID			0x1E
#define BTERR_UNKNOW				0x1F
/* XXX ... */
