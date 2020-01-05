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

#define BT_TIMEOUT	SEC_TO_NSEC(5)

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
};

void bluetooth_attach(struct bluetooth_softc *,  struct bthci *);

/* bt hci cmd packet, header 3, data max 255, total 258 bytes */
struct bt_cmd_head {
	uint16_t	op;
	uint8_t		len;
};
struct bt_cmd {
	struct bt_cmd_head	head;
	uint8_t			data[255];
};

/* bt hci event packet, header 2, data max 255, total 257 bytes */
struct bt_evt_head {
	uint8_t		op;
	uint8_t		len;
};
struct bt_evt {
	struct bt_evt_head	head;
	uint8_t			data[255];
};

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
};
struct bt_acl {
	struct bt_acl_head	head;
	uint8_t			data[27];
};

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
};
struct bt_sco {
	struct bt_sco_head	head;
	uint8_t			data[28];
};

#define BTERR_UNKNOW_HCI_COMMAND	0x01
#define BTERR_UNKNOW_CONN_ID		0x02
#define BTERR_HW_FAILURE		0x03
#define BTERR_PAGE_TIMEOUT		0x04
#define BTERR_AUTH_FAILURE		0x05
#define BTERR_PIN_KEY_MISSING		0x06
#define BTERR_NOMEM			0x07
#define BTERR_CONN_TIMEOUT		0x08
#define BTERR_CONN_LIMIT		0x09
#define BTERR_SCO_LIMIT			0x0A
#define BTERR_CONN_EXISTS		0x0B
#define BTERR_COMMAND_DISALLOWED	0x0C
#define BTERR_CONN_REJ_NOMEM		0x0D
#define BTERR_CONN_REJ_SECU		0x0E
#define BTERR_CONN_REJ_BDADDR		0x0F
#define BTERR_CONN_ACC_TIMEOUT		0x10
#define BTERR_UNSUPPORTED		0x11
#define BTERR_INVALID_HCI_COMMAND	0x12
#define BTERR_CONN_TERM_REMOTE		0x13
#define BTERR_CONN_TERM_NOMEM		0x14
#define BTERR_CONN_TERM_PWOFF		0x15
#define BTERR_CONN_TERM_HOST		0x16
#define BTERR_REPEATED_ATTEMPS		0x17
#define BTERR_PAIRING_DISALLOWED	0x18
#define BTERR_LMP_UNKNOW_PDU		0x19
#define BTERR_LMP_UNSUPPORTED		0x1A
#define BTERR_SCO_OFFSET_REJ		0x1B
#define BTERR_SCO_INTERVAL_REJ		0x1C
#define BTERR_SCO_MODE_REJ		0x1D
#define BTERR_LMPLL_INVALID		0x1E
#define BTERR_UNKNOW			0x1F
/* XXX ... */
