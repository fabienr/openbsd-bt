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
#include <sys/device.h>

/* bt hci cmd packet, header 3, data max 255, total 258 bytes */ 
struct bt_cmd {
	union {
		struct {
			u_int8_t ocf;
			u_int8_t ogf;
		} u;
		u_int16_t code;
	}		op;
	u_int8_t	length;
	u_int8_t	data[255];
};

/* bt hci acl packet, header 4, data max 27, total 31 bytes
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
struct bt_acl {
	u_int16_t	h_f;
	u_int16_t	length;
	u_int8_t	data[27];
};

/* bt hci sco packet, header 3, data max 28, total 31 bytes
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
struct bt_sco {
	u_int16_t	h_f;
	u_int8_t	length;
	u_int8_t	data[28];
};

/* bt hci evt packet, header 2, data max 255, total 257 bytes */
struct bt_evt {
	u_int8_t	code;
	u_int8_t	length;
	u_int8_t	data[255];
};

struct bthci_softc;

struct bthci_ops {
	void		(*cmd)(struct bthci_softc *, struct bt_cmd *);
	void		(*acl)(struct bthci_softc *, struct bt_acl *);
	void		(*sco)(struct bthci_softc *, struct bt_sco *);
};

struct bthci_softc {
	struct device sc_dev;
	struct bthci_ops ops;
};
