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

/* USB endpoints addresses. */
#define UBT_CMD_OUT	(UE_DIR_OUT | 0) /* default_pipe, handle by usbd */
#define UBT_EVT_IN	(UE_DIR_IN  | 1)
#define UBT_ACL_OUT	(UE_DIR_OUT | 2)
#define UBT_ACL_IN	(UE_DIR_IN  | 2)
#define UBT_SCO_OUT	(UE_DIR_OUT | 3)
#define UBT_SCO_IN	(UE_DIR_IN  | 3)

#define UBT_TIMEOUT	1000
#define UBT_CMD_TIMEOUT	UBT_TIMEOUT
