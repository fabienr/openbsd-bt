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

#include <stdio.h>
#include <fcntl.h>
#include <err.h>
#include <stdlib.h>
#include <sys/ioctl.h>

#include <bluetooth/bluetooth.h>

#define NDEV	4
#define DEVNAME	"bluetooth"

int btopen(int);
void btinfo(int);

int
main(int argc, char **argv)
{
	int dev;
	int i;

	for (i = 0; i < NDEV; i++) {
		dev = btopen(i);
		if (dev < 0) {
			warn("btopen(%d)", i);
			continue;
		}
		btinfo(dev);
	}
	return (0);
}

int
btopen(int minor)
{
	char *path = NULL;
	int dev;
	asprintf(&path, "/dev/%s%d", DEVNAME, minor);
	if (path == NULL)
		return (-1);
	dev = open(path, O_RDWR);
	free(path);
	return (dev);
}

void
btinfo(int dev)
{
	struct bt_hci_info info;
	int err;
	err = ioctl(dev, DIOCBTINFO, &info);
	if (err) {
		warn("DIOCBTINFO");
		return;
	}
	printf("manufacturer %d, hci %d(%d), lmp %d(%d)\n",
	    info.manufacturer, info.hci_version, info.hci_revision,
	    info.lmp_version, info.lmp_revision);
}
