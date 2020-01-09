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

#include <util.h>
#include <unistd.h>
#include <stdio.h>
#include <fcntl.h>
#include <err.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/syslimits.h>

#include <bluetooth/bluetooth.h>

#include "btstrings.h"

#define NDEV	4
#define DEVNAME	"bluetooth"

#define BTCONFIG_OPEN		1
#define BTCONFIG_QUERY		2
#define BTCONFIG_INQUIRY	3
#define BTCONFIG_MATCH		4
#define BTCONFIG_ATTACH		5
#define BTCONFIG_DETACH		6
#define BTCONFIG_RUN		7

int bthci_open(int);

void btconfig_open(int, int);
void btconfig_query(int, int);

static void __dead
usage(void)
{
	fprintf(stderr, "usage: btconfig [-oqimadr] [device|unit]\n");
	exit(1);
}

int
main(int argc, char **argv)
{
	int dev, unit, cmd = 0;
	char *path, *realpath, *s, ch;

	if (argc == 1) {
		for (unit = 0; unit < NDEV; unit++) {
			dev = bthci_open(unit);
			if (dev < 0) {
				warn("bthci_open(%d)", unit);
				continue;
			}
			btconfig_open(dev, unit);
			close(dev);
		}
		return (0);
	}
	while ((ch = getopt(argc, argv, "oq")) != -1) {
		switch (ch) {
		case 'o':
			cmd = BTCONFIG_OPEN;
			break;
		case 'q':
			cmd = BTCONFIG_QUERY;
			break;
		default:
			usage();
			break;
		}
	}
	argv += optind;
	argc -= optind;
	if (argc > 1 || cmd == 0)
		usage();
	unit = 0;
	if (*argv) {
		unit = strtol(*argv, &s, 10);
		if (*argv[0] != '\0' && *s == '\0')
			dev = bthci_open(unit);
		else {
			dev = opendev(*argv, O_RDWR, 0, &realpath);
			/* XXX get unit number */
		}
	} else
		dev = bthci_open(unit);
	if (dev < 0) {
		warn("invalid device");
		usage();
		exit(1);
	}
	switch(cmd) {
	case BTCONFIG_OPEN:
		/* open and display controller summary */
		btconfig_open(dev, unit);
		break;
	case BTCONFIG_QUERY:
		/* open and display controller extended information */
		break;
	};
	return (0);
}

int
bthci_open(int minor)
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
btconfig_open(int dev, int u)
{
	struct bluetooth_info info;
	int err, i;
	err = ioctl(dev, DIOCBTINFO, &info);
	if (err) {
		warn("DIOCBTINFO");
		return;
	}
	printf("%0X", u);
	for (i = 0; i < BT_ADDR_LEN; i++)
		printf("%s%0X", ((i)?":":"# "), info.bt_addr.bdaddr[i]);
	printf(" manufacturer %s\n"
	    "hci %s (rev %d)\n"
	    "lmp %s (rev %d)\n",
	    btmanufacturer(info.bt_manufacturer),
	    bthciversion(info.hci_version), info.hci_revision,
	    btlmpversion(info.lmp_version), info.lmp_revision);
}
