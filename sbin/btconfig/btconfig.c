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
#include "btutil.h"

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
void btconfig_inquiry(int, int);

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
	while ((ch = getopt(argc, argv, "oqi")) != -1) {
		switch (ch) {
		case 'o':
			cmd = BTCONFIG_OPEN;
			break;
		case 'q':
			cmd = BTCONFIG_QUERY;
			break;
		case 'i':
			cmd = BTCONFIG_INQUIRY;
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
		btconfig_open(dev, unit);
		printf("\n");
		btconfig_query(dev, unit);
		break;
	case BTCONFIG_INQUIRY:
		/* start inquiry and display devices */
		btconfig_inquiry(dev, unit);
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
btconfig_open(int dev, int unit)
{
	struct bluetooth_info info;
	int err, i;
	err = ioctl(dev, DIOCBTINFO, &info);
	if (err) {
		btwarn("DIOCBTINFO");
		return;
	}
	printf("%0X#", unit);
	for (i = BT_ADDR_LEN; --i >= 0;)
		printf("%0X%c", info.bt_addr.b[i], (i)?':':' ');
	printf(" %s\n"
	    "HCI %s (rev %d)\n"
	    "LMP %s (rev %d)\n"
	    "ACL buffer of %d packets with %d bytes payload\n"
	    "SCO buffer of %d packets with %d bytes payload\n",
	    btmanufacturer(info.bt_manufacturer),
	    bthciversion(info.hci_version), info.hci_revision,
	    btlmpversion(info.lmp_version), info.lmp_revision,
	    info.acl_bufferlen, info.acl_size,
	    info.sco_bufferlen, info.sco_size);
}

void
btconfig_query(int dev, int unit)
{
	struct bluetooth_info_extended info;
	int err, i, j;
	const char *desc;

	err = ioctl(dev, DIOCBTINFOEXT, &info);
	if (err) {
		btwarn("DIOCBTINFOEXT");
		return;
	}
	printf("*** Flow control lag : %d bytes\n", info.flow_control_lag);
	printf("*** Features not supported :\n");
	for (i = 0; i < BT_EXTENDED_PAGE_MAX*BT_FEATURES_BITMASK_LEN; i++) {
		for (j = 0; j < 8; j++) {
			desc = btfeaturebitmask(i, j);
			if (desc[0] == 'R') /* Reserved for future use */
				continue;
			if ((((uint8_t *)info.features)[i] & (0x01<<j)) == 0)
				printf("%s : No\n", desc);
		}
	}
	printf("*** Commands not supported :\n");
	for (i = 0; i < BT_COMMANDS_BITMASK_LEN; i++) {
		for (j = 0; j < 8; j++) {
			desc = btcommandbitmask(i, j);
			if (desc[0] == 'R') /* Reserved for future use */
				continue;
			if ((info.commands[i] & (0x01<<j)) == 0)
				printf("%s : No\n", desc);
		}
	}
}

void
btconfig_inquiry(int dev, int unit)
{
	struct bluetooth_device *device;
	int i;
	ssize_t k;

	if (ioctl(dev, DIOCBTINQUIRY, NULL)) {
		btwarn("DIOCBTINQUIRY");
		return;
	}
	device = malloc(sizeof(*device));
	while((k = read(dev, device, sizeof(*device))) == sizeof(*device)) {
		printf("%0X#%0X ", unit, device->unit);
		for (i = BT_ADDR_LEN; --i >= 0;)
			printf("%0X%c", device->bt_addr.b[i], (i)?':':' ');
		printf("%0X-%0X-%0X, %s\n", device->bt_class.c[0],
		    device->bt_class.c[1], device->bt_class.c[2],
		    device->name);
		fflush(stdout);
	}
	free(device);
	if (k != 0) {
		btwarn("read device");
		return;
	}
	return;
}
