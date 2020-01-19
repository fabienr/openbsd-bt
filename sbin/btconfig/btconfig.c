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
#include <string.h>
#include <sys/ioctl.h>
#include <sys/syslimits.h>

#include "btconfig.h"
#include "btstrings.h"
#include "btutil.h"

#define NDEV	4
#define DEVNAME	"bluetooth"

#define BTCONFIG_VERSION	1
#define BTCONFIG_INQUIRY	2
#define BTCONFIG_MATCH		3
#define BTCONFIG_ATTACH		4
#define BTCONFIG_DETACH		5
#define BTCONFIG_RUN		6

#define BT_UNIT_SEP		'#'
#define BT_ADDR_SEP		':'

int bthci_open(int);

void btconfig_version(int, int, int);
void btconfig_inquiry(int, int);
int btconfig_match(int, int, struct bluetooth_device_match *, int);

struct btconfig_hcis hcis;

static void __dead
usage(void)
{
	fprintf(stderr, "usage: btconfig [-vimadr] (-v(v))"
	    " [/dev/path|hci(#device)|XX:XX:XX:XX:XX:XX] (XX:XX:XX:XX:XX:XX)\n");
	exit(1);
}

int
main(int argc, char **argv)
{
	struct btconfig_hci *hci;
	struct bluetooth_device_match device;
	char *path, *realpath, *s, ch;
	int i, error = 0, cmd = 0, dev = -1, dev_minor = -1, verbose = 0;

	SLIST_INIT(&hcis);
	memset(&device, 0, sizeof(device));
	for (i = 0; i < NDEV; i++) {
		dev = bthci_open(i);
		if (argc == 1) {
			if (dev < 0) {
				warn("%X# open", i);
				continue;
			}
			btconfig_version(dev, i, 0);
			continue;
		}
		if (dev < 0)
			continue;
		if (dev_minor < 0)
			dev_minor = i;
		if ((hci = malloc(sizeof(*hci))) == NULL)
			err(1, NULL);
		error = ioctl(dev, DIOCBTVERSION, &hci->version);
		if (error) {
			btwarn("%X# DIOCBTVERSION", i);
			free(hci);
			hci = NULL;
			error = 0;
			continue;
		}
		hci->unit = i;
		SLIST_INSERT_HEAD(&hcis, hci, sl);
		close(dev);
	}
	if (argc == 1)
		goto exit;

	while ((ch = getopt(argc, argv, "vim")) != -1) {
		switch (ch) {
		case 'v':
			if (cmd)
				verbose++;
			else
				cmd = BTCONFIG_VERSION;
			break;
		case 'i':
			cmd = BTCONFIG_INQUIRY;
			break;
		case 'm':
			cmd = BTCONFIG_MATCH;
			break;
		default:
			usage();
			break;
		}
	}
	argv += optind;
	argc -= optind;
	if (argc > 2 || cmd == 0)
		usage();

	if (argc == 0) {
		dev = bthci_open(dev_minor);
		if (dev < 0) {
			warn("no controller found");
			usage();
		}
	} else {
		dev = dev_minor = -1;
	}
	for (; argc && *argv && *argv[0] != '\0'; argv++, argc--) {
		if (dev >= 0 && (device.unit > 0 || device.bt_addr.b[0] != 0)) {
			warnx("too many arguments, hci and device already set");
			usage();
		}
		i = (int)strtol(*argv, &s, 16);
		if (*argv[0] != '\0' && (*s == '\0' ||
		    *s == BT_UNIT_SEP || *s == BT_ADDR_SEP)) {
			if (*s == '\0' || *s == BT_UNIT_SEP) {
				dev = bthci_open(i);
				if (dev < 0) {
					warn("bthci_open %d / %s", i, *argv);
					usage();
				}
				dev_minor = i;
			}
			switch (*s) {
			case '\0':
				continue;
			case BT_UNIT_SEP:
				if (*++s == '\0')
					continue;
				device.unit = (int)strtol(s, &s, 16);
				if (*s == '\0' && device.unit > 0)
					continue;
				warnx("invalid bluetooth unit format,"
				    " xx(#xx) != %s", *argv);
				usage();
			case BT_ADDR_SEP:
				/* read addr in reverse order, same as printing */
				device.bt_addr.b[BT_ADDR_LEN - 1] = i;
				i = BT_ADDR_LEN - 1;
				do {
					i--;
					s++;
					if (*s == '\0')
						break;
					device.bt_addr.b[i] = (uint8_t)strtol(
					    s,&s,16);
				} while(*s == BT_ADDR_SEP && i);
				if (*s != '\0' || i) {
					warnx("invalid bluetooth address format,"
					    " xx:xx:xx:xx:xx:xx != %s", *argv);
					usage();
				}
				if (dev >= 0)
					continue;
				SLIST_FOREACH(hci, &hcis, sl) {
					for (i = BT_ADDR_LEN; --i >= 0;)
						if (hci->version.bt_addr.b[i]
						    != device.bt_addr.b[i])
							break;
					if (i < 0)
						break;
				}
				memset(&device.bt_addr, 0, sizeof(device.bt_addr));
				if (hci == NULL) {
					warnx("unknow bluetooth hci address %s",
					    *argv);
					usage();
				}
				dev = bthci_open(hci->unit);
				if (dev < 0) {
					warn("bthci_open %d / %s", i, *argv);
					usage();
				}
				dev_minor = hci->unit;
			}
		} else {
			dev = opendev(*argv, O_RDWR, 0, &realpath);
			if (dev < 0) {
				warn("invalid device %s", *argv);
				usage();
			}
			dev_minor = -1;
		}
	}

	switch(cmd) {
	case BTCONFIG_VERSION:
		/* open and display controller summary */
		btconfig_version(dev, dev_minor, verbose);
		break;
	case BTCONFIG_INQUIRY:
		/* start inquiry and display devices */
		btconfig_inquiry(dev, dev_minor);
		break;
	case BTCONFIG_MATCH:
		/* XXX debug */
		printf("match on controller %d, ", dev_minor);
		if (device.unit)
			printf("device %d", device.unit);
		else
			for (i = BT_ADDR_LEN; --i >= 0;)
				printf("%02X%c", device.bt_addr.b[i], (i)?':':' ');
		printf(" : \r\n");
		if (device.unit == 0 && device.bt_addr.b[0] == 0) {
			warnx("-m : remote device unit or addresse needed");
			usage();
		}
		error = btconfig_match(dev, dev_minor, &device, verbose);
		break;
	};

 exit:
	while (!SLIST_EMPTY(&hcis)) {
		hci = SLIST_FIRST(&hcis);
		SLIST_REMOVE_HEAD(&hcis, sl);
		free(hci);
	}
	return (error);
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
btconfig_version(int dev, int dev_minor, int verbose)
{
	struct bluetooth_version version;
	const char *desc;
	int error, i, j;
	error = ioctl(dev, DIOCBTVERSION, &version);
	if (error) {
		btwarn("DIOCBTVERSION");
		return;
	}
	printf("%X# ", (dev_minor<0)?0xEE:dev_minor);
	for (i = BT_ADDR_LEN; --i >= 0;)
		printf("%02X%c", version.bt_addr.b[i], (i)?':':' ');
	printf("%s\n"
	    "HCI %s (rev %d)\n"
	    "LMP %s (rev %d)\n"
	    "ACL buffer of %d packets with %d bytes payload\n"
	    "SCO buffer of %d packets with %d bytes payload\n",
	    btmanufacturer(version.bt_manufacturer),
	    bthciversion(version.hci_version), version.hci_revision,
	    btlmpversion(version.lmp_version), version.lmp_revision,
	    version.acl_bufferlen, version.acl_size,
	    version.sco_bufferlen, version.sco_size);
	if (verbose == 0)
		goto out;
	printf("*** Features supported :\n");
	for (i = 0; i < BT_EXTENDED_PAGE_MAX*BT_FEATURES_BITMASK_LEN; i++) {
		/* XXX debug */
		/* printf("parse byte %d : %02X\r\n",
		 *     i,((uint8_t *)version.features)[i]);
		 */
		for (j = 0; j < 8; j++) {
			desc = btfeaturebitmask(i, j);
			if (desc[0] == 'R') /* Reserved for future use */
				continue;
			if ((((uint8_t *)version.features)[i] & (0x01<<j)) == 0)
				printf("%s : No\n", desc);
			else
				printf("%s : Yes\n", desc);
		}
	}
	printf("*** Flow control lag : %d bytes\n", version.flow_control_lag);
	if (verbose <= 1)
		goto out;
	printf("*** Commands supported :\n");
	for (i = 0; i < BT_COMMANDS_BITMASK_LEN; i++) {
		/* XXX debug */
		/*printf("parse byte %d : %02X\r\n", i, version.commands[i]);*/
		for (j = 0; j < 8; j++) {
			desc = btcommandbitmask(i, j);
			if (desc[0] == 'R') /* Reserved for future use */
				continue;
			if ((version.commands[i] & (0x01<<j)) == 0)
				printf("%s : No\n", desc);
			else
				printf("%s : Yes\n", desc);
		}
	}
 out:
	fflush(stdout);
}

void
btconfig_inquiry(int dev, int dev_minor)
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
		printf("%X#%X ", (dev_minor<0)?0xEE:dev_minor, device->unit);
		for (i = BT_ADDR_LEN; --i >= 0;)
			printf("%02X%c", device->bt_addr.b[i], (i)?':':' ');
		printf("%02X-%02X-%02X, %s\n", device->bt_class.c[0],
		    device->bt_class.c[1], device->bt_class.c[2],
		    device->name);
		fflush(stdout);
	}
	free(device);
	if (k < 0)
		btwarn("inquiry");
	return;
}

int
btconfig_match(int dev, int dev_minor, struct bluetooth_device_match *device,
    int verbose)
{
	const char *desc;
	int error, i, j;
	error = ioctl(dev, DIOCBTMATCH, device);
	if (error) {
		btwarn("DIOCBTMATCH");
		return (error);
	}
	printf("%X#%X ", (dev_minor<0)?0xEE:dev_minor, device->unit);
	for (i = BT_ADDR_LEN; --i >= 0;)
		printf("%02X%c", device->bt_addr.b[i], (i)?':':' ');
	printf("%s\n"
	    "LMP %s (rev %d)\n",
	    btmanufacturer(device->bt_manufacturer),
	    btlmpversion(device->lmp_version), device->lmp_revision);
	if (verbose == 0)
		goto out;
	printf("*** Features supported :\n");
	for (i = 0; i < BT_EXTENDED_PAGE_MAX*BT_FEATURES_BITMASK_LEN; i++) {
		for (j = 0; j < 8; j++) {
			desc = btfeaturebitmask(i, j);
			if (desc[0] == 'R') /* Reserved for future use */
				continue;
			if ((((uint8_t *)device->features)[i] & (0x01<<j)) == 0)
				printf("%s : No\n", desc);
			else
				printf("%s : Yes\n", desc);
		}
	}
	printf("*** Flow control lag : %d bytes\n", device->flow_control_lag);
 out:
	fflush(stdout);
	return (error);
}
