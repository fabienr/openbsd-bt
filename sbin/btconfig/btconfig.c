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

#define BTCONFIG_OPEN		1
#define BTCONFIG_QUERY		2
#define BTCONFIG_INQUIRY	3
#define BTCONFIG_MATCH		4
#define BTCONFIG_ATTACH		5
#define BTCONFIG_DETACH		6
#define BTCONFIG_RUN		7

#define BT_UNIT_SEP		'#'
#define BT_ADDR_SEP		':'

int bthci_open(int);

void btconfig_open(int, int);
void btconfig_query(int);
void btconfig_inquiry(int, int);
int btconfig_match(int, int, struct bluetooth_device_match *);

struct btconfig_hcis hcis;

static void __dead
usage(void)
{
	fprintf(stderr, "usage: btconfig [-oqimadr]"
	    " [/dev/path|hci(#device)|XX:XX:XX:XX:XX:XX] (XX:XX:XX:XX:XX:XX)\n");
	exit(1);
}

int
main(int argc, char **argv)
{
	struct btconfig_hci *hci;
	struct bluetooth_device_match device;
	char *path, *realpath, *s, ch;
	int i, error = 0, cmd = 0, dev = -1, dev_minor = -1;

	SLIST_INIT(&hcis);
	memset(&device, 0, sizeof(device));
	for (i = 0; i < NDEV; i++) {
		dev = bthci_open(i);
		if (dev < 0) {
			if (argc == 1)
				warn("bthci_open(%d)", i);
			continue;
		}
		if (dev_minor < 0)
			dev_minor = i;
		if ((hci = malloc(sizeof(*hci))) == NULL)
			err(1, NULL);
		error = ioctl(dev, DIOCBTINFO, &hci->info);
		if (error) {
			btwarn("DIOCBTINFO");
			free(hci);
			hci = NULL;
			error = 0;
			continue;
		}
		hci->unit = i;
		SLIST_INSERT_HEAD(&hcis, hci, sl);
		if (argc == 1)
			btconfig_open(dev, i);
		close(dev);
	}
	if (argc == 1)
		goto exit;

	while ((ch = getopt(argc, argv, "oqim")) != -1) {
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
			warn("invalid device");
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
						if (hci->info.bt_addr.b[i]
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
	case BTCONFIG_OPEN:
		/* open and display controller summary */
		btconfig_open(dev, dev_minor);
		break;
	case BTCONFIG_QUERY:
		/* open and display controller extended information */
		btconfig_open(dev, dev_minor);
		printf("\n");
		btconfig_query(dev);
		break;
	case BTCONFIG_INQUIRY:
		/* start inquiry and display devices */
		btconfig_inquiry(dev, dev_minor);
		break;
	case BTCONFIG_MATCH:
		/* XXX debug */
		printf("match on controller %d, device %d or ", dev_minor, device.unit);
		for (i = BT_ADDR_LEN; --i >= 0;)
			printf("%0X%c", device.bt_addr.b[i], (i)?':':' ');
		printf(" : \r\n");
		if (device.unit == 0 && device.bt_addr.b[0] == 0) {
			warnx("-m : remote device unit or addresse needed");
			usage();
		}
		error = btconfig_match(dev, dev_minor, &device);
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
btconfig_open(int dev, int dev_minor)
{
	struct bluetooth_info info;
	int error, i;
	error = ioctl(dev, DIOCBTINFO, &info);
	if (error) {
		btwarn("DIOCBTINFO");
		return;
	}
	printf("%0X# ", (dev_minor<0)?0xEE:dev_minor);
	for (i = BT_ADDR_LEN; --i >= 0;)
		printf("%0X%c", info.bt_addr.b[i], (i)?':':' ');
	printf("%s\n"
	    "HCI %s (rev %d)\n"
	    "LMP %s (rev %d)\n"
	    "ACL buffer of %d packets with %d bytes payload\n"
	    "SCO buffer of %d packets with %d bytes payload\n",
	    btmanufacturer(info.bt_manufacturer),
	    bthciversion(info.hci_version), info.hci_revision,
	    btlmpversion(info.lmp_version), info.lmp_revision,
	    info.acl_bufferlen, info.acl_size,
	    info.sco_bufferlen, info.sco_size);
	fflush(stdout);
}

void
btconfig_query(int dev)
{
	struct bluetooth_info_extended info;
	int error, i, j;
	const char *desc;

	error = ioctl(dev, DIOCBTINFOEXT, &info);
	if (error) {
		btwarn("DIOCBTINFOEXT");
		return;
	}
	printf("*** Flow control lag : %d bytes\n", info.flow_control_lag);
	printf("*** Features supported :\n");
	for (i = 0; i < BT_EXTENDED_PAGE_MAX*BT_FEATURES_BITMASK_LEN; i++) {
		/* XXX debug */
		/* printf("parse byte %d : %0X\r\n",
		 *     i,((uint8_t *)info.features)[i]);
		 */
		for (j = 0; j < 8; j++) {
			desc = btfeaturebitmask(i, j);
			if (desc[0] == 'R') /* Reserved for future use */
				continue;
			if ((((uint8_t *)info.features)[i] & (0x01<<j)) == 0)
				printf("%s : No\n", desc);
			else
				printf("%s : Yes\n", desc);
		}
	}
	printf("*** Commands supported :\n");
	for (i = 0; i < BT_COMMANDS_BITMASK_LEN; i++) {
		/* XXX debug */
		/*printf("parse byte %d : %0X\r\n", i, info.commands[i]);*/
		for (j = 0; j < 8; j++) {
			desc = btcommandbitmask(i, j);
			if (desc[0] == 'R') /* Reserved for future use */
				continue;
			if ((info.commands[i] & (0x01<<j)) == 0)
				printf("%s : No\n", desc);
			else
				printf("%s : Yes\n", desc);
		}
	}
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
		printf("%0X#%0X ", (dev_minor<0)?0xEE:dev_minor, device->unit);
		for (i = BT_ADDR_LEN; --i >= 0;)
			printf("%0X%c", device->bt_addr.b[i], (i)?':':' ');
		printf("%0X-%0X-%0X, %s\n", device->bt_class.c[0],
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
btconfig_match(int dev, int dev_minor, struct bluetooth_device_match *device)
{
	int error = 0;
	error = ioctl(dev, DIOCBTMATCH, device);
	if (error) {
		btwarn("DIOCBTMATCH");
		return (error);
	}
	return (error);
}
