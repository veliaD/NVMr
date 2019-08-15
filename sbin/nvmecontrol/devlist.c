/*-
 * SPDX-License-Identifier: BSD-2-Clause-FreeBSD
 *
 * Copyright (C) 2012-2013 Intel Corporation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#include <sys/cdefs.h>
__FBSDID("$FreeBSD$");

#include <sys/param.h>

#include <err.h>
#include <errno.h>
#include <fcntl.h>
#include <paths.h>
#include <stddef.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <dirent.h>
#include <sysexits.h>

#include "nvmecontrol.h"

static void
devlist_usage(void)
{
	fprintf(stderr, "usage:\n");
	fprintf(stderr, DEVLIST_USAGE);
	exit(1);
}

static inline uint32_t
ns_get_sector_size(struct nvme_namespace_data *nsdata)
{
	uint8_t flbas_fmt, lbads;

	flbas_fmt = (nsdata->flbas >> NVME_NS_DATA_FLBAS_FORMAT_SHIFT) &
		NVME_NS_DATA_FLBAS_FORMAT_MASK;
	lbads = (nsdata->lbaf[flbas_fmt] >> NVME_NS_DATA_LBAF_LBADS_SHIFT) &
		NVME_NS_DATA_LBAF_LBADS_MASK;

	return (1 << lbads);
}

void
devlist(int argc, char *argv[])
{
	struct nvme_controller_data	cdata;
	struct nvme_namespace_data	nsdata;
	char				name[64], *eptr;
	uint8_t				mn[64];
	uint32_t			i;
	int				ch, ctrlr, fd, found, ret;
	DIR				*dirp;
	struct dirent			*dp;
	size_t				prfxlen;

	while ((ch = getopt(argc, argv, "")) != -1) {
		switch ((char)ch) {
		default:
			devlist_usage();
		}
	}

	ctrlr = -1;
	found = 0;
	dirp = opendir(_PATH_DEV);
	if (dirp == NULL) {
		err(EX_OSFILE, "Could not open dir "_PATH_DEV);
	}

	prfxlen = strlen(NVME_CTRLR_PREFIX);
	while (dp = readdir(dirp), dp != NULL) {
		if (strncmp(dp->d_name, NVME_CTRLR_PREFIX, prfxlen) != 0) {
			continue;
		}

		ctrlr = (int)strtol(dp->d_name + prfxlen, &eptr, 10);
		if ((errno != 0) || (*eptr != '\0')) {
			continue;
		}

		found++;

		ret = open_dev(dp->d_name, &fd, 0, 0);

		if (ret != 0) {
			warnx("could not open "_PATH_DEV"%s\n", dp->d_name);
				continue;
		}

		read_controller_data(fd, &cdata);
		nvme_strvis(mn, cdata.mn, sizeof(mn), NVME_MODEL_NUMBER_LENGTH);
		printf("%14s: %s\n", dp->d_name, mn);

		for (i = 0; i < cdata.nn; i++) {
			sprintf(name, "%s%s%d", dp->d_name, NVME_NS_PREFIX,
			    i+1);
			read_namespace_data(fd, i+1, &nsdata);
			if (nsdata.nsze == 0)
				continue;
			printf("  %10s (%lldMB)\n",
				name,
				nsdata.nsze *
				(long long)ns_get_sector_size(&nsdata) /
				1024 / 1024);
		}

		close(fd);
	}

	closedir(dirp);

	if (found == 0)
		printf("No NVMe controllers found.\n");

	exit(1);
}
