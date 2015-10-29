/*
 * bq24261_charger.h: platform data structure for bq24261 driver
 *
 * Copyright (C) 2014 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#ifndef __BQ24261_CHARGER_H__
#define __BQ24261_CHARGER_H__

struct bq24261_platform_data {
	int def_cc;	/* in mA */
	int def_cv;	/* in mV */
	int iterm;	/* in mA */
	int max_cc;	/* in mA */
	int max_cv;	/* in mV */
	int min_temp;	/* in DegC */
	int max_temp;	/* in DegC */
	int thermal_sensing;
};

#endif
