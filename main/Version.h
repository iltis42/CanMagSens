/*
 * Version.h
 *
 *  Created on: Feb 10, 2019
 *      Author: iltis
 */
#pragma once


class Version {
public:
	Version();
	virtual ~Version();
	static char _version[32];
	static char *version()  { return _version; };

	static constexpr int RELEASE_NR = 2;
};

