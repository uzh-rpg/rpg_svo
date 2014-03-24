#!/usr/bin/python

import subprocess, re

def getCpuInfo():
	command = "cat /proc/cpuinfo"
	all_info = subprocess.check_output(command, shell=True).strip()
	for line in all_info.split("\n"):
		if "model name" in line:
			return re.sub( ".*model name.*:", "", line,1)