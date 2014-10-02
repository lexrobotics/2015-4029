import glob

FILES = glob.glob("*/*.c")
CONFIG = "config.h"
DELIMETER = "//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//\n"

for path in FILES:
	originalFile = []
	configFile = []
	f = open(path, 'r')
	for line in f:
		if line == DELIMETER: 
			originalFile = [] #reached the end of config, start over again
		else:
			originalFile.append(line)
	f.close()

	f = open(CONFIG, 'r')
	for line in f:
		configFile.append(line)
	f.close()

	f = open(path, 'w')
	for line in configFile:
		f.write(line)
	f.write(DELIMETER)
	for line in originalFile:
		f.write(line)
	f.close()