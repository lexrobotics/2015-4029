FILES = ["../test/servotest.c"]
CONFIG = "config.h"
DELIMETER = "//Inserted by auto-config.py script, do not modify lines from this comment up\n"

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