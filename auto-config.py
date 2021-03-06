import os, fnmatch

CONFIG = "config.c"
DELIMETER = "//*!!Code automatically generated by 'ROBOTC' configuration wizard               !!*//\n"

# http://stackoverflow.com/questions/2186525/use-a-glob-to-find-files-recursively-in-python
def find_files(directory, pattern):
    for root, dirs, files in os.walk(directory):
        for basename in files:
            if fnmatch.fnmatch(basename, pattern):
                filename = os.path.join(root, basename)
                yield filename

configFile = []
f = open(CONFIG, 'r')
for line in f:
	if not (line == DELIMETER.strip('\n') or line == DELIMETER):
		configFile.append(line)
f.close()

for path in find_files(".", "*.c"):
	print path
	originalFile = []
	f = open(path, 'r')
	for line in f:
		if line == DELIMETER: 
			originalFile = [] #reached the end of config, start over again
		else:
			originalFile.append(line)
	f.close()

	f = open(path, 'w')
	for line in configFile:
		f.write(line)
	f.write(DELIMETER)
	for line in originalFile:
		f.write(line)
	f.close()