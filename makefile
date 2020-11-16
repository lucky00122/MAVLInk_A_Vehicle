# makefile

RESULT_FILE=result
SOURCE=mavlink_udp.c mavlink_udp.h

INCLUDE_PATH=$(shell pwd)/include/common



# Build
$(RESULT_FILE): $(SOURCE) 
	gcc -std=c99 -I $(INCLUDE_PATH) -o $(RESULT_FILE) $(SOURCE)



# Some make commands
.PHONY: rebuild
rebuild:
	make clean
	make

.PHONY: clean
clean:
	rm $(RESULT_FILE)