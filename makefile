#
#   Makefile fuer alarmd
#

alarmd_9: alarmd_9.c
	gcc -lm alarmd_9.c -o alarmd_9

saverfid: saverfid.c
	gcc -lm saverfid.c -o saverfid

alarmd: alarmd.c
	gcc -lm alarmd.c -o alarmd

clean:
	rm -f *.o
	rm -f alarmd
	sudo rm -f /usr/local/sbin/alarmd

install:
	sudo rm -f /usr/local/sbin/alarmd
	sudo cp alarmd /usr/local/sbin/
