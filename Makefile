#
#   Makefile fuer alarmd
#

alarmd_23: alarmd_23.c
	gcc -Wall -pthread -lm alarmd_23.c gpiolib.c -o alarmd_23 -L.. libiniparser.a

parse: parse.c
	gcc -lm parse.c -o parse -L.. libiniparser.a

gpio_in: gpio_in.c
	gcc gpio_in.c gpiolib.c -o gpio_in

gpio_in_flanke: gpio_in_flanke.c
	gcc gpio_in_flanke.c gpiolib.c -o gpio_in_flanke

pthread_test: pthread_test.c
	gcc -Wall -pthread pthread_test.c -o pthread_test

alarmd: alarmd.c
	gcc -Wall -pthread -lm alarmd.c gpiolib.c -o alarmd -L.. libiniparser.a

clean:
	rm -f *.o
	rm -f alarmd
	rm -f /usr/local/sbin/alarmd

install:
	rm -f /usr/local/sbin/alarmd
	cp alarmd /usr/local/sbin/
