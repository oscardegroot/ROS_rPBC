all:
	gcc -c ../usb-comm.c ../elisa3-lib.c
	ar -r libelisa3.a usb-comm.o elisa3-lib.o

clean:
	rm *.a
	rm *.o