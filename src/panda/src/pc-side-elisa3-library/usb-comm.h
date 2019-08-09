#include <stdio.h>

static int find_nrf_device(void);

int usb_send(char* data, int nbytes);

int usb_receive(char* data, int nbytes);

int openCommunication();

void closeCommunication();


