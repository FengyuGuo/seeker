obj-m += pps_s.o
 
all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	gcc pps_s_main.c -o pps_s_main
clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
