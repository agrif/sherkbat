obj-m += src/sherkbat.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules
	mv src/sherkbat.ko ./

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -f sherkbat.ko
