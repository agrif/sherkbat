obj-m += src/sherkbat.o

all: sherkbat.dtbo
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
	rm -f sherkbat.dtbo

sherkbat.dtbo: sherkbat.dts
	dtc -@ -I dts -O dtb -o sherkbat.dtbo sherkbat.dts
