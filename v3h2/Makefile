obj-m :=cpurttmod2_v3h2.o
cpurttmod2_v3h2-objs := smoni_api.o cpurttdrv.o

SRC := $(shell pwd)

all:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC)

modules_install:
	$(MAKE) -C $(KERNEL_SRC) M=$(SRC) modules_install

clean:
	rm -f *.o *.a *~ core .depend .*.cmd *.ko *.mod*
	rm -f Module.markers Module.symvers modules.order
	rm -rf .tmp_versions Modules.symvers

