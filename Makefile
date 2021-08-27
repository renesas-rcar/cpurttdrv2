SRC_V3H1 := $(shell pwd)/v3h1
SRC_V3H2 := $(shell pwd)/v3h2
SRC_V3M2 := $(shell pwd)/v3m2

all:
	$(MAKE) -C $(SRC_V3H1)
	$(MAKE) -C $(SRC_V3H2)
	$(MAKE) -C $(SRC_V3M2)

modules_install:
	$(MAKE) -C $(SRC_V3H1) modules_install
	$(MAKE) -C $(SRC_V3H2) modules_install
	$(MAKE) -C $(SRC_V3M2) modules_install

clean:
	$(MAKE) -C $(SRC_V3H1) clean
	$(MAKE) -C $(SRC_V3H2) clean
	$(MAKE) -C $(SRC_V3M2) clean

PHONY: all modules_install clean
