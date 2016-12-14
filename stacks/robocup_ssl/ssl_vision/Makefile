OSTYPE ?= $(shell uname)

ifeq ($(OSTYPE),linux)
OSTYPE=Linux
endif

ifneq ($(findstring MINGW,$(OSTYPE)),)
OSTYPE=Windows
endif

ifeq ($(OSTYPE),msys)
OSTYPE=Windows
endif

ifeq ($(OSTYPE),Window)
ROSMAKE=echo 'ROS controllers are not supported under Windows'
endif

ifeq ($(OSTYPE),Darwin)
ROSMAKE=echo 'ROS controllers not compiled under Mac OS X'
endif

ifeq ($(OSTYPE),Linux)
ROSMAKE=rosmake
endif

compil: ssl_vision

ssl_vision: src/main.cpp
	@cp Makefile Makefile.backup
	@cp Makefile.ros Makefile
	@-$(ROSMAKE)
	@mv Makefile.backup Makefile
	@cp bin/ssl_vision ../ssl_wsim/controllers/ssl_vision/ssl_vision
#	@rm -rf bin

clean:
	@rm -rf bin build ../ssl_wsim/controllers/ssl_vision/ssl_vision