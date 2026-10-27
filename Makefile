################################################################################
######################### User configurable parameters #########################
# filename extensions
CEXTS:=c
ASMEXTS:=s S
CXXEXTS:=cpp c++ cc

# probably shouldn't modify these, but you may need them below
ROOT=.
FWDIR:=$(ROOT)/firmware
BINDIR=$(ROOT)/bin
SRCDIR=$(ROOT)/src
INCDIR=$(ROOT)/include

WARNFLAGS+=-Wall -Wextra -Wpedantic -Wconversion
EXTRA_CFLAGS=
EXTRA_CXXFLAGS=

# Set to 1 to enable hot/cold linking
USE_PACKAGE:=1

# liblvgl starts a display task that draws the brain on its own. The V5
# program uses pros::screen for the splash and the selector, so that task
# is not linked. LLEMU stays on the host build.
EXCLUDE_COLD_LIBRARIES:=$(FWDIR)/liblvgl.a

# Set this to 1 to add additional rules to compile your project as a PROS library template
IS_LIBRARY:=0
# TODO: CHANGE THIS!
# Be sure that your header files are in the include directory inside of a folder with the
# same name as what you set LIBNAME to below.
LIBNAME:=libbest
VERSION:=1.0.0
# EXCLUDE_SRC_FROM_LIB= $(SRCDIR)/unpublishedfile.c
# this line excludes opcontrol.c and similar files
EXCLUDE_SRC_FROM_LIB+=$(foreach file, $(SRCDIR)/main,$(foreach cext,$(CEXTS),$(file).$(cext)) $(foreach cxxext,$(CXXEXTS),$(file).$(cxxext)))

# files that get distributed to every user (beyond your source archive) - add
# whatever files you want here. This line is configured to add all header files
# that are in the directory include/LIBNAME
TEMPLATE_FILES=$(INCDIR)/$(LIBNAME)/*.h $(INCDIR)/$(LIBNAME)/*.hpp

# Desktop simulation (src/host) is not part of the V5 image.
EXCLUDE_SRCDIRS=$(SRCDIR)/host

.DEFAULT_GOAL=quick

################################################################################
################################################################################
########## Nothing below this line should be edited by typical users ###########
-include ./common.mk

# common.mk picks up every firmware archive. Drop liblvgl from the hot link
# as well as the cold image, or its display task is still in the program.
LIBRARIES := $(filter-out $(FWDIR)/liblvgl.a,$(LIBRARIES))
COLD_LIBRARIES := $(filter-out $(FWDIR)/liblvgl.a,$(COLD_LIBRARIES))
ARCHIVE_TEXT_LIST := $(subst $(SPACE),$(COMMA),$(notdir $(basename $(LIBRARIES))))

# Desktop simulation. Defines ROBOT_HOST, links src/host, and leaves the
# physical chassis (TankDrive / XDrive / ParenthesisDrive) out of the link.
# The V5 image is `make quick` and does not define ROBOT_HOST.
HOST_CXX ?= clang++
# Windows clang++ defaults to x86_64-pc-windows-msvc. Pin the GNU triple so
# the desktop sim does not depend on the MSVC standard library or lld-link.
HOST_CXXFLAGS := --target=x86_64-w64-windows-gnu --std=gnu++26 -I./include -D_POSIX_THREADS -D_UNIX98_THREAD_MUTEX_ATTRIBUTES -D_POSIX_TIMERS -D_POSIX_MONOTONIC_CLOCK -D_USE_MATH_DEFINES -D_PROS_KERNEL_SUPPRESS_LLEMU_WARNING -DROBOT_HOST=1 -g -Os
HOST_SRCS := $(wildcard src/main.cpp src/control/*.cpp src/estimators/*.cpp src/logic/*.cpp src/math/*.cpp src/trajectory/*.cpp src/platform/*.cpp src/host/*.cpp src/auton/*.cpp)
HOST_LIBS := -fuse-ld=lld -luser32 -lgdi32

.PHONY: host
host: simulation_bin.exe

simulation_bin.exe: $(HOST_SRCS)
	$(HOST_CXX) $(HOST_CXXFLAGS) $(HOST_SRCS) $(HOST_LIBS) -o $@
