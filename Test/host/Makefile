# Host test suite for log-and-stream-common.
#
#   make            build and run everything
#   make build      build only
#   make clean      remove the build directory
#   make test_rtc   build and run one suite
#
# CI runs this same Makefile (.github/workflows/host-tests.yml), so a suite that
# passes locally passes there, and adding a suite here adds it to CI. Nothing
# needs a cross-compiler, a HAL or a device: every module below is reachable on
# a PC either directly or through the stub platform in stubs/ - see
# stubs/README.md for what that is and how to extend it.
#
# TWO PLATFORMS, ONE LIBRARY. This repository is a submodule of both
# shimmer3-firmware (MSP430) and shimmer3r-firmware (STM32U5), and every change
# here ships to both. Suites listed in DUAL_PLATFORM_SUITES are therefore built
# and run TWICE, once with -DSHIMMER3 and once with -DSHIMMER3R, because the
# code under them differs between the two - LED colour widths, which revision
# gates exist, which helpers are compiled in at all. That doubling is the point:
# it is the cheapest check there is that a change has not broken the platform
# you were not looking at.
#
# WHAT IT CANNOT CHECK. The host compiler has a 32-bit int; the MSP430's is 16.
# An expression that overflows on Shimmer3 is silently fine here. Only the Code
# Composer build catches that class of fault, which is why it is a release gate
# in docs/SHIMMER3_TEST_PROCEDURE.md and not something CI stands in for.

CC      ?= gcc
PYTHON  ?= python3

REPO    := ../..
BUILD   := out

# -Werror on purpose: these are small, self-contained translation units, so a
# new warning here is a real finding rather than noise from a vendor header.
CFLAGS  := -std=c11 -O2 -Wall -Wextra -Werror -DSHIMMER_HOST_TEST

# stubs/ first, so its log_and_stream_includes.h shadows the repository's own
# aggregate. Nothing else in stubs/ shadows anything - see stubs/README.md.
# The two Shimmer_Driver paths mirror how the firmware builds resolve vendor
# headers, including Boards/'s "../BMPX80/bmpX80.h".
INCLUDES := -I . \
            -I stubs \
            -I stubs/Shimmer_Driver \
            -I stubs/Shimmer_Driver/5xx_HAL \
            -I $(REPO)

# Suites that need no stub platform at all - the modules under them include
# nothing but the C library.
STANDALONE_SUITES := test_swcrc test_packet_ring test_util

# Suites built once, against the stub platform.
SINGLE_PLATFORM_SUITES := test_crc

# Suites built twice, once per platform. See the note above.
DUAL_PLATFORM_SUITES := test_rtc test_battery test_boards

PLATFORMS := SHIMMER3 SHIMMER3R

# Per-suite sources. The stub platform (host_stubs.c) is added automatically to
# everything except the standalone suites.
SRC_test_swcrc       := test_swcrc.c $(REPO)/CRC/shimmer_swCrc.c
SRC_test_packet_ring := test_packet_ring.c $(REPO)/Sensing/shimmer_packet_ring.c
SRC_test_util        := test_util.c $(REPO)/Util/shimmer_util.c
SRC_test_crc         := test_crc.c $(REPO)/CRC/shimmer_crc.c \
                        $(REPO)/CRC/shimmer_swCrc.c $(REPO)/Platform/platform_api.c
SRC_test_rtc         := test_rtc.c $(REPO)/RTC/shimmer_rtc.c
SRC_test_battery     := test_battery.c $(REPO)/Battery/shimmer_battery.c
SRC_test_boards      := test_boards.c $(REPO)/Boards/shimmer_boards.c

# test_swcrc and test_packet_ring predate the stub platform and use their own
# include paths, so they are built with a narrower -I set.
INC_test_swcrc       := -I $(REPO)/CRC
INC_test_packet_ring := -I $(REPO)
INC_test_util        := -I . -I $(REPO)

ALL_SUITES := $(STANDALONE_SUITES) $(SINGLE_PLATFORM_SUITES) $(DUAL_PLATFORM_SUITES)

.PHONY: all build test clean help platform-check $(ALL_SUITES)

all: test

help:
	@echo "make            build and run every suite"
	@echo "make build      build only"
	@echo "make test       run the suites and the Python cross-checks"
	@echo "make clean      remove $(BUILD)/"
	@echo "make platform-check   compile every host-clean module for BOTH MCUs"
	@echo ""
	@echo "Suites: $(ALL_SUITES)"
	@echo "Run one with e.g. 'make test_rtc'."

$(BUILD):
	@mkdir -p $(BUILD)

# --- build rules ------------------------------------------------------------

define STANDALONE_RULE
$(BUILD)/$(1): $$(SRC_$(1)) | $(BUILD)
	$$(CC) $$(CFLAGS) $$(INC_$(1)) $$(SRC_$(1)) -o $$@
endef
$(foreach s,$(STANDALONE_SUITES),$(eval $(call STANDALONE_RULE,$(s))))

define SINGLE_RULE
$(BUILD)/$(1): $$(SRC_$(1)) host_stubs.c | $(BUILD)
	$$(CC) $$(CFLAGS) -DSHIMMER3R $$(INCLUDES) $$(SRC_$(1)) host_stubs.c -o $$@
endef
$(foreach s,$(SINGLE_PLATFORM_SUITES),$(eval $(call SINGLE_RULE,$(s))))

define DUAL_RULE
$(BUILD)/$(1).$(2): $$(SRC_$(1)) host_stubs.c | $(BUILD)
	$$(CC) $$(CFLAGS) -D$(2) $$(INCLUDES) $$(SRC_$(1)) host_stubs.c -o $$@
endef
$(foreach s,$(DUAL_PLATFORM_SUITES),\
  $(foreach p,$(PLATFORMS),$(eval $(call DUAL_RULE,$(s),$(p)))))

BINARIES := $(foreach s,$(STANDALONE_SUITES) $(SINGLE_PLATFORM_SUITES),$(BUILD)/$(s)) \
            $(foreach s,$(DUAL_PLATFORM_SUITES),\
              $(foreach p,$(PLATFORMS),$(BUILD)/$(s).$(p)))

build: $(BINARIES)

# --- running ----------------------------------------------------------------

# Every binary is run, then the Python cross-checks. The shell loop keeps going
# after a failure and reports the tally at the end: one firmware change usually
# breaks several suites, and which ones is the useful signal.
test: build
	@echo "=============================================================="
	@echo " log-and-stream-common host tests"
	@echo "=============================================================="
	@failed=""; \
	for bin in $(BINARIES); do \
	  echo ""; \
	  echo "--------------------------------------------------------------"; \
	  echo " $$bin"; \
	  echo "--------------------------------------------------------------"; \
	  if ! $$bin; then failed="$$failed $$bin"; fi; \
	done; \
	echo ""; \
	echo "--------------------------------------------------------------"; \
	echo " cross-checks against independent references"; \
	echo "--------------------------------------------------------------"; \
	if ! $(PYTHON) crosscheck_swcrc.py $(BUILD)/test_swcrc; then \
	  failed="$$failed crosscheck_swcrc.py"; fi; \
	if ! $(PYTHON) crosscheck_rtc.py $(BUILD)/test_rtc.SHIMMER3R; then \
	  failed="$$failed crosscheck_rtc.py"; fi; \
	if ! $(PYTHON) crosscheck_board_revisions.py $(BUILD)/test_boards.SHIMMER3R; then \
	  failed="$$failed crosscheck_board_revisions.py"; fi; \
	echo ""; \
	echo "=============================================================="; \
	if [ -n "$$failed" ]; then \
	  echo " FAILED:$$failed"; \
	  echo "=============================================================="; \
	  exit 1; \
	fi; \
	echo " all host tests passed"; \
	echo "=============================================================="

# 'make test_rtc' runs both platform builds of that suite; 'make test_util' the
# single one. Handy while iterating on one module.
$(STANDALONE_SUITES) $(SINGLE_PLATFORM_SUITES): %: $(BUILD)/%
	@$(BUILD)/$@

$(DUAL_PLATFORM_SUITES): %: $(foreach p,$(PLATFORMS),$(BUILD)/%.$(p))
	@for p in $(PLATFORMS); do $(BUILD)/$@.$$p || exit 1; done

# --- cross-platform compile check -------------------------------------------
#
# Every module below is compiled twice, once per platform, and only compiled -
# nothing is run. It exists because of the one thing AGENTS.md calls the most
# important to hold in mind here: this library ships to an MSP430 and an STM32
# at once, and "nothing in this repo will tell you at edit time" when a change
# has broken the other one. Neither vendor IDE is available in CI, and a
# developer normally has only one of them installed.
#
# It cannot replace either real build - it does not see the vendor headers, the
# linker scripts or the word sizes - but it catches the cheap and common half:
# a symbol used outside the #if that defines it, an #ifdef arm that stopped
# compiling, a header that only resolves on one platform. Those are most of what
# actually breaks, and they break in a pull request rather than in a release
# build hours later.
#
# ADDING TO THE LIST is the goal, not the exception. A module qualifies as soon
# as it compiles for both without a new stub; the ones missing today
# (Comms/, SDCard/, SDSync/, Sensing/shimmer_sensing.c, Configuration/,
# TaskList/, Calibration/, GSR/, LEDs/, EEPROM/) each need one or two more
# headers under stubs/Shimmer_Driver - FatFs, FileX, RN4X, ADXL371, gpio.h - or,
# for EEPROM/, the BAUD_* enum out of Comms/shimmer_bt_uart.h. Each one added
# brings its subsystem within reach of a host test as well.
DUAL_COMPILE_MODULES := \
  Battery/shimmer_battery.c \
  Boards/shimmer_boards.c \
  Button/shimmer_button.c \
  CRC/shimmer_crc.c \
  CRC/shimmer_swCrc.c \
  Platform/platform_api.c \
  RTC/shimmer_rtc.c \
  Sensing/shimmer_packet_ring.c \
  Util/shimmer_util.c

platform-check: | $(BUILD)
	@fail=0; \
	for p in $(PLATFORMS); do \
	  for m in $(DUAL_COMPILE_MODULES); do \
	    if $(CC) $(CFLAGS) -D$$p $(INCLUDES) -c $(REPO)/$$m -o $(BUILD)/pc.o \
	         2>$(BUILD)/pc.err; then \
	      printf '  ok    -D%-10s %s\n' "$$p" "$$m"; \
	    else \
	      printf '  FAIL  -D%-10s %s\n' "$$p" "$$m"; \
	      sed 's/^/          /' $(BUILD)/pc.err; \
	      fail=1; \
	    fi; \
	  done; \
	done; \
	rm -f $(BUILD)/pc.o $(BUILD)/pc.err; \
	if [ $$fail -ne 0 ]; then \
	  echo ""; \
	  echo "A module no longer compiles for one of the two MCUs."; \
	  exit 1; \
	fi; \
	echo ""; \
	echo "every listed module compiles for both Shimmer3 and Shimmer3R"

clean:
	rm -rf $(BUILD)
