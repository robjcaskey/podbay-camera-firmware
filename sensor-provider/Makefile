CC ?= cc
CFLAGS ?= -O2 -g
override CPPFLAGS += -Iinclude
override CFLAGS += -std=c11 -Wall -Wextra -Werror -pedantic

.PHONY: all test clean

all: test-imx582-mode test-sigmastar-sensor-handle

test-imx582-mode: src/imx582_mode.c tests/test_imx582_mode.c include/podbay/imx582_mode.h
	$(CC) $(CPPFLAGS) $(CFLAGS) src/imx582_mode.c tests/test_imx582_mode.c -o $@

test-sigmastar-sensor-handle: src/sigmastar_sensor_handle.c tests/test_sigmastar_sensor_handle.c include/podbay/sigmastar_sensor_handle.h
	$(CC) $(CPPFLAGS) $(CFLAGS) src/sigmastar_sensor_handle.c tests/test_sigmastar_sensor_handle.c -o $@

test: test-imx582-mode test-sigmastar-sensor-handle
	./test-imx582-mode
	./test-sigmastar-sensor-handle

clean:
	$(RM) test-imx582-mode test-sigmastar-sensor-handle
