/*
 * Host-test stub for TI's MSP430 device header (Shimmer3 only).
 *
 * The real msp430.h ships with the TI compiler. Calibration/shimmer_calibration.c
 * includes it for the _delay_cycles() intrinsic alone, which it uses to wait
 * 50 ms after closing a file. An empty statement is the right stand-in: the
 * host tests assert what the module did, never how long it took.
 */
#ifndef HOST_TEST_STUB_MSP430_H
#define HOST_TEST_STUB_MSP430_H

#define _delay_cycles(cycles) ((void) (cycles))

#endif /* HOST_TEST_STUB_MSP430_H */
