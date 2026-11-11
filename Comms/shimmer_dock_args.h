/*
 * shimmer_dock_args.h
 *
 * The LEN a dock request must carry for the properties whose arguments have a
 * layout of their own (docs/SHIMMER3_DOCK_PROTOCOL.md Appendix A).
 */

#ifndef SHIMMER_DOCK_ARGS_H_
#define SHIMMER_DOCK_ARGS_H_

#include <stdint.h>

/*
 * 1 if a request's LEN fits its arguments, else 0, which the dispatcher
 * answers with BAD_ARG.
 *
 * LEN counts the component, the property and the arguments. The variable-length
 * properties read their arguments at fixed offsets, so without this check a
 * frame shorter than its arguments has its own CRC, and then whatever an
 * earlier frame left in the buffer, taken as length, offset and data. A short
 * GET then answers with data nobody asked for, and a short SET writes stale
 * bytes into the configuration or the expansion-board EEPROM (DEV-1279).
 *
 * `firstArg` is the byte after the property: a SET's data length. It is looked
 * at only once LEN says the frame holds it.
 *
 * A property not listed has no layout to check here; the dispatcher judges it
 * as before.
 */
uint8_t ShimDock_isArgLenOk(uint8_t action, uint8_t comp, uint8_t prop, uint8_t len, uint8_t firstArg);

#endif /* SHIMMER_DOCK_ARGS_H_ */
