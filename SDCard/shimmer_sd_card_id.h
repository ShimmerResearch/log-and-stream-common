/*
 * shimmer_sd_card_id.h
 *
 * Names a unit's microSD card from its CID register, for the factory test
 * report (DEV-1398). Both platforms print the same block, one report write a
 * line, from the register's 16 bytes and the card's size:
 *
 *    - Manufacturer: Integral, Manufacture Date=2025-09
 *    - Model: matches Integral UltimaPro 16GB INMSDH16G10-90U1
 *    - CID: MID=0x6F, OID=0x0303, PNM=SDABC, PRV=1.0, PSN=0xAA001102, Size=14.56GB
 *
 * The platform reads the register, since how differs (Shimmer3R's SDMMC
 * peripheral keeps it from initialisation; Shimmer3's SPI driver sends CMD10),
 * and this decodes and formats it. Kept free of any platform dependency so the
 * decoding and the model table can be host-tested (Test/host/test_sd_card_id.c)
 * against reports captured from real cards. No floating point: Shimmer3's
 * printf is built without it.
 *
 * The MID names the maker of the card's controller or NAND, not the brand on
 * its label. Most microSD brands sell other makers' cards and report that
 * maker's MID, or a code in no public list, so the brand comes from the model
 * table, which recognises a card by its whole signature. See
 * docs/SHIMMER3R_FACTORY_TEST_REPORT.md §5.5.
 */

#ifndef LOG_AND_STREAM_COMMON_SDCARD_SHIMMER_SD_CARD_ID_H_
#define LOG_AND_STREAM_COMMON_SDCARD_SHIMMER_SD_CARD_ID_H_

#include <stdint.h>

/* The CID register's length, as the card sends it, most significant byte
 * first: MID, OID[2], PNM[5], PRV, PSN[4], then 4 reserved bits, the 12-bit
 * MDT, and the CRC7. */
#define SD_CID_LEN          16U

/* Room each report line needs, the terminator included. The longest possible
 * line is a CID line with every text field in hex, about 92 characters, well
 * under the report's 128-character truncation (MAX_TEST_REPORT_LENGTH). */
#define SD_CARD_ID_LINE_MAX 100U

/* The CID fields the report prints. */
typedef struct
{
  uint8_t mid;    /* manufacturer ID */
  uint8_t oid[2]; /* OEM/application ID, normally 2 ASCII characters */
  uint8_t pnm[5]; /* product name, normally 5 ASCII characters */
  uint8_t prv;    /* product revision, BCD n.m */
  uint32_t psn;   /* product serial number */
  uint16_t year;  /* manufacture date */
  uint8_t month;
} sd_cid_t;

/* A card model the report names, recognised by its CID MID, OID and PNM plus
 * its nominal capacity. oid and pnm hold the register's bytes, NUL-terminated
 * only for convenience: they are compared by length, and may hold NULs. */
typedef struct
{
  uint8_t mid;
  char oid[3];
  char pnm[6];
  uint8_t sizeGb;
  const char *brand;
  const char *part;
} sd_card_model_t;

/* Decodes the register's 16 bytes, most significant first. */
void ShimSdCardId_decode(const uint8_t *raw, sd_cid_t *cid);

/* The company a MID is assigned to, or NULL when it is not in the short list
 * this carries. */
const char *ShimSdCardId_makerName(uint8_t mid);

/* The card's capacity as sold, from its count of 512-byte sectors: the
 * smallest power of two GB (10^9 bytes) that holds it, up to 128. So 7.95e9
 * bytes is 8 and 31.3e9 is 32. */
uint8_t ShimSdCardId_nominalGb(uint32_t sectors);

/* The model in the table the card matches, or NULL. "Matches" is all it
 * claims: some signatures are generic, so another brand's card can carry one. */
const sd_card_model_t *ShimSdCardId_findModel(const sd_cid_t *cid, uint32_t sectors);

/* Writes report line `line` of the card's block to out, which needs
 * SD_CARD_ID_LINE_MAX bytes, and returns 1: line 0 is the Manufacturer line,
 * 1 the Model line and 2 the CID line, each " - " to "\r\n". Past the last line
 * it writes an empty string and returns 0, so the caller sends each line as it
 * comes:
 *
 *    for (i = 0; ShimSdCardId_reportLine(buf, i, raw, sectors); i++)
 *    {
 *      ShimFactoryTest_sendReport(buf);
 *    }
 *
 * raw is the register's 16 bytes, most significant first, and sectors the
 * card's count of 512-byte sectors. */
uint8_t ShimSdCardId_reportLine(char *out, uint8_t line, const uint8_t *raw, uint32_t sectors);

/* Writes the line a platform prints in place of the block when it has no CID
 * to decode, " - CID: unavailable (<reason>)\r\n", to out, which needs
 * SD_CARD_ID_LINE_MAX bytes. Keep the reason under 50 characters. */
void ShimSdCardId_unavailableLine(char *out, const char *reason);

#endif /* LOG_AND_STREAM_COMMON_SDCARD_SHIMMER_SD_CARD_ID_H_ */
