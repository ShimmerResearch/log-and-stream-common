/*
 * Host-side test for SDCard/shimmer_sd_card_id.c (DEV-1398).
 *
 * The factory test report names the unit's microSD card from its CID, and
 * production uses those lines to confirm a unit carries the approved card. So
 * the cases that matter most here are real: each is a card bought for the
 * DEV-204 evaluation, and its expected lines are what a Shimmer3R printed for
 * it in November 2026, with the code this module was moved from. A change that
 * alters any of them changes what production reads.
 *
 * Where a card was also read on Linux (/sys/block/mmcblk0/device/cid), the CID
 * is those 16 bytes. Otherwise it is rebuilt from the report's own fields, with
 * a CRC byte of 0x01, which nothing here reads. The reports print the size to
 * 0.01 GB, not the sector count, so each count is the printed size times
 * 2097152, rounded.
 *
 * shimmer_sd_card_id.c includes nothing but the C library, so this needs no
 * stubs. Run by .github/workflows/host-tests.yml.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions, so every .c file
 * under it is compiled into the firmware. This file defines main(). Only the
 * host-test build passes -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include "host_test.h"

#include "SDCard/shimmer_sd_card_id.h"
#include "Test/shimmer_test.h"

/* 32 hex digits, most significant byte first, as Linux prints a CID */
static void hexToCid(const char *hex, uint8_t *raw)
{
  unsigned int i, b;

  expectU("CID hex digits", strlen(hex), 2 * SD_CID_LEN);
  for (i = 0; i < SD_CID_LEN; i++)
  {
    b = 0;
    sscanf(&hex[2 * i], "%2x", &b);
    raw[i] = (uint8_t) b;
  }
}

typedef struct
{
  const char *card; /* the card's label in the DEV-204 results */
  const char *cid;
  uint32_t sectors;
  const char *lines[3];
} captured_t;

static const captured_t captured[] = {
  /* The card shipped today */
  { "control", "0353445353303847808197b4b3019301", 15518925,
      { " - Manufacturer: SanDisk, Manufacture Date=2025-03\r\n", " - Model: matches SanDisk 8GB SDSDQAB-008G\r\n",
          " - CID: MID=0x03, OID=SD, PNM=SS08G, PRV=8.0, PSN=0x8197B4B3, "
          "Size=7.40GB\r\n" } },
  { "SDU-A", "0353445344333247853218b7a801a501", 62327357,
      { " - Manufacturer: SanDisk, Manufacture Date=2026-05\r\n",
          " - Model: matches SanDisk Ultra 32GB SDSQUA4-032G\r\n",
          " - CID: MID=0x03, OID=SD, PNM=SD32G, PRV=8.5, PSN=0x3218B7A8, "
          "Size=29.72GB\r\n" } },
  { "INT8-A", "2750485344303847606e253cda01a101", 15267267,
      { " - Manufacturer: Integral, Manufacture Date=2026-01\r\n",
          " - Model: matches Integral UltimaPro 8GB INMSDH8G10-90U1\r\n",
          " - CID: MID=0x27, OID=PH, PNM=SD08G, PRV=6.0, PSN=0x6E253CDA, "
          "Size=7.28GB\r\n" } },
  { "INT8-B", "2750485344303847607208d4bb01a401", 15267267,
      { " - Manufacturer: Integral, Manufacture Date=2026-04\r\n",
          " - Model: matches Integral UltimaPro 8GB INMSDH8G10-90U1\r\n",
          " - CID: MID=0x27, OID=PH, PNM=SD08G, PRV=6.0, PSN=0x7208D4BB, "
          "Size=7.28GB\r\n" } },
  /* Read on Linux too. An MID in no public list, and an unprintable OID */
  { "INT16-A", "6f0303534441424310aa001102019901", 30534533,
      { " - Manufacturer: Integral, Manufacture Date=2025-09\r\n",
          " - Model: matches Integral UltimaPro 16GB INMSDH16G10-90U1\r\n",
          " - CID: MID=0x6F, OID=0x0303, PNM=SDABC, PRV=1.0, PSN=0xAA001102, "
          "Size=14.56GB\r\n" } },
  /* The Silicon Power 8 GB and 16 GB differ only in capacity */
  { "SP8-A", "9f5449535043432020911401a2019101", 15434839,
      { " - Manufacturer: Silicon Power, Manufacture Date=2025-01\r\n",
          " - Model: matches Silicon Power 8GB SP008GBSTHBU1V10SP\r\n",
          " - CID: MID=0x9F, OID=TI, PNM=SPCC , PRV=2.0, PSN=0x911401A2, "
          "Size=7.36GB\r\n" } },
  { "SP8-B", "9f5449535043432020a114001b019101", 15434839,
      { " - Manufacturer: Silicon Power, Manufacture Date=2025-01\r\n",
          " - Model: matches Silicon Power 8GB SP008GBSTHBU1V10SP\r\n",
          " - CID: MID=0x9F, OID=TI, PNM=SPCC , PRV=2.0, PSN=0xA114001B, "
          "Size=7.36GB\r\n" } },
  { "SP16-A", "9f54495350434320202778019101a601", 30366761,
      { " - Manufacturer: Silicon Power, Manufacture Date=2026-06\r\n",
          " - Model: matches Silicon Power 16GB SP016GBSTHBU1V10SP\r\n",
          " - CID: MID=0x9F, OID=TI, PNM=SPCC , PRV=2.0, PSN=0x27780191, "
          "Size=14.48GB\r\n" } },
  { "V82-A", "9f5449303030303000115b043e01a301", 30429676,
      { " - Manufacturer: Verbatim, Manufacture Date=2026-03\r\n", " - Model: matches Verbatim 16GB 44082\r\n",
          " - CID: MID=0x9F, OID=TI, PNM=00000, PRV=0.0, PSN=0x115B043E, "
          "Size=14.51GB\r\n" } },
  /* A Kingston ID, and a serial that counts up from 1 */
  { "V83-A", "4134325344313647200000000d01a501", 61069066,
      { " - Manufacturer: Verbatim, Manufacture Date=2026-05\r\n", " - Model: matches Verbatim 32GB 44083\r\n",
          " - CID: MID=0x41, OID=42, PNM=SD16G, PRV=2.0, PSN=0x0000000D, "
          "Size=29.12GB\r\n" } },
  /* An impossible manufacture date, printed as the card reports it */
  { "V83-B", "4134325344313647200000000200c901", 61069066,
      { " - Manufacturer: Verbatim, Manufacture Date=2012-09\r\n", " - Model: matches Verbatim 32GB 44083\r\n",
          " - CID: MID=0x41, OID=42, PNM=SD16G, PRV=2.0, PSN=0x00000002, "
          "Size=29.12GB\r\n" } },
  /* Read on Linux too. NULs in the product name, so it prints in hex */
  { "V41-B", "9f54495344000000200000495301a701", 61069066,
      { " - Manufacturer: Verbatim, Manufacture Date=2026-07\r\n",
          " - Model: matches Verbatim 32GB industrial 47041\r\n",
          " - CID: MID=0x9F, OID=TI, PNM=0x5344000000, PRV=2.0, "
          "PSN=0x00004953, Size=29.12GB\r\n" } },
};

#define NUM_CAPTURED (sizeof(captured) / sizeof(captured[0]))

static void test_captured_reports(void)
{
  unsigned int i, line;
  uint8_t raw[SD_CID_LEN];
  char out[SD_CARD_ID_LINE_MAX];
  char what[48];

  testCase("each DEV-204 card prints the lines a Shimmer3R printed for it");
  for (i = 0; i < NUM_CAPTURED; i++)
  {
    hexToCid(captured[i].cid, raw);
    for (line = 0; line < 3; line++)
    {
      snprintf(what, sizeof(what), "%s line %u", captured[i].card, line);
      expectU(what,
          ShimSdCardId_reportLine(out, (uint8_t) line, raw, captured[i].sectors), 1);
      expectStr(what, out, captured[i].lines[line]);
    }
    snprintf(what, sizeof(what), "%s past the last line", captured[i].card);
    expectU(what, ShimSdCardId_reportLine(out, 3, raw, captured[i].sectors), 0);
    expectStr(what, out, "");
  }
}

static void test_decode(void)
{
  uint8_t raw[SD_CID_LEN];
  sd_cid_t cid;

  /* INT16-A as Linux read it */
  testCase("the CID's fields decode from its bytes, most significant first");
  hexToCid("6f0303534441424310aa001102019901", raw);
  ShimSdCardId_decode(raw, &cid);
  expectX("MID", cid.mid, 0x6F);
  expectX("OID[0]", cid.oid[0], 0x03);
  expectX("OID[1]", cid.oid[1], 0x03);
  expectTrue("PNM", memcmp(cid.pnm, "SDABC", 5) == 0);
  expectX("PRV", cid.prv, 0x10);
  expectX("PSN", cid.psn, 0xAA001102);
  expectU("year", cid.year, 2025);
  expectU("month", cid.month, 9);

  testCase("the reserved bits above the manufacture date are ignored");
  raw[13] = 0xF1;
  ShimSdCardId_decode(raw, &cid);
  expectU("year", cid.year, 2025);
  expectU("month", cid.month, 9);

  testCase("the manufacture date spans 2000 to 2255");
  raw[13] = 0x00;
  raw[14] = 0x00;
  ShimSdCardId_decode(raw, &cid);
  expectU("earliest year", cid.year, 2000);
  expectU("month field low", cid.month, 0);
  raw[13] = 0x0F;
  raw[14] = 0xFF;
  ShimSdCardId_decode(raw, &cid);
  expectU("latest year", cid.year, 2255);
  expectU("month field high", cid.month, 15);
}

static void test_linux_only_cards(void)
{
  /* No Shimmer3R report of these cards used the model table, so these are the
   * lines the table gives, checked by hand against it */
  uint8_t raw[SD_CID_LEN];
  char out[SD_CARD_ID_LINE_MAX];

  testCase("the second Verbatim 47041 supplier's card is recognised too");
  hexToCid("fe3432534400000020000002fe01a401", raw);
  ShimSdCardId_reportLine(out, 0, raw, 61069066);
  expectStr("V41-A maker", out, " - Manufacturer: Verbatim, Manufacture Date=2026-04\r\n");
  ShimSdCardId_reportLine(out, 1, raw, 61069066);
  expectStr("V41-A model", out, " - Model: matches Verbatim 32GB industrial 47041\r\n");
  ShimSdCardId_reportLine(out, 2, raw, 61069066);
  expectStr("V41-A CID",
      out, " - CID: MID=0xFE, OID=42, PNM=0x5344000000, PRV=2.0, PSN=0x000002FE, Size=29.12GB\r\n");

  testCase("the B cards Linux read match their models");
  hexToCid("6f0303534441424310aa00012500a201", raw);
  ShimSdCardId_reportLine(out, 1, raw, 30534533);
  expectStr("INT16-B", out, " - Model: matches Integral UltimaPro 16GB INMSDH16G10-90U1\r\n");
  hexToCid("9f5449535043432020110602a501a601", raw);
  ShimSdCardId_reportLine(out, 1, raw, 30366761);
  expectStr("SP16-B", out, " - Model: matches Silicon Power 16GB SP016GBSTHBU1V10SP\r\n");
  hexToCid("9f5449303030303000265a042301a301", raw);
  ShimSdCardId_reportLine(out, 1, raw, 30429676);
  expectStr("V82-B", out, " - Model: matches Verbatim 16GB 44082\r\n");
}

static void test_unrecognised_cards(void)
{
  uint8_t raw[SD_CID_LEN];
  char out[SD_CARD_ID_LINE_MAX];

  /* The current card's ID at 64 GB, which no row has */
  testCase("a card of a listed maker but no listed model names the maker");
  hexToCid("0353445353303847808197b4b3019301", raw);
  ShimSdCardId_reportLine(out, 0, raw, 124735488);
  expectStr("maker", out, " - Manufacturer: SanDisk, Manufacture Date=2025-03\r\n");
  ShimSdCardId_reportLine(out, 1, raw, 124735488);
  expectStr("model", out, " - Model: not recognised\r\n");

  /* INT16-A's ID at 8 GB */
  testCase("a card of an unlisted maker prints its MID");
  hexToCid("6f0303534441424310aa001102019901", raw);
  ShimSdCardId_reportLine(out, 0, raw, 15518925);
  expectStr("maker", out, " - Manufacturer: Unknown (0x6F), Manufacture Date=2025-09\r\n");
  ShimSdCardId_reportLine(out, 1, raw, 15518925);
  expectStr("model", out, " - Model: not recognised\r\n");

  testCase("a card that differs from a row in one field is not that row");
  hexToCid("0353445353303847808197b4b3019301", raw);
  raw[0] = 0x02; /* MID */
  ShimSdCardId_reportLine(out, 1, raw, 15518925);
  expectStr("MID", out, " - Model: not recognised\r\n");
  hexToCid("0353445353303847808197b4b3019301", raw);
  raw[2] = 'E'; /* second OID byte */
  ShimSdCardId_reportLine(out, 1, raw, 15518925);
  expectStr("OID", out, " - Model: not recognised\r\n");
  hexToCid("0353445353303847808197b4b3019301", raw);
  raw[7] = 'H'; /* last PNM byte */
  ShimSdCardId_reportLine(out, 1, raw, 15518925);
  expectStr("PNM", out, " - Model: not recognised\r\n");
}

static void test_text_fields(void)
{
  uint8_t raw[SD_CID_LEN];
  char out[SD_CARD_ID_LINE_MAX];

  testCase("a field with any unprintable byte prints in hex, else as text");
  hexToCid("00ffff7e7f2020200000000000000001", raw);
  ShimSdCardId_reportLine(out, 2, raw, 0);
  expectStr("0xFF in OID, 0x7F in PNM",
      out, " - CID: MID=0x00, OID=0xFFFF, PNM=0x7E7F202020, PRV=0.0, PSN=0x00000000, Size=0.00GB\r\n");
  hexToCid("00207e7e2020207e0000000000000001", raw);
  ShimSdCardId_reportLine(out, 2, raw, 0);
  expectStr("space and tilde are printable",
      out, " - CID: MID=0x00, OID= ~, PNM=~   ~, PRV=0.0, PSN=0x00000000, Size=0.00GB\r\n");
}

/* The CID line from "Size=" on, for the card's sector count */
static const char *sizeField(char *out, const uint8_t *raw, uint32_t sectors)
{
  const char *size;

  ShimSdCardId_reportLine(out, 2, raw, sectors);
  size = strstr(out, "Size=");
  return size != NULL ? size : out;
}

static void test_size(void)
{
  uint8_t raw[SD_CID_LEN];
  char out[SD_CARD_ID_LINE_MAX];

  hexToCid("0353445353303847808197b4b3019301", raw);
  testCase("the size is GiB to two decimals, rounded half up");
  expectStr("zero", sizeField(out, raw, 0), "Size=0.00GB\r\n");
  expectStr("1 GiB", sizeField(out, raw, 2097152), "Size=1.00GB\r\n");
  expectStr("just under 0.005 over", sizeField(out, raw, 2097152 + 10485), "Size=1.00GB\r\n");
  expectStr("just over 0.005 over", sizeField(out, raw, 2097152 + 10486), "Size=1.01GB\r\n");
  expectStr("a sector short of 2 GiB carries",
      sizeField(out, raw, 2 * 2097152 - 1), "Size=2.00GB\r\n");
  expectStr("the largest count", sizeField(out, raw, 0xFFFFFFFF), "Size=2048.00GB\r\n");
}

static void test_nominal_capacity(void)
{
  testCase("nominal capacity is the smallest power of two GB that holds it");
  expectU("empty", ShimSdCardId_nominalGb(0), 1);
  expectU("1 GB exactly", ShimSdCardId_nominalGb(1953125), 1);
  expectU("1 GB and a sector", ShimSdCardId_nominalGb(1953126), 2);
  expectU("the current card, 7.40 GiB", ShimSdCardId_nominalGb(15518925), 8);
  expectU("8 GB exactly", ShimSdCardId_nominalGb(15625000), 8);
  expectU("8 GB and a sector", ShimSdCardId_nominalGb(15625001), 16);
  expectU("14.56 GiB", ShimSdCardId_nominalGb(30534533), 16);
  expectU("29.12 GiB", ShimSdCardId_nominalGb(61069066), 32);
  expectU("128 GB exactly", ShimSdCardId_nominalGb(250000000), 128);
  expectU("past 128 GB stays 128", ShimSdCardId_nominalGb(0xFFFFFFFF), 128);
}

static void test_line_lengths(void)
{
  uint8_t raw[SD_CID_LEN];
  char out[SD_CARD_ID_LINE_MAX];
  unsigned int i, line;

  /* Every text field in hex, every number at its widest */
  testCase("no line reaches the report's truncation");
  memset(raw, 0xFF, sizeof(raw));
  ShimSdCardId_reportLine(out, 2, raw, 0xFFFFFFFF);
  expectStr("widest CID line", out,
      " - CID: MID=0xFF, OID=0xFFFF, PNM=0xFFFFFFFFFF, PRV=15.15, "
      "PSN=0xFFFFFFFF, "
      "Size=2048.00GB\r\n");
  expectTrue("widest CID line is not cut short", strlen(out) < SD_CARD_ID_LINE_MAX - 1);
  ShimSdCardId_reportLine(out, 0, raw, 0xFFFFFFFF);
  expectStr("widest unknown-maker line", out,
      " - Manufacturer: Unknown (0xFF), Manufacture Date=2255-15\r\n");
  for (i = 0; i < NUM_CAPTURED; i++)
  {
    hexToCid(captured[i].cid, raw);
    for (line = 0; line < 3; line++)
    {
      ShimSdCardId_reportLine(out, (uint8_t) line, raw, captured[i].sectors);
      expectTrue("every captured line is under 100 characters", strlen(out) < 100);
    }
  }
  expectTrue("the line buffer is within the report's truncation",
      SD_CARD_ID_LINE_MAX <= MAX_TEST_REPORT_LENGTH);
}

static void test_unavailable(void)
{
  char out[SD_CARD_ID_LINE_MAX];

  testCase("a platform with no CID says why, in the form host parsers expect");
  ShimSdCardId_unavailableLine(out, "card not initialised");
  expectStr("reason", out, " - CID: unavailable (card not initialised)\r\n");
}

int main(void)
{
  printf("shimmer_sd_card_id host tests\n\n");
  hostTestSilenceUnused();

  test_captured_reports();
  test_decode();
  test_linux_only_cards();
  test_unrecognised_cards();
  test_text_fields();
  test_size();
  test_nominal_capacity();
  test_line_lengths();
  test_unavailable();

  return hostTestReport("shimmer_sd_card_id");
}

#endif /* SHIMMER_HOST_TEST */
