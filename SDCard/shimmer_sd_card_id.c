/*
 * shimmer_sd_card_id.c
 *
 * See shimmer_sd_card_id.h for the report lines this formats (DEV-1398).
 */

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "shimmer_sd_card_id.h"

/* 512-byte sectors in a GiB, and in a GB (10^9 bytes) */
#define SECTORS_PER_GIB 2097152UL
#define SECTORS_PER_GB  1953125UL

/* The values were read off the DEV-204 candidate cards and the current SanDisk
 * card in November 2026. Some signatures are generic (Phison SD08G), so
 * another brand's card can carry the same one. Two models differ only in
 * capacity, hence sizeGb. */
static const sd_card_model_t sdCardModels[] = {
  { 0x03, "SD", "SS08G", 8, "SanDisk", "8GB SDSDQAB-008G" },
  { 0x03, "SD", "SD32G", 32, "SanDisk", "Ultra 32GB SDSQUA4-032G" },
  { 0x27, "PH", "SD08G", 8, "Integral", "UltimaPro 8GB INMSDH8G10-90U1" },
  { 0x6F, "\x03\x03", "SDABC", 16, "Integral", "UltimaPro 16GB INMSDH16G10-90U1" },
  { 0x9F, "TI", "SPCC ", 8, "Silicon Power", "8GB SP008GBSTHBU1V10SP" },
  { 0x9F, "TI", "SPCC ", 16, "Silicon Power", "16GB SP016GBSTHBU1V10SP" },
  { 0x9F, "TI", "00000", 16, "Verbatim", "16GB 44082" },
  { 0x41, "42", "SD16G", 32, "Verbatim", "32GB 44083" },
  /* Two 47041 cards bought together came from two suppliers */
  { 0xFE, "42", "SD\x00\x00\x00", 32, "Verbatim", "32GB industrial 47041" },
  { 0x9F, "TI", "SD\x00\x00\x00", 32, "Verbatim", "32GB industrial 47041" },
};

void ShimSdCardId_decode(const uint8_t *raw, sd_cid_t *cid)
{
  /* MDT: 4 reserved bits, then the year since 2000 in 8 bits and the month in
   * 4, across bytes 13 and 14 */
  uint16_t mdt = (uint16_t) (((uint16_t) (raw[13] & 0x0F) << 8) | raw[14]);

  cid->mid = raw[0];
  memcpy(cid->oid, &raw[1], sizeof(cid->oid));
  memcpy(cid->pnm, &raw[3], sizeof(cid->pnm));
  cid->prv = raw[8];
  cid->psn = ((uint32_t) raw[9] << 24) | ((uint32_t) raw[10] << 16)
      | ((uint32_t) raw[11] << 8) | raw[12];
  cid->year = (uint16_t) (2000U + (mdt >> 4));
  cid->month = (uint8_t) (mdt & 0x0F);
}

const char *ShimSdCardId_makerName(uint8_t mid)
{
  switch (mid)
  {
    case 0x01:
      return "Panasonic";
    case 0x02:
      return "Toshiba";
    case 0x03:
      return "SanDisk";
    case 0x1b:
      return "Samsung";
    case 0x1d:
      return "AData";
    case 0x27:
      return "Phison";
    case 0x28:
      return "Lexar";
    case 0x31:
      return "Silicon Power";
    case 0x41:
      return "Kingston";
    case 0x74:
      return "Transcend";
    case 0x76:
      return "Patriot";
    case 0x82:
    case 0x9c:
      return "Sony";
    default:
      return NULL;
  }
}

uint8_t ShimSdCardId_nominalGb(uint32_t sectors)
{
  uint8_t gb = 1;

  /* 128 GB is 2.5e8 sectors, so the product stays inside 32 bits */
  while ((uint32_t) gb * SECTORS_PER_GB < sectors && gb < 128)
  {
    gb <<= 1;
  }
  return gb;
}

const sd_card_model_t *ShimSdCardId_findModel(const sd_cid_t *cid, uint32_t sectors)
{
  uint8_t sizeGb = ShimSdCardId_nominalGb(sectors);
  uint8_t i;

  for (i = 0; i < sizeof(sdCardModels) / sizeof(sdCardModels[0]); i++)
  {
    const sd_card_model_t *m = &sdCardModels[i];

    if (m->mid == cid->mid && memcmp(m->oid, cid->oid, sizeof(cid->oid)) == 0
        && memcmp(m->pnm, cid->pnm, sizeof(cid->pnm)) == 0 && m->sizeGb == sizeGb)
    {
      return m;
    }
  }
  return NULL;
}

/* A CID text field (OID, PNM) as its characters when every byte is printable,
 * else as hex ("0x" and two digits a byte), so that no byte is lost:
 * white-label cards often carry unprintable OID and PNM bytes. out needs room
 * for 2 + 2 * len + 1 characters. */
static void sdCidField(char *out, const uint8_t *bytes, uint8_t len)
{
  uint8_t i;

  for (i = 0; i < len; i++)
  {
    if (bytes[i] < 0x20 || bytes[i] > 0x7E)
    {
      break;
    }
  }
  if (i == len)
  {
    memcpy(out, bytes, len);
    out[len] = '\0';
    return;
  }
  out += sprintf(out, "0x");
  for (i = 0; i < len; i++)
  {
    out += sprintf(out, "%02X", (unsigned int) bytes[i]);
  }
}

/* The size in GiB, to two decimals and rounded, although it prints as GB as
 * older firmware's did. Integer only: a remainder under 2^21 sectors times 100
 * still fits in 32 bits. */
static void sdCardSizeStr(char *out, uint32_t sectors)
{
  uint32_t whole = sectors / SECTORS_PER_GIB;
  uint32_t hundredths
      = ((sectors % SECTORS_PER_GIB) * 100UL + SECTORS_PER_GIB / 2UL) / SECTORS_PER_GIB;

  if (hundredths == 100UL)
  {
    whole++;
    hundredths = 0;
  }
  sprintf(out, "%lu.%02luGB", (unsigned long) whole, (unsigned long) hundredths);
}

uint8_t ShimSdCardId_reportLine(char *out, uint8_t line, const uint8_t *raw, uint32_t sectors)
{
  sd_cid_t cid;
  const sd_card_model_t *model;
  const char *maker;
  char oid[2 + 2 * sizeof(cid.oid) + 1];
  char pnm[2 + 2 * sizeof(cid.pnm) + 1];
  char size[16];

  ShimSdCardId_decode(raw, &cid);
  model = ShimSdCardId_findModel(&cid, sectors);

  switch (line)
  {
    case 0:
      /* The shape older firmware printed, which host parsers already read. It
       * names the brand of a card in the table, else the company its MID is
       * assigned to. */
      maker = (model != NULL) ? model->brand : ShimSdCardId_makerName(cid.mid);
      if (maker != NULL)
      {
        snprintf(out, SD_CARD_ID_LINE_MAX, " - Manufacturer: %s, Manufacture Date=%u-%02u\r\n",
            maker, (unsigned int) cid.year, (unsigned int) cid.month);
      }
      else
      {
        snprintf(out, SD_CARD_ID_LINE_MAX,
            " - Manufacturer: Unknown (0x%02X), Manufacture Date=%u-%02u\r\n",
            (unsigned int) cid.mid, (unsigned int) cid.year, (unsigned int) cid.month);
      }
      return 1;
    case 1:
      if (model != NULL)
      {
        snprintf(out, SD_CARD_ID_LINE_MAX, " - Model: matches %s %s\r\n",
            model->brand, model->part);
      }
      else
      {
        snprintf(out, SD_CARD_ID_LINE_MAX, " - Model: not recognised\r\n");
      }
      return 1;
    case 2:
      sdCidField(oid, cid.oid, sizeof(cid.oid));
      sdCidField(pnm, cid.pnm, sizeof(cid.pnm));
      sdCardSizeStr(size, sectors);
      /* PRV is BCD n.m */
      snprintf(out, SD_CARD_ID_LINE_MAX, " - CID: MID=0x%02X, OID=%s, PNM=%s, PRV=%u.%u, PSN=0x%08lX, Size=%s\r\n",
          (unsigned int) cid.mid, oid, pnm, (unsigned int) (cid.prv >> 4),
          (unsigned int) (cid.prv & 0x0F), (unsigned long) cid.psn, size);
      return 1;
    default:
      out[0] = '\0';
      return 0;
  }
}

void ShimSdCardId_unavailableLine(char *out, const char *reason)
{
  snprintf(out, SD_CARD_ID_LINE_MAX, " - CID: unavailable (%s)\r\n", reason);
}
