/*
 * Host-side test for Comms/shimmer_bt_reply.c: command replies on a Bluetooth
 * TX ring that stream data has filled.
 *
 * Before this module, a reply that arrived while data packets filled the ring
 * was refused by ShimBt_writeToTxBufAndSend() and the refusal ignored, so the
 * host timed out on a command the sensor had run (DEV-1150). On the bench this
 * took a ring filled by a stream outrunning a slow link; here it is a fake ring
 * and a handful of calls.
 *
 * The fake ring stands in for the two functions the module uses from
 * Comms/shimmer_bt_uart.c. It has the real ring's size and its rule that one
 * byte always stays free, and it records the order bytes went in, which is the
 * order the host would receive them.
 *
 * Built once per platform: the ring size, the reserve and the hold buffer all
 * differ between Shimmer3 and Shimmer3R.
 *
 * Guarded like every suite here: both firmware projects compile every .c in
 * this repository, and this file defines main(). Only the host-test build
 * passes -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include "host_test.h"

#include "Comms/shimmer_bt_reply.h"

#if defined(SHIMMER3R)
#define RING_SIZE 4096U
#else
#define RING_SIZE 256U
#endif

/* --- fake ring ------------------------------------------------------------ */

static uint8_t wire[16384];
static uint32_t wireLen;  /* bytes queued, in order, since the test began */
static uint32_t ringUsed; /* bytes queued and not yet drained */
static uint32_t replyWrites;

uint16_t ShimBt_getSpaceInBtTxBuf(void)
{
  return (uint16_t) (RING_SIZE - 1U - ringUsed);
}

static uint8_t ringPush(const uint8_t *buf, uint16_t len)
{
  /* The real ring refuses unless strictly more space than len is free */
  if (ShimBt_getSpaceInBtTxBuf() <= len)
  {
    return 1;
  }
  memcpy(&wire[wireLen], buf, len);
  wireLen += len;
  ringUsed += len;
  return 0;
}

uint8_t ShimBt_writeReplyToTxBuf(uint8_t *buf, uint16_t len)
{
  replyWrites++;
  return ringPush(buf, len);
}

/* A data packet, gated the way ShimBt_writeToTxBufAndSend() gates SENSOR_DATA */
static uint8_t pushData(uint16_t len)
{
  static uint8_t pkt[1100];
  memset(pkt, 0x00, len); /* DATA_PACKET opcode and zero payload */
  if (!ShimBtReply_isRoomForSensorData(len))
  {
    return 1;
  }
  return ringPush(pkt, len);
}

static void drain(uint32_t n)
{
  ringUsed = (n > ringUsed) ? 0 : ringUsed - n;
}

static void reset(void)
{
  ShimBtReply_reset();
  wireLen = 0;
  ringUsed = 0;
  replyWrites = 0;
}

/* Fill with data packets of len until the gate refuses one */
static uint32_t fillWithData(uint16_t len)
{
  uint32_t n = 0;
  while (pushData(len) == 0)
  {
    n++;
  }
  return n;
}

static int wireEndsWith(const uint8_t *buf, uint16_t len)
{
  return wireLen >= len && memcmp(&wire[wireLen - len], buf, len) == 0;
}

/* --- cases ---------------------------------------------------------------- */

static void testReplyOnIdleRing(void)
{
  testCase("a reply on an idle ring goes straight in");
  reset();
  uint8_t ack[] = { 0xFF };
  expectU("send result", ShimBtReply_send(ack, 1), 0);
  expectU("pending", ShimBtReply_isPending(), 0);
  expectTrue("ACK on the wire", wireEndsWith(ack, 1));
}

static void testDataLeavesRoomForReply(void)
{
  testCase("data filling the ring still leaves room for a reply (DEV-1150)");
  reset();
  /* 24 bytes: an IMU packet at 128 Hz on the bench that showed the loss */
  uint32_t packets = fillWithData(24);
  expectTrue("data filled the ring", packets > 0);
  /* A packet is accepted only while it leaves the reserve free, so the last
   * one accepted left at least the reserve */
  expectTrue("data stopped short of the reserve",
      ShimBt_getSpaceInBtTxBuf() >= BT_TX_REPLY_RESERVE);

  /* ACK + in-stream status reply with a 2-byte CRC: the reply the bench lost */
  uint8_t status[] = { 0xFF, 0x8A, 0x71, 0x20, 0x01, 0x12, 0x34 };
  expectU("send result", ShimBtReply_send(status, sizeof(status)), 0);
  expectU("not held", ShimBtReply_isPending(), 0);
  expectTrue("status reply on the wire", wireEndsWith(status, sizeof(status)));
}

static void testLongReplyHeldThenSent(void)
{
  testCase("a reply too long for the reserve is held, not dropped");
  reset();
  fillWithData(24);

  static uint8_t longRsp[RESPONSE_PACKET_SIZE + 2U];
  for (uint16_t i = 0; i < sizeof(longRsp); i++)
  {
    longRsp[i] = (uint8_t) (0x80U | i);
  }
  uint32_t before = wireLen;
  expectU("send result", ShimBtReply_send(longRsp, sizeof(longRsp)), 0);
  expectU("held", ShimBtReply_isPending(), 1);
  expectU("nothing written yet", wireLen, before);

  /* While held, data is refused even once the ring drains: the reply goes first */
  drain(RING_SIZE);
  expectU("data refused while a reply is held", pushData(24), 1);

  ShimBtReply_retry();
  expectU("released", ShimBtReply_isPending(), 0);
  expectTrue("held reply on the wire", wireEndsWith(longRsp, sizeof(longRsp)));
  expectU("data accepted again", pushData(24), 0);
}

static void testOrderKeptBehindHeldReply(void)
{
  testCase("a reply sent while another is held goes behind it");
  reset();
  fillWithData(24);

  static uint8_t first[RESPONSE_PACKET_SIZE];
  memset(first, 0xA5, sizeof(first));
  uint8_t second[] = { 0xFF, 0x8A, 0x71, 0x00, 0x00 };

  ShimBtReply_send(first, sizeof(first));
  expectU("first held", ShimBtReply_isPending(), 1);
  /* The second would fit in the reserve on its own, but must not overtake */
  uint32_t before = wireLen;
  ShimBtReply_send(second, sizeof(second));
  expectU("second not written ahead of the first", wireLen, before);

  drain(RING_SIZE);
  ShimBtReply_retry();
  expectU("both released", ShimBtReply_isPending(), 0);
  expectU("both written", wireLen, before + sizeof(first) + sizeof(second));
  expectTrue("first then second",
      memcmp(&wire[before], first, sizeof(first)) == 0
          && memcmp(&wire[before + sizeof(first)], second, sizeof(second)) == 0);
}

static void testRetryWithoutRoomKeepsReply(void)
{
  testCase("a retry with too little room keeps the reply held, whole");
  reset();
  fillWithData(24);
  static uint8_t longRsp[RESPONSE_PACKET_SIZE];
  memset(longRsp, 0x5A, sizeof(longRsp));
  ShimBtReply_send(longRsp, sizeof(longRsp));
  uint32_t before = wireLen;
  drain(8); /* a little room, not enough */
  ShimBtReply_retry();
  expectU("still held", ShimBtReply_isPending(), 1);
  expectU("no partial write", wireLen, before);
}

static void testHoldOverflowCounted(void)
{
  testCase("a reply that overflows the hold buffer is dropped and counted");
  reset();
  fillWithData(24);
  uint16_t drops = ShimBtReply_getDropCount();
  static uint8_t big[RESPONSE_PACKET_SIZE + 2U];
  memset(big, 0x11, sizeof(big));
  expectU("first held", ShimBtReply_send(big, sizeof(big)), 0);
  expectU("second dropped", ShimBtReply_send(big, sizeof(big)), 1);
  expectU("drop counted", ShimBtReply_getDropCount(), drops + 1U);
  expectU("first still held", ShimBtReply_isPending(), 1);
}

static void testResetForgetsHeldReply(void)
{
  testCase("clearing the ring forgets a held reply");
  reset();
  fillWithData(24);
  static uint8_t longRsp[RESPONSE_PACKET_SIZE];
  ShimBtReply_send(longRsp, sizeof(longRsp));
  ShimBtReply_reset();
  expectU("not held", ShimBtReply_isPending(), 0);
  uint32_t writes = replyWrites;
  ShimBtReply_retry();
  expectU("retry writes nothing", replyWrites, writes);
}

static void testEmptyReplyIgnored(void)
{
  testCase("an empty reply writes nothing");
  reset();
  expectU("send result", ShimBtReply_send(wire, 0), 0);
  expectU("no write", replyWrites, 0);
  expectU("not held", ShimBtReply_isPending(), 0);
}

static void testReserveFitsHoldBuffer(void)
{
  testCase("the hold buffer always fits in an empty ring");
  /* Otherwise a held reply could never be released */
  expectTrue("BT_PENDING_REPLY_SIZE < ring", BT_PENDING_REPLY_SIZE < RING_SIZE - 1U);
}

int main(void)
{
  hostTestSilenceUnused();
  testReplyOnIdleRing();
  testDataLeavesRoomForReply();
  testLongReplyHeldThenSent();
  testOrderKeptBehindHeldReply();
  testRetryWithoutRoomKeepsReply();
  testHoldOverflowCounted();
  testResetForgetsHeldReply();
  testEmptyReplyIgnored();
  testReserveFitsHoldBuffer();
  return hostTestReport("test_bt_reply");
}

#endif /* SHIMMER_HOST_TEST */
