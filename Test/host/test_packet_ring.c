/*
 * Host-side test for the sample packet ring.
 *
 * shimmer_packet_ring.c depends on nothing but <stdint.h> and <string.h>, so
 * this needs no MCU toolchain, no HAL and no target hardware. Run by
 * .github/workflows/host-tests.yml.
 *
 * WHAT THIS IS FOR. The ring is driven from three contexts - a timer ISR, a
 * gather completion, and the task loop - and the interleavings that matter
 * cannot be produced on demand on hardware: they need an SD write to run long
 * enough to delay a gather past the fail-safe. Here they are just a sequence of
 * calls, so the failure that took a customer's recording apart is a handful of
 * lines and runs on every push.
 *
 * WHY THE GUARD BELOW - do not remove it. Both consuming firmware projects add
 * this repository as a source-path root with no exclusions (see the
 * log-and-stream-common <entry kind="sourcePath"> in the Shimmer3R .cproject),
 * so every .c file under it is discovered and compiled into the firmware. This
 * file defines main(). Without the guard it collides with the firmware's own
 * main() at link time, in both the STM32 and the MSP430 build. The guard makes
 * the translation unit empty for anyone who does not ask for it; only the CI
 * workflow passes -DSHIMMER_HOST_TEST.
 */
#if defined(SHIMMER_HOST_TEST)

#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "Sensing/shimmer_packet_ring.h"

/* 504.123 Hz - the customer's rate, and the one the field evidence is in. */
#define PERIOD_TICKS 65U

static int failures;

static void expect(const char *what, uint32_t got, uint32_t want)
{
  if (got != want)
  {
    printf("  FAIL %-46s got %lu want %lu\n", what, (unsigned long) got,
        (unsigned long) want);
    failures++;
  }
}

/* A sample tick, a gather and a drain are the only three things that touch the
 * ring. Modelling the sample clock explicitly keeps the timestamps in the tests
 * the same shape as the ones in a real file. */
typedef struct
{
  PacketRing ring;
  uint32_t nowTicks;
} Sim;

static void sim_init(Sim *sim, uint16_t stallLimitPeriods)
{
  PktRing_init(&sim->ring, 0x00);
  sim->ring.stallLimitPeriods = stallLimitPeriods;
  sim->nowTicks = 1000000U; /* mid-range, far from a real 24-bit wrap */
}

/* Mirrors ShimSens_saveTimestampToPacket(): the 32-bit tick for the firmware's
 * own use, and its low three bytes little-endian into the packet. */
static void sim_stamp(Sim *sim)
{
  PACKETBufferTypeDef *slotPtr = PktRing_wrSlot(&sim->ring);

  slotPtr->timestampTicks = sim->nowTicks;
  slotPtr->dataBuf[PACKET_TIMESTAMP_IDX + 0] = (uint8_t) (sim->nowTicks >> 0);
  slotPtr->dataBuf[PACKET_TIMESTAMP_IDX + 1] = (uint8_t) (sim->nowTicks >> 8);
  slotPtr->dataBuf[PACKET_TIMESTAMP_IDX + 2] = (uint8_t) (sim->nowTicks >> 16);
}

/* One sample timer interrupt, and what ShimSens_sampleTimerTriggered() does
 * with the answer. */
static PktTickAction sim_tick(Sim *sim)
{
  PktTickAction action;

  sim->nowTicks += PERIOD_TICKS;
  action = PktRing_onTick(&sim->ring);
  if (action == PKT_TICK_START)
  {
    PktRing_markInProgress(&sim->ring);
    sim_stamp(sim);
  }
  return action;
}

/* One gather completion, as ShimSens_stageCompleteCb() delivers it. */
static uint8_t sim_gather_complete(Sim *sim)
{
  return PktRing_onComplete(&sim->ring);
}

/* One TASK_SAVEDATA run. Records the timestamp of every packet handed to the
 * SD and Bluetooth writers, which is exactly what ends up in a .bin file. */
static uint8_t sim_drain(Sim *sim, uint32_t *emittedOut, uint8_t emittedMax)
{
  uint8_t budget = PktRing_count(&sim->ring);
  uint8_t emitted = 0;
  PACKETBufferTypeDef *slotPtr;

  while (budget-- && PktRing_drainNext(&sim->ring, &slotPtr))
  {
    if (emitted < emittedMax)
    {
      emittedOut[emitted] = slotPtr->timestampTicks;
    }
    emitted++;
    PktRing_drainDone(&sim->ring);
  }
  return emitted;
}

/* A packet starts, is gathered, and is written out. */
static void test_normal_cycle(void)
{
  Sim sim;
  uint32_t emitted[4];
  uint32_t firstTs;

  printf("test_normal_cycle\n");
  sim_init(&sim, PKT_RING_STALL_LIMIT_DEFAULT);

  expect("tick starts a packet", sim_tick(&sim), PKT_TICK_START);
  firstTs = sim.nowTicks;
  expect("completion accepted", sim_gather_complete(&sim), 1);
  expect("one packet queued", PktRing_count(&sim.ring), 1);
  expect("one packet emitted", sim_drain(&sim, emitted, 4), 1);
  expect("emitted the stamp it started with", emitted[0], firstTs);
  expect("ring drained", PktRing_count(&sim.ring), 0);
  expect("nothing was skipped", sim.ring.diag.drainSkipped, 0);
}

/*
 * The DEV-1023 field interleaving, as reconstructed from a customer .bin.
 *
 * An SD write holds the main loop long enough that the gather queued by tick 1
 * has not run by tick 5, so the fail-safe releases the slot. Before the guards
 * this is where it came apart: the stale gather ran against the released slot,
 * a tick restarted that same slot mid-gather, and the completion of the stale
 * gather closed it - leaving a second gather queued that landed on the next
 * slot, which nothing had stamped. That slot went to the card with a 00 00 00
 * timestamp, and Consensys read it as a 24-bit roll-over worth 512 s.
 *
 * With the entry guard the stale gather never runs, so there is no second
 * gather and no unstamped packet. The cost is the samples the stall ate, which
 * is what a dropped sample should cost.
 */
static void test_field_interleaving_is_contained(void)
{
  Sim sim;
  uint32_t emitted[4];
  uint32_t restartTs;
  uint8_t i;

  printf("test_field_interleaving_is_contained\n");
  sim_init(&sim, PKT_RING_STALL_LIMIT_DEFAULT);

  /* tick 1: the packet starts and a gather is queued behind the SD write. */
  expect("tick 1 starts a packet", sim_tick(&sim), PKT_TICK_START);

  /* ticks 2-4: the gather has not run; the packet is still in progress. */
  for (i = 0; i < PKT_RING_STALL_LIMIT_DEFAULT; i++)
  {
    expect("tick while gather outstanding", sim_tick(&sim), PKT_TICK_BUSY);
  }

  /* tick 5: the fail-safe gives up and releases the slot. */
  expect("tick 5 releases the slot", sim_tick(&sim), PKT_TICK_STALL_RESET);
  expect("one stall reset recorded", sim.ring.diag.stallResets, 1);

  /* The SD write finishes and the queued gather is finally dispatched - but
   * the packet it was queued for no longer exists. This is the guard that
   * stops the cascade. */
  expect("stale gather refused", PktRing_gatherMayProceed(&sim.ring), 0);
  expect("refusal recorded", sim.ring.diag.gathersRefused, 1);

  /* tick 6 starts a fresh packet, 390 ticks after the last good sample - the
   * gap that identifies this fault in a file. */
  expect("tick 6 starts a fresh packet", sim_tick(&sim), PKT_TICK_START);
  restartTs = sim.nowTicks;
  expect("restart is 6 sample periods on", restartTs - 1000000U - PERIOD_TICKS,
      5U * PERIOD_TICKS);

  /* Its own gather is allowed through and completes normally. */
  expect("fresh gather allowed", PktRing_gatherMayProceed(&sim.ring), 1);
  expect("completion accepted", sim_gather_complete(&sim), 1);

  expect("exactly one packet queued", PktRing_count(&sim.ring), 1);
  expect("exactly one packet emitted", sim_drain(&sim, emitted, 4), 1);
  expect("and it carries the stamp it started with", emitted[0], restartTs);
  expect("no unstamped packet was written", sim.ring.diag.drainSkipped, 0);
}

/* On Shimmer3R the gather is asynchronous and its completion interrupt runs at
 * a higher priority than the sample tick, so a completion can arrive after the
 * fail-safe has already released the packet it belonged to. It must not close
 * a slot that is no longer in progress. */
static void test_stale_completion_is_dropped(void)
{
  Sim sim;
  uint32_t emitted[4];
  uint32_t restartTs;
  uint8_t i;

  printf("test_stale_completion_is_dropped\n");
  sim_init(&sim, PKT_RING_STALL_LIMIT_DEFAULT);

  expect("packet starts", sim_tick(&sim), PKT_TICK_START);
  for (i = 0; i < PKT_RING_STALL_LIMIT_DEFAULT; i++)
  {
    sim_tick(&sim);
  }
  expect("fail-safe releases the slot", sim_tick(&sim), PKT_TICK_STALL_RESET);

  /* The abandoned gather finishes now. */
  expect("stale completion dropped", sim_gather_complete(&sim), 0);
  expect("drop recorded", sim.ring.diag.completionsDropped, 1);
  expect("write index did not move", PktRing_count(&sim.ring), 0);

  /* Sampling carries on unharmed. */
  expect("next packet starts", sim_tick(&sim), PKT_TICK_START);
  restartTs = sim.nowTicks;
  expect("its completion accepted", sim_gather_complete(&sim), 1);
  expect("one packet emitted", sim_drain(&sim, emitted, 4), 1);
  expect("with the right stamp", emitted[0], restartTs);
}

/* Belt to the completion guard's braces: if an unstamped slot ever does end up
 * inside the drain window, it is dropped rather than written out. */
static void test_drain_skips_incomplete_slot(void)
{
  Sim sim;
  uint32_t emitted[4];
  uint32_t secondTs;

  printf("test_drain_skips_incomplete_slot\n");
  sim_init(&sim, PKT_RING_STALL_LIMIT_DEFAULT);

  sim_tick(&sim);
  sim_gather_complete(&sim);
  sim_tick(&sim);
  secondTs = sim.nowTicks;
  sim_gather_complete(&sim);
  expect("two packets queued", PktRing_count(&sim.ring), 2);

  /* Plant an unstamped slot where the first packet is. */
  PktRing_rdSlot(&sim.ring)->samplingStatus = SAMPLING_PACKET_IDLE;
  PktRing_rdSlot(&sim.ring)->timestampTicks = 0;

  expect("only the complete packet is emitted", sim_drain(&sim, emitted, 4), 1);
  expect("and it is the second one", emitted[0], secondTs);
  expect("the incomplete slot was skipped", sim.ring.diag.drainSkipped, 1);
  expect("ring fully drained", PktRing_count(&sim.ring), 0);
}

/* Sampling stops while the drain is behind, and resumes once it catches up. */
static void test_ring_full(void)
{
  Sim sim;
  uint32_t emitted[DATA_BUF_QTY];
  uint8_t i;

  printf("test_ring_full\n");
  sim_init(&sim, PKT_RING_STALL_LIMIT_DEFAULT);

  for (i = 0; i < DATA_BUF_QTY_IN_USE; i++)
  {
    expect("tick starts a packet", sim_tick(&sim), PKT_TICK_START);
    expect("completion accepted", sim_gather_complete(&sim), 1);
    expect("count never exceeds the in-use limit",
        PktRing_count(&sim.ring) <= DATA_BUF_QTY_IN_USE, 1);
  }

  expect("ring reports full", PktRing_isFull(&sim.ring), 1);
  expect("tick refused while full", sim_tick(&sim), PKT_TICK_FULL);
  expect("refusal recorded", sim.ring.diag.startsRefusedFull, 1);
  /* The write index always moves on a completion, so filling the ring can
   * never strand a completed slot under it - which would stall sampling for
   * good, because no tick can start on a slot that is already complete. */
  expect("write slot is not left completed",
      PktRing_wrSlot(&sim.ring)->samplingStatus != SAMPLING_COMPLETE, 1);

  expect("all packets emitted", sim_drain(&sim, emitted, DATA_BUF_QTY), DATA_BUF_QTY_IN_USE);
  expect("sampling resumes after the drain", sim_tick(&sim), PKT_TICK_START);
}

/* The fail-safe fires on the tick after the limit, and its counter restarts
 * with each new packet. */
static void test_stall_boundary(void)
{
  Sim sim;
  uint8_t i;

  printf("test_stall_boundary\n");
  sim_init(&sim, 5U);

  expect("packet starts", sim_tick(&sim), PKT_TICK_START);
  for (i = 0; i < 5U; i++)
  {
    expect("busy up to the limit", sim_tick(&sim), PKT_TICK_BUSY);
  }
  expect("reset on the tick after the limit", sim_tick(&sim), PKT_TICK_STALL_RESET);

  /* A fresh packet starts the count again. */
  expect("packet restarts", sim_tick(&sim), PKT_TICK_START);
  expect("stall count restarted", sim.ring.stallCount, 0);

  /* Disabled means never. */
  sim_init(&sim, PKT_RING_STALL_LIMIT_DISABLED);
  expect("packet starts", sim_tick(&sim), PKT_TICK_START);
  for (i = 0; i < 50U; i++)
  {
    expect("never gives up when disabled", sim_tick(&sim), PKT_TICK_BUSY);
  }
  expect("no stall resets recorded", sim.ring.diag.stallResets, 0);
}

/* The indices are free-running 16-bit counters masked at use, so the only
 * interesting moment is where they wrap. */
static void test_index_wrap(void)
{
  Sim sim;
  uint32_t emitted[2];
  uint8_t i;

  printf("test_index_wrap\n");
  sim_init(&sim, PKT_RING_STALL_LIMIT_DEFAULT);
  sim.ring.wrIdx = sim.ring.rdIdx = 0xFFF8U;

  for (i = 0; i < 30U; i++)
  {
    uint32_t startedTs;
    expect("tick starts a packet across the wrap", sim_tick(&sim), PKT_TICK_START);
    startedTs = sim.nowTicks;
    expect("completion accepted", sim_gather_complete(&sim), 1);
    expect("one packet emitted", sim_drain(&sim, emitted, 2), 1);
    expect("stamp survives the wrap", emitted[0], startedTs);
    expect("ring empty again", PktRing_isEmpty(&sim.ring), 1);
  }
}

/* A completion that lands while the drain is running is picked up by the next
 * drain, exactly once. */
static void test_completion_during_drain(void)
{
  Sim sim;
  uint32_t emitted[4];
  uint32_t firstTs, secondTs;

  printf("test_completion_during_drain\n");
  sim_init(&sim, PKT_RING_STALL_LIMIT_DEFAULT);

  sim_tick(&sim);
  firstTs = sim.nowTicks;
  sim_gather_complete(&sim);

  /* The drain snapshots one packet. */
  expect("first drain emits one", sim_drain(&sim, emitted, 4), 1);
  expect("first drain emitted the first packet", emitted[0], firstTs);

  /* A second packet completes afterwards. */
  sim_tick(&sim);
  secondTs = sim.nowTicks;
  sim_gather_complete(&sim);

  expect("second drain emits one", sim_drain(&sim, emitted, 4), 1);
  expect("second drain emitted the second packet", emitted[0], secondTs);
  expect("nothing left behind", PktRing_count(&sim.ring), 0);
}

int main(void)
{
  printf("packet ring host tests\n\n");

  test_normal_cycle();
  test_field_interleaving_is_contained();
  test_stale_completion_is_dropped();
  test_drain_skips_incomplete_slot();
  test_ring_full();
  test_stall_boundary();
  test_index_wrap();
  test_completion_during_drain();

  printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "PASSED", failures,
      failures == 1 ? "" : "s");
  return failures ? 1 : 0;
}

#endif /* SHIMMER_HOST_TEST */
