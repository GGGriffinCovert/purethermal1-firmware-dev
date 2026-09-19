/*
 * wedge_lab.h - v1.3.0 wedge lab (scratch branch, not for release).
 *
 * v1.3.0 wedges roughly once per ~120 single-frame grabs. This adds counters
 * and a set of recovery actions that can be fired at a held wedge, one at a
 * time, to find out which one (if any) brings frames back. Nothing here runs
 * unless asked: with LAB_AUTO_CMD = 0 (default) and g_lab.cmd left at 0 the
 * acquisition path behaves exactly like 1.3.0.
 *
 * Two ways to fire an action:
 *   - by hand: write the action number to g_lab.cmd over SWD (Memory tab,
 *     Hot plug). It reads back 0 once the action has run.
 *   - automatically: build with `make LAB_AUTO=n`; action n fires every time
 *     the firmware detects the wedge (see LAB_WEDGE_*).
 *
 * Find the struct with:  arm-none-eabi-nm main.out | grep g_lab
 */
#ifndef WEDGE_LAB_H_
#define WEDGE_LAB_H_

#include <stdint.h>

#define LAB_MAGIC (0x1AB0C0DEu)

#ifndef LAB_AUTO_CMD
#define LAB_AUTO_CMD (0)
#endif

/* Wedge detector: either signature fires it. */
#define LAB_WEDGE_BAD_RUN      (60)    /* consecutive rejected VSYNC reads */
#define LAB_WEDGE_DISCARD_RUN  (5000)  /* consecutive discards in one v1.3.0 walk
                                          (a healthy walk sees < ~100) */

enum lab_cmd {
  LAB_CMD_NONE       = 0,
  LAB_CMD_CS_ONLY    = 1,  /* /CS high, SCK idle for `arg` ms, /CS low; then
                              back to VSYNC-triggered reads */
  LAB_CMD_CS_WALK    = 2,  /* as 1, then clock continuously, one packet at a
                              time, to packet 0 and read the rest of that
                              segment (the full datasheet procedure) */
  LAB_CMD_IDLE_WALK  = 3,  /* control for 2: SCK idle `arg` ms with /CS LOW,
                              then the same walk */
  LAB_CMD_CCI_CYCLE  = 4,  /* LPM2, 250 ms, power on, 250 ms, VSYNC + stream
                              format config (the old CCI escape hatch) */
  LAB_CMD_HW_RESET   = 5,  /* /CS high; RESET_L + PWR_DWN_L as at boot; 1.5 s;
                              type/VSYNC/LUT + format config; `arg` ms more
                              with /CS high; /CS low; walk as 2 */
  LAB_CMD_SPI_REINIT = 6,  /* MCU side only: reset SPI2 (RCC) and both DMA
                              streams with /CS held low and SCK held idle, then
                              walk as 2 */
  LAB_CMD_MCU_RESET  = 7,  /* NVIC_SystemReset(): the known cure, as a control.
                              Counters are lost and USB re-enumerates. */
};

struct wedge_lab {
  uint32_t magic;            /* +0x00  0x1AB0C0DE */
  uint32_t cmd;              /* +0x04  WRITE an action here; reads 0 when done */
  uint32_t arg;              /* +0x08  hold time in ms for 1/2/3/5 (0 = 250) */
  uint32_t last_cmd;         /* +0x0C  last action run */
  uint32_t cmds_done;        /* +0x10  actions run (manual + auto) */
  uint32_t frames_ok_before; /* +0x14  frames_ok when the last action started;
                                        frames_ok moving past it = recovered */
  uint32_t vsync_irqs;       /* +0x18  EXTI13 callbacks */
  uint32_t reads;            /* +0x1C  VSYNC-triggered reads completed */
  uint32_t frames_ok;        /* +0x20  reads that validated */
  uint32_t frames_bad;       /* +0x24  reads rejected */
  uint32_t bad_run;          /* +0x28  current run of rejected reads */
  uint32_t worst_bad_run;    /* +0x2C  longest run seen */
  uint32_t resyncs;          /* +0x30  v1.3.0 resync walks entered */
  uint32_t walk_packets;     /* +0x34  packets read by all walks */
  uint32_t walk_discard_run; /* +0x38  consecutive discards in the current or
                                        last v1.3.0 walk (thousands = stuck) */
  uint32_t walk_last_header; /* +0x3C  ID word of the last packet a walk read */
  uint32_t lab_walk_found;   /* +0x40  action walks that reached packet 0 */
  uint32_t lab_walk_giveups; /* +0x44  action walks that hit the 2000 cap */
  uint32_t cs_stuck_low;     /* +0x48  PB12 driven high but read back low */
  uint32_t cs_idr_last;      /* +0x4C  GPIOB IDR sampled after last /CS release */
  uint32_t wedges_detected;  /* +0x50  detector fired (both signatures) */
  uint32_t auto_fired;       /* +0x54  LAB_AUTO_CMD actions fired */
  uint32_t auto_recovered;   /* +0x58  a frame validated after an auto action,
                                        before the detector fired again */
  uint32_t auto_cmd;         /* +0x5C  LAB_AUTO_CMD this build was made with */
  uint32_t state;            /* +0x60  1 idle, 2 wait VSYNC, 3 read, 4 v1.3.0
                                        walk, 5 action */
};

extern volatile struct wedge_lab g_lab;

#endif /* WEDGE_LAB_H_ */
