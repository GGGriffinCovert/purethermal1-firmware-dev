# PureThermal "sticky frame grab" — investigation handoff

Status as of 2026-09-18. Written so a fresh session can pick up without
repeating work. Everything below is either measured or explicitly flagged as
hypothesis.

---

## The bug

Repeated single-frame V4L2 grabs (open → STREAMON → DQBUF → STREAMOFF → close)
eventually wedge the camera. Once wedged it never produces another frame.
Reproduces on Raspbian, Ubuntu, Windows and Jetson. Long-standing; upstream
issue #23 is a *different* bug (RPi dwc_otg FIQ timeouts).

**Hardware:** PureThermal, Lepton 3/3.5, 160x120 UYVY. MCU reports STM32F412,
1 MB flash. Test unit serial `0053001e-…`.

**Build:** master builds as `-DSTM32F411xE` with `STM32F411CEUx_FLASH.ld`
(128 K RAM / 512 K flash) even on the F412. Works, but leaves half the part
unaddressed. Build on Ubuntu: `sudo apt install gcc-arm-none-eabi
binutils-arm-none-eabi make`, then `make clean && make`. Ignore the readme's
`terry.guo` PPA, long dead. Flash with `st-flash --reset write main.bin
0x8000000`. Version string comes from `git describe`, so `dmesg` shows the
running build — confirm flashes that way.

**Keep `USART_DEBUG` OFF for soaks.** With it on, `DEBUG_PRINTF` calls `_write`
which blocks ~5 ms in `HAL_UART_Transmit` inside the resync loop, changing the
timing of the thing being measured.

The Makefile has no `-g`, so gdb cannot print `g_dbg` or other typed symbols
from `main.out` — read raw addresses (map file / `arm-none-eabi-nm`) instead.

---

## Reproduction

`scripts/repro_sticky_grab.sh` — Mode A is N single-frame grabs with teardown
between each; Mode B is N frames in one continuous stream.

- **Mode B is always clean.** Continuous streaming is healthy. The bug is
  entirely in the teardown/restart path.
- A sysfs `authorized` toggle re-enumerates the USB device but does **not**
  reset the STM32, so it does not clear the wedge. Only removing power does.
  The script defaults to prompting for a physical replug for this reason.
- `scripts/probe_wedged.sh` — run while wedged; separates control path (EP0)
  from video path (EP 0x81).
- A single hanging `ffmpeg` frame grab holds the wedge open indefinitely,
  which is how live counters get read.

Failure rate is a **random hazard per teardown**, not a counter running out.
First-failure iteration scatters enormously (observed 20, 108, 113, 163, 173,
590, 2995, 12000). Any single run proves almost nothing — pool across runs.

| build state | grabs | wedges | rate |
|---|---|---|---|
| pre-fix | 246 | 2 | 1 in 123 |
| + EXTI fix | 284 | 2 | 1 in 142 |
| + breadcrumbs | 3157 | 2 | 1 in 1579 |
| + packet-0 + VSYNC + escape hatch | ~15500 | 3 | 1 in 5200 |

The last row is ~4× better than the third, p ≈ 0.09 — suggestive, not
established, and confounded by `USART_DEBUG` being turned off at the same time.

---

## Instrumentation

`Inc/dbg_counters.h` defines `volatile struct dbg_counters g_dbg`. Read it over
SWD with STM32CubeProgrammer in **Hot plug** mode (Normal/Under-reset resets the
MCU and destroys the evidence). Memory tab, Address + Size, 32-bit.

**`g_dbg` moves between builds — always `grep g_dbg main.map` first.** Field
offsets from the base are stable; new fields are only ever appended.

| offset | field |
|---|---|
| +0x00 | `magic` = `0xDBC0FFEE` (sanity check) |
| +0x04 | `vsync_irqs` |
| +0x08 | `transfers_done` |
| +0x0C | `transfer_fails` |
| +0x10 | `desync_events` |
| +0x14 | `resync_entries` |
| +0x18 | `resync_packets` |
| +0x1C | `frames_completed` |
| +0x20 | `frames_dropped` |
| +0x24 | `phase` (see `enum dbg_phase`) |
| +0x28 | `phase_entry_ms` |
| +0x2C | `main_loop_ticks` |
| +0x30 | `stall_detected` |
| +0x34 | `stall_phase` |
| +0x38 | `stall_ms` |
| +0x3C | `phase_max_ms[16]` |
| +0x7C | `phase_count[16]` |
| +0xBC | `first_line_bad` |
| +0xC0 | `resync_giveups` |
| +0xC4 | `vsync_cfg_fails` |
| +0xC8 | `hard_recoveries` |
| +0xCC | `worst_desync_run` |

`main_loop_ticks` distinguishes *blocked* from *waiting* — it advances at
~470 k/s while the scheduler runs. `phase` alone cannot, because several phases
legitimately sit for a long time.

Static locals are not in `main.map` (local symbols); use
`arm-none-eabi-nm main.out | grep <name>`.

`scripts/analyze_vospi_dump.py` decodes a raw dump of `lepton_buffers[]`
(4 × 15384 = `0xF060` bytes from the `lepton_buffers` address in `main.map`)
and reports whether the VoSPI packet headers sit at byte 0 of each 244-byte
record, at some other halfword offset, or at a bit offset. `--selftest` checks
it against synthetic buffers.

### SWD register map for the SPI path

**SPI2 is serviced by DMA1, not DMA2.** SPI2_RX = DMA1 Stream 3 ch0, SPI2_TX =
DMA1 Stream 4 ch0 (`stm32f4xx_hal_msp.c`). DMA2 Stream 0 is
`hdma_memtomem_dma2_stream0`, initialised in `MX_DMA_Init()` and never used, so
`DMA2_LISR` says nothing about this bug.

| address | register | expected when idle | abnormal |
|---|---|---|---|
| `0x40003800` | SPI2_CR1 | `0x0947` (SPE, MSTR, DFF=16-bit, CPOL/CPHA=1, BR=/2; SSI set by HAL, inert) | SPE (bit 6) or MSTR (bit 2) clear |
| `0x40003804` | SPI2_CR2 | `0x0004` (SSOE); `0x0007` mid-transfer | DMA enables set while DMA1 streams are idle |
| `0x40003808` | SPI2_SR | `0x0002` (TXE) | OVR b6, MODF b5, BSY b7 while idle, RXNE b0 |
| `0x40026000` | DMA1_LISR | stream 3 = bits 22–27; HTIF3 (b26) may be set, harmless | TEIF3 b25, DMEIF3 b24 |
| `0x40026004` | DMA1_HISR | stream 4 = bits 0–5; HTIF4 (b4) may be set, harmless | TEIF4 b3, DMEIF4 b2 |
| `0x40026058` | DMA1_S3CR (RX) | `0x00032C00`; `0x00032C11` mid-transfer | EN set with NDTR frozen; TCIE set while idle |
| `0x4002605C` | DMA1_S3NDTR | `0` | non-zero and not changing |
| `0x40026060` | DMA1_S3PAR | `0x4000380C` (SPI2_DR) | anything else |
| `0x40026064` | DMA1_S3M0AR | `lepton_buffers + n×15384` | outside `lepton_buffers` |
| `0x40026070` | DMA1_S4CR (TX) | `0x00032C40`; `0x00032C51` mid-transfer | as S3 |
| `0x40026074` | DMA1_S4NDTR | `0` | ≠ S3NDTR while both idle |
| `0x4002607C` | DMA1_S4M0AR | same as S3M0AR | different from S3M0AR |
| `0x40020400` | GPIOB_MODER | bits 25:24 = `10` (PB12 = SPI2_NSS) | — |
| `0x40020410` | GPIOB_IDR | **bit 12 = 0: /CS is asserted, always** | — |
| `0x40013C14` | EXTI_PR | bit 13 = pending VSYNC edge | — |

---

## Measured facts

Taken on wedged units, 10 s apart, fault held open with ffmpeg.

**The MCU is alive.** `uwTick` advances, `main_loop_ticks` advances, USB
enumerates, EP0 control transfers answer in ~100 ms, Lepton I2C answers.

**Every frame is rejected, forever.** Over one interval: `transfers_done`
+1170, `desync_events` +1170 — 100%, not 99%. `resync_entries` +390 = exactly
1170/3, matching the `current_frame_count > 2` gate. `resync_packets` +27290 =
70.0 per resync, every time. `frames_completed` **+0** across 89 seconds.
`transfer_fails` 0 — because `complete_lepton_transfer()` is a TODO stub that
validates nothing.

**The rejecting comparison** is
`last_end_line != (IMAGE_NUM_LINES + g_telemetry_num_lines - 1)`.
On a wedged unit `last_end_line` reads **0xFF** with `g_telemetry_num_lines`
= 0, so it expects 59 and sees 255. Meanwhile `current_segment`, read from line
20 of the *same buffer*, is a valid 1. An earlier build measured
`first_line_bad` climbing past 1800 on a wedged unit with `resync_giveups`
static at 3. (Earlier reading: "0xFF is an idle bus, real packets at the front,
nothing at the back." That was never checked against the buffer itself; under
a framing offset these fields are just whatever pixel bytes sit where the
header should be, and TTT=1 is a 1-in-8 coincidence. The buffer dump settles
which.)

**SPI2 is clean (2026-09-18).** `SPI2_SR` = `0x0002` on a wedged unit: TXE only.
OVR, BSY, RXNE, MODF, FRE all clear. The `DMA2_LISR` = 0 read taken with it is
the wrong controller (see the register map).

**The LED** is toggled only in `lepton_task`: 1 Hz in the idle blink loop,
once per *validated* frame while streaming. Frozen means neither — the task is
running but nothing validates. It blinks normally between grab attempts.

**`first_line_bad` ≈ one per grab** during healthy operation. The first
VSYNC-triggered read after each restart lands wherever it lands and resync
corrects it. (Likely cause: the sensor lost VoSPI sync at the previous
teardown, see hypothesis; and/or the EXTI clear at stream start happens
*before* `lepton_power_on()`, so edges from the sensor powering up are latched
and fire the moment the IRQ is enabled.)

**The escape hatch fires and does not help.** `hard_recoveries` = 95 and
climbing on a wedged unit, `worst_desync_run` = 60 (the threshold).

**The Lepton is exonerated** — *with a caveat*. Physically removing the sensor
and re-inserting it did not clear the wedge; only an STM32 reset does.
Caveat (2026-09-18): after re-insertion nothing re-applies the sensor config
(VSYNC GPIO mode, RGB888, AGC) until a stream restart or the escape hatch runs,
and the escape hatch needs desyncs, which need VSYNC edges. So "still wedged"
after a re-insert can also mean "parked in `PHASE_WAIT_BUFFER`, `vsync_irqs`
frozen", which is a different state. Also, if the MCU was clocking SPI at the
moment the sensor booted, the sensor's framing could be offset from birth. If
the counters weren't read after the re-insert, this result is weaker than it
looks — the /CS test below settles the question directly.

Slow grabs of 1–2 s are FFC events on the sensor's automatic interval. Benign.

---

## Fixes committed (branch `fix/uvc-stream-restart`)

1. **EXTI pending-bit clear** — `__HAL_GPIO_EXTI_CLEAR_IT()` takes a pin mask;
   the code passed `EXTI15_10_IRQn` (40 = 0x28), clearing lines 3 and 5 and
   leaving the Lepton's line 13 pending. **Confirmed fix**: eliminated the
   recoverable multi-second stalls entirely (0 in 284 grabs against an 18–27%
   baseline). Did not affect the wedge.
2. **`695c3c7` packet-0 resync** — resync stopped at the first non-discard
   packet; VoSPI only guarantees alignment from packet 0. Also bounded, and
   frame validation now checks the first packet as well as the last. Real
   defect, unproven impact on the wedge.
3. **`1a867ae` VSYNC phase delay restore** — `LEP_SetOemGpioVsyncPhaseDelay`
   and `LEP_SetOemGpioMode` were set once at boot in `set_lepton_type()`, while
   `lepton_task` OEM power-cycles the sensor on every grab. Real defect,
   unproven impact.
4. **`d9a18e8` escape hatch** — after 60 consecutive rejected frames,
   re-initialise the sensor. Fires, but does not recover.
5. **`_write()` retarget in `main.c`** — before this, `USART_DEBUG` builds
   produced **no output at all**: no `_write` existed anywhere and the link uses
   `-specs=nosys.specs`, whose stub discards everything. Every `DEBUG_PRINTF` in
   the tree was writing to nowhere. Arguably a real fix in its own right.

---

## Code defects found in the 2026-09-18 audit (not yet fixed)

1. **/CS is never deasserted.** `MX_SPI2_Init()` uses `SPI_NSS_HARD_OUTPUT`. On
   the F4 that holds NSS low for as long as SPE = 1 ("driven low when the master
   starts the communication and kept low until the SPI is disabled"), and SPE
   is set once in `lepton_init()` and never cleared. The Lepton datasheet's
   only documented (re)sync procedure is "deassert /CS **and** idle SCK for
   > 185 ms". The resync path's 185 ms wait only idles SCK. Nothing after boot
   ever performs a datasheet-compliant VoSPI resync.
2. **The escape hatch never hard-resets the sensor.** It uses CCI
   (`LEP_RunOemLowPowerMode2` / `LEP_RunOemPowerOn`). `RESET_L` (PA9) is only
   toggled in `lepton_init()` at boot. The only recovery that works, an MCU
   reset, is also the only thing that pulses `RESET_L` and floats the SPI pins.
3. **SCK is 24 MHz; the Lepton's max is 20 MHz.** SYSCLK 96 MHz on both PT1
   (14.318 MHz HSE) and PT2 (8 MHz) clock trees, APB1 = 48 MHz, BR = /2. One
   SCK edge missed by the sensor is exactly a bit-level framing offset. Mode B
   being clean argues against it as the main cause. Not trivially fixable:
   /4 gives 12 MHz, below what RGB888 needs (~12.4 Mbit/s), so it would take a
   PLL change to hit 20 MHz.
4. **EXTI clear order.** At stream start the pending bit is cleared *before*
   `lepton_power_on()` and `lepton_restore_vsync_config()`, so edges produced
   while the sensor powers up and GPIO3 changes mode are latched and fire
   immediately at `HAL_NVIC_EnableIRQ`. The per-frame re-enable at the top of
   the loop does no clear at all.
5. **DMA start sequence** (`start_lepton_spi_dma`) clears EN and writes
   NDTR/PAR/M0AR without waiting for EN to read back 0 (writes are ignored while
   EN = 1), and never clears the stream's flags in LIFCR/HIFCR before
   re-enabling, which RM0402 requires. It also enables TCIE on the TX stream.
   Latent: every transfer currently completes, so EN is always already 0.
6. **The resync exit test is weak.** It stops on one halfword with a non-F
   nibble and low byte 0. On a stream whose framing is shifted, that is pixel
   data, and dark RGB888 pixels satisfy it often. So "resync finds packet 0
   every time" is **not** evidence that the stream is aligned. Also, the
   post-resync read (`lepton_transfer(current_buffer, 59)`) writes into line 0,
   overwriting the packet 0 it just found. Harmless, since the buffer is thrown
   away, but the comment is wrong.
7. **No CRC check.** `complete_lepton_transfer()` is a stub. The VoSPI CRC
   would catch a framing error on the first packet, deterministically. The
   datasheet recommends a /CS resync after any CRC error.

---

## Theories that are dead

Do not re-litigate these; each died to a measurement.

- **Unaligned-access hard faults** — an artifact of STM32CubeProgrammer's Fault
  Analyzer setting `UNALIGN_TRP`. The descriptors are `__attribute__((packed))`
  and CFLAGS lack `-mno-unaligned-access`, so packed-field reads fault on
  *every* Probe/Commit GET once the trap is armed. Also, two of the three
  register captures were all-identical values — failed ST-LINK AP reads.
- **`TxState` latch in `usbd_uvc.c`** — would leave the LED blinking.
- **Lepton settle time** — delay sweep was non-monotonic (0 s clean, 0.5 s
  failed, 2 s clean).
- **Blocked in an unbounded SDK poll loop** — `main_loop_ticks` advances.
- **Telemetry line-count divergence** — `g_telemetry_num_lines` reads 0, which
  is correct for the UYVY path, and does not change between healthy and wedged.
- **SPI2 overrun / MCU-side byte misalignment** (the previous leading
  hypothesis) — `SPI2_SR` = `0x0002`, OVR clear. It was also already excluded by
  the counters: per the reference manual, once OVR latches "all other
  subsequently transmitted data are lost", so the RX DMA would stall, every
  transfer would hit the 200 ms timeout, and `transfer_fails` would climb. It
  reads 0. The F4 SPI has no RX FIFO to hold a stale byte, and with DFF = 16-bit
  and the MCU as master generating every clock, the MCU's side cannot be
  byte-shifted.

One methodological note: `ISER1` bit 8 clear is **not** evidence the task failed
to re-enable EXTI. The EXTI callback disables its own IRQ, so that is the normal
state for most of `lepton_task`'s loop body.

---

## Current leading hypothesis (unverified)

**The sensor's VoSPI packet framing no longer matches the MCU's 244-byte
records, and the firmware has no way to re-establish it.**

The datasheet lists three ways to lose VoSPI sync: a packet not clocked out
within 3 line periods, a segment not fully read before the next arrives, and
any segment (unique or invalid frame) left unread. **Every single-frame grab
almost certainly breaks the third rule**: at STREAMOFF the MCU stops reading
while the sensor keeps producing segments until the LPM2 command lands.
Whether LPM2 / CCI power-on resets VoSPI state is not documented. If it
doesn't, every restart begins with the sensor out of sync and depends on the
firmware's recovery. That recovery idles SCK for 185 ms with /CS still low, which is not
the documented procedure. Usually the sensor comes back anyway (that is the
"one `first_line_bad` per grab" floor). The hypothesis is that occasionally it
comes back with its packet boundaries at a different point in the byte stream
than the MCU's, and from then on:

- every read is rejected, 100%, because the "header" the firmware checks is
  pixel data;
- the resync walk still "finds packet 0", because dark pixel data passes its
  one-halfword test (defect 6), which is why `resync_giveups` stays flat;
- USB re-enumeration, sensor CCI power cycles and the escape hatch change
  nothing, since none of them touch /CS or `RESET_L`;
- an MCU reset fixes it, since it hard-resets the sensor and releases the SPI
  pins.
- Mode B never loses sync, so it never wedges.

Not yet explained: the sensor re-insert result (see the caveat above), and the
exact event that creates the offset. Candidates: the sensor's VoSPI restarting
while a premature read is in flight (defect 4 makes a read start the instant
EXTI is enabled after `lepton_power_on()`), or a missed SCK edge at 24 MHz
(defect 3).

### Tests, on the currently hung unit, in this order

Steps 1–2 are non-destructive. Steps 3–4 may clear the wedge, so they go last.

1. **Registers**, Hot plug, core running: the table above. `GPIOB_IDR` bit 12
   = 0 confirms /CS is held asserted while idle.
2. **Halt the core** (MCU core panel → Halt). Any in-flight transfer finishes in
   ~6 ms and nothing starts a new one. Now read the DMA1 registers again (stable
   snapshot), then dump `lepton_buffers` (Address from `main.map`, Size `0xF060`,
   Read, Save As .bin) and `current_buffer` (4 bytes, address from
   `arm-none-eabi-nm main.out | grep current_buffer`). Run
   `python3 scripts/analyze_vospi_dump.py lepton_buffers.bin`.
3. **Sham control:** leave the core halted ~3 s in total, then Run. This idles
   SCK for far longer than the resync's 185 ms, with /CS still low. Watch the
   LED and ffmpeg for ~10 s.
4. **/CS test** (only if still wedged): Halt again, then
   - read `GPIOB_MODER` (`0x40020400`), call it M
   - write `GPIOB_BSRR` (`0x40020418`) = `0x00001000` (ODR12 = 1)
   - write `GPIOB_MODER` = `(M & ~0x03000000) | 0x01000000` → PB12 becomes a GPIO
     driving /CS high; `GPIOB_IDR` bit 12 should now read 1
   - wait ≥ 1 s
   - write `GPIOB_MODER` = M → PB12 back to NSS, /CS low with SCK idle
   - Run, watch the LED / ffmpeg / `frames_completed`.

   If the host re-enumerates during the halt, do one fresh grab afterwards
   before judging.

| dump says | sham | /CS test | conclusion |
|---|---|---|---|
| FRAMING OFFSET or BIT SLIP | no | recovers | **confirmed**: fix = datasheet resync (below) |
| FRAMING OFFSET or BIT SLIP | recovers | — | sensor times out on SCK idle alone, but only after > 185 ms: lengthen the idle *and* add /CS |
| ALIGNED/PARTIAL | — | — | framing is fine; the problem is VSYNC-to-read timing (packet level). Look at EXTI/latency, defect 4 |
| NO VOSPI | — | — | the data didn't come from the sensor: check S3M0AR vs `current_buffer`, S3NDTR, S3PAR |
| any | no | no | MCU-side state: compare the DMA1 snapshot against the table |

A dump of a *healthy* streaming unit is a useful baseline (published buffers
have their RGB bytes swapped in place, so CRC fails on those. Rely on the
headers.)

### Fix sketch (apply only after step 4 confirms)

```c
/* lepton.c: datasheet VoSPI (re)sync, step 1. NSS is a hardware output that
 * stays low while SPE = 1, so take PB12 away from the SPI briefly and drive it
 * high as a GPIO. Caller guarantees no transfer is in flight. */
void lepton_cs_deassert(void)
{
  while (SPI2->SR & SPI_SR_BSY) {}
  GPIOB->BSRR  = GPIO_PIN_12;                                        /* ODR12 = 1 */
  GPIOB->MODER = (GPIOB->MODER & ~(3u << 24)) | (1u << 24);          /* output */
}

void lepton_cs_assert(void)
{
  GPIOB->MODER = (GPIOB->MODER & ~(3u << 24)) | (2u << 24);          /* AF5 NSS */
}
```

In `lepton_task`:

- resync path: wrap the existing 185 ms wait in `lepton_cs_deassert()` /
  `lepton_cs_assert()` and make it ≥ 200 ms;
- escape hatch: deassert /CS before `lepton_low_power()`, assert after the
  settle wait;
- stream start: after `lepton_restore_vsync_config()`, do a /CS-high ≥ 200 ms,
  *then* clear EXTI pending, then enable the IRQ (defect 4);
- tighten the resync exit and validation: require the packets after packet 0
  to count 1, 2, 3…, and ideally implement the CRC check in
  `complete_lepton_transfer()`.
