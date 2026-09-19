# Wedge lab (scratch branch `scratch/wedge-lab`, based on tag `1.3.0`)

Not for release. v1.3.0 wedges roughly once per ~120 single-frame grabs, so it
is the fastest way to get a wedge to experiment on. This branch leaves 1.3.0's
acquisition path unchanged and adds:

- `g_lab`, a counter block (`Inc/wedge_lab.h`), and
- seven recovery actions that can be fired at a held wedge, by hand over SWD
  or automatically when the firmware detects the wedge.

With no action requested (the default), behaviour is 1.3.0's. The build has
the same warnings as 1.3.0.

## Build and find the counters

```
make clean && make                 # manual actions only
make clean && make LAB_AUTO=2      # also fire action 2 whenever a wedge is detected
arm-none-eabi-nm main.out | grep -E ' (g_lab|lepton_buffers)$'
```

`make clean` is needed when changing `LAB_AUTO` (it's a compiler flag).
`scripts/repro_sticky_grab.sh` and `scripts/analyze_vospi_dump.py` are copied
from `fix/uvc-stream-restart` so the test runs from this branch. The
`lepton_buffers` layout is 1.3.0's, the same as on the fix branch, so the
analyzer works unchanged.

## Actions

Write the number to `g_lab.cmd` (+0x04). It reads back 0 once the action has
run. Optional `g_lab.arg` (+0x08) sets the hold time in ms for 1/2/3/5
(0 = 250). Actions run only while a stream is open (a grab is holding it).

| cmd | action | what it isolates |
|---|---|---|
| 1 | /CS high, SCK idle `arg` ms, /CS low, back to VSYNC reads | /CS timeout alone (what `9bae07e`'s escape hatch did) |
| 2 | as 1, then clock continuously to packet 0 and read the rest of that segment | the full datasheet procedure |
| 3 | SCK idle `arg` ms with /CS **low**, then the same walk | control for 2: is /CS the ingredient? |
| 4 | LPM2, 250 ms, power on, 250 ms, VSYNC + format config | the old CCI escape hatch |
| 5 | /CS high; `RESET_L`/`PWR_DWN_L` as at boot; 1.5 s; reinit; `arg` ms; /CS low; walk | sensor hardware reset |
| 6 | reset SPI2 (RCC) and both DMA streams, /CS held low and SCK held idle; walk | MCU SPI/DMA state only |
| 7 | `NVIC_SystemReset()` | the known cure (control); counters are lost |

## Counters (`g_lab`, offsets)

| off | field | meaning |
|---|---|---|
| +0x00 | `magic` | `0x1AB0C0DE` |
| +0x04 | `cmd` | write an action; reads 0 when done |
| +0x08 | `arg` | hold ms (0 = 250) |
| +0x0C | `last_cmd` | last action run |
| +0x10 | `cmds_done` | actions run |
| +0x14 | `frames_ok_before` | `frames_ok` when the last action started |
| +0x18 | `vsync_irqs` | EXTI13 callbacks |
| +0x1C | `reads` | VSYNC-triggered reads completed |
| +0x20 | `frames_ok` | reads that validated |
| +0x24 | `frames_bad` | reads rejected |
| +0x28 | `bad_run` | current run of rejected reads |
| +0x2C | `worst_bad_run` | longest run seen |
| +0x30 | `resyncs` | 1.3.0 resync walks entered |
| +0x34 | `walk_packets` | packets read by all walks |
| +0x38 | `walk_discard_run` | consecutive discards in the current/last 1.3.0 walk |
| +0x3C | `walk_last_header` | ID word of the last packet a walk read (`xFxx` = discard) |
| +0x40 | `lab_walk_found` | action walks that reached packet 0 |
| +0x44 | `lab_walk_giveups` | action walks that hit the 2000-packet cap |
| +0x48 | `cs_stuck_low` | PB12 driven high but read back low (must stay 0) |
| +0x4C | `cs_idr_last` | GPIOB IDR after the last /CS release |
| +0x50 | `wedges_detected` | 60 rejected reads in a row, or 5000 discards in one walk |
| +0x54 | `auto_fired` | `LAB_AUTO` actions fired |
| +0x58 | `auto_recovered` | a read validated after an auto action, same stream |
| +0x5C | `auto_cmd` | the `LAB_AUTO` value this build was made with |
| +0x60 | `state` | 1 idle, 2 wait VSYNC, 3 read, 4 1.3.0 walk, 5 action |

**Recovered** means `frames_ok` moves past `frames_ok_before` (and the LED
toggles at frame rate, ffmpeg returns). Read with the Memory tab in Hot plug
mode. Don't open the MCU core tab: halting the core breaks the USB side
(5 s control timeouts) and muddies every result.

## Suggested runs

**A. Manual, one held wedge, least invasive first.** Wedge the unit with the
repro script, hold it with one ffmpeg grab, and confirm it's the known state:
`walk_discard_run` in the thousands and `walk_last_header` = `xFxx`, or dump
`lepton_buffers` and run the analyzer. Then fire, waiting ~5 s after each and
reading `frames_ok` vs `frames_ok_before`:

3 → 1 → 2 → 4 → 6 → 5 → 7

Stop at the first action that recovers. Because an earlier action could have
loosened the state, confirm the winner by using it *first* on a fresh wedge.

**B. Automatic A/B.** `make clean && make LAB_AUTO=n`, flash, run Mode A as
usual (`-t 30`). Each detection fires action n. Compare `auto_recovered` /
`auto_fired` across builds with n = 1, 2, 3, 5. A working action turns
failures into single slow grabs. With the wedge this frequent, a few hundred
grabs per build is enough.

**C. Does the action itself break a healthy stream?** On `9818951` the
full-procedure resync made grabs fail quickly, but ffmpeg worked again
afterwards, i.e. per stream, not a wedge. With a healthy continuous stream
running, fire 1 or 2 by hand. If frames stop until the stream is restarted,
raising /CS is what breaks it (e.g. VSYNC stops, or the sensor needs a CCI
power-on afterwards); watch `vsync_irqs`, `lab_walk_found`,
`lab_walk_giveups`.
