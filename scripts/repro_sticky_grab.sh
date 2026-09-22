#!/usr/bin/env bash
# repro_sticky_grab.sh v3 - A/B test for the PureThermal "sticky frame grab" bug.
#
#   Mode A  N independent single-frame grabs. Each does
#           open -> SET_INTERFACE(alt=N) -> DQBUF -> SET_INTERFACE(alt=0) -> close.
#   Mode B  N frames in ONE continuous stream. One setup, one teardown.
#
# v3 changes - the escape hatch now RECOVERS wedges, so "grab failed" stopped
# being the signal and the script went blind:
#   - Slow grabs are classified by duration instead of just counted. Note that
#     duration ALONE cannot identify a wedge: an FFC closes the shutter and
#     costs about the same ~2 s as an escape-hatch hardware reset, and FFCs
#     happen routinely. The 2s band is therefore reported as ambiguous, and
#     only hard_recoveries moving (-C) distinguishes the two.
#   - Optional SWD counter reads (-C) using STM32CubeProgrammer's CLI in
#     hotplug mode, which does NOT halt the core. Halting breaks USB and makes
#     every grab take 5.1 s (the uvcvideo control timeout), so gdb/st-util is
#     deliberately not used here. Counters are read before and after the run,
#     and optionally after every grab, so a slow grab can be attributed to a
#     specific counter moving rather than guessed at from its duration.
#   - Per-grab CSV (-o) with duration and counter deltas, so the distribution
#     can be looked at afterwards instead of only the worst case.
#
# Usage: ./repro_sticky_grab.sh [-d /dev/video0] [-n 200] [-t 10] [-m ab]
#                               [-r 1] [-k 5] [-s 0] [-A] [-C] [-P] [-o FILE]

set -uo pipefail

trap 'echo; echo "aborted."; exit 130' INT TERM

DEV=/dev/video0; N=200; TIMEOUT=10; MODES=ab; REPEATS=1; ABORT_AFTER=5
DELAY=0        # -s: idle seconds between grabs (accepts fractions)
SOFT_RESET=0   # -A: sysfs re-enumerate instead of asking for a replug
COUNTERS=0     # -C: read g_dbg over SWD before/after each run
POLL=0         # -P: also read g_dbg after every grab (implies -C)
CSV=""         # -o: per-grab log

# Duration bands, ms. TWOSEC_LO/HI bracket BOTH of the things that cost about
# two seconds:
#   - an FFC: the shutter closes, video pauses, and the grab waits it out.
#     Routine, benign, and periodic.
#   - the escape hatch: two LEPTON_HW_RESET_STEP_MS steps (190 each) +
#     LEPTON_HW_BOOT_MS (1500) + LEPTON_VOSPI_RESYNC_MS (250) + CCI
#     reconfiguration.
# They are indistinguishable by duration. Run with -C and read hard_recoveries:
# it moves for a wedge and not for an FFC.
SLOW_LO=1000
TWOSEC_LO=1700
TWOSEC_HI=3500

while getopts "d:n:t:m:r:k:s:o:ACPh" o; do case $o in
  d) DEV=$OPTARG ;; n) N=$OPTARG ;; t) TIMEOUT=$OPTARG ;; m) MODES=$OPTARG ;;
  r) REPEATS=$OPTARG ;; k) ABORT_AFTER=$OPTARG ;; A) SOFT_RESET=1 ;;
  s) DELAY=$OPTARG ;; o) CSV=$OPTARG ;;
  C) COUNTERS=1 ;; P) POLL=1; COUNTERS=1 ;;
  h) sed -n '2,30p' "$0"; exit 0 ;; *) exit 2 ;;
esac; done

command -v v4l2-ctl >/dev/null || { echo "need v4l-utils: sudo apt install v4l-utils"; exit 1; }
[ -e "$DEV" ] || { echo "no such device: $DEV"; exit 1; }

# ------------------------------------------------------------------- counters
# g_dbg layout, from Inc/dbg_counters.h. Offsets are stable: the struct is
# append-only by design. The base address is found by scanning SRAM for the
# magic rather than parsing main.map, so this works against a board whose
# build tree you do not have.
DBG_MAGIC=0xDBC0FFEE
DBG_BASE=${DBG_BASE:-}
CLI=${STM32_PROGRAMMER_CLI:-}

# name:offset - only the ones worth watching during a soak
DBG_FIELDS="vsync_irqs:0x04 transfer_fails:0x0C desync_events:0x10 \
resync_entries:0x14 frames_completed:0x1C first_line_bad:0xBC \
resync_giveups:0xC0 vsync_cfg_fails:0xC4 hard_recoveries:0xC8 \
worst_desync_run:0xCC wedge_recoveries:0xD4 last_recovery_firings:0xD8 \
cs_stuck_low:0xE0 hw_resets:0xE8 giveup_recoveries:0xEC"

find_cli() {
  [ -n "$CLI" ] && { command -v "$CLI" >/dev/null && return 0 || return 1; }
  local c
  for c in STM32_Programmer_CLI STM32_Programmer_CLI.exe; do
    command -v "$c" >/dev/null && { CLI=$c; return 0; }
  done
  for c in /opt/st/stm32cubeprog*/bin/STM32_Programmer_CLI \
           "$HOME"/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI \
           "/mnt/c/Program Files/STMicroelectronics/STM32Cube/STM32CubeProgrammer/bin/STM32_Programmer_CLI.exe"; do
    [ -x "$c" ] && { CLI=$c; return 0; }
  done
  return 1
}

# read_words <addr> <count> -> one hex word per line
# HOTPLUG leaves the core running. Never use "mode=UR" or a gdb attach here:
# a halted core stops servicing USB and every grab then takes 5.1 s.
read_words() {
  "$CLI" -c port=SWD mode=HOTPLUG -q -r32 "$1" "$(( $2 * 4 ))" 2>/dev/null |
    sed -n 's/^0x[0-9A-Fa-f]*[[:space:]]*:\?[[:space:]]*//p' |
    tr ' \t' '\n\n' | grep -oE '^[0-9A-Fa-f]{8}$' | head -n "$2"
}

locate_dbg() {
  [ -n "$DBG_BASE" ] && return 0
  local a w
  for a in 0x20000014 0x20000024 0x20000000; do
    w=$(read_words "$a" 1 | head -1)
    [ "0x${w^^}" = "${DBG_MAGIC^^}" ] && { DBG_BASE=$a; return 0; }
  done
  # brute scan of the first 2 KiB of SRAM, 4-byte aligned
  local i
  mapfile -t w < <(read_words 0x20000000 512)
  for i in "${!w[@]}"; do
    if [ "0x${w[$i]^^}" = "${DBG_MAGIC^^}" ]; then
      DBG_BASE=$(printf "0x%08X" $(( 0x20000000 + i * 4 )))
      return 0
    fi
  done
  return 1
}

# snapshot -> "name=value name=value ..." on stdout
snapshot() {
  local words f name off idx
  mapfile -t words < <(read_words "$DBG_BASE" 60)
  [ "${#words[@]}" -lt 60 ] && return 1
  local out=""
  for f in $DBG_FIELDS; do
    name=${f%%:*}; off=${f##*:}
    idx=$(( off / 4 ))
    out+="$name=$(( 16#${words[$idx]} )) "
  done
  echo "$out"
}

val() { # val "<snapshot>" <name>
  local v; v=$(echo " $1" | grep -oE " $2=[0-9]+" | head -1); echo "${v##*=}"
}

print_delta() { # print_delta <before> <after> <indent>
  local f name b a moved=0
  for f in $DBG_FIELDS; do
    name=${f%%:*}
    b=$(val "$1" "$name"); a=$(val "$2" "$name")
    [ -z "$b" ] && continue
    if [ "$b" != "$a" ]; then
      printf "%s%-24s %8s -> %-8s (%+d)\n" "$3" "$name" "$b" "$a" "$((a-b))"
      moved=1
    fi
  done
  [ $moved -eq 0 ] && printf "%sno counters moved\n" "$3"
}

if [ "$COUNTERS" = "1" ]; then
  if ! find_cli; then
    echo "!! STM32_Programmer_CLI not found - counter reads disabled."
    echo "   Set STM32_PROGRAMMER_CLI=/path/to/STM32_Programmer_CLI to enable."
    COUNTERS=0; POLL=0
  elif ! locate_dbg; then
    echo "!! g_dbg magic not found in SRAM - counter reads disabled."
    echo "   Is the board attached and running a build with dbg_counters?"
    COUNTERS=0; POLL=0
  else
    echo "counters: g_dbg at $DBG_BASE via $(basename "$CLI") (hotplug, core keeps running)"
  fi
fi

# ---------------------------------------------------------------- device reset
usb_dev_path() {
  local link="/sys/class/video4linux/$(basename "$DEV")/device"
  [ -e "$link" ] || return 1
  local p; p=$(dirname "$(readlink -f "$link")")
  while [ "$p" != "/" ]; do
    [ -f "$p/authorized" ] && { echo "$p"; return 0; }
    p=$(dirname "$p")
  done
  return 1
}

wait_for_dev() {
  local i
  for ((i=0;i<30;i++)); do [ -e "$DEV" ] && { sleep 1; return 0; }; sleep 1; done
  echo "  !! $DEV never came back"; return 1
}

reset_device() {
  local p
  # A sysfs authorized 0/1 makes the HOST re-enumerate. The STM32 never
  # reboots, so every firmware-side static - protothread state, the lepton
  # buffer ring, VoSPI sync - survives it untouched. Measured: a wedged unit
  # stays wedged across re-enumeration and fails on the very next grab. Only
  # removing power actually resets the device, so that is the default here.
  if [ "$SOFT_RESET" = "1" ]; then
    if p=$(usb_dev_path 2>/dev/null); then
      if [ -w "$p/authorized" ]; then
        echo 0 > "$p/authorized"; sleep 1; echo 1 > "$p/authorized"
      elif sudo -n true 2>/dev/null; then
        sudo sh -c "echo 0 > $p/authorized"; sleep 1; sudo sh -c "echo 1 > $p/authorized"
      fi
      echo "  [re-enumerated $(basename "$p") - HOST ONLY, firmware state intact]"
      wait_for_dev; return
    fi
  fi
  echo "  >> Unplug the camera, wait 2s, plug it back in."
  read -rp "  >> Press Enter once it is back: " _
  wait_for_dev
}

# ---------------------------------------------------------------- mode A
mode_a() {
  local tag="$1" fails=0 first=0 slow=0 resets=0 worst=0 consec=0 done_n=0 i ms rc g0
  local snap_before="" snap_after="" prev="" now="" hr_b hr_a wr_b wr_a band
  local t0; t0=$(date +%s%N)

  if [ "$COUNTERS" = "1" ]; then
    snap_before=$(snapshot); prev=$snap_before
    [ -n "$CSV" ] && echo "grab,ms,rc,band,moved" > "$CSV"
  elif [ -n "$CSV" ]; then
    echo "grab,ms,rc,band" > "$CSV"
  fi

  for ((i=1;i<=N;i++)); do
    g0=$(date +%s%N)
    timeout "$TIMEOUT" v4l2-ctl -d "$DEV" --stream-mmap --stream-count=1 \
        --stream-to=/dev/null >/dev/null 2>&1
    rc=$?
    ms=$(( ($(date +%s%N) - g0) / 1000000 ))
    (( ms > worst )) && worst=$ms

    # classify before reporting, so the label is the same everywhere
    if   [ $rc -ne 0 ];            then band=FAIL
    elif [ $ms -ge $TWOSEC_LO ] && [ $ms -le $TWOSEC_HI ]; then band=TWOSEC
    elif [ $ms -gt $TWOSEC_HI ];   then band=STALL
    elif [ $ms -ge $SLOW_LO ];     then band=SLOW
    else                                band=ok
    fi

    local moved=""
    if [ "$POLL" = "1" ]; then
      now=$(snapshot)
      hr_b=$(val "$prev" hard_recoveries); hr_a=$(val "$now" hard_recoveries)
      wr_b=$(val "$prev" wedge_recoveries); wr_a=$(val "$now" wedge_recoveries)
      [ "$hr_b" != "$hr_a" ] && moved+="hard_recoveries+$((hr_a-hr_b)) "
      [ "$wr_b" != "$wr_a" ] && moved+="wedge_recoveries+$((wr_a-wr_b)) "
      prev=$now
    fi

    [ -n "$CSV" ] && echo "$i,$ms,$rc,$band,${moved% }" >> "$CSV"

    case $band in
      FAIL)
        fails=$((fails+1)); consec=$((consec+1))
        if [ $first -eq 0 ]; then
          first=$i
          printf "  >>> FIRST FAILURE at grab %d  (rc=%d, %dms) %s\n" "$i" "$rc" "$ms" "$moved"
        fi
        if [ $consec -ge $ABORT_AFTER ]; then
          printf "  >>> %d consecutive failures - wedged and NOT recovering, stopping early\n" "$consec"
          done_n=$i; break
        fi ;;
      TWOSEC)
        consec=0; done_n=$i; resets=$((resets+1)); slow=$((slow+1))
        printf "  grab %-5d %dms  [~2s: FFC or escape hatch - see hard_recoveries] %s\n" "$i" "$ms" "$moved" ;;
      STALL)
        consec=0; done_n=$i; slow=$((slow+1))
        printf "  grab %-5d %dms  [longer than a hardware reset - investigate] %s\n" "$i" "$ms" "$moved" ;;
      SLOW)
        consec=0; done_n=$i; slow=$((slow+1))
        printf "  grab %-5d %dms  [resync churn] %s\n" "$i" "$ms" "$moved" ;;
      *) consec=0; done_n=$i ;;
    esac

    # Idle gap between grabs. Each teardown calls lepton_low_power() and each
    # restart calls lepton_power_on(), which issues LEP_RunOemPowerOn and
    # returns immediately - no settle wait before VoSPI is clocked again. If
    # the wedge is really about sensor settle time, it should move with this.
    [ "$DELAY" != "0" ] && sleep "$DELAY"
  done

  printf "  %s: first_failure=%s  failures=%d  slow=%d  two_sec=%d  worst=%dms  elapsed=%ds\n" \
    "$tag" "$( [ $first -eq 0 ] && echo none || echo "$first" )" \
    "$fails" "$slow" "$resets" "$worst" "$(( ($(date +%s%N) - t0)/1000000000 ))"

  if [ "$COUNTERS" = "1" ]; then
    snap_after=$(snapshot)
    echo "  --- g_dbg over this run ---"
    print_delta "$snap_before" "$snap_after" "    "
    local hr wr lf gu
    hr=$(val "$snap_after" hard_recoveries); wr=$(val "$snap_after" wedge_recoveries)
    lf=$(val "$snap_after" last_recovery_firings); gu=$(val "$snap_after" resync_giveups)
    echo "  --- verdict ---"
    local hb; hb=$(val "$snap_before" hard_recoveries)
    if [ "${hr:-0}" != "${hb:-0}" ]; then
      echo "    WEDGED $(( hr - hb ))x. Recovered by the escape hatch: wedge_recoveries=$wr," \
           "last needed $lf firing(s)."
      echo "    These are real wedges. Without the hatch each one would have needed a replug."
    elif [ "$resets" -gt 0 ]; then
      echo "    No wedges. The $resets two-second grab(s) are FFCs - hard_recoveries"
      echo "    did not move, so the escape hatch never fired."
    else
      echo "    No wedges this run."
    fi
    [ "${gu:-0}" != "0" ] && echo "    resync_giveups=$gu - the walk hit RESYNC_MAX_PACKETS; deep desync short of a wedge."
  fi

  FIRST_FAILS+=("$( [ $first -eq 0 ] && echo none || echo "$first" )")
}

# ---------------------------------------------------------------- report
echo "device : $DEV"
v4l2-ctl -d "$DEV" --get-fmt-video 2>/dev/null | sed -n 's/^\s*/  /p' | head -3
echo "config : n=$N timeout=${TIMEOUT}s abort_after=$ABORT_AFTER repeats=$REPEATS delay=${DELAY}s"
echo "bands  : ok <${SLOW_LO}ms | SLOW | TWOSEC ${TWOSEC_LO}-${TWOSEC_HI}ms (FFC or hatch) | STALL >${TWOSEC_HI}ms | FAIL"
[ -n "$CSV" ] && echo "csv    : $CSV"
echo

DMESG_MARK=$(dmesg 2>/dev/null | wc -l); DMESG_MARK=${DMESG_MARK:-0}
[ "$DMESG_MARK" -eq 0 ] && echo "(dmesg unreadable - run with sudo for kernel messages)" && echo

FIRST_FAILS=()

if [[ $MODES == *a* ]]; then
  for ((r=1;r<=REPEATS;r++)); do
    echo "=== Mode A run $r/$REPEATS: up to $N single-frame grabs, teardown between each ==="
    reset_device
    mode_a "run$r"
    echo
  done
fi

if [[ $MODES == *b* ]]; then
  echo "=== Mode B: $N frames in one continuous stream (FRESH device) ==="
  reset_device
  [ "$COUNTERS" = "1" ] && B_BEFORE=$(snapshot)
  b0=$(date +%s%N)
  timeout $((TIMEOUT * 10)) v4l2-ctl -d "$DEV" --stream-mmap \
      --stream-count="$N" --stream-to=/dev/null 2>&1 | tail -2
  rc=${PIPESTATUS[0]}
  ms=$(( ($(date +%s%N) - b0) / 1000000 )); [ "$ms" -lt 1 ] && ms=1
  case $rc in
    124) echo "  HUNG (killed after $((TIMEOUT*10))s)" ;;
      0) echo "  OK - $N frames in $((ms/1000))s (~$(( N * 1000 / ms )) fps)" ;;
      *) echo "  FAILED rc=$rc after ${ms}ms" ;;
  esac
  if [ "$COUNTERS" = "1" ]; then
    echo "  --- g_dbg over mode B ---"
    print_delta "$B_BEFORE" "$(snapshot)" "    "
  fi
  echo
fi

NEW=$(dmesg 2>/dev/null | tail -n +$((DMESG_MARK+1)) | grep -iE "uvc|usb" | tail -20)
[ -n "$NEW" ] && { echo "=== new kernel messages ==="; echo "$NEW"; echo; }

if [ ${#FIRST_FAILS[@]} -gt 1 ]; then
  echo "=== first-failure iteration across runs: ${FIRST_FAILS[*]} ==="
  cat <<'VAR'
  Clustered (all within ~20% of each other) -> something COUNTS UP and runs out:
    a leak, a FIFO filling, a buffer index. Deterministic, so bisectable by
    instrumenting the counter.
  Scattered (e.g. 40, 190, 95)              -> a RACE that latches once hit.
    Now that the escape hatch recovers, read the shape off hard_recoveries
    rather than off first_failure: with -C the run prints how many wedges
    happened even when every grab succeeded.
VAR
fi
