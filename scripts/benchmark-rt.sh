#!/usr/bin/env bash
#
# benchmark-rt.sh -- run the controller N times under a named configuration and
# aggregate cycle_jitter_us / pdo_exchange_us across runs.
#
# Single 10 s runs are not enough to judge an RT tuning change. The body of the
# latency distribution (p50..p99) settles quickly, but the tail is dominated by a
# handful of rare events -- four outliers in one run is well inside noise. This
# script collects several runs under a label, pools them for the percentile
# table, and reports the per-run maxima separately so tail variance stays visible
# instead of being averaged away.
#
# It does NOT change any tuning itself. Apply a configuration by hand (chrt, IRQ
# affinity, isolcpus, ...), then label the runs you collect under it. The config
# state at collect time is snapshotted to <label>.meta so a comparison can be
# traced back to what was actually set.
#
# Usage:
#   sudo scripts/benchmark-rt.sh --label NAME [--runs N] [--iface IFACE]
#   sudo scripts/benchmark-rt.sh --compare LABEL [LABEL ...]
#   sudo scripts/benchmark-rt.sh --list
#
#   --label NAME     collect runs and file them under NAME (e.g. baseline, chrt85)
#   --runs N         number of runs to collect (default: 5)
#   --iface IFACE    EtherCAT interface (default: enp2s0)
#   --settle S       seconds to wait between runs (default: 3)
#   --skip-s S       seconds of each run to discard as startup transient
#                    (default: 1.0 -- matches the DC sync settling time)
#   --compare ...    print the aggregate tables for one or more existing labels
#   --list           list labels collected so far
#   --meta LABEL     show the config snapshot recorded for a label
#
# Typical use:
#   sudo scripts/benchmark-rt.sh --label baseline --runs 5
#   sudo chrt -f -p 85 $(ps -eLo tid,comm | awk '/irq\/.*-enp2s0/{print $1}')
#   sudo scripts/benchmark-rt.sh --label chrt85 --runs 5
#   sudo scripts/benchmark-rt.sh --compare baseline chrt85
#
set -euo pipefail

IFACE="enp2s0"
RUNS=5
LABEL=""
SETTLE=3
SKIP_S="1.0"
MODE=""
COMPARE_LABELS=()

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"
DATA_DIR="$REPO_DIR/data"
BENCH_DIR="$DATA_DIR/benchmark"
BINARY="$REPO_DIR/ethercat-voice-coil-controller"

while [[ $# -gt 0 ]]; do
	case "$1" in
		--label)   LABEL="$2"; MODE="collect"; shift 2 ;;
		--runs)    RUNS="$2"; shift 2 ;;
		--iface)   IFACE="$2"; shift 2 ;;
		--settle)  SETTLE="$2"; shift 2 ;;
		--skip-s)  SKIP_S="$2"; shift 2 ;;
		--compare) MODE="compare"; shift
		           while [[ $# -gt 0 && "$1" != --* ]]; do COMPARE_LABELS+=("$1"); shift; done ;;
		--list)    MODE="list"; shift ;;
		--meta)    MODE="meta"; LABEL="$2"; shift 2 ;;
		-h|--help) grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
		*) echo "unknown argument: $1" >&2; exit 2 ;;
	esac
done

if [[ -z "$MODE" ]]; then
	echo "error: give --label NAME, --compare LABEL..., --list or --meta LABEL" >&2
	echo "       (--help for usage)" >&2
	exit 2
fi

mkdir -p "$BENCH_DIR"

# Cycle period drives the budget percentages. Read it from the source of truth so
# the table can never disagree with what was actually compiled.
CYCLE_MS="$(grep -oP '#define\s+CYCLE_TIME_MS\s+\K[0-9.]+' "$REPO_DIR/main.h" 2>/dev/null || echo 0.5)"
CYCLE_US="$(awk -v c="$CYCLE_MS" 'BEGIN{printf "%.1f", c*1000}')"

# ---------------------------------------------------------------- collect ----

snapshot_config() {  # snapshot_config <outfile>
	local out="$1"
	{
		echo "# config snapshot -- $(date -Is)"
		echo "label:        $LABEL"
		echo "iface:        $IFACE"
		echo "kernel:       $(uname -r)"
		echo "cycle_time:   ${CYCLE_US} us (CYCLE_TIME_MS=$CYCLE_MS)"
		echo "cmdline:      $(cat /proc/cmdline)"
		echo "isolated:     $(cat /sys/devices/system/cpu/isolated 2>/dev/null)"
		echo "nohz_full:    $(cat /sys/devices/system/cpu/nohz_full 2>/dev/null)"
		echo "governor:     $(cat /sys/devices/system/cpu/cpu1/cpufreq/scaling_governor 2>/dev/null)"
		echo "no_turbo:     $(cat /sys/devices/system/cpu/intel_pstate/no_turbo 2>/dev/null)"
		echo "link:         $(cat /sys/class/net/$IFACE/speed 2>/dev/null) Mb/s"
		local irq
		irq="$(awk -F: -v i="$IFACE" '$0 ~ i {gsub(/ /,"",$1); print $1; exit}' /proc/interrupts)"
		if [[ -n "$irq" ]]; then
			echo "nic_irq:      $irq"
			echo "irq_affinity: $(cat /proc/irq/$irq/smp_affinity_list 2>/dev/null)"
		fi
		echo "irq_threads:"
		ps -eLo tid,cls,rtprio,psr,comm | awk '/irq\// {printf "  tid=%-6s %s prio=%-3s cpu=%s %s\n", $1,$2,$3,$4,$5}'
	} > "$out"
}

collect() {
	[[ -x "$BINARY" ]] || { echo "error: $BINARY not found or not executable" >&2; exit 1; }
	[[ $EUID -eq 0 ]] || { echo "error: must run as root (sudo) -- the controller needs raw socket + RT priority" >&2; exit 1; }

	local runlist="$BENCH_DIR/$LABEL.runs"
	: > "$runlist"
	snapshot_config "$BENCH_DIR/$LABEL.meta"

	echo "-- collecting $RUNS run(s) under label '$LABEL' on $IFACE"
	echo "   cycle ${CYCLE_US} us, discarding first ${SKIP_S} s of each run"
	echo

	local i before after log
	for (( i=1; i<=RUNS; i++ )); do
		before="$(ls "$DATA_DIR"/voice_coil_log_*.csv 2>/dev/null | sort)"
		printf "   run %d/%d ... " "$i" "$RUNS"

		if ! "$BINARY" "$IFACE" > "$BENCH_DIR/$LABEL.run$i.stdout" 2>&1; then
			echo "FAILED (see $BENCH_DIR/$LABEL.run$i.stdout)"
			continue
		fi

		after="$(ls "$DATA_DIR"/voice_coil_log_*.csv 2>/dev/null | sort)"
		log="$(comm -13 <(echo "$before") <(echo "$after") | tail -1)"

		if [[ -z "$log" ]]; then
			echo "no new log produced -- skipping"
			continue
		fi

		echo "$(basename "$log")" >> "$runlist"
		# Per-run headline so a bad run is obvious immediately, not at the end.
		awk -F, -v skip="$SKIP_S" 'NR>1 && NF>10 && $1+0>skip {p=$11+0; s+=p; n++; if(p>mx)mx=p}
			END{ if(n) printf "ok  mean pdo=%.1f us  max=%.1f us\n", s/n, mx; else print "ok  (no samples past skip window)" }' "$log"

		(( i < RUNS )) && sleep "$SETTLE"
	done

	echo
	echo "-- filed $(wc -l < "$runlist") run(s) under '$LABEL'"
	echo "   config snapshot: $BENCH_DIR/$LABEL.meta"
	echo
	compare "$LABEL"
}

# ---------------------------------------------------------------- compare ----

# Pool one label's samples into a single column file. Pooling is right for the
# distribution shape; per-run maxima are reported separately because pooling
# would hide exactly the tail variance we are trying to measure.
pool() {  # pool <label> <column: 10=jitter 11=pdo> <outfile>
	local label="$1" col="$2" out="$3"
	: > "$out"
	local f
	while read -r f; do
		[[ -f "$DATA_DIR/$f" ]] || continue
		awk -F, -v c="$col" -v skip="$SKIP_S" 'NR>1 && NF>10 && $1+0>skip {print $c+0}' "$DATA_DIR/$f" >> "$out"
	done < "$BENCH_DIR/$label.runs"
	sort -n "$out" -o "$out"
}

pctl() {  # pctl <sorted file> -- emits: n p50 p90 p99 p999 max mean sd p01
	awk '{v[n++]=$1; s+=$1}
		END{
			if(n==0){print "0 0 0 0 0 0 0 0 0"; exit}
			m=s/n; for(i=0;i<n;i++){d=v[i]-m; ss+=d*d}
			printf "%d %.1f %.1f %.1f %.1f %.1f %.1f %.2f %.1f\n",
				n, v[int(.5*n)], v[int(.9*n)], v[int(.99*n)], v[int(.999*n)], v[n-1], m, sqrt(ss/n), v[int(.001*n)]
		}' "$1"
}

compare() {
	local labels=("$@") label
	local tmp; tmp="$(mktemp -d)"
	trap 'rm -rf "$tmp"' RETURN

	for label in "${labels[@]}"; do
		[[ -f "$BENCH_DIR/$label.runs" ]] || { echo "error: no runs collected under label '$label'" >&2; exit 1; }
	done

	echo "=== PDO exchange time (us), pooled across runs -- cycle budget ${CYCLE_US} us ==="
	printf "%-14s %5s %8s %8s %8s %8s %8s %9s %9s\n" label runs n p50 p90 p99 p99.9 max "%budget"
	for label in "${labels[@]}"; do
		pool "$label" 11 "$tmp/$label.pdo"
		read -r n p50 p90 p99 p999 max mean sd p01 <<< "$(pctl "$tmp/$label.pdo")"
		printf "%-14s %5d %8d %8.1f %8.1f %8.1f %8.1f %8.1f %8.1f%%\n" \
			"$label" "$(wc -l < "$BENCH_DIR/$label.runs")" "$n" "$p50" "$p90" "$p99" "$p999" "$max" \
			"$(awk -v a="$max" -v b="$CYCLE_US" 'BEGIN{printf "%.1f", 100*a/b}')"
	done

	echo
	echo "=== per-run max PDO (us) -- tail variance across runs ==="
	for label in "${labels[@]}"; do
		printf "%-14s" "$label"
		local f
		while read -r f; do
			[[ -f "$DATA_DIR/$f" ]] || continue
			awk -F, -v skip="$SKIP_S" 'NR>1 && NF>10 && $1+0>skip {p=$11+0; if(p>mx)mx=p} END{printf " %7.1f", mx}' "$DATA_DIR/$f"
		done < "$BENCH_DIR/$label.runs"
		echo
	done

	echo
	echo "=== cycle jitter (us), pooled ==="
	printf "%-14s %8s %8s %8s %8s %10s\n" label mean sd p0.1 p99.9 "spread"
	for label in "${labels[@]}"; do
		pool "$label" 10 "$tmp/$label.jit"
		read -r n p50 p90 p99 p999 max mean sd p01 <<< "$(pctl "$tmp/$label.jit")"
		printf "%-14s %8.1f %8.2f %8.1f %8.1f %10.1f\n" "$label" "$mean" "$sd" "$p01" "$p999" \
			"$(awk -v a="$p999" -v b="$p01" 'BEGIN{printf "%.1f", a-b}')"
	done

	echo
	echo "=== outlier counts (pooled) and faults ==="
	printf "%-14s %10s %10s %10s %12s %10s\n" label ">100us" ">150us" ">250us" ">50%budget" faults
	for label in "${labels[@]}"; do
		local half; half="$(awk -v b="$CYCLE_US" 'BEGIN{printf "%.1f", b/2}')"
		printf "%-14s" "$label"
		awk -v h="$half" '{ if($1>100)a++; if($1>150)b++; if($1>250)c++; if($1>h)d++ }
			END{printf " %10d %10d %10d %12d", a+0, b+0, c+0, d+0}' "$tmp/$label.pdo"
		# Fault rows are the ground truth for missed deadlines / WKC mismatches;
		# a clean latency table with a non-zero count here is not a clean run.
		local faults=0 f fault_f
		while read -r f; do
			fault_f="$DATA_DIR/${f/voice_coil_log_/voice_coil_faults_}"
			[[ -f "$fault_f" ]] && faults=$(( faults + $(( $(wc -l < "$fault_f") - 1 )) ))
		done < "$BENCH_DIR/$label.runs"
		printf " %10d\n" "$faults"
	done
	echo
}

# ------------------------------------------------------------------- main ----

case "$MODE" in
	collect) collect ;;
	compare)
		[[ ${#COMPARE_LABELS[@]} -gt 0 ]] || { echo "error: --compare needs at least one label" >&2; exit 2; }
		compare "${COMPARE_LABELS[@]}" ;;
	list)
		if compgen -G "$BENCH_DIR/*.runs" > /dev/null; then
			printf "%-16s %5s  %s\n" label runs "collected"
			for f in "$BENCH_DIR"/*.runs; do
				label="$(basename "$f" .runs)"
				printf "%-16s %5d  %s\n" "$label" "$(wc -l < "$f")" \
					"$(awk -F': *' '/^# config snapshot/{print $0}' "$BENCH_DIR/$label.meta" 2>/dev/null | sed 's/# config snapshot -- //')"
			done
		else
			echo "no labels collected yet under $BENCH_DIR"
		fi ;;
	meta)
		[[ -f "$BENCH_DIR/$LABEL.meta" ]] || { echo "error: no snapshot for label '$LABEL'" >&2; exit 1; }
		cat "$BENCH_DIR/$LABEL.meta" ;;
esac
