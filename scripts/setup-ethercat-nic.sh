#!/usr/bin/env bash
#
# setup-ethercat-nic.sh -- dedicate and RT-tune a NIC for the EtherCAT fieldbus.
#
# SOEM talks to the slave over a raw AF_PACKET socket (EtherType 0x88A4); it uses
# no IP. Anything the OS does with IP on that interface -- DHCP retries, IPv6
# router solicitation, Energy Efficient Ethernet renegotiation, autoneg flaps --
# only injects stray frames and, worst case, bounces the link mid-run and drops
# the slave to SAFE-OP. This script removes all of that and pins the NIC's
# link/offload settings for predictable sub-millisecond PDO exchange.
#
# It configures ONLY the EtherCAT interface. It never touches the interface that
# carries your internet / SSH session. Moving internet to the other NIC is a
# manual step (see the notes printed at the end).
#
# Recommended NIC on this machine: enp2s0 (Intel I210-T1, igb, PCIe, MSI-X with
# per-queue vectors) in preference to eno1 (82579LM, e1000e, single MSI vector,
# shares DMI with the rest of the PCH) or enp3s2 (82544GC, e1000, shared legacy
# IRQ 18 on 33/66 MHz PCI). See docs/realtime-tuning.md.
#
# --undo reverses a previous run: it hands the interface back to NetworkManager
# with offloads and autonegotiation restored, so a former fieldbus NIC can be
# used as an ordinary network interface again.
#
# Usage:
#   sudo scripts/setup-ethercat-nic.sh [--iface IFACE] [--speed 100|1000|auto]
#                                      [--rt-core N] [--install-service] [--yes]
#   sudo scripts/setup-ethercat-nic.sh --undo --iface IFACE [--yes]
#
#   --iface IFACE        EtherCAT interface (default: enp2s0)
#   --speed 100|1000|auto Fix link speed to avoid autoneg flaps (default: 100,
#                        the normal EtherCAT 100BASE-TX rate; use auto to keep
#                        autonegotiation, e.g. through a switch that needs it)
#   --rt-core N          RT loop core to check IRQ affinity against
#                        (default: read RT_CPU_CORE from main.h, else 1)
#   --install-service    Also install a systemd oneshot so the tuning is
#                        re-applied automatically on every boot
#   --undo               Undo a previous run on IFACE: remove the drop-ins,
#                        disable the boot service, restore offloads/EEE/autoneg,
#                        and return the interface to NetworkManager
#   --yes                Don't prompt for confirmation
#
set -euo pipefail

IFACE="enp2s0"
SPEED="100"
RT_CORE=""
INSTALL_SERVICE=0
ASSUME_YES=0
UNDO=0

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(dirname "$SCRIPT_DIR")"

while [[ $# -gt 0 ]]; do
	case "$1" in
		--iface)           IFACE="$2"; shift 2 ;;
		--speed)           SPEED="$2"; shift 2 ;;
		--rt-core)         RT_CORE="$2"; shift 2 ;;
		--install-service) INSTALL_SERVICE=1; shift ;;
		--undo)            UNDO=1; shift ;;
		--yes|-y)          ASSUME_YES=1; shift ;;
		-h|--help)         grep '^#' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
		*) echo "unknown argument: $1" >&2; exit 2 ;;
	esac
done

if [[ $EUID -ne 0 ]]; then
	echo "error: must run as root (sudo)" >&2
	exit 1
fi

for tool in ethtool ip; do
	command -v "$tool" >/dev/null || { echo "error: '$tool' not found" >&2; exit 1; }
done

if [[ ! -e "/sys/class/net/$IFACE" ]]; then
	echo "error: interface '$IFACE' does not exist" >&2
	echo "available: $(ls /sys/class/net | grep -v '^lo$' | tr '\n' ' ')" >&2
	exit 1
fi

try() { echo "   + $*"; "$@" 2>/dev/null || echo "     (not supported -- ignored)"; }

# Apply ethtool sub-options ONE AT A TIME. ethtool fails the whole request if any
# single key/value pair is unsupported by the driver, so bundling (e.g. passing
# rx-frames to e1000e, which only takes rx-usecs) would silently drop every other
# setting in the call -- including the important one.
try_each() {  # try_each <ethtool-flag> <key1> <val1> [<key2> <val2> ...]
	local flag="$1"; shift
	while [[ $# -ge 2 ]]; do
		try ethtool "$flag" "$IFACE" "$1" "$2"
		shift 2
	done
}

# ---------------------------------------------------------------------------
# --undo: hand the interface back to the OS as an ordinary network device.
# Reverses every persistent change a previous run made. Used when the fieldbus
# moves to a different NIC and the old one becomes a normal network interface.
# ---------------------------------------------------------------------------
undo_iface() {
	echo "=============================================================="
	echo " Undo EtherCAT dedication of $IFACE  (driver: ${DRIVER:-?})"
	echo "=============================================================="

	if [[ $ASSUME_YES -ne 1 ]]; then
		echo
		read -r -p "Return '$IFACE' to normal network use? [y/N] " ans
		[[ "$ans" == "y" || "$ans" == "Y" ]] || { echo "aborted."; exit 1; }
	fi

	echo
	echo "-- [1/4] disabling boot-time tuning service"
	if systemctl is-enabled "ethercat-nic@${IFACE}.service" >/dev/null 2>&1; then
		systemctl disable --now "ethercat-nic@${IFACE}.service" >/dev/null 2>&1 || true
		echo "   disabled ethercat-nic@${IFACE}.service"
	else
		echo "   not enabled -- nothing to do"
	fi

	echo "-- [2/4] removing drop-ins"
	local f found=0
	for f in "/etc/NetworkManager/conf.d/99-ethercat-${IFACE}.conf" \
	         "/etc/sysctl.d/99-ethercat-${IFACE}.conf" \
	         "/etc/systemd/network/99-ethercat-${IFACE}.network"; do
		[[ -e "$f" ]] || continue
		rm -f "$f"; echo "   removed $f"; found=1
	done
	[[ $found -eq 1 ]] || echo "   none present"
	sysctl -qw "net.ipv6.conf.${IFACE//./\/}.disable_ipv6=0" 2>/dev/null || true
	echo "   re-enabled IPv6 on $IFACE"

	# Restore driver defaults. An internet NIC wants the offloads and adaptive
	# coalescing that the fieldbus path deliberately turns off.
	echo "-- [3/4] restoring link settings"
	try_each -K tso on gso on gro on sg on rxvlan on txvlan on
	try ethtool --set-eee "$IFACE" eee on
	try_each -A autoneg on rx on tx on
	try ethtool -C "$IFACE" adaptive-rx on
	try ethtool -s "$IFACE" autoneg on

	echo "-- [4/4] returning $IFACE to NetworkManager"
	if command -v nmcli >/dev/null && systemctl is-active --quiet NetworkManager; then
		nmcli general reload 2>/dev/null || systemctl reload NetworkManager || true
		nmcli device set "$IFACE" managed yes 2>/dev/null || true
		echo "   $IFACE is managed again"
	else
		echo "   NetworkManager not active -- bring the interface up yourself"
	fi
	systemctl is-active --quiet systemd-networkd 2>/dev/null && networkctl reload 2>/dev/null || true
	ip link set "$IFACE" up 2>/dev/null || true

	cat <<EOF

==============================================================
 Done. $IFACE is an ordinary network interface again.

 It has no connection profile yet if the previous run deleted one.
 Give it an address, e.g. DHCP:
     sudo nmcli connection add type ethernet ifname $IFACE \\
          con-name internet ipv4.method auto ipv6.method auto
     sudo nmcli connection up internet

 Check:  ip -br addr show $IFACE ; ip route
==============================================================
EOF
}

if [[ $UNDO -eq 1 ]]; then
	undo_iface
	exit 0
fi

# Resolve the RT core (only used for the IRQ-affinity sanity check).
if [[ -z "$RT_CORE" ]]; then
	RT_CORE="$(grep -oP '#define\s+RT_CPU_CORE\s+\K[0-9]+' "$REPO_DIR/main.h" 2>/dev/null || true)"
	RT_CORE="${RT_CORE:-1}"
fi

DRIVER="$(ethtool -i "$IFACE" 2>/dev/null | awk -F': ' '/^driver:/{print $2}')"
BUSINFO="$(ethtool -i "$IFACE" 2>/dev/null | awk -F': ' '/^bus-info:/{print $2}')"

echo "=============================================================="
echo " EtherCAT NIC setup"
echo "   interface : $IFACE  (driver: ${DRIVER:-?}, bus: ${BUSINFO:-?})"
echo "   link speed: $SPEED"
echo "   RT core   : $RT_CORE  (IRQ affinity check only)"
echo "   service   : $([[ $INSTALL_SERVICE -eq 1 ]] && echo 'install systemd oneshot' || echo 'no (settings not persisted across reboot)')"
echo "=============================================================="

# Warn loudly if this interface is the one carrying the default route / an IP.
IFACE_IP="$(ip -4 -o addr show dev "$IFACE" 2>/dev/null | awk '{print $4}' | paste -sd, -)"
DEFAULT_DEV="$(ip -4 route show default 2>/dev/null | awk '{print $5; exit}')"
if [[ -n "$IFACE_IP" || "$DEFAULT_DEV" == "$IFACE" ]]; then
	echo
	echo "  !! WARNING: $IFACE currently has IP(s) [${IFACE_IP:-none}]"
	[[ "$DEFAULT_DEV" == "$IFACE" ]] && echo "  !! WARNING: $IFACE currently carries the DEFAULT ROUTE."
	echo "  !! If your session runs over this interface it WILL drop."
	echo "  !! Make sure internet is already moved to the other NIC first."
fi

if [[ $ASSUME_YES -ne 1 ]]; then
	echo
	read -r -p "Proceed configuring '$IFACE' for EtherCAT? [y/N] " ans
	[[ "$ans" == "y" || "$ans" == "Y" ]] || { echo "aborted."; exit 1; }
fi

# ---------------------------------------------------------------------------
# 1. Take the interface away from the OS network managers (no DHCP / IPv6 / EEE
#    renegotiation / autoconnect on the fieldbus link).
# ---------------------------------------------------------------------------
echo
echo "-- [1/5] removing $IFACE from NetworkManager management"
NM_DROPIN="/etc/NetworkManager/conf.d/99-ethercat-${IFACE}.conf"
if command -v nmcli >/dev/null && systemctl is-active --quiet NetworkManager; then
	cat > "$NM_DROPIN" <<EOF
# Managed by scripts/setup-ethercat-nic.sh -- $IFACE is a dedicated EtherCAT
# fieldbus link and must not be touched by NetworkManager (no DHCP, no autoconnect).
[keyfile]
unmanaged-devices=interface-name:$IFACE
EOF
	echo "   wrote $NM_DROPIN"
	# Delete any leftover connection profiles bound to this interface. Match on
	# connection.interface-name, not the DEVICE column: DEVICE is empty while the
	# device is unavailable, and NAME may contain spaces ("Wired connection 1").
	while IFS= read -r name; do
		[[ -n "$name" ]] || continue
		bound="$(nmcli -g connection.interface-name connection show "$name" 2>/dev/null)"
		[[ "$bound" == "$IFACE" ]] || continue
		echo "   deleting stale connection profile: $name"
		nmcli connection delete "$name" >/dev/null 2>&1 || true
	done < <(nmcli -t -g NAME connection show 2>/dev/null)
	nmcli general reload 2>/dev/null || systemctl reload NetworkManager || true
	nmcli device set "$IFACE" managed no 2>/dev/null || true
else
	echo "   NetworkManager not active -- skipping (check netplan/systemd-networkd yourself)"
fi

# systemd-networkd, if present, should also leave it alone.
if systemctl is-active --quiet systemd-networkd 2>/dev/null; then
	mkdir -p /etc/systemd/network
	cat > "/etc/systemd/network/99-ethercat-${IFACE}.network" <<EOF
[Match]
Name=$IFACE
[Link]
Unmanaged=yes
EOF
	echo "   wrote /etc/systemd/network/99-ethercat-${IFACE}.network"
	networkctl reload 2>/dev/null || true
fi

# ---------------------------------------------------------------------------
# 2. Kill IPv6 on the fieldbus link (stops RS/RA/DAD frames).
# ---------------------------------------------------------------------------
echo "-- [2/5] disabling IPv6 on $IFACE"
sysctl -qw "net.ipv6.conf.${IFACE//./\/}.disable_ipv6=1" || true
SYSCTL_DROPIN="/etc/sysctl.d/99-ethercat-${IFACE}.conf"
echo "net.ipv6.conf.${IFACE//./\/}.disable_ipv6 = 1" > "$SYSCTL_DROPIN"
echo "   wrote $SYSCTL_DROPIN"

# ---------------------------------------------------------------------------
# 3. RT link tuning via ethtool. Each setting is best-effort: old NICs / drivers
#    reject some of them, which is fine.
# ---------------------------------------------------------------------------
echo "-- [3/5] applying ethtool RT tuning"

# Offloads: batching/segmentation add latency and jitter, and TSO on the 82579
# family is linked to "Detected Hardware Unit Hang". Turn them all off.
try_each -K tso off gso off gro off lro off rx-gro-hw off sg off rxvlan off txvlan off

# Energy Efficient Ethernet (802.3az) / LPI -- causes sporadic link blips.
try ethtool --set-eee "$IFACE" eee off
try ethtool --set-eee "$IFACE" tx-lpi off

# Flow control / pause frames -- a paused TX = a missed cycle.
try ethtool -A "$IFACE" autoneg off
try ethtool -A "$IFACE" rx off
try ethtool -A "$IFACE" tx off

# Interrupt coalescing / throttling -- deliver every frame immediately.
# On e1000/e1000e, rx-usecs maps to InterruptThrottleRate: 0 = throttling OFF,
# 3 = "dynamic conservative" (~4000 ints/s = ~250 us floor -- the default, and a
# ~250 us hit on pdo_exchange_us). Force 0. rx-frames/adaptive-rx are not
# supported by e1000e and would fail the whole call if bundled.
try_each -C adaptive-rx off adaptive-tx off rx-usecs 0 tx-usecs 0 rx-frames 1 tx-frames 1

# Link speed -- a fixed speed can't spend 2-4 s renegotiating mid-run.
case "$SPEED" in
	auto) try ethtool -s "$IFACE" autoneg on ;;
	100)  try ethtool -s "$IFACE" speed 100  duplex full autoneg off ;;
	1000) try ethtool -s "$IFACE" speed 1000 duplex full autoneg off ;;
	*) echo "   unknown --speed '$SPEED' (use 100|1000|auto)"; exit 2 ;;
esac

# ---------------------------------------------------------------------------
# 4. Bring the link up (no address).
# ---------------------------------------------------------------------------
echo "-- [4/5] bringing $IFACE up (no IP)"
ip addr flush dev "$IFACE" 2>/dev/null || true
ip link set "$IFACE" up
# Give a fixed-speed link a moment to come up before we check it.
[[ "$SPEED" != "auto" ]] && sleep 3 || true

# ---------------------------------------------------------------------------
# 5. Sanity checks.
# ---------------------------------------------------------------------------
echo "-- [5/5] verification"

LINK="$(ethtool "$IFACE" 2>/dev/null | awk -F': ' '/Link detected/{print $2}')"
NEG_SPEED="$(cat "/sys/class/net/$IFACE/speed" 2>/dev/null || echo '?')"
echo "   link detected : ${LINK:-?}   speed: ${NEG_SPEED} Mb/s"
[[ "$LINK" == "yes" ]] || echo "   !! no link -- check the cable to the slave"

# IRQ(s) for this NIC, their type (MSI vs shared legacy), and CPU affinity.
echo "   interrupts:"
IRQS="$(awk -v i="$IFACE" 'index($0, i) { gsub(/[: ]/, "", $1); print $1 }' /proc/interrupts)"
for irq in $IRQS; do
	[[ "$irq" =~ ^[0-9]+$ ]] || continue
	line="$(grep -E "^\s*$irq:" /proc/interrupts || true)"
	kind="$(echo "$line" | grep -oE 'PCI-MSI|IO-APIC|edge|fasteoi' | paste -sd/ -)"
	shared="$(echo "$line" | sed 's/.*fasteoi//; s/.*edge//' | tr ',' '\n' | grep -v "$IFACE" | grep -c '[a-z]' || true)"
	aff="$(cat "/proc/irq/$irq/smp_affinity_list" 2>/dev/null || echo '?')"
	echo "     IRQ $irq  [$kind]  affinity=$aff"
	if echo ",$aff," | grep -qE "(^|,)$RT_CORE(,|$)|(^|,)$RT_CORE-|-$RT_CORE(,|$)"; then
		echo "       !! this IRQ can land on RT core $RT_CORE -- repin it off:"
		echo "          echo <mask without core $RT_CORE> > /proc/irq/$irq/smp_affinity_list"
	fi
	if [[ "$kind" == *IO-APIC* && "${shared:-0}" -gt 0 ]]; then
		echo "       !! legacy IRQ shared with another device -- expect extra jitter."
		echo "          Prefer a NIC with its own MSI/MSI-X vectors (e.g. enp2s0 on this box)."
	fi
done

echo
echo "-- current ethtool state for $IFACE --"
ethtool -k "$IFACE" 2>/dev/null | grep -E 'tcp-segmentation-offload|generic-(segmentation|receive)-offload|large-receive-offload' | sed 's/^/   /'
ethtool --show-eee "$IFACE" 2>/dev/null | grep -E 'EEE status' | sed 's/^/   /' || true

# ---------------------------------------------------------------------------
# Optional: persist the ethtool tuning across reboots via a systemd oneshot.
# ---------------------------------------------------------------------------
if [[ $INSTALL_SERVICE -eq 1 ]]; then
	echo
	echo "-- installing systemd oneshot: ethercat-nic@.service"
	cat > /etc/systemd/system/ethercat-nic@.service <<EOF
[Unit]
Description=RT tuning for EtherCAT NIC %i
Wants=network-pre.target
Before=network-pre.target
BindsTo=sys-subsystem-net-devices-%i.device
After=sys-subsystem-net-devices-%i.device

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=$SCRIPT_DIR/setup-ethercat-nic.sh --iface %i --speed $SPEED --rt-core $RT_CORE --yes

[Install]
WantedBy=multi-user.target
EOF
	systemctl daemon-reload
	systemctl enable "ethercat-nic@${IFACE}.service" >/dev/null
	echo "   enabled ethercat-nic@${IFACE}.service"
fi

cat <<EOF

==============================================================
 Done. $IFACE is now a dedicated, RT-tuned EtherCAT link.

 Run the fieldbus against it:
     sudo ./ethercat-voice-coil-controller $IFACE

 If internet still needs moving to another NIC, release the old fieldbus NIC
 first, then give the new one an address:
     sudo $0 --undo --iface <OLD_IFACE>
     sudo nmcli connection add type ethernet ifname <NET_IFACE> \\
          con-name internet ipv4.method auto ipv6.method auto
     sudo nmcli connection up internet

 A/B check vs. the old NIC -- run 30k cycles on each and compare the
 CSV summary line (missed deadlines / max jitter) and the
 cycle_jitter_us + pdo_exchange_us columns in data/voice_coil_log_*.csv.
==============================================================
EOF
