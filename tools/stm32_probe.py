#!/usr/bin/env python3
"""
stm32_probe.py - identify and test an STM32 target through an ST-LINK + OpenOCD.

Standard library only (pyserial is used when installed, and is required for
the UART test on Windows). Requires OpenOCD >= 0.12 on PATH (or --openocd).

Subcommands
  probes                  list ST-LINK probes on USB and their serial ports
  identify                core, device id/family/revision, flash size, UID,
                          vector table of the image in flash
  read ADDR [ADDR...]     read 32 bit words (halted, read twice, compared)
  clock [--hse HZ]        decode SYSCLK/HCLK/PCLK from RCC (F2/F4/F7 only)
  backup --out FILE|DIR/  dump the whole flash to a file (+ .sha256);
                          a directory gets <device>-<uid>-<time>.bin
  flash FILE              program + verify + reset (ELF/HEX, or BIN with --address)
  uart-echo [--port P]    send text over the ST-LINK VCP and check the echo
  uart-scan [--port P]    try common baud rates, report which gives readable text

Common options: --serial SN (select probe), --speed KHZ, --allow-reset
(connect under reset if a normal attach fails; this RESETS the target),
--json (machine readable output).

Read reliability: with ST-LINK in HLA mode, flash/system memory reads while
the core is running were seen returning stale values, and bus faults are not
reported (unmapped addresses return stale data). Therefore every read here is
done with the core halted (and resumed afterwards if it was running), done
twice and compared, and values are validated against known tables.
"""

import argparse
import glob
import hashlib
import json
import os
import platform
import re
import shutil
import subprocess
import sys
import time

HERE = os.path.dirname(os.path.abspath(__file__))
GENERIC_CFG = os.path.join(HERE, "stm32-generic.cfg")

ST_VID = 0x0483
STLINK_PIDS = {
    0x3744: ("ST-LINK/V1", False),
    0x3748: ("ST-LINK/V2", False),
    0x374A: ("ST-LINK/V2-1", True),
    0x374B: ("ST-LINK/V2-1", True),
    0x3752: ("ST-LINK/V2-1 (no MSD)", True),
    0x374E: ("STLINK-V3E", True),
    0x374F: ("STLINK-V3", True),
    0x3753: ("STLINK-V3 (2 VCP)", True),
    0x3754: ("STLINK-V3 (no MSD)", True),
    0x3757: ("STLINK-V3PWR", True),
}  # (name, has virtual COM port)

CORES = {0xC20: "Cortex-M0", 0xC60: "Cortex-M0+", 0xC23: "Cortex-M3",
         0xC24: "Cortex-M4", 0xC27: "Cortex-M7", 0xD21: "Cortex-M33"}

# Where DBGMCU_IDCODE is, tried in order, by core part number
DBGMCU_BY_CORE = {
    0xC20: [0x40015800], 0xC60: [0x40015800],
    0xC23: [0xE0042000], 0xC24: [0xE0042000],
    0xC27: [0xE0042000, 0x5C001000],  # F7, H7
    0xD21: [0xE0044000],
}

# Per family: flash size register (16 bit, KB), unique id (96 bit), OpenOCD cfg
FAMILIES = {
    "F0": dict(flash_size=0x1FFFF7CC, uid=0x1FFFF7AC, cfg="target/stm32f0x.cfg"),
    "F1": dict(flash_size=0x1FFFF7E0, uid=0x1FFFF7E8, cfg="target/stm32f1x.cfg"),
    "F2": dict(flash_size=0x1FFF7A22, uid=0x1FFF7A10, cfg="target/stm32f2x.cfg"),
    "F3": dict(flash_size=0x1FFFF7CC, uid=0x1FFFF7AC, cfg="target/stm32f3x.cfg"),
    "F4": dict(flash_size=0x1FFF7A22, uid=0x1FFF7A10, cfg="target/stm32f4x.cfg"),
    "F7": dict(flash_size=0x1FF0F442, uid=0x1FF0F420, cfg="target/stm32f7x.cfg"),
    "H7": dict(flash_size=0x1FF1E880, uid=0x1FF1E800, cfg="target/stm32h7x.cfg"),
    "L0": dict(flash_size=0x1FF8007C, uid=0x1FF80050, cfg="target/stm32l0.cfg"),
    "L1": dict(flash_size=0x1FF800CC, uid=0x1FF800D0, cfg="target/stm32l1.cfg"),
    "L4": dict(flash_size=0x1FFF75E0, uid=0x1FFF7590, cfg="target/stm32l4x.cfg"),
    "L5": dict(flash_size=0x0BFA05E0, uid=0x0BFA0590, cfg="target/stm32l5x.cfg"),
    "G0": dict(flash_size=0x1FFF75E0, uid=0x1FFF7590, cfg="target/stm32g0x.cfg"),
    "G4": dict(flash_size=0x1FFF75E0, uid=0x1FFF7590, cfg="target/stm32g4x.cfg"),
    "WB": dict(flash_size=0x1FFF75E0, uid=0x1FFF7590, cfg="target/stm32wbx.cfg"),
    "WL": dict(flash_size=0x1FFF75E0, uid=0x1FFF7590, cfg="target/stm32wlx.cfg"),
    "U5": dict(flash_size=0x0BFA07A0, uid=0x0BFA0700, cfg="target/stm32u5x.cfg"),
}

# DBGMCU DEV_ID -> (description, family, per-device overrides)
DEVICES = {
    0x412: ("STM32F10x low-density", "F1", {}),
    0x410: ("STM32F10x medium-density", "F1", {}),
    0x414: ("STM32F10x high-density", "F1", {}),
    0x430: ("STM32F10x XL-density", "F1", {}),
    0x418: ("STM32F105/107 connectivity line", "F1", {}),
    0x420: ("STM32F100 value line low/medium-density", "F1", {}),
    0x428: ("STM32F100 value line high-density", "F1", {}),
    0x411: ("STM32F2xx", "F2", {}),
    0x413: ("STM32F405/407/415/417", "F4", {}),
    0x419: ("STM32F42x/43x", "F4", {}),
    0x423: ("STM32F401xB/C", "F4", {}),
    0x433: ("STM32F401xD/E", "F4", {}),
    0x458: ("STM32F410", "F4", {}),
    0x431: ("STM32F411", "F4", {}),
    0x441: ("STM32F412", "F4", {}),
    0x421: ("STM32F446", "F4", {}),
    0x434: ("STM32F469/479", "F4", {}),
    0x463: ("STM32F413/423", "F4", {}),
    0x440: ("STM32F030x8/F05x", "F0", {}),
    0x444: ("STM32F03x", "F0", {}),
    0x445: ("STM32F04x/F070x6", "F0", {}),
    0x448: ("STM32F07x", "F0", {}),
    0x442: ("STM32F09x/F030xC", "F0", {}),
    0x422: ("STM32F302xB/C, F303xB/C, F358", "F3", {}),
    0x432: ("STM32F37x", "F3", {}),
    0x438: ("STM32F303x6/8, F334, F328", "F3", {}),
    0x439: ("STM32F301, F302x6/8, F318", "F3", {}),
    0x446: ("STM32F302xD/E, F303xD/E, F398", "F3", {}),
    0x449: ("STM32F74x/75x", "F7", {}),
    0x451: ("STM32F76x/77x", "F7", {}),
    0x452: ("STM32F72x/73x", "F7", dict(flash_size=0x1FF07A22, uid=0x1FF07A10)),
    0x450: ("STM32H74x/75x", "H7", {}),
    0x480: ("STM32H7A3/B3/B0", "H7", {}),
    0x483: ("STM32H72x/73x", "H7", {}),
    0x457: ("STM32L0 category 1", "L0", {}),
    0x425: ("STM32L0 category 2", "L0", {}),
    0x417: ("STM32L0 category 3", "L0", {}),
    0x447: ("STM32L0 category 5", "L0", {}),
    0x416: ("STM32L1 category 1", "L1", dict(flash_size=0x1FF8004C, uid=0x1FF80050)),
    0x429: ("STM32L1 category 2", "L1", dict(flash_size=0x1FF8004C, uid=0x1FF80050)),
    0x427: ("STM32L1 category 3", "L1", {}),
    0x436: ("STM32L1 category 4", "L1", {}),
    0x437: ("STM32L1 category 5/6", "L1", {}),
    0x415: ("STM32L47x/48x", "L4", {}),
    0x435: ("STM32L43x/44x", "L4", {}),
    0x462: ("STM32L45x/46x", "L4", {}),
    0x461: ("STM32L496/4A6", "L4", {}),
    0x464: ("STM32L41x/42x", "L4", {}),
    0x470: ("STM32L4R/L4S", "L4", {}),
    0x471: ("STM32L4P5/Q5", "L4", {}),
    0x472: ("STM32L55x/56x", "L5", {}),
    0x466: ("STM32G03x/04x", "G0", {}),
    0x460: ("STM32G07x/08x", "G0", {}),
    0x456: ("STM32G05x/06x", "G0", {}),
    0x467: ("STM32G0Bx/0Cx", "G0", {}),
    0x468: ("STM32G43x/44x", "G4", {}),
    0x469: ("STM32G47x/48x", "G4", {}),
    0x479: ("STM32G49x/4Ax", "G4", {}),
    0x495: ("STM32WB5x", "WB", {}),
    0x497: ("STM32WLE/WL5", "WL", {}),
    0x482: ("STM32U575/585", "U5", {}),
    0x481: ("STM32U59x/5Ax", "U5", {}),
}

# Silicon revision codes are family specific; print them raw with common names
REV_NAMES = {0x1000: "A", 0x1001: "Z", 0x1003: "Y", 0x1007: "1", 0x2000: "B",
             0x2001: "X/2", 0x2003: "Y/3", 0x1005: "5"}

FLASH_BASE = 0x08000000


class ProbeError(Exception):
    pass


# --------------------------------------------------------------------------
# OpenOCD
# --------------------------------------------------------------------------
def openocd_bin(args):
    exe = args.openocd or shutil.which("openocd")
    if not exe:
        raise ProbeError("openocd not found on PATH (install OpenOCD >= 0.12 or pass --openocd)")
    return exe


def run_openocd(args, cmds, target_cfg=None, under_reset=False, speed=None,
                finish=("shutdown",), timeout=120):
    """Run one OpenOCD session; returns (returncode, combined output)."""
    cmd = [openocd_bin(args), "-f", "interface/stlink.cfg"]
    if args.serial:
        cmd += ["-c", "adapter serial %s" % args.serial]
    cmd += ["-c", "adapter speed %d" % (speed or args.speed)]
    cmd += ["-f", target_cfg or GENERIC_CFG]
    if under_reset:
        cmd += ["-c", "reset_config srst_only srst_nogate connect_assert_srst"]
    cmd += ["-c", "init"]
    for c in list(cmds) + list(finish):
        cmd += ["-c", c]
    try:
        p = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                           universal_newlines=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        raise ProbeError("OpenOCD timed out")
    if args.verbose:
        sys.stderr.write("$ %s\n%s\n" % (" ".join(cmd), p.stdout))
    return p.returncode, p.stdout


def parse_probe_info(out):
    info = {}
    m = re.search(r"STLINK (\S+) \(API v(\d+)\) VID:PID ([0-9A-Fa-f]{4}):([0-9A-Fa-f]{4})", out)
    if m:
        info["stlink_firmware"] = m.group(1)
        info["stlink_pid"] = "0x" + m.group(4).upper()
    m = re.search(r"Target voltage: ([0-9.]+)", out)
    if m:
        info["target_voltage"] = float(m.group(1))
    m = re.search(r"(Cortex-M\S*) (r\dp\d) processor detected", out)
    if m:
        info["core_reported"] = "%s %s" % (m.group(1), m.group(2))
    return info


def parse_reads(out):
    """'RD32 0xe000ed00: 410fc241' lines -> {addr: [values...]} in order."""
    vals = {}
    for m in re.finditer(r"^RD(32|16) (?:0x)?([0-9a-fA-F]+):? (ERR|(?:0x)?[0-9a-fA-F]+)", out, re.M):
        addr = int(m.group(2), 16)
        v = None if m.group(3) == "ERR" else int(m.group(3), 16)
        vals.setdefault(addr, []).append(v)
    return vals


def halted_reads(args, reads, under_reset=False):
    """reads: list of (width, addr). Each read twice with the core halted."""
    cmds = ["probe_begin"]
    for _ in range(2):
        cmds += ["rd%d 0x%08x" % (w, a) for (w, a) in reads]
    cmds += ["probe_end"]
    rc, out = run_openocd(args, cmds, under_reset=under_reset)
    info = parse_probe_info(out)
    if "core_reported" not in info:
        return None, info, out
    vals = parse_reads(out)
    result = {}
    for (_, a) in reads:
        v = vals.get(a, [])
        if len(v) >= 2 and v[0] is not None and v[0] == v[1]:
            result[a] = v[0]
        else:
            result[a] = None  # failed or inconsistent
    info["halted_was_running"] = "PROBE_STATE running" in out
    return result, info, out


def attach_reads(args, reads):
    """halted_reads with fallbacks: slower clock, then (if allowed) under reset."""
    attempts = [dict(speed=args.speed, under_reset=False)]
    if args.speed > 200:
        attempts.append(dict(speed=200, under_reset=False))
    if args.allow_reset:
        attempts.append(dict(speed=200, under_reset=True))
    log = []
    for a in attempts:
        saved = args.speed
        args.speed = a["speed"]
        try:
            res, info, out = halted_reads(args, reads, under_reset=a["under_reset"])
        finally:
            args.speed = saved
        log.append("speed %d kHz%s: %s" % (a["speed"], ", connect under reset" if a["under_reset"] else "",
                                           "ok" if res is not None else "attach failed"))
        if res is not None:
            info["attach"] = log
            return res, info
    hint = "" if args.allow_reset else " (try --allow-reset to connect under reset; this resets the target)"
    raise ProbeError("could not attach to the target: %s%s" % ("; ".join(log), hint))


# --------------------------------------------------------------------------
# USB / serial port discovery
# --------------------------------------------------------------------------
def usb_probes():
    system = platform.system()
    found = []
    if system == "Darwin":
        out = subprocess.run(["ioreg", "-p", "IOUSB", "-l", "-w0"], stdout=subprocess.PIPE,
                             universal_newlines=True).stdout
        cur = None
        for line in out.splitlines():
            m = re.search(r"\+-o (.+?)@([0-9a-fA-F]+)\s+<class", line)
            if m:
                cur = {"name": m.group(1), "location": m.group(2)}
                found.append(cur)
                continue
            m = re.search(r'"(idVendor|idProduct|USB Serial Number|USB Product Name)" = (.+)$', line)
            if m and cur is not None:
                cur[m.group(1)] = m.group(2).strip().strip('"')
        devs = []
        for d in found:
            if d.get("idVendor") and int(d["idVendor"]) == ST_VID:
                devs.append(dict(pid=int(d.get("idProduct", "0")), serial=d.get("USB Serial Number", ""),
                                 product=d.get("USB Product Name", d["name"]), location=d["location"]))
        return devs
    if system == "Linux":
        devs = []
        for path in glob.glob("/sys/bus/usb/devices/*"):
            try:
                vid = int(open(os.path.join(path, "idVendor")).read(), 16)
                if vid != ST_VID:
                    continue
                pid = int(open(os.path.join(path, "idProduct")).read(), 16)
                rd = lambda n: open(os.path.join(path, n)).read().strip() if os.path.exists(os.path.join(path, n)) else ""
                devs.append(dict(pid=pid, serial=rd("serial"), product=rd("product"), location=os.path.basename(path)))
            except (OSError, ValueError):
                continue
        return devs
    if system == "Windows":
        ps = ("Get-CimInstance Win32_PnPEntity | Where-Object { $_.DeviceID -like 'USB\\VID_0483*' } | "
              "Select-Object DeviceID, Name | ConvertTo-Json")
        out = subprocess.run(["powershell", "-NoProfile", "-Command", ps], stdout=subprocess.PIPE,
                             universal_newlines=True).stdout
        try:
            items = json.loads(out) if out.strip() else []
        except ValueError:
            items = []
        if isinstance(items, dict):
            items = [items]
        devs = []
        for it in items:
            m = re.match(r"USB\\VID_0483&PID_([0-9A-F]{4})\\(.+)$", it.get("DeviceID", ""), re.I)
            if m and "&MI_" not in it["DeviceID"].upper():
                devs.append(dict(pid=int(m.group(1), 16), serial=m.group(2), product=it.get("Name", ""), location=""))
        return devs
    return []


def serial_ports():
    """List of dicts: port, serial (USB serial number if known), description."""
    try:
        from serial.tools import list_ports  # pyserial, all OSes
        return [dict(port=p.device, serial=p.serial_number or "", description=p.description or "")
                for p in list_ports.comports() if p.vid == ST_VID]
    except ImportError:
        pass
    system = platform.system()
    if system == "Darwin":
        return [dict(port=p, serial="", description="usbmodem") for p in sorted(glob.glob("/dev/cu.usbmodem*"))]
    if system == "Linux":
        ports = []
        for p in sorted(glob.glob("/dev/serial/by-id/*")):
            if re.search(r"STM|STLink|0483", p, re.I):
                m = re.search(r"_([0-9A-Fa-f]{12,})-if", p)
                ports.append(dict(port=os.path.realpath(p), serial=m.group(1) if m else "", description=os.path.basename(p)))
        return ports
    if system == "Windows":
        ps = ("Get-CimInstance Win32_PnPEntity | Where-Object { $_.DeviceID -like 'USB\\VID_0483*' -and $_.Name -match 'COM\\d+' } | "
              "Select-Object DeviceID, Name | ConvertTo-Json")
        out = subprocess.run(["powershell", "-NoProfile", "-Command", ps], stdout=subprocess.PIPE,
                             universal_newlines=True).stdout
        try:
            items = json.loads(out) if out.strip() else []
        except ValueError:
            items = []
        if isinstance(items, dict):
            items = [items]
        return [dict(port=re.search(r"(COM\d+)", it["Name"]).group(1), serial="", description=it["Name"]) for it in items]
    return []


def port_for_probe(probe, ports):
    for p in ports:
        if p["serial"] and probe["serial"] and p["serial"].upper() == probe["serial"].upper():
            return p["port"], "serial number match"
    loc = probe.get("location", "")
    if platform.system() == "Darwin" and loc:
        prefix = loc.rstrip("0")  # 14200000 -> 142 ; usbmodem14203
        for p in ports:
            if prefix and re.search(r"usbmodem%s" % re.escape(prefix), p["port"]):
                return p["port"], "USB location match"
    if len(ports) == 1:
        return ports[0]["port"], "only ST serial port"
    return None, None


def resolve_port(args):
    """--port, or the VCP of the (selected) ST-LINK."""
    if args.port:
        return args.port
    probes = [p for p in usb_probes() if STLINK_PIDS.get(p["pid"], ("", False))[1]]
    if args.serial:
        probes = [p for p in probes if p["serial"].upper() == args.serial.upper()]
    if len(probes) != 1:
        raise ProbeError("%d ST-LINK(s) with a VCP found; pass --port (or --serial)" % len(probes))
    port, _ = port_for_probe(probes[0], serial_ports())
    if not port:
        raise ProbeError("VCP serial port of the ST-LINK not found; pass --port")
    return port


# --------------------------------------------------------------------------
# Commands
# --------------------------------------------------------------------------
def cmd_probes(args):
    probes = usb_probes()
    ports = serial_ports()
    rows = []
    for p in probes:
        name, vcp = STLINK_PIDS.get(p["pid"], ("unknown ST device", False))
        port, how = port_for_probe(p, ports) if vcp else (None, None)
        rows.append(dict(model=name, pid="0x%04X" % p["pid"], serial=p["serial"], product=p["product"],
                         has_vcp=vcp, vcp_port=port, vcp_match=how))
    if args.json:
        print(json.dumps(dict(probes=rows, serial_ports=ports), indent=2))
        return
    if not rows:
        print("No ST-LINK found on USB. Check the cable (data, not charge-only) and the CN4 jumpers on Discovery boards.")
    for r in rows:
        print("%-22s PID %s  SN %s" % (r["model"], r["pid"], r["serial"]))
        if r["has_vcp"]:
            print("  VCP port: %s%s" % (r["vcp_port"] or "not found", " (%s)" % r["vcp_match"] if r["vcp_match"] else ""))
        else:
            print("  no virtual COM port on this ST-LINK version")
    if len(rows) > 1:
        print("\nSeveral probes: pass --serial <SN> to the other commands.")


def identify(args):
    core_reads = [(32, 0xE000ED00), (32, 0xE0042000), (32, 0x40015800), (32, 0x5C001000), (32, 0xE0044000)]
    vals, info = attach_reads(args, core_reads)
    cpuid = vals[0xE000ED00]
    if cpuid is None:
        raise ProbeError("CPUID read failed/inconsistent")
    part = (cpuid >> 4) & 0xFFF
    r = dict(info)
    r["cpuid"] = "0x%08X" % cpuid
    r["core"] = "%s r%dp%d" % (CORES.get(part, "unknown (0x%03X)" % part), (cpuid >> 20) & 0xF, cpuid & 0xF)

    dev = None
    for addr in DBGMCU_BY_CORE.get(part, [0xE0042000, 0x40015800, 0x5C001000, 0xE0044000]):
        v = vals.get(addr)
        if v is not None and (v & 0xFFF) in DEVICES:
            dev = (addr, v)
            break
    if dev is None:
        r["warning"] = "DBGMCU_IDCODE not recognised; check the reference manual (DBGMCU chapter)"
        return r
    addr, idcode = dev
    dev_id, rev_id = idcode & 0xFFF, (idcode >> 16) & 0xFFFF
    name, family, overrides = DEVICES[dev_id]
    fam = dict(FAMILIES[family], **overrides)
    r.update(idcode="0x%08X" % idcode, idcode_addr="0x%08X" % addr, dev_id="0x%03X" % dev_id,
             device=name, family=family, rev_id="0x%04X" % rev_id,
             revision=REV_NAMES.get(rev_id, "see errata sheet"), openocd_target=fam["cfg"])

    fam_reads = [(16, fam["flash_size"]), (32, fam["uid"]), (32, fam["uid"] + 4), (32, fam["uid"] + 8),
                 (32, FLASH_BASE), (32, FLASH_BASE + 4)]
    v2, _ = attach_reads(args, fam_reads)
    fs = v2[fam["flash_size"]]
    r["flash_kb"] = fs if fs not in (None, 0, 0xFFFF) else None
    uid = [v2[fam["uid"] + o] for o in (0, 4, 8)]
    r["uid"] = "-".join("%08X" % u for u in uid) if None not in uid else None
    sp, rv = v2[FLASH_BASE], v2[FLASH_BASE + 4]
    img = dict(initial_sp=None if sp is None else "0x%08X" % sp, reset_handler=None if rv is None else "0x%08X" % rv)
    if sp == 0xFFFFFFFF and rv == 0xFFFFFFFF:
        img["state"] = "flash erased (no image)"
    elif sp is not None and rv is not None:
        sp_ok = (sp & 0xFF000000) in (0x20000000, 0x24000000, 0x10000000) or (sp & 0xFFF00000) == 0x30000000
        size = (r["flash_kb"] or 0) * 1024
        rv_ok = (rv & 1) == 1 and FLASH_BASE <= (rv & ~1) < FLASH_BASE + (size or 0x200000)
        img["state"] = "valid vector table" if sp_ok and rv_ok else "unexpected vector table (not a standard image?)"
    r["image"] = img
    return r


def cmd_identify(args):
    r = identify(args)
    if args.json:
        print(json.dumps(r, indent=2))
        return
    rows = [("ST-LINK firmware", r.get("stlink_firmware")), ("Target voltage", "%.2f V" % r["target_voltage"] if "target_voltage" in r else None),
            ("Core", r.get("core")), ("CPUID", r.get("cpuid")),
            ("DBGMCU_IDCODE", "%s @ %s" % (r.get("idcode"), r.get("idcode_addr")) if r.get("idcode") else None),
            ("Device", "%s (DEV_ID %s, family %s)" % (r["device"], r["dev_id"], r["family"]) if "device" in r else None),
            ("Revision", "%s (REV_ID %s)" % (r["revision"], r["rev_id"]) if "rev_id" in r else None),
            ("Flash", "%d KB" % r["flash_kb"] if r.get("flash_kb") else None), ("Unique ID", r.get("uid")),
            ("Image in flash", "%s - SP %s, Reset %s" % (r["image"]["state"], r["image"]["initial_sp"], r["image"]["reset_handler"]) if "image" in r else None),
            ("OpenOCD target", r.get("openocd_target")), ("Attach", "; ".join(r.get("attach", [])))]
    for k, v in rows:
        if v:
            print("%-17s %s" % (k, v))
    if "warning" in r:
        print("WARNING: " + r["warning"])


def cmd_read(args):
    addrs = [int(a, 0) for a in args.addr]
    vals, _ = attach_reads(args, [(32, a) for a in addrs])
    out = {"0x%08X" % a: (None if vals[a] is None else "0x%08X" % vals[a]) for a in addrs}
    print(json.dumps(out, indent=2) if args.json else "\n".join("%s: %s" % (k, v or "read failed/inconsistent") for k, v in out.items()))


def cmd_clock(args):
    r = identify(args)
    if r.get("family") not in ("F2", "F4", "F7"):
        raise ProbeError("clock decode supports F2/F4/F7 RCC only (family %s); use 'read' with the RM addresses" % r.get("family"))
    RCC, FLASH_ACR = 0x40023800, 0x40023C00
    v, _ = attach_reads(args, [(32, RCC), (32, RCC + 4), (32, RCC + 8), (32, FLASH_ACR)])
    cr, pll, cfgr, acr = v[RCC], v[RCC + 4], v[RCC + 8], v[FLASH_ACR]
    if None in (cr, pll, cfgr, acr):
        raise ProbeError("RCC read failed/inconsistent")
    hsi, hse = 16000000, args.hse
    sws = (cfgr >> 2) & 3
    src = {0: "HSI", 1: "HSE", 2: "PLL", 3: "?"}[sws]
    m, n, p = pll & 0x3F, (pll >> 6) & 0x1FF, (((pll >> 16) & 3) + 1) * 2
    pllsrc = "HSE" if pll & (1 << 22) else "HSI"
    if sws == 0:
        sysclk = hsi
    elif sws == 1:
        sysclk = hse
    else:
        sysclk = (hse if pllsrc == "HSE" else hsi) // m * n // p if m else 0
    ahb = [1] * 8 + [2, 4, 8, 16, 64, 128, 256, 512]
    apb = [1] * 4 + [2, 4, 8, 16]
    hclk = sysclk // ahb[(cfgr >> 4) & 0xF]
    pclk1, pclk2 = hclk // apb[(cfgr >> 10) & 7], hclk // apb[(cfgr >> 13) & 7]
    res = dict(sysclk_source=src, pll_source=pllsrc, pllm=m, plln=n, pllp=p, hse_assumed=hse,
               hse_on=bool(cr & (1 << 16)), hse_ready=bool(cr & (1 << 17)), hse_bypass=bool(cr & (1 << 18)),
               pll_ready=bool(cr & (1 << 25)), sysclk_hz=sysclk, hclk_hz=hclk, pclk1_hz=pclk1, pclk2_hz=pclk2,
               flash_latency_ws=acr & 0xF, rcc_cr="0x%08X" % cr, rcc_pllcfgr="0x%08X" % pll, rcc_cfgr="0x%08X" % cfgr)
    if args.json:
        print(json.dumps(res, indent=2))
        return
    print("SYSCLK source  %s%s" % (src, " (PLL from %s, M=%d N=%d P=%d)" % (pllsrc, m, n, p) if src == "PLL" else ""))
    print("HSE            on=%s ready=%s bypass=%s (assumed %d Hz, --hse to change)" % (res["hse_on"], res["hse_ready"], res["hse_bypass"], hse))
    print("SYSCLK/HCLK    %.3f / %.3f MHz" % (sysclk / 1e6, hclk / 1e6))
    print("PCLK1/PCLK2    %.3f / %.3f MHz" % (pclk1 / 1e6, pclk2 / 1e6))
    print("Flash latency  %d WS" % res["flash_latency_ws"])


def cmd_backup(args):
    r = identify(args)
    size = args.size or (r.get("flash_kb") or 0) * 1024
    if not size:
        raise ProbeError("flash size unknown; pass --size BYTES")
    out = os.path.abspath(args.out)
    if args.out.endswith(("/", "\\")) or os.path.isdir(out):
        name = "%s-%s-%s.bin" % ((r.get("device") or "stm32").split("/")[0].replace(" ", "_"),
                                 r.get("uid") or "nouid", time.strftime("%Y%m%d-%H%M%S"))
        os.makedirs(out, exist_ok=True)
        out = os.path.join(out, name)
    rc, log = run_openocd(args, ["probe_begin", "dump_image {%s} 0x%08x %d" % (out, FLASH_BASE, size), "probe_end"], timeout=600)
    if rc != 0 or not os.path.exists(out) or os.path.getsize(out) != size:
        raise ProbeError("backup failed:\n" + log[-2000:])
    digest = hashlib.sha256(open(out, "rb").read()).hexdigest()
    with open(out + ".sha256", "w") as f:
        f.write("%s  %s\n" % (digest, os.path.basename(out)))
    meta = dict(file=out, bytes=size, sha256=digest, device=r.get("device"), uid=r.get("uid"),
                restore="python3 stm32_probe.py flash %s --address 0x%08X" % (out, FLASH_BASE))
    print(json.dumps(meta, indent=2) if args.json else
          "Saved %d bytes of %s (UID %s) to %s\nsha256 %s\nRestore with: %s" % (size, r.get("device"), r.get("uid"), out, digest, meta["restore"]))


def cmd_flash(args):
    path = os.path.abspath(args.file)
    if not os.path.exists(path):
        raise ProbeError("file not found: " + path)
    cfg = args.target_cfg
    if not cfg:
        cfg = identify(args).get("openocd_target")
        if not cfg:
            raise ProbeError("could not determine the OpenOCD target; pass --target-cfg")
    if path.lower().endswith(".bin"):
        if args.address is None:
            raise ProbeError(".bin files need --address (usually 0x08000000)")
        prog = "program {%s} verify reset exit 0x%08x" % (path, int(args.address, 0))
    else:
        prog = "program {%s} verify reset exit" % path
    rc, log = run_openocd(args, [prog], target_cfg=cfg, under_reset=args.allow_reset, finish=(), timeout=600)
    ok = rc == 0 and "Verified OK" in log
    if args.json:
        print(json.dumps(dict(file=path, target_cfg=cfg, verified=ok), indent=2))
    else:
        print(("Programmed and verified %s (%s), target reset." if ok else "FLASH FAILED for %s (%s)") % (path, cfg))
    if not ok:
        sys.stderr.write(log[-3000:])
        sys.exit(1)


def uart_exchange(port, baud, data, timeout):
    """Send data byte by byte (an RX-interrupt echo has no buffer) and collect replies."""
    received = b""
    try:
        import serial  # pyserial
        with serial.Serial(port, baud, timeout=0.2) as s:
            s.reset_input_buffer()
            time.sleep(0.1)
            for b in data:
                s.write(bytes([b]))
                s.flush()
                received += s.read(1)
            end = time.time() + timeout
            while time.time() < end:
                chunk = s.read(256)
                received += chunk
                if not chunk and len(received) >= len(data):
                    break
        return received
    except ImportError:
        if platform.system() == "Windows":
            raise ProbeError("the UART commands need pyserial on Windows: python -m pip install pyserial")
    import select
    import termios
    import tty
    fd = os.open(port, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        old = termios.tcgetattr(fd)
        tty.setraw(fd)
        attrs = termios.tcgetattr(fd)
        speed = getattr(termios, "B%d" % baud, None)
        if speed is None:
            if platform.system() != "Darwin":  # macOS speed_t is the plain baud value
                raise ProbeError("baud %d not supported by termios here; install pyserial" % baud)
            speed = baud
        attrs[4] = attrs[5] = speed
        attrs[2] |= termios.CLOCAL | termios.CREAD
        attrs[2] &= ~(termios.PARENB | termios.CSTOPB)
        termios.tcsetattr(fd, termios.TCSANOW, attrs)
        termios.tcflush(fd, termios.TCIOFLUSH)
        time.sleep(0.1)
        for b in data:
            os.write(fd, bytes([b]))
            r, _, _ = select.select([fd], [], [], 0.2)
            if r:
                received += os.read(fd, 256)
        end = time.time() + timeout
        while time.time() < end:
            r, _, _ = select.select([fd], [], [], 0.2)
            if r:
                received += os.read(fd, 256)
            elif len(received) >= len(data):
                break
        termios.tcsetattr(fd, termios.TCSANOW, old)
    finally:
        os.close(fd)
    return received


def cmd_uart_echo(args):
    data = (args.text + "\n").encode() if not args.no_newline else args.text.encode()
    args.port = resolve_port(args)
    received = uart_exchange(args.port, args.baud, data, args.timeout)
    ok = received == data
    res = dict(port=args.port, baud=args.baud, sent=data.decode(errors="replace"),
               received=received.decode(errors="replace"), echo_ok=ok)
    if args.json:
        print(json.dumps(res, indent=2))
    else:
        print("sent     %r\nreceived %r\n%s" % (data, received, "ECHO OK" if ok else "ECHO MISMATCH"))
    if not ok:
        sys.exit(1)


def cmd_uart_scan(args):
    """Find the baud rate of whatever firmware is running: send a probe, score the reply."""
    data = (args.text + "\r\n").encode()
    args.port = resolve_port(args)
    results = []
    for baud in args.bauds:
        try:
            rx = uart_exchange(args.port, baud, data, args.timeout)
        except (ProbeError, OSError) as e:
            results.append(dict(baud=baud, bytes=0, printable_ratio=0.0, reply="skipped: %s" % e, echo=False))
            continue
        printable = sum(1 for c in rx if 32 <= c < 127 or c in (9, 10, 13))
        score = printable / len(rx) if rx else 0.0
        results.append(dict(baud=baud, bytes=len(rx), printable_ratio=round(score, 2),
                            reply=rx.decode(errors="replace")[:80], echo=rx == data))
    best = max(results, key=lambda r: (r["printable_ratio"] if r["bytes"] else -1, r["bytes"]))
    if args.json:
        print(json.dumps(dict(results=results, likely_baud=best["baud"] if best["printable_ratio"] > 0.9 else None), indent=2))
        return
    for r in results:
        print("%7d  %3d bytes  %3d%% printable  %r" % (r["baud"], r["bytes"], r["printable_ratio"] * 100, r["reply"]))
    if best["printable_ratio"] > 0.9:
        print("Likely baud rate: %d%s" % (best["baud"], " (echo)" if best["echo"] else ""))
    else:
        print("No readable reply: firmware may not use this UART, or needs a specific command")


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--serial", help="ST-LINK serial number (when several probes are connected)")
    ap.add_argument("--speed", type=int, default=1000, help="SWD clock in kHz (default 1000)")
    ap.add_argument("--allow-reset", action="store_true", help="allow connect-under-reset (RESETS the target)")
    ap.add_argument("--openocd", help="path to the openocd executable")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("-v", "--verbose", action="store_true", help="print OpenOCD commands and output")
    sub = ap.add_subparsers(dest="command")
    sub.required = True
    sub.add_parser("probes").set_defaults(func=cmd_probes)
    sub.add_parser("identify").set_defaults(func=cmd_identify)
    p = sub.add_parser("read")
    p.add_argument("addr", nargs="+")
    p.set_defaults(func=cmd_read)
    p = sub.add_parser("clock")
    p.add_argument("--hse", type=int, default=8000000, help="HSE frequency in Hz (default 8 MHz)")
    p.set_defaults(func=cmd_clock)
    p = sub.add_parser("backup")
    p.add_argument("--out", required=True)
    p.add_argument("--size", type=lambda s: int(s, 0), help="bytes (default: full flash)")
    p.set_defaults(func=cmd_backup)
    p = sub.add_parser("flash")
    p.add_argument("file")
    p.add_argument("--address", help="load address for .bin files")
    p.add_argument("--target-cfg", help="OpenOCD target cfg (default: from identify)")
    p.set_defaults(func=cmd_flash)
    p = sub.add_parser("uart-echo")
    p.add_argument("--port", help="serial port (default: the ST-LINK VCP)")
    p.add_argument("--baud", type=int, default=115200)
    p.add_argument("--text", default="Hello STM32 0123456789")
    p.add_argument("--no-newline", action="store_true")
    p.add_argument("--timeout", type=float, default=2.0)
    p.set_defaults(func=cmd_uart_echo)
    p = sub.add_parser("uart-scan")
    p.add_argument("--port", help="serial port (default: the ST-LINK VCP)")
    p.add_argument("--bauds", type=int, nargs="+", default=[9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600])
    p.add_argument("--text", default="?")
    p.add_argument("--timeout", type=float, default=0.8)
    p.set_defaults(func=cmd_uart_scan)
    args = ap.parse_args()
    try:
        args.func(args)
    except ProbeError as e:
        sys.stderr.write("error: %s\n" % e)
        sys.exit(2)


if __name__ == "__main__":
    main()
