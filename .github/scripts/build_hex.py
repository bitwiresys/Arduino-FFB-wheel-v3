#!/usr/bin/env python3
"""Builds every supported hardware variant of the firmware and packages them
for release. Variants are declared explicitly in .github/variants/*.txt
(one letter-combo per line, "-" for the base/no-letters build) rather than
being inferred by scanning old .hex filenames - the .hex files themselves are
not tracked in git, they're only produced here and shipped via the release
zip/artifact.
"""
from __future__ import annotations

import json
import os
import re
import shutil
import subprocess
import zipfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CONFIG_PATH = ROOT / "brWheel_my" / "Config.h"
SKETCH_DIR = ROOT / "brWheel_my"
VARIANTS_DIR = ROOT / ".github" / "variants"
DIST_DIR = ROOT / "dist"
FW_VERSION = os.environ.get("FW_VERSION", "v250")
ZIP_PATH = Path(os.environ.get("ZIP_PATH", "build.zip"))
# CI sets this to the GitHub Actions run number (same number used in the "fw-build-N" release
# tag), baked into every compiled variant so the control panel can compare its connected
# board's FW_BUILD_ID against the latest release with a single integer comparison.
FW_BUILD_ID = os.environ.get("FW_BUILD_ID", "0")

LIB_DIR = Path(os.environ.get("ARDUINO_LIB_DIR", Path.home() / "Arduino" / "libraries"))

# letter -> (#define name, human description shown in the discoverability manifest)
DEFINE_MAP: dict[str, tuple[str, str]] = {
    "a": ("USE_AUTOCALIB", "Automatic pedal calibration"),
    "b": ("USE_TWOFFBAXIS", "2-axis force feedback (X+Y), 4-channel PWM output"),
    "w": ("USE_AS5600", "Magnetic encoder (AS5600) instead of an optical encoder"),
    "z": ("USE_ZINDEX", "Encoder Z-index (index pulse) support"),
    "h": ("USE_HATSWITCH", "Hat switch (POV) support"),
    "s": ("USE_ADS1015", "External 12-bit ADS1015 ADC for pedals"),
    "t": ("USE_BTNMATRIX", "4x4 button matrix"),
    "f": ("USE_XY_SHIFTER", "XY analog H-shifter support"),
    "i": ("AVG_INPUTS", "Averaging of analog pedal inputs"),
    "e": ("USE_EXTRABTN", "Two extra buttons on pins A2/A3"),
    "x": ("USE_ANALOGFFBAXIS", "Analog FFB axis output"),
    "l": ("USE_LOAD_CELL", "HX711 load cell support"),
    "g": ("USE_MCP4725", "Analog FFB output via external DAC (MCP4725/4275)"),
    "u": ("USE_TCA9548", "Dual magnetic encoders via I2C multiplexer (TCA9548A)"),
    "c": ("USE_CENTERBTN", "Hardware wheel recenter button"),
    "k": ("USE_SPLITAXIS", "Gas axis split into separate gas/brake axes"),
    "o": ("USE_MOTOR_NTC", "Motor NTC 100k thermistor over-temperature FFB cutoff"),
    "v": ("USE_AXIS_TWEAKS", "Per-axis invert/disable via serial commands"),
}

# letters that don't map to a single #define but change other build behavior;
# described here purely for the manifest.
EXTRA_LETTER_DESCRIPTIONS: dict[str, str] = {
    "n": "Extra buttons via shift register (nano button box)",
    "r": "SN74ALS166N shift register chip for extra buttons",
    "d": "No optical encoder (paired with w for magnetic encoder, or a pot for X-axis)",
}


def set_define(lines: list[str], name: str, enabled: bool) -> list[str]:
    pattern = re.compile(rf"^(\s*)(//\s*)?#\s*define\s+{re.escape(name)}(\b.*)$")
    found = False
    updated = []
    for line in lines:
        match = pattern.match(line)
        if match:
            indent, _comment, rest = match.group(1), match.group(2), match.group(3)
            updated.append(f"{indent}#define {name}{rest}\n" if enabled else f"{indent}//#define {name}{rest}\n")
            found = True
        else:
            updated.append(line)
    if not found:
        raise RuntimeError(f"Define not found in Config.h: {name}")
    return updated


def set_define_value(lines: list[str], name: str, value: str) -> list[str]:
    pattern = re.compile(rf"^(\s*)#\s*define\s+{re.escape(name)}\s+\S+(.*)$")
    found = False
    updated = []
    for line in lines:
        match = pattern.match(line)
        if match:
            indent, rest = match.group(1), match.group(2)
            updated.append(f"{indent}#define {name}              {value}{rest}\n")
            found = True
        else:
            updated.append(line)
    if not found:
        raise RuntimeError(f"Define not found in Config.h: {name}")
    return updated


def apply_options(base_config: list[str], letters: str, promicro: bool) -> list[str]:
    letter_set = set(letters)

    enabled_defines = {DEFINE_MAP[ch][0] for ch in letter_set if ch in DEFINE_MAP}

    use_shift_register = "n" in letter_set or "r" in letter_set
    use_sn74 = "r" in letter_set
    use_eeprom = "p" not in letter_set
    use_quadrature = "d" not in letter_set and "w" not in letter_set

    updated = base_config[:]
    for name, _desc in DEFINE_MAP.values():
        updated = set_define(updated, name, name in enabled_defines)

    updated = set_define(updated, "USE_SHIFT_REGISTER", use_shift_register)
    updated = set_define(updated, "USE_SN74ALS166N", use_sn74)
    updated = set_define(updated, "USE_EEPROM", use_eeprom)
    updated = set_define(updated, "USE_PROMICRO", promicro)
    updated = set_define(updated, "USE_QUADRATURE_ENCODER", use_quadrature)
    updated = set_define_value(updated, "FW_BUILD_ID", FW_BUILD_ID)

    return updated


def read_variant_list(path: Path) -> list[str]:
    if not path.exists():
        raise RuntimeError(f"Missing variant list: {path}")
    variants = []
    for raw in path.read_text(encoding="utf-8").splitlines():
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        variants.append("" if line == "-" else line)
    return variants


def describe_letters(letters: str) -> list[str]:
    descriptions = []
    for ch in letters:
        if ch in DEFINE_MAP:
            descriptions.append(DEFINE_MAP[ch][1])
        elif ch in EXTRA_LETTER_DESCRIPTIONS:
            descriptions.append(EXTRA_LETTER_DESCRIPTIONS[ch])
        else:
            descriptions.append(f"Unknown option '{ch}'")
    return descriptions


def compile_variant(fqbn: str, letters: str, promicro: bool, output_name: str, output_dir: Path) -> None:
    build_dir = ROOT / "build" / fqbn.replace(":", "_") / output_name.replace(".hex", "")
    build_dir.mkdir(parents=True, exist_ok=True)

    base_config = CONFIG_PATH.read_text(encoding="utf-8").splitlines(keepends=True)
    updated = apply_options(base_config, letters, promicro)
    CONFIG_PATH.write_text("".join(updated), encoding="utf-8")

    cmd = [
        "arduino-cli", "compile", "--fqbn", fqbn, str(SKETCH_DIR),
        "--build-path", str(build_dir), "--libraries", str(LIB_DIR),
    ]
    try:
        subprocess.check_call(cmd)
    finally:
        CONFIG_PATH.write_text("".join(base_config), encoding="utf-8")

    hex_candidates = sorted(build_dir.glob("*.hex"))
    if not hex_candidates:
        raise RuntimeError(f"No HEX produced for {output_name}")
    preferred = [p for p in hex_candidates if "with_bootloader" not in p.name]
    hex_path = preferred[0] if preferred else hex_candidates[0]

    output_dir.mkdir(parents=True, exist_ok=True)
    shutil.copy2(hex_path, output_dir / output_name)


def build_board(board: str, fqbn: str, promicro: bool, filename_fmt: str) -> tuple[list[dict], list[str]]:
    variants = read_variant_list(VARIANTS_DIR / f"{board}.txt")
    output_dir = DIST_DIR / board
    manifest_entries = []
    failures = []

    for letters in variants:
        output_name = filename_fmt.format(letters=letters)
        try:
            compile_variant(fqbn, letters, promicro, output_name, output_dir)
        except subprocess.CalledProcessError:
            failures.append(output_name)
            continue
        manifest_entries.append({
            "board": board,
            "file": output_name,
            "letters": letters,
            "features": describe_letters(letters),
        })

    return manifest_entries, failures


def main() -> None:
    if DIST_DIR.exists():
        shutil.rmtree(DIST_DIR)
    DIST_DIR.mkdir(parents=True)

    leo_entries, leo_failures = build_board(
        "leonardo", "arduino:avr:leonardo", promicro=False,
        filename_fmt="brWheel_my.ino.leonardo_" + FW_VERSION + "{letters}.hex",
    )
    pro_entries, pro_failures = build_board(
        "promicro", "arduino:avr:micro", promicro=True,
        filename_fmt="brWheel_my.ino.micro_" + FW_VERSION + "{letters}m.hex",
    )

    manifest = {
        "firmware_version": FW_VERSION,
        "build_id": FW_BUILD_ID,
        "variants": leo_entries + pro_entries,
    }
    (DIST_DIR / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")

    ZIP_PATH.parent.mkdir(parents=True, exist_ok=True)
    if ZIP_PATH.exists():
        ZIP_PATH.unlink()

    with zipfile.ZipFile(ZIP_PATH, "w", compression=zipfile.ZIP_DEFLATED) as zipf:
        for path in sorted(DIST_DIR.rglob("*")):
            if path.is_file():
                zipf.write(path, path.relative_to(DIST_DIR))

    failures = leo_failures + pro_failures
    if failures:
        raise RuntimeError("Build failed for: " + ", ".join(sorted(failures)))


if __name__ == "__main__":
    main()
