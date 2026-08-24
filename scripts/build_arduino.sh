#!/usr/bin/env bash
set -euo pipefail

# Compile one of the three sketch folders for the Uno and report how much of the
# board it uses, without opening the Arduino IDE.
#
#   bash scripts/build_arduino.sh arduino_mini_base_controller
#   bash scripts/build_arduino.sh --all
#
# It drives the toolchain the IDE already installed under ~/.arduino15, so
# there is nothing extra to install if you have ever used the IDE on this
# machine. The .hex lands in build/<sketch>/<sketch>.hex, ready for
# scripts/flash_arduino.py.
#
# The size report is the point of running this before uploading. The base
# controller is well past 80% of the Uno's 32 KB, so a change that looks small
# can be the one that does not fit -- and the IDE only tells you after you have
# already picked a board and a port.

REPO_ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD_ROOT="${BUILD_ROOT:-$REPO_ROOT/build}"
ARDUINO15="${ARDUINO15:-$HOME/.arduino15/packages/arduino}"

# Uno numbers: 32256 bytes of application flash (the rest is the bootloader)
# and 2048 bytes of SRAM.
FLASH_LIMIT=32256
RAM_LIMIT=2048

find_one() {
  # Toolchain and core versions change with IDE updates, so discover them
  # rather than pinning a version that will rot.
  local pattern="$1"
  local found
  found="$(ls -d $pattern 2>/dev/null | sort -V | tail -1 || true)"
  [ -n "$found" ] || return 1
  printf '%s' "$found"
}

GCC_DIR="$(find_one "$ARDUINO15/tools/avr-gcc/*/bin")" || {
  echo "ERROR: no avr-gcc under $ARDUINO15/tools/avr-gcc/." >&2
  echo "Install the Arduino IDE's AVR board package (Tools > Board > Boards Manager" >&2
  echo "> 'Arduino AVR Boards'), or point ARDUINO15 at another install." >&2
  exit 1
}
AVR_CORE="$(find_one "$ARDUINO15/hardware/avr/*")" || {
  echo "ERROR: no AVR core under $ARDUINO15/hardware/avr/." >&2
  exit 1
}

CORE_DIR="$AVR_CORE/cores/arduino"
VARIANT_DIR="$AVR_CORE/variants/standard"
WIRE_DIR="$AVR_CORE/libraries/Wire/src"

build_sketch() {
  local sketch_dir="$1"
  local name
  name="$(basename "$sketch_dir")"
  local out="$BUILD_ROOT/$name"

  [ -f "$sketch_dir/$name.ino" ] || {
    echo "ERROR: $sketch_dir has no $name.ino" >&2
    return 1
  }

  rm -rf "$out"
  mkdir -p "$out/core"

  local defines="-DF_CPU=16000000L -DARDUINO=10819 -DARDUINO_AVR_UNO -DARDUINO_ARCH_AVR"
  local includes="-I$CORE_DIR -I$VARIANT_DIR -I$WIRE_DIR -I$WIRE_DIR/utility -I$sketch_dir"
  local common="-c -g -Os -w -ffunction-sections -fdata-sections -flto -mmcu=atmega328p"
  local cflags="$common -std=gnu11 -fno-fat-lto-objects"
  local cxxflags="$common -std=gnu++11 -fpermissive -fno-exceptions -fno-threadsafe-statics"

  # The core and Wire, archived once per sketch.
  local file
  for file in "$CORE_DIR"/*.c "$WIRE_DIR"/utility/*.c; do
    [ -e "$file" ] || continue
    "$GCC_DIR/avr-gcc" $cflags $defines $includes "$file" -o "$out/core/$(basename "$file").o"
  done
  for file in "$CORE_DIR"/*.cpp "$WIRE_DIR"/*.cpp; do
    [ -e "$file" ] || continue
    "$GCC_DIR/avr-g++" $cxxflags $defines $includes "$file" -o "$out/core/$(basename "$file").o"
  done
  "$GCC_DIR/avr-gcc-ar" rcs "$out/core.a" "$out"/core/*.o

  # The sketch. The .ino is C++ with an implicit Arduino.h, which is the only
  # preprocessing the IDE does that matters here: every function in these
  # sketches is declared in a header before use, so no prototype generation is
  # needed.
  local objects=()
  { echo '#include <Arduino.h>'; cat "$sketch_dir/$name.ino"; } > "$out/$name.ino.cpp"
  "$GCC_DIR/avr-g++" $cxxflags $defines $includes "$out/$name.ino.cpp" -o "$out/$name.ino.cpp.o"
  objects+=("$out/$name.ino.cpp.o")
  for file in "$sketch_dir"/*.cpp; do
    [ -e "$file" ] || continue
    "$GCC_DIR/avr-g++" $cxxflags $defines $includes "$file" -o "$out/$(basename "$file").o"
    objects+=("$out/$(basename "$file").o")
  done

  "$GCC_DIR/avr-gcc" -w -Os -g -flto -fuse-linker-plugin -Wl,--gc-sections -mmcu=atmega328p \
    -o "$out/$name.elf" "${objects[@]}" "$out/core.a" -lm
  "$GCC_DIR/avr-objcopy" -O ihex -R .eeprom "$out/$name.elf" "$out/$name.hex"

  # .text + .data is what occupies flash; .data + .bss is the static RAM floor,
  # before the stack and any heap.
  local sizes text data bss flash ram
  sizes="$("$GCC_DIR/avr-size" -A "$out/$name.elf")"
  text=$(awk '/^\.text/{print $2}' <<<"$sizes")
  data=$(awk '/^\.data/{print $2}' <<<"$sizes")
  bss=$(awk '/^\.bss/{print $2}' <<<"$sizes")
  flash=$((text + data))
  ram=$((data + bss))

  printf '%-38s flash %6d / %d (%2d%%)   static RAM %5d / %d (%2d%%)\n' \
    "$name" "$flash" "$FLASH_LIMIT" "$((flash * 100 / FLASH_LIMIT))" \
    "$ram" "$RAM_LIMIT" "$((ram * 100 / RAM_LIMIT))"
  echo "  -> $out/$name.hex"

  if [ "$flash" -gt "$FLASH_LIMIT" ]; then
    echo "  DOES NOT FIT: over the Uno's application flash." >&2
    return 1
  fi
  if [ "$ram" -gt $((RAM_LIMIT * 3 / 4)) ]; then
    echo "  WARNING: static RAM is over 75% of the Uno's 2 KB, leaving little" >&2
    echo "  for the stack. Stack overflow on an AVR is silent corruption." >&2
  fi
}

if [ "${1:-}" = "--all" ] || [ $# -eq 0 ]; then
  status=0
  for sketch in "$REPO_ROOT"/arduino_mini_*/; do
    build_sketch "${sketch%/}" || status=1
  done
  exit "$status"
fi

sketch_dir="$1"
[ -d "$sketch_dir" ] || sketch_dir="$REPO_ROOT/$1"
[ -d "$sketch_dir" ] || { echo "ERROR: no such sketch folder: $1" >&2; exit 1; }
build_sketch "$(cd "$sketch_dir" && pwd)"
