# Releasing firmware

*Last updated: 2026-11-06*

A published image must be built from a clean checkout of a tag, and the tag must spell the version the firmware reports. The 1.1.6 beta 4 bundle showed why: its files were named `beta4`, but the images reported `v1.1.6-beta3-16-g557bce7`. That is an untagged build 16 commits past beta 3. The `RELEASE` CMake option makes that mistake fail the build instead of shipping.

Version numbering (what `FW_VERSION_*` and the tag must say) is in `Features/firmware_versioning_spec.md`. This page is the procedure.

## 1. What `-DRELEASE=ON` enforces

`RELEASE` is OFF by default, and dev builds are unaffected. With it ON:

- **At configure time** the bench options (`DSPI_LOOPBACK`, `DSPI_CLOCK_DIAG`, `DSPI_LOOP_PROF`, `DSPI_DEBUG_PRINTF`) must be OFF, or configure fails.
- **On every build** `scripts/gen_build_info.cmake` refuses with a `RELEASE BUILD REFUSED` banner unless all of these hold:
  - `HEAD` is exactly a tag (`git describe --tags --exact-match`);
  - the tree has no uncommitted changes (the stamp has no `-dirty`);
  - the tag is `v<major>.<minor>.<patch>[-beta<N>][-<label>]` and matches `FW_VERSION_MAJOR/MINOR/PATCH/BETA` in `firmware/DSPi/config.h`. No `-beta<N>` means `FW_VERSION_BETA 0`. The optional `-<label>` (for example `-bofr1`) is not checked.

  On a refusal the build also deletes the previous `DSPi.*` outputs in the build directory, so a stale image cannot be picked up by mistake.
- **After a passing build** the UF2 is copied to `<build>/release/DSPi-<tag>-<variant>.uf2`, next to a `.sha256` file in `sha256sum` format. The tag in the file name is read back from the `build_info.h` that was compiled in, so the name always matches what the device reports through `REQ_GET_BUILD_INFO` (0x80) and `picotool info`. The variant is the platform plus `-midi` for `DSPI_USB_MIDI` plus `-vreg<mV>` for a non-default core voltage (K340: `rp2350-midi-vreg1200`). Override it with `-DRELEASE_VARIANT=<name>`.

## 2. Procedure

1. **Freeze.** All changes for the release are merged. `git status` is clean. Host tests pass (`firmware/tests/run.sh` in a container), and `tools/midi_ctrl` unit tests pass if MIDI is shipped.
2. **Set the version** in `firmware/DSPi/config.h`: `FW_VERSION_MAJOR/MINOR/PATCH` and `FW_VERSION_BETA` (0 for a final release). Commit: `Release v1.1.7-beta1`.
3. **Tag that commit** with an annotated tag that spells the same version:
   ```sh
   git tag -a v1.1.7-beta1 -m "DSPi v1.1.7 beta 1"
   ```
   A fork release that keeps an upstream version adds a label instead of a new number, for example `v1.1.6-bofr1` (`FW_VERSION_*` stays 1.1.6 beta 0).
4. **Build every published variant in fresh build directories**, each with `-DRELEASE=ON`. Keep them under the repo (git-excluded `build-*`) or `~/.cache/dspi-build`, never on tmpfs:
   ```sh
   source ~/prog/DSPi/dspi-env.sh
   for v in "rp2040 pico OFF 1150" "rp2350-arm-s pico2 OFF 1150" \
            "rp2040 pico ON 1150"  "rp2350-arm-s pico2 ON 1150" \
            "rp2350-arm-s pico2 ON 1200"; do
     set -- $v
     d=~/.cache/dspi-build/rel-$1-midi$3-vreg$4
     cmake -S firmware -B "$d" -G Ninja -DPICO_PLATFORM=$1 -DPICO_BOARD=$2 \
           -DDSPI_USB_MIDI=$3 -DDSPI_VREG_MV=$4 -DRELEASE=ON
     cmake --build "$d" -j || break
   done
   ```
   Check that the build printed `RELEASE build: v1.1.7-beta1 (config.h 1.1.7 beta 1)` and a `Release image:` line for each variant.
5. **Check RAM placement** for every variant: `python3 scripts/check_ram_placement.py <build>/DSPi/DSPi.elf ...` must report `VERDICT: PASS`.
6. **Verify each image before publishing.** `picotool info <file>.uf2` shows the bare tag (no `-N-g<hash>`, no `-dirty`). `sha256sum -c <file>.uf2.sha256` passes. On a bench unit, `REQ_GET_PLATFORM` (0x7F) reports the version and beta ordinal, and `REQ_GET_BUILD_INFO` (0x80) reports the tag.
7. **Publish** only the files from `<build>/release/`, with their `.sha256` files. Never rename an image to a different version, and never publish `<build>/DSPi/DSPi.uf2` directly. When an app bundles the images (the DSPi Android app, DSPi Console), copy the files under their release names and record the sha256 values in the app's release notes.
8. **Push** the release commit and the tag together: `git push origin <branch> v1.1.7-beta1`.

## 3. Bench and test builds

Leave `RELEASE` OFF. The stamp then says exactly what was built, for example `v1.1.6-18-gb2a13d9-dirty`. Bench builds keep the version they are based on (`Features/firmware_versioning_spec.md`, "Private test builds"). If a bench image has to leave the bench, label it with its full stamp, never with a release name.
