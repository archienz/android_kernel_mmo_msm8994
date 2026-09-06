# Cursor agent — talkman kernel

Git: `archienz/android_kernel_mmo_msm8994` branch `lineage-18.1-talkman`.

Workspace: `/home/deck/android/talkman.code-workspace`. Rules: `.cursor/rules/talkman.mdc`.

## Build (Cloud Agent)

`.cursor/install.sh` (run by `.cursor/environment.json`) installs host deps and the
`aarch64-linux-android-4.9` GCC toolchain to `~/toolchains/`, matching CI
(`.github/workflows/build-on-commit.yml`). To cross-compile the arm64 kernel:

```
export CROSS_COMPILE="$HOME/toolchains/aarch64-linux-android-4.9/bin/aarch64-linux-android-"
make ARCH=arm64 mmo_defconfig
make ARCH=arm64 CROSS_COMPILE="$CROSS_COMPILE" -j"$(nproc)"
```

Output: `arch/arm64/boot/Image.gz-dtb`. Package a flashable zip with `./anykernel.sh`.
