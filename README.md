# rtw88-rtl8723bs

Out-of-tree port of the `rtw88` driver to Mac80211 for the Realtek RTL8723BS (SDIO) chipset.

This repository extracts the modern `rtw88` framework so you can build the driver modules independently of a full kernel tree.

**Contents:** a small set of kernel modules implementing the `rtw88` core and RTL8723B/8723BS support with an SDIO frontend.

**Supported use:** compile against your running kernel and load the modules manually or via modprobe.

## Prerequisites

Install build tools and the kernel headers for the target kernel:

- Arch Linux: `sudo pacman -S linux-headers base-devel`
- Debian/Ubuntu: `sudo apt install build-essential linux-headers-$(uname -r)`
- Fedora: `sudo dnf install kernel-devel development-tools`

Ensure the headers match the kernel you plan to load the modules into.

## Build

Build against the currently running kernel with:

```bash
make
```

This produces the following modules (filenames may vary slightly): `rtw88_core.ko`, `rtw88_8723x.ko`, `rtw88_8723b.ko`, `rtw88_sdio.ko`, and `rtw88_8723bs.ko`.

## Loading modules

Load modules in this order to satisfy symbol dependencies:

```bash
sudo insmod rtw88_core.ko
sudo insmod rtw88_88xxa.ko   # helper module that provides shared PHY helpers
sudo insmod rtw88_8723x.ko
sudo insmod rtw88_8723b.ko
sudo insmod rtw88_sdio.ko
sudo insmod rtw88_8723bs.ko
```
 Load the modules in this order so helper symbols are available. When
 unloading, reverse the order (unload `rtw88_8723bs`/`rtw88_sdio`
 first, then `rtw88_8723b`, `rtw88_8723x`, `rtw88_88xxa`, and finally
 `rtw88_core`) to avoid unresolved-symbol or "module in use" errors.

Alternatively, use `modprobe` after installing the modules into the kernel module path.

## Validation / Tests

A simple validation script is provided: `test_rtw.sh`.

Run it with root privileges to verify module loading, symbol resolution, and basic dmesg checks:

```bash
sudo bash test_rtw.sh
```

What the script checks:

- Metadata: `modinfo` fields and firmware paths
- Chain loading: modules load in correct order without unresolved symbols
- dmesg scan: look for oopses, NULL dereferences, or locking warnings
- Unload: verify modules can be removed cleanly

## Firmware

This driver requires the Realtek firmware files to be present, specifically: rtw8723b_fw.bin

If missing, you can usually find them in your distribution's linux-firmware package.

## Notes

- Build the modules against the exact kernel version you will use them with.
- If you see unresolved symbols, ensure `rtw88_core.ko` is loaded first and that you used the right kernel headers.

## License

See the source files for license headers and details.