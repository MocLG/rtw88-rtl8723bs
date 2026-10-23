# rtw88-rtl8723bs

> ## ⚠️ OBSOLETE: this repository is no longer maintained
>
> RTL8723B/RTL8723BS support has been **merged into mainline Linux** (the
> `rtw88` driver built into the kernel) and is part of **Linux 7.4**. The old
> `drivers/staging/rtl8723bs` driver was removed in the same release.
>
> **If you run Linux 7.4 or newer, use the mainline driver and uninstall
> this one.** See the "Moving to the mainline driver" section below.
>
> This tree is **frozen**. It receives no fixes, updates or support, and
> issues and pull requests about the code in *this repository* (the
> external DKMS build) will not be handled. It stays online only as a
> historical archive and its code is outdated and not synced to mainline.
>
> **Issues are still monitored here, but only for the mainline driver**
> (Linux 7.4 or newer). See the "Reporting Issues" section below.

### Realtek `rtw88` driver with working RTL8723BS (SDIO) support.

This repository is a downstream fork of the `rtw88` driver, which was
maintained to bring support for the **RTL8723B(S)** chipset to Linux kernels
before it was upstreamed into mainline `rtw88`.

🌟 **The Star Chipset: RTL8723BS (SDIO)**
While `rtw88` supports many chips, this repo is focused on the 8723B series. The
SDIO variant (8723BS) was long notorious for bus timeouts and scanning failures;
those are now fixed here through corrected power sequencing, a faithful
reimplementation of the vendor association/join sequence, WiFi/BT coexistence
antenna handling, and refined H2C command handling. **The driver associates and
passes traffic end-to-end and holds up over multi-hour soak tests.**

---

## 📡 Status

* **Merged upstream.** RTL8723B/RTL8723BS support went into mainline `rtw88`
  through the wireless and networking trees and was pulled by Linus Torvalds
  in the networking merge for 7.4,
  [`734bef548a96`](https://git.kernel.org/torvalds/c/734bef548a96e8774d6eef5dfa2026c6edc4e80b).
  Main commits:
  * [`d726c70ce082`](https://git.kernel.org/torvalds/c/d726c70ce082) wifi: rtw88: 8723b: add the RTL8723B chip driver
  * [`269946cba92e`](https://git.kernel.org/torvalds/c/269946cba92e) wifi: rtw88: 8723bs: add the RTL8723BS SDIO bind
  * [`b2d251aa7f16`](https://git.kernel.org/torvalds/c/b2d251aa7f16) wifi: rtw88: 8723bs: enable building the RTL8723BS driver
  * [`c520f3af7df4`](https://git.kernel.org/torvalds/c/c520f3af7df4) staging: rtl8723bs: remove the obsolete driver
* **This repository is obsolete and unmaintained.** It was last updated before
  the upstream series was finalized, and the upstream version differs from
  it. Do not expect it to match the mainline driver's behavior.

---

## 🔁 Moving to the mainline driver

On Linux 7.4 or newer:

1. Remove everything installed from this repository, from a checkout of it:
   ```bash
   sudo make dkms-uninstall      # if you installed with DKMS
   sudo make uninstall           # if you installed with `make install`
   sudo rm -f /etc/modprobe.d/rtw88.conf
   ```
   The `rtw88.conf` file from this repository blacklists the mainline rtw88
   and `r8723bs` modules, so it must not be left in place. Both drivers can
   also claim the same SDIO device, so do not leave this one installed next
   to the mainline driver. Reboot afterwards.
2. Use a kernel with `CONFIG_RTW88_8723BS` enabled (distribution kernels
   normally build it as a module) and load it:
   `sudo modprobe rtw88_8723bs`.
3. The driver needs the RTL8723B firmware (`rtw88/rtw8723b_fw.bin`). If your
   distribution's firmware package does not include it yet, copy just that
   file from this repository:
   ```bash
   sudo install -Dm644 firmware/rtw8723b_fw.bin /lib/firmware/rtw88/rtw8723b_fw.bin
   ```
   Do not run `make install_fw` for this: it copies every firmware file in
   this repository over the ones in `/lib/firmware/rtw88`.

Problems with the mainline driver can be reported in this repository's issues.
See the "Reporting Issues" section below.

***

## 🛠 Supported Chipsets

* **Tested in this fork (no longer developed):**
  - **RTL8723BS** (SDIO) — the focus of this repository.
- **Inherited from upstream `rtw88` (built, but not validated or the focus
  here):** other rtw88 SDIO/PCIe/USB chips (8723CS/DS, 8821CS, 8822BS/CS,
  8723DE, 8821CE, 8822BE/CE, and the various USB parts). Use the mainline
  `rtw88` driver for those unless you specifically need this tree.
---

## 🚀 Installation Guide

> **Only for kernels older than 7.4**, and unmaintained. On 7.4 or newer use
> the mainline driver instead (see above).

### 1. Prerequisites 📋
You must install your kernel headers and build tools before compiling.

* **Arch Linux**: `sudo pacman -S base-devel git linux-headers`
* **Ubuntu / Linux Mint / Kali**: `sudo apt update && sudo apt install linux-headers-$(uname -r) build-essential git dkms`
* **Fedora**: `sudo dnf install kernel-devel git dkms`
* **Raspberry Pi OS**: `sudo apt install raspberrypi-kernel-headers build-essential git dkms`

### 2. Install with DKMS (Recommended) 🔄
DKMS rebuilds the driver automatically whenever you update your kernel. The
`dkms` make target stages the tree with a git-derived version and installs it:

```bash
git clone https://github.com/MocLG/rtw88-rtl8723bs.git
cd rtw88-rtl8723bs
sudo make dkms
sudo make install_fw
sudo cp rtw88.conf /etc/modprobe.d/
```

Then load it:

```bash
sudo modprobe rtw_8723bs
```

> **Developing / debugging?** For an edit–build–test loop, skip DKMS and load
> the freshly built modules directly instead:
> ```bash
> make -j$(nproc)
> sudo make install_fw          # first time only
> sudo insmod rtw_core.ko        # + rtw_8723x.ko, rtw_8723b.ko, rtw_sdio.ko, rtw_8723bs.ko
> # or, after `sudo make install && sudo depmod -a`:
> sudo modprobe rtw_8723bs
> ```

### 3. Secure Boot 🔐
If you have Secure Boot enabled, you must enroll the DKMS signing key:
```bash
sudo mokutil --import /var/lib/dkms/mok.pub
```
Reboot and select **"Enroll MOK"** from the blue-screen menu.

## 🚨 Reporting Issues
**Issues are monitored only for the mainline driver** (`rtw88_8723bs` in
Linux 7.4 or newer), **not for the code in this repository.** Issues that
involve the DKMS or external build from this repository will be closed
without a fix. If you are using it, please move to the mainline driver first
(see above).

When reporting a bug with the mainline driver, please include:

- Output of `uname -r` (kernel version)
- Your hardware platform (e.g., PC, PinePhone, Samsung Note 20)
- Issue(s) you have with logs and any other relevant information

## ⚖️ License & Credits
Licensed under Dual BSD/GPL-2.0.

Maintainer: Luka Gejak (MocLG)

Original 8723B/BS Logic: Michael Straube

Upstream Core: Larry Finger and the rtw88 community.
