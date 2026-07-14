# rtw88-rtl8723bs

### Realtek `rtw88` driver with working RTL8723BS (SDIO) support.

This repository is a downstream fork of the `rtw88` driver, maintained to bring
stable support for the **RTL8723B(S)** chipset to modern Linux kernels and to
prepare that support for upstreaming into mainline `rtw88`.

🌟 **The Star Chipset: RTL8723BS (SDIO)**
While `rtw88` supports many chips, this repo is focused on the 8723B series. The
SDIO variant (8723BS) was long notorious for bus timeouts and scanning failures;
those are now fixed here through corrected power sequencing, a faithful
reimplementation of the vendor association/join sequence, WiFi/BT coexistence
antenna handling, and refined H2C command handling. **The driver associates and
passes traffic end-to-end and holds up over multi-hour soak tests.**

---

## 📡 Status

- **RTL8723BS (SDIO): working & stable.** Scan → auth → associate → sustained
  throughput, validated on hardware.
- **Upstreaming in progress.** A cleaned, bisectable patch series adding
  RTL8723B/RTL8723BS support is being prepared for submission to mainline
  `rtw88` (wireless-next). This fork remains the place to get it running on
  today's distro kernels until that lands.

---

## 🛠 Supported Chipsets

- **Tested & actively developed in this fork:**
  - **RTL8723BS** (SDIO) — the focus of this repository.
- **Inherited from upstream `rtw88` (built, but not validated or the focus
  here):** other rtw88 SDIO/PCIe/USB chips (8723CS/DS, 8821CS, 8822BS/CS,
  8723DE, 8821CE, 8822BE/CE, and the various USB parts). Use the mainline
  `rtw88` driver for those unless you specifically need this tree.

---

## 🚀 Installation Guide

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
When reporting a bug, please include:

- Output of `uname -r` (kernel version)
- Your hardware platform (e.g., PC, PinePhone, Samsung Note 20)

And attach a diagnostic bundle:

```bash
sudo ./diagnose.sh
```

Then send me the resulting `diagnostic-<hash>.tar`.

## ⚖️ License & Credits
Licensed under Dual BSD/GPL-2.0.

Maintainer: Luka Gejak (MocLG)

Original 8723B/BS Logic: Michael Straube

Upstream Core: Larry Finger and the rtw88 community.
