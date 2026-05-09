# rtw88-rtl8723bs

### Optimized Realtek rtw88 drivers with a primary focus on RTL8723B (PCIe) and RTL8723BS (SDIO) support.

This repository is a downstream fork of the `rtw88` driver, specifically maintained to bring stable support for the **RTL8723B(S)** chipset series to modern Linux kernels and prepare it for upstreaming.

🌟 **The Star Chipset: RTL8723B / RTL8723BS**
While `rtw88` supports many chips, this repo prioritizes the development and debugging of the 8723B series. The SDIO version (8723BS) is notorious for bus timeouts and scanning failures—this project aims to fix those issues through improved power sequencing and refined H2C command handling.



---

## 🛠 Supported Chipsets
- **SDIO**: **RTL8723BS** (Active Development), RTL8723CS, RTL8723DS, RTL8821CS, RTL8822BS, RTL8822CS
- **PCIe**: **RTL8723BE**, RTL8723DE, RTL8812AE, RTL8814AE, RTL8821AE, RTL8821CE, RTL8822BE, RTL8822CE
- **USB** : RTL8723DU, RTL8811AU/CU, RTL8812AU/BU/CU, RTL8814AU, RTL8821AU/CU, RTL8822BU/CU

---

## 🚀 Installation Guide

### 1. Prerequisites 📋
You must install your kernel headers and build tools before compiling.

* **Arch Linux**: `sudo pacman -S base-devel git linux-headers`
* **Ubuntu / Kali**: `sudo apt update && sudo apt install linux-headers-$(uname -r) build-essential git`
* **Fedora**: `sudo dnf install kernel-devel git`
* **Raspberry Pi OS**: `sudo apt install raspberrypi-kernel-headers build-essential git`

### 2. Installation Using DKMS (Recommended) 🔄
## Note
In case you are testing for development process insmoding .ko files is recommended way to load the driver.

---

DKMS automatically rebuilds the driver when you update your kernel.

```bash
git clone https://github.com/MocLG/rtw88-rtl8723bs.git
cd rtw88-rtl8723bs
sudo dkms install .
sudo make install_fw
sudo cp rtw88.conf /etc/modprobe.d/
```
### 3. Secure Boot 🔐
If you have Secure Boot enabled, you must enroll the signing key:
```bash
sudo mokutil --import /var/lib/dkms/mok.pub
```
### Reboot and select "Enroll MOK" from the blue screen menu.

## 🚨 Reporting Issues
When reporting a bug, please include:

Output of uname -r (Kernel version)

Your hardware platform (e.g., PC, PinePhone, Samsung Note 20)

And follow these steps:

### Run diagnose.sh
```bash
sudo diagnose.sh
```

### Send me appropriate diagnostic-(hash).tar

## ⚖️ License & Credits
Licensed under Dual BSD/GPL-2.0.

Maintainer: Luka Gejak (MocLG)

Original 8723B/BS Logic: Michael Straube

Upstream Core: Larry Finger and the rtw88 community.
