[Raspberry Pi](RaspberryPi.md)
- [Image Management](#image-management)
- [Setup](#setup)
  - [Raspberry Pi Imager](#raspberry-pi-imager)
  - [Pishrink](#pishrink)
- [Copy Card Image To Host (after Following Setup)](#copy-card-image-to-host-after-following-setup)
- [Copy Image on Host to Card (after Following Setup)](#copy-image-on-host-to-card-after-following-setup)

# Image Management

# Setup
Setup the following tools:
## Raspberry Pi Imager
1. Download Raspberry Pi Imager
Run:
```bash
sudo snap install rpi-imager
```

## Pishrink
```bash
sudo apt update && sudo apt install -y wget parted gzip pigz xz-utils udev e2fsprogs
wget https://raw.githubusercontent.com/Drewsif/PiShrink/master/pishrink.sh
chmod +x pishrink.sh
sudo mv pishrink.sh /usr/local/bin/pishrink
```

# Copy Card Image To Host (after Following [Setup](#setup))
Use this when you have an Image available, and want to commission a new SD Card.

1. Insert the SD Card into a computer and run the following:
```bash
sudo fdisk -l
```
2. Determine the appropriate device name.  For example:

| Partition        | Whole Device Name to Use | Umount Command               |
| ---------------- | ------------------------ | ---------------------------- |
| `/dev/mmcblk0p1` | `/dev/mmcblk0`           | `sudo umount /dev/mmcblk0p*` |
| `/dev/mmcblk0p2` | `/dev/mmcblk0`           | `sudo umount /dev/mmcblk0p*` |

1. Unmount the card with the above Umount command given in the previous table.
2. Copy the Image to the Host with:
```bash
sudo dd if=<Whole Device Name Above> of=<Image File Location>.img bs=4M status=progress
```

1. Shrink the image with Pishrink
```bash
sudo pishrink <Old Image>.img <Shrinked Image>.img
```
   

When the command completes, you may remove the card.

# Copy Image on Host to Card (after Following [Setup](#setup))
Use Raspberry Pi Imager to copy the Image to the Card.

When prompted to make OS customization settings, either click "NO", or edit the settings and change the Hostname.



