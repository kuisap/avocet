
# Setup for IMU
## I2C Configuration
### 1. raspi-conig
```
$ sudo raspi-config
```

Select `5 Interfacing Options` > `P5 I2C` > `Yes` > `OK`.

### 2.  Edit config.txt

```
$ sudo vim /boot/config.txt
```

Append `dtparam=i2c_baudrate=200000` in the last section.

### 3. Reboot
Reboot raspi.
