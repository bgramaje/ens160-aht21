```bash
python -m esptool --chip esp32-c3 erase_flash
```
```cpp
        if (hasFlag(ens160.getDeviceStatus(), ENS16x::DeviceStatus::NewGeneralPurposeData))
        {
            debug("RS0: ");
            debug(ens160.getRs0());
            debug("\tRS1: ");
            debug(ens160.getRs1());
            debug("\tRS2: ");
            debug(ens160.getRs2());
            debug("\tRS3: ");
            debugln(ens160.getRs3());
        }
```