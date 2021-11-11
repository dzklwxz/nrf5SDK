@echo off
echo 复制 sd与app的hex文件到当前目录
copy .\pca10040\s132\arm5_no_packs\_build\nrf52832_xxaa.hex app.hex
copy ..\..\..\components\softdevice\s132\hex\s132_nrf52_7.2.0_softdevice.hex s132_nrf52_7.2.0_softdevice.hex
mergehex.exe -m s132_nrf52_7.2.0_softdevice.hex app.hex -o sd_app_complete_dev_1_52832.hex

echo 删除中间文件
del app.hex
del s132_nrf52_7.2.0_softdevice.hex

pause