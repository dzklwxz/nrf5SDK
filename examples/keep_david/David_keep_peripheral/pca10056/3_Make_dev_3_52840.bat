@echo off
echo ���� sd��app��hex�ļ�����ǰĿ¼
copy .\s140\arm5_no_packs\_build\nrf52840_xxaa.hex app.hex
copy ..\..\..\..\components\softdevice\s140\hex\s140_nrf52_7.2.0_softdevice.hex s140_nrf52_7.2.0_softdevice.hex
mergehex.exe -m s140_nrf52_7.2.0_softdevice.hex app.hex -o sd_app_complete_dev_3_52840.hex

echo ɾ���м��ļ�
del app.hex
del s140_nrf52_7.2.0_softdevice.hex

pause