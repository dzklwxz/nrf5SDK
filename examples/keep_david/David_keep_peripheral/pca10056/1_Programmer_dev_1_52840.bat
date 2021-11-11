@echo off
echo иуб╪нд╪Ч
nrfjprog.exe --snr 683600486 --family NRF52 --eraseall
nrfjprog.exe --snr 683600486 --family NRF52 --program sd_app_complete_dev_1_52840.hex 
nrfjprog.exe --snr 683600486 --family NRF52 --verify sd_app_complete_dev_1_52840.hex
nrfjprog.exe --snr 683600486 --reset

pause