@echo off
echo ��¼�ļ�
nrfjprog.exe --snr 683054148 --family NRF52 --eraseall
nrfjprog.exe --snr 683054148 --family NRF52 --program sd_app_complete_dev_1_52840.hex 
nrfjprog.exe --snr 683054148 --family NRF52 --verify sd_app_complete_dev_1_52840.hex
nrfjprog.exe --snr 683054148 --reset

pause