@echo off
echo ��¼�ļ�
nrfjprog.exe --snr 682177958 --family NRF52 --eraseall
nrfjprog.exe --snr 682177958 --family NRF52 --program sd_app_complete_dev_1_52832.hex 
nrfjprog.exe --snr 682177958 --family NRF52 --verify sd_app_complete_dev_1_52832.hex
nrfjprog.exe --snr 682177958 --reset

pause