
STEP 1: Make sure you have installed the latest BLE software from www.ti.com/lprf. In my case it was "BLE-CC254x-1.2"
You will need IAR 8051 to open up the project. Check out the user guide that is installed with the TI BLE software

1) Copy "HelloBluetoothProfile" folder under C:\Texas Instruments\BLE-CC254x-1.2\Projects\ble\Profiles
2) Copy "HelloBluetoothPeripheral" folder under C:\Texas Instruments\BLE-CC254x-1.2\Projects\ble

After building the project, you need to download the hex file to the CC2540 mini-DK. You can use SMART RF Flash Programmer (available at www.ti.com/lprf) or use IAR to burn the image.





