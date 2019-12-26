****************************************************************************************************
* This application is to showcase monitoring and advanced frame feature of AWR1243 ES 3.0          *
*  mmWave device and mmWaveLink APIs usage on External Host environment for the same feature.      *
****************************************************************************************************

How to run-
    Connect AWR1243 ES 3.0 devpack and boosterpack to PC.
    Erase sFlash before running this application.
    Run mmwavelink_monitoring.exe.
    

Execution flow of the application
    - App sets device in SOP4 mode
    - downloads MSS, BSS and memSwap images over SPI
    - after Patched MSS comes up, it'll start sending all commands including monitoring APIs, where 
      it reads API parameters from mmwaveconfig.txt
    - after sensorStart it'll wait in while-sleep mode to get all monitoring Async Event from the Device.
    

To modify and re-run application, please user Visual Studio based project provided in the same directory.


Note - Advanced frame and monitoring feature is based on global TAGs (gLinkMonitoringTest, gLinkAdvanceFrameTest
[default TRUE]) where user can toggle these feature in this application.

