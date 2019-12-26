****************************************************************************************************
* This application is to showcase basic radar feature of AWR1243 ES 3.0 mmWave device              *
*  and mmWaveLink APIs usage on External Host environment for the same.                            *
****************************************************************************************************

How to run-
    Connect AWR1243 ES 3.0 devpack and boosterpack to PC.
    Erase sFlash before running this application.
    Run mmwavelink_example.exe.
    

Execution flow of the application
    - App sets device in SOP4 mode
    - downloads MSS, BSS and memSwap images over SPI
    - after Patched MSS comes up, it'll start sending all commands, where it reads API parameters from mmwaveconfig.txt



To modify and re-run application, please use Visual Studio based project provided in the same directory.