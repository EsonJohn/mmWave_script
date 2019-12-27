--BSS and MSS firmware download
info = debug.getinfo(1,'S');
file_path = (info.source);
file_path = string.gsub(file_path, "@","");
file_path = string.gsub(file_path, "Configure.lua","");
fw_path   = file_path.."\\rf_eval_firmware"

--Export bit operation file
bitopfile = file_path.."\\".."bitoperations.lua"
dofile(bitopfile)

--Read part ID
--This register address used to find part number for ES2 and ES3 devices
res, efusedevice = ar1.ReadRegister(0xFFFFE214, 0, 31)
res, efuseES1device = ar1.ReadRegister(0xFFFFE210, 0, 31)
efuseES2ES3Device = bit_and(efusedevice, 0x03FC0000)
efuseES2ES3Device = bit_rshift(efuseES2ES3Device, 18)

--if part number is zero then those are ES1 devices 
if(efuseES2ES3Device == 0) then
	if (bit_and(efuseES1device, 3) == 0) then
		partId = 1243
	elseif (bit_and(efuseES1device, 3) == 1) then
		partId = 1443
	else
		partId = 1642
	end
elseif(efuseES2ES3Device == 0xE0 and (bit_and(efuseES1device, 3) == 2)) then
		partId = 6843
		ar1.frequencyBandSelection("60G")
--if part number is non-zero then those are ES12 and ES3 devices
else
   if(efuseES2ES3Device == 0x20 or efuseES2ES3Device == 0x21 or efuseES2ES3Device == 0x80) then
		partId = 1243
	elseif(efuseES2ES3Device == 0xA0 or efuseES2ES3Device == 0x40)then
		partId = 1443
	elseif(efuseES2ES3Device == 0x60 or efuseES2ES3Device == 0x61 or efuseES2ES3Device == 0x04 or efuseES2ES3Device == 0x62 or efuseES2ES3Device == 0x67) then
		partId = 1642
	elseif(efuseES2ES3Device == 0x66 or efuseES2ES3Device == 0x01 or efuseES2ES3Device == 0xC0 or efuseES2ES3Device == 0xC1) then
		partId = 1642
	elseif(efuseES2ES3Device == 0x70 or efuseES2ES3Device == 0x71 or efuseES2ES3Device == 0xD0 or efuseES2ES3Device == 0x05) then
		partId = 1843
	elseif(efuseES2ES3Device == 0xE0 or efuseES2ES3Device == 0xE1 or efuseES2ES3Device == 0xE2 or efuseES2ES3Device == 0xE3 or efuseES2ES3Device == 0xE4) then
		partId = 6843
		ar1.frequencyBandSelection("60G")
	else
		WriteToLog("Inavlid Device part number in ES2 and ES3 devices\n" ..partId)
    end
end 

--ES version
res, ESVersion = ar1.ReadRegister(0xFFFFE218, 0, 31)
ESVersion = bit_and(ESVersion, 15)

--ADC_Data file path
data_path     = file_path.."\\PostProc"
adc_data_path = data_path.."\\adc_data.bin"

-- Download Firmware
if(partId == 1642) then
    BSS_FW    = fw_path.."\\radarss\\xwr16xx_radarss.bin"
    MSS_FW    = fw_path.."\\masterss\\xwr16xx_masterss.bin"
elseif(partId == 1243) then
    BSS_FW    = fw_path.."\\radarss\\xwr12xx_xwr14xx_radarss.bin"
    MSS_FW    = fw_path.."\\masterss\\xwr12xx_xwr14xx_masterss.bin"
elseif(partId == 1443) then
    BSS_FW    = fw_path.."\\radarss\\xwr12xx_xwr14xx_radarss.bin"
    MSS_FW    = fw_path.."\\masterss\\xwr12xx_xwr14xx_masterss.bin"
elseif(partId == 1843) then
    BSS_FW    = fw_path.."\\radarss\\xwr18xx_radarss.bin"
    MSS_FW    = fw_path.."\\masterss\\xwr18xx_masterss.bin"
elseif(partId == 6843) then
    BSS_FW    = fw_path.."\\radarss\\xwr68xx_radarss.bin"
    MSS_FW    = fw_path.."\\masterss\\xwr68xx_masterss.bin"
else
    WriteToLog("Invalid Device partId FW\n" ..partId)
    WriteToLog("Invalid Device ESVersion\n" ..ESVersion)
end

-- Download BSS Firmware
if (ar1.DownloadBSSFw(BSS_FW) == 0) then
    WriteToLog("BSS FW Download Success\n", "green")
else
    WriteToLog("BSS FW Download failure\n", "red")
end

-- Download MSS Firmware
if (ar1.DownloadMSSFw(MSS_FW) == 0) then
    WriteToLog("MSS FW Download Success\n", "green")
else
    WriteToLog("MSS FW Download failure\n", "red")
end

-- SPI Connect
if (ar1.PowerOn(1, 1000, 0, 0) == 0) then
    WriteToLog("Power On Success\n", "green")
else
   WriteToLog("Power On failure\n", "red")
end

-- RF Power UP
if (ar1.RfEnable() == 0) then
    WriteToLog("RF Enable Success\n", "green")
else
    WriteToLog("RF Enable failure\n", "red")
end

if (ar1.ChanNAdcConfig(1, 1, 0, 1, 1, 1, 1, 2, 1, 0) == 0) then
    WriteToLog("ChanNAdcConfig Success\n", "green")
else
    WriteToLog("ChanNAdcConfig failure\n", "red")
end

if (partId == 1642) then
    if (ar1.LPModConfig(0, 1) == 0) then
        WriteToLog("LPModConfig Success\n", "green")
    else
        WriteToLog("LPModConfig failure\n", "red")
    end
else
    if (ar1.LPModConfig(0, 0) == 0) then
        WriteToLog("Regualar mode Cfg Success\n", "green")
    else
        WriteToLog("Regualar mode Cfg failure\n", "red")
    end
end

if (ar1.RfInit() == 0) then
    WriteToLog("RfInit Success\n", "green")
else
    WriteToLog("RfInit failure\n", "red")
end

RSTD.Sleep(1000)

if (ar1.DataPathConfig(1, 1, 0) == 0) then
    WriteToLog("DataPathConfig Success\n", "green")
else
    WriteToLog("DataPathConfig failure\n", "red")
end

if (ar1.LvdsClkConfig(1, 1) == 0) then
    WriteToLog("LvdsClkConfig Success\n", "green")
else
    WriteToLog("LvdsClkConfig failure\n", "red")
end

if((partId == 1642) or (partId == 1843) or (partId == 6843)) then
    if (ar1.LVDSLaneConfig(0, 1, 1, 0, 0, 1, 0, 0) == 0) then
        WriteToLog("LVDSLaneConfig Success\n", "green")
    else
        WriteToLog("LVDSLaneConfig failure\n", "red")
    end
elseif ((partId == 1243) or (partId == 1443)) then
    if (ar1.LVDSLaneConfig(0, 1, 1, 1, 1, 1, 0, 0) == 0) then
        WriteToLog("LVDSLaneConfig Success\n", "green")
    else
        WriteToLog("LVDSLaneConfig failure\n", "red")
    end
end

if (ar1.SetTestSource(4, 3, 0, 0, 0, 0, -327, 0, -327, 327, 327, 327, -2.5, 327, 327, 0, 0, 0, 0, -327, 0, -327, 
                      327, 327, 327, -95, 0, 0, 0.5, 0, 1, 0, 1.5, 0, 0, 0, 0, 0, 0, 0) == 0) then
    WriteToLog("Test Source Configuration Success\n", "green")
else
    WriteToLog("Test Source Configuration failure\n", "red")
end

if((partId == 1642) or (partId == 1843)) then
    if(ar1.ProfileConfig(0, 77, 100, 6, 60, 0, 0, 0, 0, 0, 0, 29.982, 0, 256, 5000, 0, 0, 30) == 0) then
        WriteToLog("ProfileConfig Success\n", "green")
    else
        WriteToLog("ProfileConfig failure\n", "red")
    end
elseif((partId == 1243) or (partId == 1443)) then
    if(ar1.ProfileConfig(0, 77, 100, 6, 60, 0, 0, 0, 0, 0, 0, 29.982, 0, 256, 10000, 0, 0, 30) == 0) then
        WriteToLog("ProfileConfig Success\n", "green")
    else
        WriteToLog("ProfileConfig failure\n", "red")
    end
elseif(partId == 6843) then
    if(ar1.ProfileConfig(0, 60.25, 100, 6, 60, 0, 0, 0, 0, 0, 0, 29.982, 0, 256, 10000, 0, 131072, 30) == 0) then
		WriteToLog("ProfileConfig Success\n", "green")
    else
        WriteToLog("ProfileConfig failure\n", "red")
    end
end

if (ar1.ChirpConfig(0, 0, 0, 0, 0, 0, 0, 1, 1, 0) == 0) then
    WriteToLog("ChirpConfig Success\n", "green")
else
    WriteToLog("ChirpConfig failure\n", "red")
end

if (ar1.EnableTestSource(1) == 0) then
    WriteToLog("Enabling Test Source Success\n", "green")
else
    WriteToLog("Enabling Test Source failure\n", "red")
end

if (ar1.FrameConfig(0, 0, 0, 128, 40, 0, 0, 1) == 0) then
    WriteToLog("FrameConfig Success\n", "green")
else
    WriteToLog("FrameConfig failure\n", "red")
end

-- select Device type
if (ar1.SelectCaptureDevice("DCA1000") == 0) then
    WriteToLog("SelectCaptureDevice Success\n", "green")
else
    WriteToLog("SelectCaptureDevice failure\n", "red")
end

--DATA CAPTURE CARD API
if (ar1.CaptureCardConfig_EthInit("192.168.33.30", "192.168.33.180", "12:34:56:78:90:12", 4096, 4098) == 0) then
    WriteToLog("CaptureCardConfig_EthInit Success\n", "green")
else
    WriteToLog("CaptureCardConfig_EthInit failure\n", "red")
end

--AWR12xx or xWR14xx-1, xWR16xx or xWR18xx or xWR68xx- 2 (second parameter indicates the device type)
if ((partId == 1642) or (partId == 1843) or (partId == 6843)) then
    if (ar1.CaptureCardConfig_Mode(1, 2, 1, 2, 3, 30) == 0) then
        WriteToLog("CaptureCardConfig_Mode Success\n", "green")
    else
        WriteToLog("CaptureCardConfig_Mode failure\n", "red")
    end
elseif ((partId == 1243) or (partId == 1443)) then
    if (ar1.CaptureCardConfig_Mode(1, 1, 1, 2, 3, 30) == 0) then
        WriteToLog("CaptureCardConfig_Mode Success\n", "green")
    else
        WriteToLog("CaptureCardConfig_Mode failure\n", "red")
    end
end

if (ar1.CaptureCardConfig_PacketDelay(25) == 0) then
    WriteToLog("CaptureCardConfig_PacketDelay Success\n", "green")
else
    WriteToLog("CaptureCardConfig_PacketDelay failure\n", "red")
end

