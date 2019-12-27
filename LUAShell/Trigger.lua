--Start Record ADC data
ar1.CaptureCardConfig_StartRecord(adc_data_path, 1)
RSTD.Sleep(1000)

--Trigger frame
WriteToLog("Now start frame .....!!!! \n", "green")
ar1.StartFrame()
RSTD.Sleep(3000)
WriteToLog("Now stop frame .....!!!! \n", "green")
ar1.StopFrame()

RSTD.Sleep(3000)
WriteToLog("Now PostProc .....!!!! \n", "green")
--Post process the Capture RAW ADC data
ar1.StartMatlabPostProc(adc_data_path)
WriteToLog("Please wait for a few seconds for matlab post processing .....!!!! \n", "green")
-- RSTD.Sleep(10000)