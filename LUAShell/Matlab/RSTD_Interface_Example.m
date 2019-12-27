clear;clc;close all;

% Initialize mmWaveStudio .NET connection
RSTD_DLL_Path='C:\ti\mmwave_studio_02_01_00_00\mmWaveStudio\Clients\RtttNetClientController\RtttNetClientAPI.dll';

ErrStatus = Init_RSTD_Connection(RSTD_DLL_Path);
if (ErrStatus ~= 30000)
    disp('Error inside Init_RSTD_Connection');
    return;
end

%Example Lua Command
%strFilename ='C:\\ti\\mmwave_studio_01_00_00_01\\mmWaveStudio\\Scripts\\Example_script_AllDevices.lua';
%Lua_String = sprintf('dofile("%s")',strFilename);
%ErrStatus =RtttNetClientAPI.RtttNetClient.SendCommand(Lua_String);