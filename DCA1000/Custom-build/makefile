CC_DLL      = g++
CC_EXE      = g++ -static-libgcc -static-libstdc++
CFLAGS_DLL  = -std=c++11 -w -shared 
CFLAGS_EXE 	= -std=c++11 -Wall 
INCFLAGS 	= -I Common/DCA1000_API/. -I Common/Json_Utils/dist/json/. -I Common/. -I Common/Validate_Utils/. -I Common/Osal_Utils/.
SOURCES_DLL = RF_API/*.cpp Common/Validate_Utils/validate_params.cpp
SOURCES_CTRL= CLI_Control/cli_control_main.cpp Common/Json_Utils/dist/jsoncpp.cpp Common/Validate_Utils/validate_params.cpp
SOURCES_REC	= CLI_Record/cli_record_main.cpp Common/Json_Utils/dist/jsoncpp.cpp Common/Validate_Utils/validate_params.cpp
DIRECTORY 	= Release

ifeq ($(OS),Windows_NT)
	RM = del /Q
	SOURCES_OSAL= Common/Osal_Utils/osal_win.cpp
	LDFLAGS_DLL = -lws2_32
	LDFLAGS_EXE = -L Release/. -lRF_API -lws2_32
	LDFLAGS_PTHREAD = 
	TARGET_DLL = Release\RF_API.dll
	TARGET_CTRL = Release\DCA1000EVM_CLI_Control.exe
	TARGET_REC = Release\DCA1000EVM_CLI_Record.exe
	TARGET_DIR = mkdir $@
else
	RM = rm -f
	SOURCES_OSAL= Common/Osal_Utils/osal_linux.cpp
	LDFLAGS_DLL = 
	LDFLAGS_EXE = -L Release/. -lRF_API
	LDFLAGS_PTHREAD = -pthread
	TARGET_DLL = Release/libRF_API.so
	TARGET_CTRL = Release/DCA1000EVM_CLI_Control
	TARGET_REC = Release/DCA1000EVM_CLI_Record
	TARGET_DIR = mkdir -p $@
endif

all: $(DIRECTORY) $(TARGET_DLL) $(TARGET_CTRL) $(TARGET_REC) 

$(DIRECTORY):
		@echo "Folder $(DIRECTORY) not exists"
		$(TARGET_DIR)

$(TARGET_DLL): $(SOURCES_DLL)
		$(CC_DLL) $(CFLAGS_DLL) -o $(TARGET_DLL) -fPIC $(LDFLAGS_PTHREAD) $(SOURCES_DLL) $(SOURCES_OSAL) $(LDFLAGS_DLL)
$(TARGET_CTRL): $(SOURCES_CTRL)
		$(CC_EXE) $(CFLAGS_EXE) -o $(TARGET_CTRL) $(SOURCES_CTRL) $(SOURCES_OSAL) $(INCFLAGS) $(LDFLAGS_EXE)
$(TARGET_REC): $(SOURCES_REC)
		$(CC_EXE) $(CFLAGS_EXE) -o $(TARGET_REC) $(SOURCES_REC) $(SOURCES_OSAL) $(LDFLAGS_PTHREAD) $(INCFLAGS) $(LDFLAGS_EXE)

clean:
ifeq ($(OS),Windows_NT)
	if exist $(TARGET_DLL) $(RM) $(TARGET_DLL)
	if exist $(TARGET_CTRL) $(RM) $(TARGET_CTRL)
	if exist $(TARGET_REC) $(RM) $(TARGET_REC)
else
		$(RM) $(TARGET_DLL) $(TARGET_CTRL) $(TARGET_REC)
endif
