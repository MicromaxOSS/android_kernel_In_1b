#ifndef _TEST_INI_H_
#define _TEST_INI_H_
char test_ini[] = {"\
[Valid_File]\n\
OnlyMultipleTest=1\n\
[Interface]\n\
IC_Type=FT5446U\n\
Normalize_Type=0\n\
CascadingType=0\n\
Interface_Type=0\n\
Slave_Addr=0x70\n\
Slave_Addr0=\n\
Slave_Addr1=\n\
Freq_Index=2\n\
Phase_Pola=-1\n\
Max_Points=5\n\
iRotationDegree=0\n\
isReversed=0\n\
ixMaxPixel=720\n\
iyMaxPixel=1520\n\
[Config]\n\
Run_Mode=0\n\
Read_Bytes=256\n\
Write_Bytes=128\n\
Test_Way=0\n\
Handle_SN=0\n\
SN_Length=18\n\
SN_AutoTest=1\n\
SnAndEnter=0\n\
SN_And_IO_AutoTest=0\n\
Set_Focus_To_SN1=0\n\
TP_Connection_AND_SN_AutoTest=0\n\
SN_IgnoreSNLength=0\n\
SN_AutoSkipBy=0\n\
SKey_Index=0\n\
SKeyValue=13\n\
TP_AutoTest=0\n\
TP_AutoTest_Time=0\n\
TP_ReplaceTP=0\n\
TP_ReplaceTP_Time=0\n\
TP_Always_Replace=0\n\
TP_Always_Replace_Time=0\n\
Store_Result=0\n\
CommOption=0\n\
Auto_Switch=0\n\
Continue_Test_After_Fail=0\n\
Break_Test_After_Fail=0\n\
Break_Test_If_Failed_Before_Download=0\n\
CB_Test_Mode=1\n\
Tip_After_TestNG=0\n\
Show_Res=0\n\
Check_Mode=0\n\
Light_Up=0\n\
Non_Common_GND=0\n\
TP_Stop=0\n\
LCDDeepStandbyAndTpStop=0\n\
LCDDeepStandbyTimeOut=30\n\
RemoveLPWGLevelSignal=0\n\
ContinueDownLoad=0\n\
CheckFwCRCBeforeDownload=0\n\
bPoll=0\n\
bCheckBinBeforeDownLoad=1\n\
bRemoveReloadInitialCode=0\n\
Output_LevelSignal=0\n\
Output_NgSignal=0\n\
Input_LevelSignal=0\n\
Reverse_Time=0\n\
Switch_Protocol=0\n\
CLB_Other_Return=0\n\
Count_Result=0\n\
Count_Result_Type=0\n\
Full_Screen=0\n\
Impulsing=0\n\
NotifyTest=0\n\
Use_LockDown=0\n\
SetCheckFwFileName=0\n\
SetFwChecksum=0\n\
DelayTimeSet=0\n\
CheckFactoryID=0\n\
SaveState=0\n\
SetVDDTP_Check=0\n\
[Android_Terminal]\n\
I2C_Interface=0\n\
I2C_Index=0\n\
RW_Byte=1\n\
CustomConfPath=0\n\
AutoSave=1\n\
ResultPath=\n\
[OnLine_Setting]\n\
IPAddress=127.0.0.1\n\
TCPPort=5555\n\
[COM_Setting]\n\
COM_ID=0\n\
BaudRate=115200\n\
[TestItem]\n\
FW_VERSION_TEST=0\n\
FACTORY_ID_TEST=0\n\
IC_VERSION_TEST=0\n\
PROJECT_CODE_TEST=0\n\
LCM_ID_TEST=0\n\
PANEL_ID_TEST=0\n\
RAWDATA_TEST=1\n\
ADC_DETECT_TEST=0\n\
PANEL_DIFFER_TEST=0\n\
PANEL_DIFFER_UNIFORMITY_TEST=0\n\
SCAP_CB_TEST=1\n\
SCAP_RAWDATA_TEST=1\n\
CHANNEL_NUM_TEST=0\n\
INT_PIN_TEST=0\n\
RESET_PIN_TEST=0\n\
NOISE_TEST=0\n\
WEAK_SHORT_CIRCUIT_TEST=1\n\
UNIFORMITY_TEST=0\n\
CM_TEST=0\n\
RAWDATA_MARGIN_TEST=0\n\
TE_TEST=0\n\
SITO_RAWDATA_UNIFORMITY_TEST=0\n\
PATTERN_TEST=0\n\
GPIO_TEST=0\n\
LCD_NOISE_TEST=0\n\
FPC_OPEN_TEST=0\n\
SREF_OPEN_TEST=0\n\
VIRTUAL_BUTTON_TEST=0\n\
ONELINE_TEST=0\n\
MULTILINE_TEST=0\n\
GRIDLINE_TEST=0\n\
DIAGONAL_TEST=0\n\
FREEPAINT_TEST=0\n\
SPECIAL_BUTTON_TEST=0\n\
HOME_KEY_TEST=0\n\
FORCE_TOUCH_TEST=0\n\
LINEARITY_TEST=0\n\
CIRCLE_TEST=0\n\
SQUARE_TEST=0\n\
KEY_TEST=0\n\
PRESS_CHANNEL_TEST=0\n\
Graph_Set_K1=0\n\
[Basic_Threshold]\n\
FW_VER_VALUE=255\n\
Factory_ID_Number=255\n\
IC_Version=0\n\
Project_Code=\n\
Ori_ProjectCode=0\n\
LCM_ID=1\n\
PANEL_ID=255\n\
UniformityTest_Check_Rx=0\n\
UniformityTest_Check_Tx=0\n\
RawDataTest_Low_Min=7000\n\
RawDataTest_Low_Max=11000\n\
RawDataTest_High_Min=6000\n\
RawDataTest_High_Max=20000\n\
RawDataTest_NonCommonGND=0\n\
RawDataTest_LowFreq=1\n\
RawDataTest_HighFreq=1\n\
SCapCbTest_OFF_Min=0\n\
SCapCbTest_OFF_Max=491\n\
SCapCbTest_ON_Min=0\n\
SCapCbTest_ON_Max=490\n\
ScapCBTest_SetWaterproof_OFF=1\n\
ScapCBTest_SetWaterproof_ON=1\n\
SCapCBTest_LetTx_Disable=0\n\
ScapCBTest_SpecialMode_Off=0\n\
SCapCbTest_MaxNum_Off=5\n\
SCapCbTest_ContinuousNum_Off=3\n\
SCapRawDataTest_OFF_Min=3000\n\
SCapRawDataTest_OFF_Max=15000\n\
SCapRawDataTest_ON_Min=3000\n\
SCapRawDataTest_ON_Max=15000\n\
SCapRawDataTest_SetWaterproof_OFF=1\n\
SCapRawDataTest_SetWaterproof_ON=1\n\
SCapRawDataTest_LetTx_Disable=0\n\
WeakShortTest_CG=1200\n\
WeakShortTest_CC=1200\n\
WeakShortTest_CC_Rsen=57\n\
WeakShortTest_CapShortTest=0\n\
Distance_Diagonal=50\n\
Type_Diagonal=2\n\
MaxNG_Diagonal=0\n\
LimitTime_Diagonal=0\n\
LinearityCheck_Diagonal=1\n\
Continue_Diagonal=0\n\
LimitTime_HomeKey=10\n\
HomeKey_LeftChannel=0\n\
HomeKey_RightChannel=0\n\
HomeKey_TopChannel=0\n\
HomeKey_BottonChannel=0\n\
HomeKey_Hole=0\n\
SET_TOUCH_THRESHOLD_INCELL=0\n\
Key_Div_Number_Incell=1\n\
Preserved_key_threshold_Incell=4\n\
SET_TOUCH_THRESHOLD_INCELL2=0\n\
Key_Div_Number_Incell2=1\n\
Preserved_key_threshold_Incell2=4\n\
Preserved_key_threshold_Dynamic=1\n\
CIRCLE_TEST_MAX_NG=0\n\
CIRCLE_TEST_LIMITE_TIME=0\n\
CIRCLE_TEST_BOARDER=360\n\
CIRCLE_TEST_EDGE=20\n\
CIRCLE_TEST_TRACK=10\n\
CIRCLE_TEST_CENTER1=270\n\
CIRCLE_TEST_CENTER2=120\n\
CIRCLE_TEST_LINEARITY=35\n\
CIRCLE_TEST_SPLITS=10\n\
SET_TOUCH_THRESHOLD=0\n\
Key_Div_Number=1\n\
Preserved_key_threshold=800\n\
1_key_threshold=800\n\
2_key_threshold=800\n\
3_key_threshold=800\n\
4_key_threshold=800\n\
Key_Threshold=800\n\
KEY_TEST_MAX_NG=0\n\
KEY_TEST_LIMITE_TIME=0\n\
KEY_TEST_KEY_NUM=21\n\
PressChannelTest_Min=5000\n\
Press_Test_LimitedTime=0\n\
Graph_Set_K1_Step=3\n\
[INVALID_NODE]\n\
InvalidNode[7][1]=0\n\
InvalidNode[8][1]=0\n\
[SiuParam]\n\
Check_Siu_Version=0\n\
Siu_MainVersion=0\n\
Siu_SubVersion=0\n\
Update_Siu2_Version=0\n\
Siu2_MainVersion=0\n\
Siu2_SubVersion=0\n\
Check_Set_IICVol=0\n\
IIC_Vol_Type=0\n\
IIC_Vdd_Type=0\n\
Check_Iovcc=0\n\
Iovcc_Vol_Type=0\n\
Iovcc_Current_Type=0\n\
Iovcc_Min_Hole=0\n\
Iovcc_Max_Hole=50\n\
Check_Vdd=0\n\
Vdd_Vol_Type=0\n\
Check_Normal=0\n\
Check_Sleep=0\n\
Vdd_Normal_Min=0\n\
Vdd_Normal_Max=500\n\
Vdd_Sleep_Min=0\n\
Vdd_Sleep_Max=150\n\
Check_Standby=0\n\
Standby_Vol_Type=0\n\
Vdd_Standby_Min=0\n\
Vdd_Standby_Max=150\n\
"};

#endif