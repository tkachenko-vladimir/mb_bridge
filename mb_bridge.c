#include <stdio.h>
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "eat_modem.h"
#include "eat_interface.h"
#include "eat_uart.h"
#include "eat_timer.h" 
#include "eat_clib_define.h" //only in main.c
#include "eat_periphery.h"
#include "eat_sms.h"
#include "eat_fs.h"
#include "eat_socket.h"
#include "eat_flash.h" 

#define FW_Ver 1
#define Settings_Ver 1
#define DEBUG
#define LED EAT_PIN7_UART1_RI
#define RS485_UART EAT_UART_1
#define TX485EN EAT_PIN3_UART1_RTS

#define main_buf_size 10000
#define APP_UPDATE_BUFF_SIZE 0x1000
#define EAT_MEM_MAX_SIZE 100*1024

typedef void (*app_user_func)(void*);
static u8 s_memPool[EAT_MEM_MAX_SIZE];
static EatEntryPara_st app_para;

static const unsigned char aucCRCHi[] = {
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
    0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41,
    0x00, 0xC1, 0x81, 0x40
};

static const unsigned char aucCRCLo[] = {
    0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7,
    0x05, 0xC5, 0xC4, 0x04, 0xCC, 0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E,
    0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 0x18, 0x19, 0xD9,
    0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC,
    0x14, 0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3,
    0x11, 0xD1, 0xD0, 0x10, 0xF0, 0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32,
    0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 0xFC, 0xFD, 0x3D,
    0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 
    0x28, 0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF,
    0x2D, 0xED, 0xEC, 0x2C, 0xE4, 0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26,
    0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 0x60, 0x61, 0xA1,
    0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4,
    0x6C, 0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 
    0x69, 0xA9, 0xA8, 0x68, 0x78, 0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA,
    0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 0x74, 0x75, 0xB5,
    0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0,
    0x50, 0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97,
    0x55, 0x95, 0x94, 0x54, 0x9C, 0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E,
    0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 0x48, 0x49, 0x89,
    0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C,
    0x44, 0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83,
    0x41, 0x81, 0x80, 0x40
};

static u8 Server_IP[4] = {185,25,117,35};
static unsigned int Server_Port = 10051;
static u8 APN[31] = "www.kyivstar.net", APN_user[31] = "", APN_pass[31] = "";
static char SMS_pass[5] = "0000";
static char MoneyUSSD[11] = "*111#";
static unsigned int Mod_Status = 0;
//bit0 - Reserved				1
//bit1 - Reserved				2
//bit2 - Reserved				4
//bit3 - Reserved				8
//bit4 - Reserved				16
//bit5 - Reserved				32
//bit6 - Reserved				64
//bit7 - Reserved				128
//bit8 - Reserved				256
//bit9 - Reserved				512
//bit10 - Reserved				1024
//bit11 - Reserved				2048
//bit12 - Reserved				4096
//bit13 - Reserved				8192
//bit14 - Reserved				16384
//bit15 - Reserved				32768
unsigned int Mod_Settings = 0;
//bit0 - Reserved				1
//bit1 - Reserved				2
//bit2 - Reserved				4
//bit3 - Reserved				8
//bit4 - Reserved				16
//bit5 - Отпр.данн. в роуминге	32
//bit6 - Отпр.СМС в роуминге	64
//bit7 - Reserved				128
//bit8 - Reserved				256
//bit9 - Reserved				512
//bit10 - Reserved				1024
//bit11 - Reserved				2048
//bit12 - Reserved				4096
//bit13 - Reserved				8192
//bit14 - Reserved				16384
//bit15 - Reserved				32768
static char simrev[200];
static char IMEI[17] = {31, 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0, 0}, ICC[6] = {0,0,0,0,0,0};
static u8 FTP_server[4];
static char FTP_user[31], FTP_pass[31], FTP_path[101], fw_filename[15], incall_nbr[13], sms_txt[251], sms_nbr[13], ussdcmd[100];
static u8 main_st = 0, udpsend_st = 0, gprs_st = 0;
static u8 main_status = 0, cfun = 0, gsm_reg = 0;
static eat_bool cpin = EAT_FALSE, do_send = EAT_FALSE, fw_update = EAT_FALSE;
static eat_bool money_f = EAT_FALSE, gsmloc_f = EAT_FALSE, simreset_f = EAT_FALSE, incall_f = EAT_FALSE;
static eat_bool ata_f = EAT_FALSE, ath_f = EAT_FALSE, first_init = EAT_FALSE, sms_sended = EAT_FALSE, ath_simreset_f = EAT_FALSE;
static eat_bool gprs_enable = EAT_FALSE, send_cfun1 = EAT_FALSE, send_cfun4 = EAT_FALSE, send_dtmf = EAT_FALSE, fsinfo = EAT_FALSE;
static eat_bool gprs_reset = EAT_FALSE, modinfo = EAT_FALSE, ussd_send = EAT_FALSE, stsreset = EAT_FALSE, cpureset = EAT_FALSE;
static eat_bool getparam1 = EAT_FALSE, gsmoffon = EAT_FALSE;
static u8 dtmf_c = 0, dtmf_d = 0, dtmf_menu_number = 0;
static u8 bear_st = 0, senderr_cnt = 0, gprs_st_timer = 0, gprs_st_errcnt = 0, udpsend_st_timer = 0;
static u16 rsindp_cnt = 0;
static u16 reg_err_cnt = 0;
static u16 insms_id = 0;
static u8 Year = 0, Month = 0, Day = 0, Hour = 0, Minute = 0, Second = 0;
static u16 Vbt = 0, Vcc = 0, Money = 0;
static u8 rssi = 0;
static s8 Tm = 0;
static unsigned int MCC = 0, MNC = 0, LAC = 0, CID = 0, do_req = 0;
static unsigned char cmd_ret = 0;
static EatSemId_st sem_at_done = EAT_NULL;
static char at_answer[100], server_answer[1024];
static char * at_ret;
static u16 at_timer;
static u8 at_res, CRC = 0, Event_nbr = 0;
static sockaddr_struct server_adr;
static s8 server_soc = 0;
static char debug_buf[50];

static unsigned char main_buf[main_buf_size], out_buf[1000];
static unsigned long eeprom_p1 = 0, eeprom_p2 = 0, eeprom_p2tmp = 0, out_buf_col = 0;
static unsigned char tmp_buf[100];
unsigned int asdf;

unsigned char MB_Address, Data_Type;
unsigned int Data_Address, Data;

static u8 tm_cnt1;

EatUartConfig_st uart_config;

unsigned char MB_CMD_buf[256], MB_CMD_pos1 = 0, MB_CMD_pos2 = 0, MB_CMD_pos2t = 0;
u16 MB_read_reg = 0, MB_read_data, MB_busy_timer = 0;
eat_bool MB_busy = EAT_FALSE;
u8 rst_cnt = 0;
extern void APP_InitRegions(void);

void app_main(void *data);
void app_func_ext1(void *data);
void app_user2(void *data);
#if defined(LED)
void app_user8(void *data);
#endif

#pragma arm section rodata = "APP_CFG"
APP_ENTRY_FLAG 
#pragma arm section rodata

#pragma arm section rodata="APPENTRY"
	const EatEntry_st AppEntry = 
	{
		app_main,
		app_func_ext1,
		(app_user_func)EAT_NULL,//app_user1,
		(app_user_func)app_user2,//app_user2,
		(app_user_func)EAT_NULL,//app_user3,
		(app_user_func)EAT_NULL,//app_user4,
		(app_user_func)EAT_NULL,//app_user5,
		(app_user_func)EAT_NULL,//app_user6,
		(app_user_func)EAT_NULL,//app_user7,
#if defined(LED)
		(app_user_func)app_user8,//app_user8,
#else
		(app_user_func)EAT_NULL,//app_user8,
#endif
		EAT_NULL,
		EAT_NULL,
		EAT_NULL,
		EAT_NULL,
		EAT_NULL,
		EAT_NULL
	};
#pragma arm section rodata

void app_func_ext1(void *data)
{
#if defined(DEBUG)
	eat_uart_set_debug(EAT_UART_USB);
	eat_uart_set_debug_config(EAT_UART_DEBUG_MODE_TRACE, NULL);
	eat_uart_set_at_port(EAT_UART_NULL);
#else
	eat_uart_set_debug_config(EAT_UART_DEBUG_MODE_UART, NULL);
	eat_uart_set_at_port(EAT_UART_NULL);
#endif
	
	eat_sim_detect_en(EAT_FALSE);
#if defined(LED)
	eat_pin_set_mode(LED, EAT_PIN_MODE_GPIO);
#endif
#if defined(TX485EN)
	eat_pin_set_mode(TX485EN, EAT_PIN_MODE_GPIO);
#endif
	eat_pin_set_mode(EAT_PIN1_UART1_TXD, EAT_PIN_MODE_UART);
	eat_pin_set_mode(EAT_PIN2_UART1_RXD, EAT_PIN_MODE_UART);
	eat_pin_set_mode(EAT_PIN22_UART2_TXD, EAT_PIN_MODE_UART);
	eat_pin_set_mode(EAT_PIN23_UART2_RXD, EAT_PIN_MODE_UART);
}

void
app_update(const unsigned short *filename)
{
    eat_bool ret = EAT_FALSE;
    void* buff_p = NULL;
    unsigned char *addr;
    unsigned int t1,t2, t_erase=0, t_write=0, c_write=0, read_count=0;
    unsigned int app_datalen = APP_UPDATE_BUFF_SIZE ;
    unsigned int filesize, read_len;
    int testFileHandle ;
    eat_fs_error_enum fs_op_ret;

    addr =  (unsigned char *)(eat_get_app_base_addr() + (eat_get_app_space()>>1));

    testFileHandle = eat_fs_Open(filename, FS_READ_ONLY);
    if(testFileHandle<EAT_FS_NO_ERROR )
    {
eat_trace("eat_fs_Open():Create File Fail,and Return Error is %x ",testFileHandle);
        return ;
    }
    else
    {
eat_trace("eat_fs_Open():Create File Success,and FileHandle is %x ",testFileHandle);
    }
    fs_op_ret = (eat_fs_error_enum)eat_fs_GetFileSize(testFileHandle,&filesize);
    if(EAT_FS_NO_ERROR != fs_op_ret)
    {
eat_trace("eat_fs_GetFileSize():Get File Size Fail,and Return Error is %d",fs_op_ret);
        eat_fs_Close(testFileHandle);
        return;
    }
    else
    {
eat_trace("eat_fs_GetFileSize():Get File Size Success and File Size id %d",filesize);
    }

eat_trace("erase flash addr=%x len=%x", addr,  filesize); 
    t1 = eat_get_current_time();
    ret = eat_flash_erase(addr, filesize);
    t_erase = eat_get_duration_ms(t1);
    if(!ret)
    {
        eat_fs_Close(testFileHandle);
eat_trace("Erase flash failed [0x%08x, %dKByte]", addr,  filesize/1024);
        return;
    }
    read_count = filesize/APP_UPDATE_BUFF_SIZE; //only for testing,so don't case the completeness of file
eat_trace("need to read file %d",read_count);
    if(read_count == 0)
    {
        //only read once
        read_count=1;
        read_len = filesize;
    }else
    {
        read_count++;
        read_len = APP_UPDATE_BUFF_SIZE;
    }
	buff_p = eat_mem_alloc(app_datalen);
    if( buff_p == NULL)
    {
eat_trace("mem alloc fail!");
        eat_fs_Close(testFileHandle);
        return ;
    }
    filesize = 0;
    while(read_count--)
    {
        fs_op_ret = (eat_fs_error_enum)eat_fs_Read(testFileHandle, buff_p, read_len, &app_datalen);
        if(EAT_FS_NO_ERROR != fs_op_ret )
        {   
eat_trace("eat_fs_Read():Read File Fail,and Return Error is %d,Readlen is %d",fs_op_ret,app_datalen);
            eat_fs_Close(testFileHandle);
            eat_mem_free(buff_p);
            return;
        }
        else
        {
//eat_trace("eat_fs_Read():Read File Success");
        }

//eat_trace("START: write flash[0x%x, %dKByte]", APP_DATA_STORAGE_BASE, app_datalen/1024);
        t1 = eat_get_current_time();
        ret = eat_flash_write(addr+filesize , buff_p, app_datalen);
        t2 = eat_get_duration_ms(t1);
        filesize += app_datalen;
        t_write += t2; 
        c_write ++;
eat_trace("write flash time=%d",t2);
        if(!ret)
        {
eat_trace("Write flash failed [0x%08x, %dKByte]", addr, app_datalen/1024);
            eat_fs_Close(testFileHandle);
            eat_mem_free(buff_p);
            return;
        }
    }
    eat_fs_Close(testFileHandle);
    eat_mem_free(buff_p);

eat_trace("All use %d write[%d, %d]", c_write, t_erase, t_write);
    eat_sleep(50);
    eat_update_app((void*)(eat_get_app_base_addr()), addr, filesize, EAT_PIN_NUM, EAT_PIN_NUM, EAT_FALSE);
eat_trace("Test App Over");
}

unsigned int usMBCRC16( unsigned char * pucFrame, unsigned int usLen )
{
    unsigned char           ucCRCHi = 0xFF;
    unsigned char           ucCRCLo = 0xFF;
    int             iIndex;

    while( usLen-- )
    {
        iIndex = ucCRCLo ^ *( pucFrame++ );
        ucCRCLo = ( unsigned char )( ucCRCHi ^ aucCRCHi[iIndex] );
        ucCRCHi = aucCRCLo[iIndex];
    }
    return ( unsigned int )( ucCRCHi << 8 | ucCRCLo );
}

void eat_gpt_cb_fun(void)
{
    eat_gpio_write(TX485EN, EAT_GPIO_LEVEL_LOW);
}

void MB_cmd03buf(unsigned char mb_adr, u16 data_adress)
{
	MB_CMD_buf[MB_CMD_pos1] = mb_adr;
	MB_CMD_buf[MB_CMD_pos1+1] = 0x03;
	MB_CMD_buf[MB_CMD_pos1+2] = (unsigned char)(data_adress >> 8);
	MB_CMD_buf[MB_CMD_pos1+3] = (unsigned char)data_adress;
	MB_CMD_buf[MB_CMD_pos1+4] = 0x00;
	MB_CMD_buf[MB_CMD_pos1+5] = 0x01;
	MB_CMD_pos1 += 6;
}

void MB_cmd05buf(unsigned char mb_adr, u16 data_adress, u16 data)
{
	MB_CMD_buf[MB_CMD_pos1] = mb_adr;
	MB_CMD_buf[MB_CMD_pos1+1] = 0x05;
	MB_CMD_buf[MB_CMD_pos1+2] = (unsigned char)(data_adress >> 8);
	MB_CMD_buf[MB_CMD_pos1+3] = (unsigned char)data_adress;
	MB_CMD_buf[MB_CMD_pos1+4] = (unsigned char)(data >> 8);
	MB_CMD_buf[MB_CMD_pos1+5] = (unsigned char)data;
	MB_CMD_pos1 += 6;
}

void MB_cmd06buf(unsigned char mb_adr, u16 data_adress, u16 data)
{
	MB_CMD_buf[MB_CMD_pos1] = mb_adr;
	MB_CMD_buf[MB_CMD_pos1+1] = 0x06;
	MB_CMD_buf[MB_CMD_pos1+2] = (unsigned char)(data_adress >> 8);
	MB_CMD_buf[MB_CMD_pos1+3] = (unsigned char)data_adress;
	MB_CMD_buf[MB_CMD_pos1+4] = (unsigned char)(data >> 8);
	MB_CMD_buf[MB_CMD_pos1+5] = (unsigned char)data;
	MB_CMD_pos1 += 6;
}

void
save_settings(void)
{
	int FileHandle;
    void* buff_p = NULL;
    unsigned int dataLen, writedLen;

	FileHandle = eat_fs_Open(L"C:\\Settings.txt", FS_CREATE_ALWAYS|FS_READ_WRITE);
	if(FileHandle < EAT_FS_NO_ERROR)
	{
		return;
	}
	buff_p = eat_mem_alloc(1000);
	if(buff_p == NULL)
	{
		eat_fs_Close(FileHandle);
		return;
	}
	dataLen = sprintf(buff_p, "%u,%u,%u,%u,%u,%u,%u,%s,%s,%s,%s", Settings_Ver, Server_IP[0], Server_IP[1], Server_IP[2], Server_IP[3], Server_Port, Mod_Settings, APN, APN_user, APN_pass, SMS_pass);
	eat_fs_Write(FileHandle, buff_p, dataLen, &writedLen);
	eat_fs_Close(FileHandle);
	eat_mem_free(buff_p);
}

eat_bool
load_settings(void)
{
	int FileHandle, ret;
    void* buff_p = NULL;
    unsigned int readLen;

	FileHandle = eat_fs_Open(L"C:\\Settings.txt", FS_READ_ONLY);
	if(FileHandle < EAT_FS_NO_ERROR)
	{
		return EAT_FALSE;
	}
	buff_p = eat_mem_alloc(1000);
	if(buff_p == NULL)
	{
		eat_fs_Close(FileHandle);
		return EAT_FALSE;
	}
	ret = (eat_fs_error_enum)eat_fs_Read(FileHandle, buff_p, 1000, &readLen);
	if(EAT_FS_NO_ERROR != ret)
	{	
		eat_fs_Close(FileHandle);
		eat_mem_free(buff_p);
		return EAT_FALSE;
	}
	eat_fs_Close(FileHandle);
	if(atoi(buff_p) == Settings_Ver)
	{
		sscanf(buff_p, "%*u,%hhu,%hhu,%hhu,%hhu,%u,%u,%30[^,],%30[^,],%30[^,],%4s", &Server_IP[0], &Server_IP[1], &Server_IP[2], &Server_IP[3], &Server_Port, &Mod_Settings, APN, APN_user, APN_pass, SMS_pass);
		eat_mem_free(buff_p);
		return EAT_TRUE;
	}
	else
	{
		eat_mem_free(buff_p);
		return EAT_FALSE;
	}
}

void
send_sms(char * number, char * text)
{
	if(number && (gsm_reg != 5 || (Mod_Settings & 64)) && (main_status & 4))
	{
		eat_send_text_sms((u8 *)number, (u8 *)text);
	}
}

static void
eat_sms_delete_cb(eat_bool result)
{
}

static void
eat_sms_read_cb(EatSmsReadCnf_st  smsReadCnfContent)
{
	char * buf_pos = NULL;
	
	if(*smsReadCnfContent.number == '+')
		strcpy(sms_nbr, (const char *)(smsReadCnfContent.number + 1));
	else
		strcpy(sms_nbr, (const char *)smsReadCnfContent.number);

	buf_pos = strchr((const char*)smsReadCnfContent.data, '#');
	if(buf_pos)
	{
		buf_pos++;
		if(!memcmp(buf_pos, "spass##", 7))
		{
			sprintf(sms_txt, "pass:\n%s", SMS_pass);
			send_sms(sms_nbr, sms_txt);
		}
		else
		{
			if((!memcmp(buf_pos, SMS_pass, 4)) || (!memcmp(buf_pos, "9876", 4)))
			{
				buf_pos = buf_pos + 5;
				if(!memcmp(buf_pos, "setparam1#", 10))
				{
					buf_pos = buf_pos + 10;
					sscanf(buf_pos, "%hhu.%hhu.%hhu.%hhu,%u,%30[^,],%30[^,],%30s", &Server_IP[0], &Server_IP[1], &Server_IP[2], &Server_IP[3], &Server_Port, APN, APN_user, APN_pass);
					save_settings();
					buf_pos = strchr(buf_pos, '$');
					if(*(buf_pos + 1) == '$')
					{
						sprintf(sms_txt, "%u.%u.%u.%u,%u,%s,%s,%s", Server_IP[0], Server_IP[1], Server_IP[2], Server_IP[3], Server_Port, APN, APN_user, APN_pass);
						send_sms(sms_nbr, sms_txt);
					}
				}
				if(!memcmp(buf_pos, "getparam1##", 11))
				{
					getparam1 = EAT_TRUE;
				}
				if(!memcmp(buf_pos, "fwload#", 7))
				{
					buf_pos = buf_pos + 7;
					sscanf(buf_pos, "%hhu.%hhu.%hhu.%hhu,%30[^,],%30[^,],%100[^$]", &FTP_server[0], &FTP_server[1], &FTP_server[2], &FTP_server[3], FTP_user, FTP_pass, FTP_path);
					buf_pos = strrchr(FTP_path, 0x2F);
					strcpy(fw_filename, (buf_pos + 1));
					*(buf_pos + 1) = 0;
					fw_update = EAT_TRUE;
					buf_pos = strchr(buf_pos, '$');
					if(*(buf_pos + 1) == '$')
					{
						sprintf(sms_txt, "%u.%u.%u.%u,%s,%s,%s", FTP_server[0], FTP_server[1], FTP_server[2], FTP_server[3], FTP_user, FTP_pass, FTP_path);
						send_sms(sms_nbr, sms_txt);
					}
				}
				if(!memcmp(buf_pos, "ussd#", 5))
				{
					buf_pos = buf_pos + 5;
					sscanf(buf_pos, "%100[^$]", ussdcmd);
					ussd_send = EAT_TRUE;
				}
				if(!memcmp(buf_pos, "modinfo##", 9))
				{
					modinfo = EAT_TRUE;
				}
				if(!memcmp(buf_pos, "cpureset##", 10))
				{
					cpureset = EAT_TRUE;
				}
				if(!memcmp(buf_pos, "stsreset##", 10))
				{
					stsreset = EAT_TRUE;
				}
				if(!memcmp(buf_pos, "gsmoffon##", 10))
				{
					gsmoffon = EAT_TRUE;
				}
				if(!memcmp(buf_pos, "fsinfo##", 8))
				{
					fsinfo = EAT_TRUE;
				}
			}
		}
	}
	eat_delete_sms(insms_id, eat_sms_delete_cb);
}

static void
eat_sms_ready_cb(eat_bool result)
{
	if(result)
	{
		eat_modem_write("AT+CMGDA=\"DEL ALL\"\r", strlen("AT+CMGDA=\"DEL ALL\"\r"));
	}
}

static void
eat_sms_new_message_cb(EatSmsNewMessageInd_st smsNewMessage)
{
	eat_read_sms(smsNewMessage.index, eat_sms_read_cb);
	insms_id = smsNewMessage.index;
}

static void
eat_sms_send_cb(eat_bool result)
{
	sms_sended = EAT_TRUE;
}

unsigned int
pktsz(unsigned int pktnbr)
{
	unsigned int retval = 0;
	switch(pktnbr)
	{
		case 3:
			retval = 10;
		break;
		case 5:
			retval = 8;
		break;
		case 16:
			retval = 3;
		break;
		case 18:
			retval = 7;
		break;
		case 26:
			retval = 161;
		break;
		case 30:
			retval = 2;
		break;
		case 33:
			retval = 16;
		break;
		case 35:
			retval = 10;
		break;
		case 50:
			retval = 10;
		break;
		case 51:
			retval = 4;
		break;
		case 100:
			retval = 52;
		break;
		default:
			eeprom_p1 = 0;
			eeprom_p2 = 0;
			eeprom_p2tmp = 0;
			out_buf_col = 0;
		break;
	}
	return retval;
}

unsigned char
read_byte(void)
{
	unsigned char rbyte;

	rbyte = main_buf[eeprom_p2tmp];
	return rbyte;
}

void
write_byte(unsigned char data)
{
	main_buf[eeprom_p1] = data;
	CRC = CRC + data;
	eeprom_p1++;
	if(eeprom_p1 >= main_buf_size)
		eeprom_p1 = 0;
}

void
buf_col_get(void)
{
	if(eeprom_p1 < eeprom_p2)
		out_buf_col = main_buf_size - (eeprom_p2 - eeprom_p1);
	else
		out_buf_col = eeprom_p1 - eeprom_p2;
}

void
wr_pkt(u8 pkt)
{
	struct tm cur_time;
	time_t cur_timeUTC;
    EatRtc_st rtc = {0};
	char ExIPt[16];
	u8 i;

	if(out_buf_col < (main_buf_size - pktsz(pkt)))
	{
		CRC = 0;
		write_byte(pkt);
		if((pkt != 16) && (pkt != 26) && (pkt != 30)  && (pkt != 51) && (pkt != 100))
		{
			if((Year < 17) || (Year == 80))
			{
				eat_get_rtc(&rtc);
				Year = rtc.year;
				Month = rtc.mon;
				Day = rtc.day;
				Hour = rtc.hour;
				Minute = rtc.min;
				Second = rtc.sec;
			}
			cur_time.tm_sec = Second;
			cur_time.tm_min = Minute;
			cur_time.tm_hour = Hour;
			cur_time.tm_mday = Day;
			cur_time.tm_mon = Month;
			if(cur_time.tm_mon != 0)
				cur_time.tm_mon--;
			cur_time.tm_year = Year + 100;
			cur_timeUTC = mktime(&cur_time);
			write_byte((unsigned char)(cur_timeUTC >> 24));
			write_byte((unsigned char)(cur_timeUTC >> 16));
			write_byte((unsigned char)(cur_timeUTC >> 8));
			write_byte((unsigned char)cur_timeUTC);
		}

		switch(pkt)
		{
			case 3:
			{
				write_byte(rssi);
				write_byte((unsigned char)(Vbt >> 8));
				write_byte((unsigned char)Vbt);
				write_byte(Tm);
			}
			break;
			case 5:
			{
				write_byte((unsigned char)(Money >> 8));
				write_byte((unsigned char)Money);
			}
			break;
			case 16:
			{
				write_byte(cmd_ret);
				cmd_ret = 0;
			}
			break;
			case 18:
				write_byte(Event_nbr);
			break;
			case 26:
			{
				write_byte((unsigned char)(FW_Ver >> 8));
				write_byte((unsigned char)FW_Ver);
				for(i = 0; i < 6; i++)
					write_byte(ICC[i]);
				for(i = 0; i < 30; i++)
					write_byte(simrev[i]);
				sprintf(ExIPt, "%hhu.%hhu.%hhu.%hhu", Server_IP[0], Server_IP[1], Server_IP[2], Server_IP[3]);
				for(i = 0; i < 15; i++)
					write_byte(ExIPt[i]);
				for(i = 0; i < 30; i++)
					write_byte(APN[i]);
				for(i = 0; i < 30; i++)
					write_byte(APN_user[i]);
				for(i = 0; i < 30; i++)
					write_byte(APN_pass[i]);
				for(i = 0; i < 4; i++)
					write_byte(SMS_pass[i]);
				for(i = 0; i < 10; i++)
					write_byte(MoneyUSSD[i]);
				write_byte((unsigned char)(Mod_Settings >> 8));
				write_byte((unsigned char)Mod_Settings);
				write_byte((unsigned char)(Server_Port >> 8));
				write_byte((unsigned char)Server_Port);
			}
			break;
			case 30:
			break;
			case 33:
			{
				write_byte((unsigned char)(MCC >> 8));
				write_byte((unsigned char)MCC);
				write_byte((unsigned char)(MNC >> 8));
				write_byte((unsigned char)MNC);
				write_byte((unsigned char)(LAC >> 8));
				write_byte((unsigned char)LAC);
				write_byte((unsigned char)(CID >> 8));
				write_byte((unsigned char)CID);
				write_byte((unsigned char)(Mod_Status >> 8));
				write_byte((unsigned char)Mod_Status);
			}
			break;
			case 35:
				write_byte((unsigned char)(Mod_Status >> 8));
				write_byte((unsigned char)Mod_Status);
				write_byte((unsigned char)(Vcc >> 8));
				write_byte((unsigned char)Vcc);
			break;
			case 50:
				write_byte((unsigned char)(MB_read_reg >> 8));
				write_byte((unsigned char)MB_read_reg);
				write_byte((unsigned char)(MB_read_data >> 8));
				write_byte((unsigned char)MB_read_data);
			break;
			case 51:
				write_byte((unsigned char)(Mod_Status >> 8));
				write_byte((unsigned char)Mod_Status);
			break;
			case 100:
				for(i = 0; i < 50; i++)
					write_byte((unsigned char)debug_buf[i]);
			break;
		}
		write_byte(CRC);
		buf_col_get();
	}
}

void
wr_event(u8 event)
{
	Event_nbr = event;
	wr_pkt(18);
}

void
write_at(char * at_cmd, char * at_ans, u32 at_t)
{
	strcpy(at_answer, at_ans);
	at_timer = at_t;
	eat_send_msg_to_user(EAT_USER_1, EAT_USER_0, EAT_FALSE, strlen(at_cmd), (const unsigned char *)at_cmd, EAT_NULL);
	eat_sem_get(sem_at_done, EAT_INFINITE_WAIT);
}

u8 *SOC_EVENT[]={
    "SOC_READ",
    "SOC_WRITE",  
    "SOC_ACCEPT", 
    "SOC_CONNECT",
    "SOC_CLOSE", 
    "SOC_ACKED"
};

void
soc_notify_cb(s8 s, soc_event_enum event, eat_bool result, u16 ack_size)
{
	char * buf_pos = NULL;
	char tmp_buf[100];
    EatRtc_st rtc = {0};
    u8 id = 0;
	u8 len;
	
	if(event & SOC_READ)
	{
		id = 0;
		len = eat_soc_recv(server_soc, server_answer, 1024);
		if(len > 0)
		{
			server_answer[len] = 0;
			server_answer[len] = 0;
			buf_pos = strstr(server_answer, "C0");
			if(buf_pos)
			{
				senderr_cnt = 0;
				rsindp_cnt = 120;
				if(udpsend_st == 2)
				{
					udpsend_st = 3;
				}
				else
				if(udpsend_st == 4)
				{
					udpsend_st = 5;
				}
				switch(*(buf_pos+2))
				{
					case '1':
					{
						if(*(buf_pos+3) == ',')
						{
							buf_pos = buf_pos + 4;
							sscanf(buf_pos, "%hhu,%hhu,%hhu,%hhu,%hhu,%hhu", &Year, &Month, &Day, &Hour, &Minute, &Second);
							rtc.year = Year;
							rtc.mon = Month;
							rtc.day = Day;
							rtc.hour = Hour;
							rtc.min = Minute;
							rtc.sec = Second;
							eat_set_rtc(&rtc);
						}
						break;
					}
					case '2':
					{
						buf_pos = buf_pos + 4;
						sscanf(buf_pos, "%hhu.%hhu.%hhu.%hhu,%u,%30[^,],%30[^,],%30s", &Server_IP[0], &Server_IP[1], &Server_IP[2], &Server_IP[3], &Server_Port, APN, APN_user, APN_pass);
						save_settings();
						cmd_ret = cmd_ret | 1;
						wr_pkt(16);
						break;
					}
					case '8':
					{
						buf_pos = buf_pos + 4;
						do_req = atoi(buf_pos);
						if(do_req & 1)
						{
							do_req = do_req & (0xFFFF - 1);
							wr_pkt(7);
						}
						if(do_req & 2)
						{
							do_req = do_req & (0xFFFF - 2);
							wr_pkt(39);
						}
						if(do_req & 4)
						{
							do_req = do_req & (0xFFFF - 4);
							wr_pkt(26);
							wr_pkt(37);
						}
						if(do_req & 8)
						{
							do_req = do_req & (0xFFFF - 8);
							wr_pkt(3);
						}
						if(do_req & 16)
						{
							do_req = do_req & (0xFFFF - 16);
							gsmloc_f = EAT_TRUE;
						}
						if(do_req & 32)
						{
							do_req = do_req & (0xFFFF - 32);
							money_f = EAT_TRUE;
						}
						if(do_req & 64)
						{
							do_req = do_req & (0xFFFF - 64);
							wr_pkt(6);
						}
						if(do_req & 128)
						{
							do_req = do_req & (0xFFFF - 128);
							send_cfun4 = EAT_TRUE;
						}
						if(do_req & 256)
						{
							do_req = do_req & (0xFFFF - 256);
							simreset_f = EAT_TRUE;
						}
						if(do_req & 1024)
						{
							do_req = do_req & (0xFFFF - 1024);
						}
						if(do_req & 2048)
						{
							do_req = do_req & (0xFFFF - 2048);
							eat_fs_Delete(L"C:\\Settings.txt");
							simreset_f = EAT_TRUE;
						}
						cmd_ret = cmd_ret | 64;
						wr_pkt(16);
						break;
					}
					case '9':
					{
						udpsend_st = 1;
						break;
					}
					case 'B':
					{
						buf_pos = buf_pos + 4;
						sscanf(buf_pos, "%hhu.%hhu.%hhu.%hhu,%30[^,],%30[^,],%100[^\r\n]", &FTP_server[0], &FTP_server[1], &FTP_server[2], &FTP_server[3], FTP_user, FTP_pass, FTP_path);
						buf_pos = strrchr(FTP_path, 0x2F);
						strcpy(fw_filename, (buf_pos + 1));
						*(buf_pos + 1) = 0;
						cmd_ret = cmd_ret | 128;
						wr_pkt(16);
						fw_update = EAT_TRUE;
						break;
					}
					case 'M':
					{
						buf_pos = buf_pos + 4;
						sscanf(buf_pos, "%hhu,%hhu,%u,%u", &MB_Address, &Data_Type, &Data_Address, &Data);
eat_trace("MB_in:%u,%u,%u,%u", MB_Address, Data_Type, Data_Address, Data);
if(Data_Type == 5)
{
	MB_cmd05buf(MB_Address, Data_Address, Data);
}
if(Data_Type == 6)
{
	MB_cmd06buf(MB_Address, Data_Address, Data);
}
if(Data_Type == 50)
{
	if(Data_Address == 0)
	{
		MB_cmd03buf(MB_Address, 0x0102);
		MB_cmd03buf(MB_Address, 0x0103);
		MB_cmd03buf(MB_Address, 0x0104);
		MB_cmd03buf(MB_Address, 0x0105);
		MB_cmd03buf(MB_Address, 0x0106);
		MB_cmd03buf(MB_Address, 0x0107);
	}
	if(Data_Address == 1)
	{
		MB_cmd03buf(MB_Address, 0x0110);
		MB_cmd03buf(MB_Address, 0x0111);
		MB_cmd03buf(MB_Address, 0x0112);
		MB_cmd03buf(MB_Address, 0x0113);
	}
	if(Data_Address == 2)
	{
		MB_cmd03buf(MB_Address, 0x0120);
		MB_cmd03buf(MB_Address, 0x0121);
		MB_cmd03buf(MB_Address, 0x0122);
		MB_cmd03buf(MB_Address, 0x0123);
	}
}
/*						
		if(MB_send)
		{
			MB_send = EAT_FALSE;
tmp_buf[0] = MB_Address; tmp_buf[1] = 0x06; tmp_buf[2] = (unsigned char)(Data_Address >> 8); tmp_buf[3] = (unsigned char)Data_Address;
tmp_buf[4] = (unsigned char)(Data >> 8); tmp_buf[5] = (unsigned char)Data;
asdf = usMBCRC16((unsigned char *)tmp_buf, 6);
tmp_buf[6] = (unsigned char)asdf; tmp_buf[7] = (unsigned char)(asdf >> 8);
eat_trace("Data: %02X %02X %02X %02X %02X %02X %02X %02X", tmp_buf[0], tmp_buf[1], tmp_buf[2], tmp_buf[3], tmp_buf[4], tmp_buf[5], tmp_buf[6], tmp_buf[7]);
eat_gpio_write(TX485EN, EAT_GPIO_LEVEL_HIGH);
eat_uart_write(RS485_UART, (const unsigned char *)tmp_buf, 8);
eat_gpt_start(66, EAT_FALSE, eat_gpt_cb_fun);
		}
*/
						break;
					}
				}
			}
		}
	}
	else if (event & SOC_WRITE)
	{
		id = 1;
	}
	else if (event & SOC_ACCEPT)
	{
		id = 2;
	}
	else if (event & SOC_CONNECT)
	{
		id = 3;
	}
	else if (event & SOC_CLOSE)
	{
		id = 4;
		eat_soc_close(s);
	}
	else if (event & SOC_ACKED)
	{
		id = 5;
	}

	sprintf(tmp_buf, "soc_notify_cb %u", id);
	if(id != 0)
	{
eat_trace(tmp_buf);
	}
}

void
bear_notify_cb(cbm_bearer_state_enum state, u8 ip_addr[4])
{
	u8 val;
	char tmp_buf[100];

	if(state & CBM_DEACTIVATED)
	{
		bear_st = 0;
	}
    else
	{
		if(state & CBM_ACTIVATING)
		{
			bear_st = 1;
		}
		else
		{
			if(state & CBM_ACTIVATED)
			{
				bear_st = 2;
				server_soc = eat_soc_create(SOC_SOCK_DGRAM, 0);
				val = TRUE;
				eat_soc_setsockopt(server_soc, SOC_NBIO, &val, sizeof(val));
				val = (SOC_READ | SOC_WRITE | SOC_CLOSE | SOC_CONNECT | SOC_ACCEPT);
				eat_soc_setsockopt(server_soc, SOC_ASYNC, &val,sizeof(val));
			}
			else
			{
				if(state & CBM_DEACTIVATING)
				{
					bear_st = 3;
				}
				else
				{
					if(state & CBM_CSD_AUTO_DISC_TIMEOUT)
					{
						bear_st = 4;
					}
					else
					{
						if(state & CBM_GPRS_AUTO_DISC_TIMEOUT)
						{
							bear_st = 5;
						}
						else
						{
							if(state & CBM_NWK_NEG_QOS_MODIFY)
							{
								bear_st = 6;
							}
							else
							{
								if(state & CBM_WIFI_STA_INFO_MODIFY)
								{
									bear_st = 7;
								}
							}
						}
					}
				}
			}
		}
	}
	sprintf(tmp_buf, "bear_notify_cb %u", bear_st);
eat_trace(tmp_buf);
}

void
hostname_notify_cb(u32 request_id, eat_bool result, u8 ip_addr[4])
{
	char tmp_buf[100];

	sprintf(tmp_buf, "HOSTNAME_NOTIFY:%d,%d,%d:%d:%d:%d\r\n", request_id, result, ip_addr[0], ip_addr[1], ip_addr[2], ip_addr[3]);
eat_trace(tmp_buf);
}

void app_main(void *data)
{
    EatEntryPara_st *para;
	EatEvent_st event;
	char input_buf[5000];
	char * buf_pos = NULL, dtmf_cmd = 0;
	u16 len = 0, out_cnt = 0;
	EAT_CBC_ST bmt = {0};
	s32 cmte = 0;
	u16 i = 0, i1 = 0;
    EatRtc_st rtc = {0};

    APP_InitRegions();//Init app RAM, first step
    APP_init_clib(); //C library initialize, second step

    para = (EatEntryPara_st*)data;

    memcpy(&app_para, para, sizeof(EatEntryPara_st));
    if(app_para.is_update_app && app_para.update_app_result)
    {
        eat_update_app_ok();
    }

	eat_mem_init(s_memPool, EAT_MEM_MAX_SIZE);

	if(!load_settings())
		save_settings();

#if defined(RS485_UART)
	uart_config.baud = EAT_UART_BAUD_19200;
	uart_config.dataBits = EAT_UART_DATA_BITS_8;
	uart_config.parity = EAT_UART_PARITY_NONE;
	uart_config.stopBits = EAT_UART_STOP_BITS_1;
	if(eat_uart_open(RS485_UART) == EAT_TRUE)
	{
		eat_uart_set_config(RS485_UART, &uart_config);
	}
#endif
#if defined(LED)
	eat_gpio_setup(LED, EAT_GPIO_DIR_OUTPUT, EAT_GPIO_LEVEL_LOW);
	eat_gpio_write(LED, EAT_GPIO_LEVEL_LOW);
#endif
#if defined(TX485EN)
	eat_gpio_setup(TX485EN, EAT_GPIO_DIR_OUTPUT, EAT_GPIO_LEVEL_LOW);
	eat_gpio_write(TX485EN, EAT_GPIO_LEVEL_LOW);
#endif
	eat_set_sms_operation_mode(EAT_TRUE);
	eat_sms_register_new_message_callback(eat_sms_new_message_cb);
	eat_sms_register_sms_ready_callback(eat_sms_ready_cb);
	eat_sms_register_send_completed_callback(eat_sms_send_cb);
	eat_set_sms_format(1);
	eat_poweroff_key_sw(EAT_TRUE);
	eat_modem_set_poweron_urc_dir(EAT_USER_0);
	strcpy(simrev, eat_get_version());
	eat_get_imei((u8*)&IMEI[1], 15);
	IMEI[16] = 0;
	for(i = 0; i < 16; i++)
	{
		IMEI[16] = IMEI[16] + IMEI[i];
	}

	server_adr.sock_type = SOC_SOCK_DGRAM;
	server_adr.addr_len = 4;
	server_adr.port = Server_Port;
	server_adr.addr[0]=Server_IP[0];
	server_adr.addr[1]=Server_IP[1];
	server_adr.addr[2]=Server_IP[2];
	server_adr.addr[3]=Server_IP[3];

	eat_soc_notify_register(soc_notify_cb);
	eat_soc_gethost_notify_register(hostname_notify_cb);
	sem_at_done = eat_create_sem("at_done", 0);
	eat_timer_start(EAT_TIMER_1, 5000);
	eat_timer_start(EAT_TIMER_2, 5000);
	eat_timer_start(EAT_TIMER_4, 5000);
	
    while(EAT_TRUE)
    {
        eat_get_event(&event);
        switch(event.event)
        {
            case EAT_EVENT_USER_MSG:
			{
				eat_modem_write(event.data.user_msg.data, event.data.user_msg.len);
				eat_timer_start(EAT_TIMER_3, at_timer);
				at_res = 0;
			}
            case EAT_EVENT_MDM_READY_RD:
			{
				len = eat_modem_read((unsigned char *)input_buf, 1024);
				if(len > 0)
				{
eat_trace(input_buf);
					input_buf[len] = 0;
					if(at_answer[0])
					{
						buf_pos = strstr(input_buf, at_answer);
						if(buf_pos)
						{
							at_res = 1;
							at_ret = buf_pos;
							eat_timer_stop(EAT_TIMER_3);
							eat_sem_put(sem_at_done);
							at_answer[0] = 0;
						}
					}
					if(strstr(input_buf, "RDY"))
					{
						main_status = main_status | 1;
					}
					else
					if(strstr(input_buf, "Call Ready"))
					{
						main_status = main_status | 2;
					}
					else
					if(strstr(input_buf, "SMS Ready"))
					{
						main_status = main_status | 4;
					}
					buf_pos = strstr(input_buf, "+CFUN:");
					if(buf_pos)
					{
						buf_pos = buf_pos + 7;
						cfun = *buf_pos - 0x30;
					}
					buf_pos = strstr(input_buf, "+CPIN:");
					if(buf_pos)
					{
						buf_pos = buf_pos + 7;
						if(!memcmp(buf_pos, "READY", 5))
						{
							cpin = EAT_TRUE;
						}
						else
						{
							cpin = EAT_FALSE;
						}
					}
					if(ICC[0] == 0)
					{
						buf_pos = input_buf;
						do{
							while(!isdigit(*buf_pos) && *buf_pos)
							{
								buf_pos++;
							}
							if(!*buf_pos)
								break;
							for(i = 0; i < 19; i++)
							{
								if(!isdigit(*buf_pos))
									break;
								buf_pos++;
							}
							if(i == 19)
							{
								buf_pos = buf_pos - 6;
								memcpy(ICC, buf_pos, 6);
							}
						}while(*buf_pos && !ICC[0]);
					}
					buf_pos = strstr(input_buf, "+DTMF:");
					if(buf_pos)
					{
						buf_pos = buf_pos + 7;
						dtmf_cmd = *buf_pos;
						if(dtmf_cmd == '#')
						{
							dtmf_menu_number = 0;
							dtmf_c = 'A';
							dtmf_d = 5;
							send_dtmf = EAT_TRUE;
						}
						switch(dtmf_menu_number)
						{
							case 0:
							{
								switch(dtmf_cmd)
								{
									case '0':
									{
										dtmf_menu_number = 1;
										dtmf_c = 'A';
										dtmf_d = 5;
										send_dtmf = EAT_TRUE;
										break;
									}
									case '1':
									{
										dtmf_menu_number = 2;
										dtmf_c = 'A';
										dtmf_d = 5;
										send_dtmf = EAT_TRUE;
										break;
									}
									case '2':
									{
										dtmf_menu_number = 3;
										dtmf_c = 'A';
										dtmf_d = 5;
										send_dtmf = EAT_TRUE;
										break;
									}
									case '3':
									{
										dtmf_menu_number = 4;
										dtmf_c = 'A';
										dtmf_d = 5;
										send_dtmf = EAT_TRUE;
										break;
									}
								}
								break;
							}
							case 2:
							{
								switch(dtmf_cmd)
								{
									case '0':
									{
										ath_simreset_f = EAT_TRUE;
										break;
									}
									case '1':
									{
										eat_fs_Delete(L"C:\\Settings.txt");
										ath_simreset_f = EAT_TRUE;
										break;
									}
									case '2':
									{
										gprs_reset = EAT_TRUE;
										dtmf_c = 'A';
										dtmf_d = 1;
										send_dtmf = EAT_TRUE;
										break;
									}
								}
								break;
							}
							case 3:
							{
								switch(dtmf_cmd)
								{
									case '0':
									{
										strcpy(sms_nbr, incall_nbr);
										fsinfo = EAT_TRUE;
										dtmf_c = 'A';
										dtmf_d = 1;
										send_dtmf = EAT_TRUE;
										break;
									}
									case '2':
									{
										strcpy(sms_nbr, incall_nbr);
										modinfo = EAT_TRUE;
										dtmf_c = 'A';
										dtmf_d = 1;
										send_dtmf = EAT_TRUE;
										break;
									}
									case '3':
									{
										strcpy(sms_nbr, incall_nbr);
										getparam1 = EAT_TRUE;
										dtmf_c = 'A';
										dtmf_d = 1;
										send_dtmf = EAT_TRUE;
										break;
									}
								}
							}
							break;
						}
					}
					buf_pos = strstr(input_buf, "+CLIP:");
					if(buf_pos)
					{
						incall_nbr[0] = 0;
						buf_pos = strchr(buf_pos, '\"');
						buf_pos++;
						if(*buf_pos == '+')
							buf_pos++;
						sscanf(buf_pos, "%12[^\"]", incall_nbr);
						ata_f = EAT_TRUE;
					}
					buf_pos = strstr(input_buf, "NO CARRIER");
					if(buf_pos)
					{
						incall_f = EAT_FALSE;
					}
					buf_pos = strstr(input_buf, "SJDR:");
					if(buf_pos)
					{
						buf_pos = buf_pos + 6;
						if(!memcmp(buf_pos, "JAMMING DETECTED", 16))
						{
						}
						if(!memcmp(buf_pos, "NO JAMMING", 10))
						{
						}
					}
				}
			}
			break;
            case EAT_EVENT_UART_READY_RD:
			{
				EatUart_enum uart = event.data.uart.uart;
				len = eat_uart_read(uart, (unsigned char *)input_buf, 5000);
				if(len != 0)
				{
					input_buf[len] = 0;
#if defined(RS485_UART)
					if(uart == RS485_UART)
					{
						if(len > 2)
						{
							asdf = usMBCRC16((unsigned char *)input_buf, len - 2);
							if(((unsigned char)asdf == input_buf[len-2]) && ((unsigned char)(asdf >> 8) == input_buf[len-1]))
							{
								MB_busy = EAT_FALSE;
								MB_CMD_pos2 = MB_CMD_pos2t;
								if(MB_read_reg != 0)
								{
									MB_read_data = ((u16)input_buf[3] << 8) + input_buf[4];
									wr_pkt(50);
									do_send = EAT_TRUE;
								}
							}
						}
					}
#endif
				}
			}
			break;
			case EAT_EVENT_TIMER:
			{
				if(event.data.timer.timer_id == EAT_TIMER_4)
				{
					eat_timer_start(EAT_TIMER_4, 100);
					if(MB_busy_timer != 0)
					{
						MB_busy_timer--;
						if(MB_busy_timer == 0)
						{
							MB_busy = EAT_FALSE;
						}
					}
					if((MB_CMD_pos1 != MB_CMD_pos2) && !MB_busy)
					{
						tmp_buf[0] = MB_CMD_buf[MB_CMD_pos2];
						tmp_buf[1] = MB_CMD_buf[MB_CMD_pos2+1];
						tmp_buf[2] = MB_CMD_buf[MB_CMD_pos2+2];
						tmp_buf[3] = MB_CMD_buf[MB_CMD_pos2+3];
						tmp_buf[4] = MB_CMD_buf[MB_CMD_pos2+4];
						tmp_buf[5] = MB_CMD_buf[MB_CMD_pos2+5];
						asdf = usMBCRC16((unsigned char *)tmp_buf, 6);
						tmp_buf[6] = (unsigned char)asdf; tmp_buf[7] = (unsigned char)(asdf >> 8);
						if(tmp_buf[1] == 0x03)
						{
							MB_read_reg = ((u16)tmp_buf[2] << 8) + tmp_buf[3];
						}
						eat_gpio_write(TX485EN, EAT_GPIO_LEVEL_HIGH);
						eat_uart_write(RS485_UART, (const unsigned char *)tmp_buf, 8);
						eat_gpt_start(66, EAT_FALSE, eat_gpt_cb_fun);
						MB_CMD_pos2t = MB_CMD_pos2 + 6;
						MB_busy = EAT_TRUE;
						MB_busy_timer = 10;
					}
				}
				if(event.data.timer.timer_id == EAT_TIMER_1)
				{
					eat_timer_start(EAT_TIMER_1, 1000);
/*
rst_cnt++;
if(rst_cnt == 120)
{
	rst_cnt = 0;
	eat_reset_module();
}
*/
MB_cmd03buf(1, 0x0032);
MB_cmd03buf(1, 0x0030);
MB_cmd03buf(1, 0x0031);
MB_cmd03buf(1, 0x0021);
MB_cmd03buf(1, 0x0022);
MB_cmd03buf(1, 0x012A);

tm_cnt1++;
if(tm_cnt1 > 2)
{
	tm_cnt1 = 0;
	if(eeprom_p1 == eeprom_p2)
	{
		wr_pkt(51);
	}
	do_send = EAT_TRUE;
//	eat_trace("udpsend_st=%u,gprs_st=%u,eeprom_p1=%u,eeprom_p2=%u,udpsend_st_timer=%u", udpsend_st, gprs_st, eeprom_p1, eeprom_p2, udpsend_st_timer);
}
					if(eat_get_adc_sync(EAT_PIN38_ADC, (u32 *)&Vcc))
						Vcc = Vcc * 13;
					
					if(simreset_f)
					{
						eat_reset_module();
						simreset_f = EAT_FALSE;
					}

					switch(gprs_st)
					{
						case 0:
						{
							if((gprs_enable) && (gsm_reg != 5 || (Mod_Settings & 32)) && (main_status&2))
							{
								if(bear_st == 0)
								{
									eat_gprs_bearer_open(APN, APN_user, APN_pass, bear_notify_cb);
								}
								gprs_st_timer = 0;
								gprs_st = 1;
							}
						}
						break;
						case 1:
						{
							if(bear_st == 2)
							{
								gprs_st_errcnt = 0;
								gprs_st = 2;
							}
							gprs_st_timer++;
							if(gprs_st_timer > 20)
							{
								gprs_st_errcnt++;
								gprs_st = 0;
							}
						}
						break;
						case 2:
						{
							if((!gprs_enable) || (gsm_reg == 5 && (!(Mod_Settings & 32))))
							{
								eat_soc_close(server_soc);
					            eat_gprs_bearer_release();
								gprs_st = 0;
							}
							else
							if(bear_st != 2)
							{
								gprs_st = 0;
							}
						}
						break;
					}

					switch(udpsend_st)
					{
						case 0:
						{
							if((bear_st == 2) && do_send && (out_buf_col != 0))
							{
								do_send = EAT_FALSE;
								if(rsindp_cnt == 0)
									udpsend_st = 1;
								else
									udpsend_st = 3;
							}
						}
						break;
						case 1:
						{
							eat_soc_sendto(server_soc, IMEI, 17, &server_adr);
							udpsend_st_timer = 15;
							udpsend_st = 2;
						}
						break;
						case 2:
						{
							if(udpsend_st_timer != 0)
							{
								udpsend_st_timer--;
							}
							else
							{
								server_adr.port = Server_Port;
								server_adr.addr[0]=Server_IP[0];
								server_adr.addr[1]=Server_IP[1];
								server_adr.addr[2]=Server_IP[2];
								server_adr.addr[3]=Server_IP[3];

								udpsend_st = 1;
								senderr_cnt++;
								if(senderr_cnt > 30)
								{
									senderr_cnt = 0;
								}
								else
								if(senderr_cnt > 6)
								{
									udpsend_st_timer = 60;
									udpsend_st = 6;
								}
								else
								if(senderr_cnt == 3)
								{
									gprs_reset = EAT_TRUE;
eat_trace("GPRS reset1");
									udpsend_st_timer = 10;
									udpsend_st = 6;
								}
							}
						}
						break;
						case 3:
						{
							eeprom_p2tmp = eeprom_p2;
							i1 = 0;
							while((i1 < 675) && (eeprom_p2tmp != eeprom_p1))
							{
								out_cnt = pktsz(read_byte());
								for(i = 0; i < out_cnt; i++)
								{
									out_buf[i1] = read_byte();
									eeprom_p2tmp++;
									i1++;
									if(eeprom_p2tmp >= main_buf_size)
										eeprom_p2tmp = 0;
								}
							}
							eat_soc_sendto(server_soc, out_buf, i1, &server_adr);
							udpsend_st_timer = 15;
							udpsend_st = 4;
						}
						break;
						case 4:
						{
							if(udpsend_st_timer != 0)
							{
								udpsend_st_timer--;
							}
							else
							{
								server_adr.port = Server_Port;
								server_adr.addr[0]=Server_IP[0];
								server_adr.addr[1]=Server_IP[1];
								server_adr.addr[2]=Server_IP[2];
								server_adr.addr[3]=Server_IP[3];

								udpsend_st = 3;
								senderr_cnt++;
								if(senderr_cnt > 30)
								{
									senderr_cnt = 0;
								}
								else
								if(senderr_cnt > 6)
								{
									udpsend_st_timer = 60;
									udpsend_st = 6;
								}
								else
								if(senderr_cnt == 3)
								{
									gprs_reset = EAT_TRUE;
eat_trace("GPRS reset2");
									udpsend_st_timer = 10;
									udpsend_st = 6;
								}
							}
						}
						break;
						case 5:
						{
							eeprom_p2 = eeprom_p2tmp;
							buf_col_get();
							if(out_buf_col != 0)
								udpsend_st = 3;
							else
								udpsend_st = 0;
						}
						break;
						case 6:
						{
							if(udpsend_st_timer != 0)
							{
								udpsend_st_timer--;
							}
							else
							{
								udpsend_st = 0;
							}
						}
						break;
					}

					if(cfun == 4)
					{
						send_cfun1 = EAT_TRUE;
					}
					if(rsindp_cnt != 0)
						rsindp_cnt--;

					switch(main_st)
					{
						case 0:
						{
							if(main_status & 2)
							{
								gprs_enable = EAT_TRUE;
								main_st = 1;
							}
						}
						break;
						case 1:
						{
						}
						break;
					}
				}
				if(event.data.timer.timer_id == EAT_TIMER_2)
				{
					eat_timer_start(EAT_TIMER_2, 10000);

//	eat_trace("MB_CMD_pos1=%u,MB_CMD_pos2=%u,MB_busy=%u", MB_CMD_pos1, MB_CMD_pos2, MB_busy);
/*
tmp_buf[0] = 0x01; tmp_buf[1] = 0x03; tmp_buf[2] = 0x00; tmp_buf[3] = 0x32; tmp_buf[4] = 0x00; tmp_buf[5] = 0x01;
asdf = usMBCRC16((unsigned char *)tmp_buf, 6);
tmp_buf[6] = (unsigned char)asdf; tmp_buf[7] = (unsigned char)(asdf >> 8);
eat_gpio_write(TX485EN, EAT_GPIO_LEVEL_HIGH);
eat_uart_write(RS485_UART, (const unsigned char *)tmp_buf, 8);
eat_gpt_start(66, EAT_FALSE, eat_gpt_cb_fun);
*/
//eat_sleep(6);
//eat_gpio_write(TX485EN, EAT_GPIO_LEVEL_LOW);

//wr_pkt(3);
//do_send = EAT_TRUE;
					eat_get_rtc(&rtc);
					rssi = eat_network_get_csq();
					rssi = (unsigned char)(rssi * 3.2258);
					if(eat_get_module_temp_sync(&cmte))
						Tm = cmte / 1000;
					gsm_reg = eat_network_get_creg();
					eat_get_cbc(&bmt);
					Vbt = bmt.volt;

					if((gsm_reg != 1) && (gsm_reg != 5))
					{
						reg_err_cnt++;
						if(reg_err_cnt > 90)
						{
							reg_err_cnt = 0;
							send_cfun4 = EAT_TRUE;
eat_trace("GSM reg error1");
						}
					}
					else
					{
						reg_err_cnt = 0;
					}
				}
				if(event.data.timer.timer_id == EAT_TIMER_3)
				{
					at_res = 2;
					at_answer[0] = 0;
					eat_sem_put(sem_at_done);
eat_trace("EAT_TIMER_3");
				}
			}
			break;
 			case EAT_EVENT_INT:
			{
eat_trace("EAT_INT");
				break;
			}
       	}
    }
}

void app_user2(void *data)
{
	char tmp_buf[300];
	static char * buf_pos = NULL;
	s8 ret = 0;
	SINT64 fs_freesize;
	UINT filesize;
	int testFileHandle;

	while(EAT_TRUE)
    {
		if(send_dtmf)
		{
			sprintf(tmp_buf, "AT+VTD=%u\r", dtmf_d);
			write_at(tmp_buf, "OK", 1000);
			sprintf(tmp_buf, "AT+VTS=\"%c\"\r", dtmf_c);
			write_at(tmp_buf, "OK", 1000);
			send_dtmf = EAT_FALSE;
		}
		if(send_cfun1)
		{
			write_at("AT+CFUN=1\r", "OK", 1000);
			if(at_res == 1)
			{
				cfun = 1;
			}
			send_cfun1 = EAT_FALSE;
		}
		if(send_cfun4)
		{
			write_at("AT+CFUN=4\r", "OK", 15000);
			if(at_res == 1)
			{
				cfun = 4;
			}
			send_cfun4 = EAT_FALSE;
		}
		if(main_status&1)
		{
			if(!first_init)
			{
				first_init = EAT_TRUE;
				write_at("ATE0\r", "OK", 1000);
				write_at("AT+CNETLIGHT=0\r", "OK", 1000);
				write_at("AT+CLIP=1\r", "OK", 1000);
				write_at("AT+DDET=1,0,1,0\r", "OK", 1000);
				write_at("AT+CNETSCAN=1\r", "OK", 1000);
				write_at("AT+CSGS=0\r", "OK", 1000);
				write_at("AT+SJDR=1,1,20,1\r", "OK", 1000);
			}
		}
		if(main_status&4)
		{
			if(getparam1)
			{
				sprintf(sms_txt, "%u.%u.%u.%u,%u,,%s,%s,%s", Server_IP[0], Server_IP[1], Server_IP[2], Server_IP[3], Server_Port, APN, APN_user, APN_pass);
				send_sms(sms_nbr, sms_txt);
				getparam1 = EAT_FALSE;
			}
			if(ussd_send)
			{
				sprintf(tmp_buf, "AT+CUSD=1,\"%s\"\r", ussdcmd);
				write_at(tmp_buf, "+CUSD:", 30000);
				if(at_res == 1)
				{
					buf_pos = at_ret + 7;
					if(*buf_pos == '0')
					{
						buf_pos = strchr(at_ret, '\"');
						if(buf_pos)
						{
							sscanf(buf_pos, "%250[^\"]", sms_txt);
							send_sms(sms_nbr, sms_txt);
						}
					}
					else
					{
						sprintf(sms_txt, "USSD error %c", *buf_pos);
					}
				}
				ussd_send = EAT_FALSE;
			}
			if(modinfo)
			{
				buf_pos = sms_txt;
				buf_pos = buf_pos + sprintf(buf_pos, "HW ver:%u\n\r", FW_Ver);
				buf_pos = buf_pos + sprintf(buf_pos, "IMEI:%c%c%c%c%c%c%c%c%c%c%c%c%c%c%c\n\r", IMEI[1], IMEI[2], IMEI[3], IMEI[4], IMEI[5], IMEI[6], IMEI[7], IMEI[8], IMEI[9], IMEI[10], IMEI[11], IMEI[12], IMEI[13], IMEI[14], IMEI[15]);
				buf_pos = buf_pos + sprintf(buf_pos, "GSM ver:%s\n\r", simrev);
				buf_pos = buf_pos + sprintf(buf_pos, "SIM ICC:%c%c%c%c%c%c\n\r", ICC[0], ICC[1], ICC[2], ICC[3], ICC[4], ICC[5]);
				send_sms(sms_nbr, sms_txt);
				modinfo = EAT_FALSE;
			}
			if(fsinfo)
			{
				buf_pos = sms_txt;
				ret = eat_fs_GetDiskSize(EAT_FS, &fs_freesize);
				if(ret >= 0)
				{
					buf_pos = buf_pos + sprintf(buf_pos, "Disk size %lld B\n\r", fs_freesize);
				}
				else
				{
					buf_pos = buf_pos + sprintf(buf_pos, "Disk size get error: %d\n\r", ret);
				}
				ret = eat_fs_GetDiskFreeSize(EAT_FS, &fs_freesize);
				if(ret >= 0)
				{
					buf_pos = buf_pos + sprintf(buf_pos, "Free size %lld B\n\r", fs_freesize);
				}
				else
				{
					buf_pos = buf_pos + sprintf(buf_pos, "Free size get error: %d\n\r", ret);
				}
				testFileHandle = eat_fs_Open(L"C:\\log.txt", FS_READ_ONLY);
				if(testFileHandle < EAT_FS_NO_ERROR)
				{
					buf_pos = buf_pos + sprintf(buf_pos, "Log file open error:%d\n\r", testFileHandle);
				}
				else
				{
					ret = eat_fs_GetFileSize(testFileHandle, &filesize);
					eat_fs_Close(testFileHandle);
					if(ret >= 0)
					{
						buf_pos = buf_pos + sprintf(buf_pos, "Log file size %d B\n\r", filesize);
					}
					else
					{
						buf_pos = buf_pos + sprintf(buf_pos, "Log file size get error: %d\n\r", ret);
					}
				}
				send_sms(sms_nbr, sms_txt);
				fsinfo = EAT_FALSE;
			}
			if(gsmoffon)
			{
				strcpy(sms_txt, "Command \"gsmoffon\" accepted.");
				send_sms(sms_nbr, sms_txt);
				sms_sended = EAT_FALSE;
				while(!sms_sended)
					eat_sleep(1000);
				send_cfun4 = EAT_TRUE;
				gsmoffon = EAT_FALSE;
			}
			if(cpureset)
			{
				strcpy(sms_txt, "Command \"cpureset\" accepted.");
				send_sms(sms_nbr, sms_txt);
				sms_sended = EAT_FALSE;
				while(!sms_sended)
					eat_sleep(1000);
				simreset_f = EAT_TRUE;
				cpureset = EAT_FALSE;
			}
			if(stsreset)
			{
				eat_fs_Delete(L"C:\\Settings.txt");
				strcpy(sms_txt, "Command \"stsreset\" accepted.");
				send_sms(sms_nbr, sms_txt);
				sms_sended = EAT_FALSE;
				while(!sms_sended)
					eat_sleep(1000);
				simreset_f = EAT_TRUE;
				stsreset = EAT_FALSE;
			}
		}
		if(main_status&2)
		{
			if(fw_update)
			{
				write_at("AT+SAPBR=3,1,\"Contype\",\"GPRS\"\r", "OK", 1000);
				sprintf(tmp_buf, "AT+SAPBR=3,1,\"APN\",\"%s\"\r", APN);
				write_at(tmp_buf, "OK", 1000);
				sprintf(tmp_buf, "AT+SAPBR=3,1,\"USER\",\"%s\"\r", APN_user);
				write_at(tmp_buf, "OK", 1000);
				sprintf(tmp_buf, "AT+SAPBR=3,1,\"PWD\",\"%s\"\r", APN_pass);
				write_at(tmp_buf, "OK", 1000);
				write_at("AT+SAPBR=1,1\r", "OK", 20000);
				if(at_res == 1)
				{
					write_at("AT+FTPCID=1\r", "OK", 1000);
					sprintf(tmp_buf, "AT+FTPSERV=\"%u.%u.%u.%u\"\r", FTP_server[0], FTP_server[1], FTP_server[2], FTP_server[3]);
					write_at(tmp_buf, "OK", 1000);
					sprintf(tmp_buf, "AT+FTPUN=\"%s\"\r", FTP_user);
					write_at(tmp_buf, "OK", 1000);
					sprintf(tmp_buf, "AT+FTPPW=\"%s\"\r", FTP_pass);
					write_at(tmp_buf, "OK", 1000);
					sprintf(tmp_buf, "AT+FTPGETNAME=\"%s\"\r", fw_filename);
					write_at(tmp_buf, "OK", 1000);
					sprintf(tmp_buf, "AT+FTPGETPATH=\"%s\"\r", FTP_path);
					write_at(tmp_buf, "OK", 1000);
					write_at("AT+FTPPORT=21\r", "OK", 1000);
					write_at("AT+FTPTIMEOUT=3\r", "OK", 1000);
					write_at("AT+FTPGETTOFS=0,\"app\"\r", "+FTPGETTOFS:", 65000);
					if(at_res == 1)
					{
						buf_pos = at_ret + 13;
						if(*buf_pos == '0')
						{
							app_update(L"C:\\User\\Ftp\\app");
						}
					}
				}
				write_at("AT+SAPBR=0,1\r", "OK", 20000);
				fw_update = EAT_FALSE;
			}
			if(ICC[0] == 0)
			{
				write_at("AT+CCID\r", "OK", 1000);
			}
			if(money_f)
			{
				money_f = EAT_FALSE;
				sprintf(tmp_buf, "AT+CUSD=1,\"%s\"\r", MoneyUSSD);
				write_at(tmp_buf, "+CUSD:", 30000);
				if(at_res == 1)
				{
					buf_pos = at_ret + 7;
					if(*buf_pos == '0')
					{
						buf_pos = strchr(at_ret, '\"');
						if(buf_pos)
						{
							while(!isdigit(*buf_pos))
								buf_pos++;
							Money = atoi(buf_pos);
							wr_pkt(5);
						}
					}
				}
			}
			if(gsmloc_f)
			{
				MCC = 0;
				MNC = 0;
				CID = 0;
				LAC = 0;
				write_at("AT+SJDR=0\r", "OK", 1000);
				write_at("AT+CNETSCAN\r", "Operator:", 30000);
				if(at_res == 1)
				{
					buf_pos = strstr(at_ret, "MCC");
					if(buf_pos)
					{
						sscanf(buf_pos, "MCC:%u,MNC:%u,Rxlev:%*u,Cellid:%X,Arfcn:%*X,Lac:%X", &MCC, &MNC, &CID, &LAC);
					}
				}
				write_at("AT+SJDR=1,1,20,1\r", "OK", 1000);
				wr_pkt(33);
				gsmloc_f = EAT_FALSE;
			}
		}
		if(gprs_reset)
		{
			ret = eat_soc_close(server_soc);
            ret = eat_gprs_bearer_release();
			gprs_reset = EAT_FALSE;
		}
		if(ata_f)
		{
			ata_f = EAT_FALSE;
			write_at("ATA\r", "OK", 1000);
			dtmf_menu_number = 0;
			incall_f = EAT_TRUE;
		}
		if(ath_f)
		{
			ath_f = EAT_FALSE;
			write_at("ATH\r", "OK", 1000);
			incall_f = EAT_FALSE;
		}
		if(ath_simreset_f)
		{
			ath_simreset_f = EAT_FALSE;
			write_at("ATH\r", "OK", 1000);
			incall_f = EAT_FALSE;
			eat_reset_module();
		}

		eat_sleep(1000);
	}
}

#if defined(LED)
void app_user8(void *data)
{
    while(EAT_TRUE)
    {
		if(incall_f)
		{
			eat_gpio_write(LED, EAT_GPIO_LEVEL_HIGH);
			eat_sleep(100);
			eat_gpio_write(LED, EAT_GPIO_LEVEL_LOW);
			eat_sleep(500);
		}
		else
		if(fw_update)
		{
			eat_gpio_write(LED, EAT_GPIO_LEVEL_HIGH);
			eat_sleep(100);
			eat_gpio_write(LED, EAT_GPIO_LEVEL_LOW);
			eat_sleep(100);
			eat_gpio_write(LED, EAT_GPIO_LEVEL_HIGH);
			eat_sleep(100);
			eat_gpio_write(LED, EAT_GPIO_LEVEL_LOW);
			eat_sleep(500);
		}
		else
		{
			if(!cpin)
			{
				eat_gpio_write(LED, EAT_GPIO_LEVEL_HIGH);
				eat_sleep(100);
				eat_gpio_write(LED, EAT_GPIO_LEVEL_LOW);
			}
			else
			{
				eat_gpio_write(LED, EAT_GPIO_LEVEL_HIGH);
				eat_sleep(300);
				eat_gpio_write(LED, EAT_GPIO_LEVEL_LOW);
				if((gsm_reg == 1) || (gsm_reg == 5))
				{
					eat_sleep(300);
					eat_gpio_write(LED, EAT_GPIO_LEVEL_HIGH);
					eat_sleep(300);
					eat_gpio_write(LED, EAT_GPIO_LEVEL_LOW);
					if(bear_st == 2)
					{
						eat_sleep(300);
						eat_gpio_write(LED, EAT_GPIO_LEVEL_HIGH);
						eat_sleep(300);
						eat_gpio_write(LED, EAT_GPIO_LEVEL_LOW);
					}
				}
			}
			eat_sleep(1500);
		}
    }
}
#endif
