/*------------------------------------------------------------------------------------
// MYPLC CORE MODULE
//
// Part of PLC interpter code comes from internet, take the license risk by yourself.
// Copyright(c) by Aixi Wang 2019 <aix.wang@hotmail.com>
//
// Note: * for your study only, don't use in commerical product
//       * compiled by vc6.0, FXGPWIN 3.0 was used to create ladder program
//
//------------------------------------------------------------------------------------

[v1 2019-08-11]
* added PLC PMW loading funciton
* added realio_to_map, map_to_realio hook functions
TODO:
1. add modbus RTU 16in/16out support

[v1 2019-08-14]
* added emulated io (keypad 0..9) = X0 - X9
TODO:
1. added motion function call

[v3 2019-08-17]
* added myMOV instruction support
* fixed LDP issue

[v4 2019-08-24]
* added config.json -- done
* restructure utils.c -- done

* added motion functions
--------------------------
int mc_CHECKONLINE(void)
int mc_CLOSELOCNOTI(void)
int mc_STOPALL(void)
int mc_STOPALLE(void)
int mc_CLRLOC(void)

int mc_GETAXISSTS(axis)
int mc_GETPARAM(axis,param_idx)       
int mc_SETPARAM(axis,low_speed,speed,accel_time)

int mc_MOVEREL(axis,d,n)
int mc_MOVERELPRE(axis,d,n)
int mc_MOVERELMULTI(axis_mask)
int mc_GETlOC(axis)

* wrap mc_xxx to move rpc interface


//------------------------------------------------------*/
#include "stdafx.h"
#include <windows.h>
#include <stdio.h>

#include "mydebug.h"


//--------------------------------------------
// utils.h  -- begin
//--------------------------------------------


#define VERSION "4"

typedef unsigned char u8;
typedef unsigned short u16;
typedef unsigned long u32;

typedef     unsigned char   byte;
typedef     unsigned char   uchar;
typedef     unsigned short  word;
typedef     unsigned short  ushort;
typedef     unsigned long   dword;

#define BIT0    0x0001
#define BIT1    0x0002
#define BIT2    0x0004
#define BIT3    0x0008
#define BIT4    0x0010
#define BIT5    0x0020
#define BIT6    0x0040
#define BIT7    0x0080
#define BIT8    0x0100
#define BIT9    0x0200
#define BIT10   0x0400
#define BIT11   0x0800
#define BIT12   0x1000
#define BIT13   0x2000
#define BIT14   0x4000
#define BIT15   0x8000

#define on 0xff
#define off 0x00

#define m_base 0x0000
#define ext_m_base 0x0060
#define y_base 0x00c0
#define c_base 0x00f0
#define t_base 0x0100
#define x_base 0x0120
#define s_base 0x0140


#define PLC_API_ERR_OFF 8200
#define PLC_API_OUT_OFF 8100
#define PLC_API_IN_OFF 8010


#define MCUJSON_ERROR_STR_TOO_LONG    -2
#define MCUJSON_ERROR_NORMAL          -1
#define MCUJSON_OK                    0


void mcujson_debug(char* p);

int     dump_ram_to_file(char * str_filename, byte * addr, dword bytes);
int     read_file_to_ram(char * str_filename, byte * addr, dword bytes, dword* rel_bytes);
void    print_256bs_hex (byte * p_byte_buffer);
byte    uart_has_char(void);
int     uart_rx(byte* p_byte);
int     uart_tx(byte c);
void    delayms(dword i);
byte    hex_to_bin(char value);
char    hexit(unsigned char value);
int mcujson_find_sub_str(char* p_str1,char* p_str2);
int mcujson_find_x_char(char* p_str1,u32 n,char c);
void mcujson_strcpy_n(char* desc,char* src,u32 n);
int mcujson_lstrip_char(char* p,char c);
int mcujson_strlen(char* p);
void mcujson_int2str(int i, char* p);
int mcujson_add_json_header(char* p1,char* p2);
int mcujson_str2int(char* p,int* d);


int mcujson_get_token(char* json_str, char* token_name, char* token_str);
int mcujson_get_int(char* token_str,int* ptr_val);
int mcujson_get_float(char* token_str,float* ptr_val);
int mcujson_get_bool (char* token_str,int* ptr_val);

int mcujson_stream_state_machine(char c, char* p);



int init_serial(char* str3);


//--------------------------------------------
// utils.h  -- end
//--------------------------------------------






//--------------------------------------------------
// global variables -- begin
//--------------------------------------------------
u8 Run = 1;
u8 RUN_STOP = 0;

u16* all_data;
u8* p_all_data;
u8*  step_status;
u8  Run_Flag; //ADD

// void backup_data(void);

u16 *p_data,process_value;
u16 *p_prog;
u8 T_number,C_number;
u16 T_value,C_value;
u16 mov_d_value;
u16 mov_d_addr;
u16 C_count[2];
u16 *program_start_addr;
u16 sub_add1,sub_add2;
u8 edit_prog;

u16 *prog_p_addr[129];  
u16 *p_save[129];      
u16 process[64];                    

u16 x[0x10000 + 0x15c];
u16 plc_ram[0x10000];
u16 d_ram[0x10000];


u8 step_status_buf[0x1024];

HANDLE Serial;

char config_json_buf[0x10000];
char mc_uart_port[64];



//--------------------------------------------------
// global variables -- end
//--------------------------------------------------



//--------------------------------------------
// utils.c  -- begin
//--------------------------------------------

void set_m(u16 offset,u8 val)
{
    u16 w;
    w = plc_ram[m_base + offset/16];
    if (val == 1)
        w |= 1 << (offset%16);
    else
        w &= 0xffff-(1 << (offset%16));

    plc_ram[m_base + offset/16] = w;

}

u8 get_m(u16 offset)
{
    u16 w;
    w = plc_ram[m_base + offset/16];
    if ((w & (1 << (offset%16))) == 0)
        return 0;
    else
        return 1;
}

//---------------------------
// AlarmBeep
//---------------------------
UINT alarm_beep(void *pArg)
{
    int i,j;
    //while(bJaanState != JA_CEFANG){
    //if ()
    //{
    for(j=0; j<10; j++)
    {
        for (i = 800; i<1500; i=i+50)
            ::Beep(i, 10);

        for (i = 1500; i>800; i=i-50)
            ::Beep(i, 10);
    //}     
    //}
    }

    //bitAlarmBeepRun = 0;
    return 1;
}

//---------------------------
// dump_ram_to_file
//---------------------------
int dump_ram_to_file(char * str_filename, byte * addr, dword bytes)
{
   FILE    * fp;
   dword   i;
   byte    c;

   if ( !( fp=fopen(str_filename, "wb")) )
   {  
       printf("Open file %s error. \n", str_filename);
       return -1; 
   }

    fseek(fp, 0, SEEK_SET);
    for (i = 0; i< bytes; i++)
    {  
        c = *(addr+i);
        fwrite(&c, 1, 1, fp); 
    }
    fclose(fp);
    return 0;
}



//----------------------
// read_file_to_ram
//----------------------
int read_file_to_ram(char * str_filename, byte* addr, dword bytes,dword* rel_bytes)
{
   FILE    * fp;
   dword   i;
   byte    c;

   *rel_bytes = 0;

   if ( !( fp=fopen(str_filename, "rb")) )
   {  
       printf("Open file %s error. \n", str_filename);
       return -1; 
   };

    fseek(fp, 0, SEEK_SET);
    for (i = 0; i< bytes; i++)
    {  
      fread(&c, 1, 1, fp); 
      
      if (feof(fp) )
      {
            fclose(fp);
            return 0;
      }

      else
      {
          *(addr+i) = c;
          //printf("c=%x,i=%d\r\n",c,i);
          (*rel_bytes)++;
      }
    }
    fclose(fp);
    return 0;

}

//-------------------------
// print_256bs_hex      
//-------------------------
void print_256bs_hex(byte * p_byte_buffer, dword virtual_address)
{
    int     i,j;
    byte    c,d;
    printf("\nBase: %lx\n", virtual_address);
    for (i = 0; i < 16; i++)
    {
       for (j = 0; j < 16; j++)
       {   c = p_byte_buffer[i * 16 + j];

       d = c & 0x0f;

       printf("%1x",c>>4);
       printf("%1x",d );
       putchar(' '); 
    }

    printf("   ");
    for (j = 0; j < 16; j++)
    {   
       c = p_byte_buffer[i * 16 + j];
       if ( c !=0x0a && c != 0x0d && c != 0x07 && c!=0x0c && c!=0x09)
            printf("%c",c); }

        putchar('\n'); 
    }

}


//-------------------------
// print_plc_bin
//-------------------------
void
print_plc_bin(u16* b, u16 n)
{
    int i;

    printf("====== PLC start ========\r\n");
    for(i=0; i<n; i++)
    {
        if (b[i] == 0xffff)
            break;

        printf("%4d(0x%4x)-> %4X\r\n",i,i,b[i]);
    }
    printf("====== PLC end ========\r\n");

}

//-----------------
// is_key_pressed
//-----------------
byte
is_key_pressed(word vk_key){
word wGAKS_return;
    wGAKS_return = GetAsyncKeyState( vk_key);
    // check num0
    if ((wGAKS_return & 0x8000) == 0x8000)
    {
        return 1;
    }
    else
        return 0;
}

//-----------------------------------------------------------------------------
// Open and configure the serial port, and put a handle to it in the global
// Serial. Print an error and exit if something fails.
//-----------------------------------------------------------------------------
int serial_init(char* str3)
{

    Serial = CreateFile( str3, GENERIC_READ | GENERIC_WRITE, 0, NULL,
                     OPEN_EXISTING, 0, NULL);

    if (Serial == INVALID_HANDLE_VALUE) {
        //printf("can't get port\n");
        return -1;
    }

    DCB dcb;

    if(!GetCommState(Serial, &dcb)) {
        //printf("can't get params\n");
        return -2;
    }

    //dcb.BaudRate = CBR_57600;
    dcb.BaudRate = CBR_115200;
    dcb.ByteSize = 8;
    dcb.Parity   = NOPARITY;
    dcb.StopBits = ONESTOPBIT;

    if(!SetCommState(Serial, &dcb)) {
        //printf("can't set params\n");
        return -3;
    }

    COMMTIMEOUTS cto;
    cto.ReadIntervalTimeout = 100;
    cto.ReadTotalTimeoutMultiplier = 0;
    cto.ReadTotalTimeoutConstant = 1000;

    cto.WriteTotalTimeoutMultiplier = 0;
    cto.WriteTotalTimeoutConstant = 0;

    if(!SetCommTimeouts(Serial, &cto)) {
        //printf("couldn't set timeouts\n");
        return -4;
    }

    return 0;
}

void serial_close(void)
{
    CloseHandle(Serial);
}
//---------------------
// uart_rx
//---------------------

int uart_rx(byte* pByte)
{

    DWORD r;

    if(!ReadFile(Serial, pByte, 1, &r, NULL)) 
    {
        return -1;
    }
    else
        return 0;



}
        
//---------------------
// uart_tx
//---------------------
byte bCmdStr[16];
byte bCmdIdx;

int uart_tx(byte c)
{
    


        dword written;
        byte b;
        b = c;
        if (!WriteFile(Serial, &b, 1, &written, NULL)){
            return -1;  
        }

        return 0;

#if 0       
        else
        {
            if (c == '[')
                bCmdIdx = 0;
            else
                bCmdIdx++;
            
            if (bCmdIdx >= 10)
                bCmdIdx = 0;

            bCmdStr[bCmdIdx] = c;

            return 1;
        }
#endif

}

//---------------------
// uart_tx_bytes
//---------------------
int uart_tx_bytes(u8* p,int n)
{
        dword written;
        //byte b;
        //b = c;
        if (!WriteFile(Serial, &p, n, &written, NULL)){
            return -1;  
        }

        return 0;
}

//---------------------
// uart_rx_bytes
//---------------------
int uart_rx_bytes(u8* p,int n)
{

    DWORD r;

    if(!ReadFile(Serial, p, n, &r, NULL)) 
    {
        return -1;
    }
    else
        return 0;

}

//---------------------
// delayms
//---------------------

void
delayms(dword i){
    Sleep(i);
}

//---------------------
// hex_to_bin
//---------------------

byte 
hex_to_bin( char value ) {

    unsigned char result;
    result =value - '0';
    if ( result > 9 ) result -= ('A' - '9' - 1);

    return( result );

}

//---------------------
// hexit
//---------------------

char 
hexit( unsigned char value ) {

    char result;
    result = value & 0x0F;
    result += '0';
    if (result > '9') result += ('A' - '9' - 1);

    return( result );

}

#define MCUJSON_JSON_MAX_LEN                256
#define MCUJSON_MAX_TOKEN_NAME_LEN           32
#define MCUJSON_MAX_TOKEN_STR_LEN            256
#define MCUJSON_MAX_TOKEN_NUM                32
#define MCUJSON_TRUE_VALUE                 "true"
#define MCUJSON_FALSE_VALUE                "false"
#define MCUJSON_NULL_VALUE                 "null"



// u8 mcujson_buff[MCUJSON_JSON_MAX_LEN];

//------------------
// mcujson_debug
//------------------
void mcujson_debug(char* p){
    //puts(p);
}
//------------------
// mcujson_get_token
//------------------
int mcujson_get_token(char* json_str, char* token_name, char* token_str){
    
    int i,j,k1,k2,off1,off2;
    
    j = mcujson_strlen(token_name);
    if (j > MCUJSON_MAX_TOKEN_NAME_LEN)
        return MCUJSON_ERROR_STR_TOO_LONG;
        
    // debug
    mcujson_debug("mcujson_get_token called");
    mcujson_debug("json_str:");
    mcujson_debug(json_str);
    
    mcujson_debug("token_name:");
    mcujson_debug(token_name);
    
    i = mcujson_find_sub_str(json_str,token_name);
    
    if (i < 0){
        mcujson_debug("can't find token_name"); 
        return MCUJSON_ERROR_NORMAL;
    }
    
    if (i >= 1) {
        // off: i+1+j  =>  "xxxx" or x.x or x or none or true or false
        off1 = i + j + 1;
        mcujson_debug("==============1");        
        mcujson_debug(json_str + off1);
        
        k1 = mcujson_lstrip_char(json_str + off1,' ');
        off1 += k1;

        k1 = mcujson_lstrip_char(json_str + off1,':');
        off1 += k1;
        
        k1 = mcujson_lstrip_char(json_str + off1,' ');
        off1 += k1;

        mcujson_debug("==============2");
        mcujson_debug(json_str + off1);
              
        
        if (json_str[off1] == '[') {
            // process array
            k2 = mcujson_find_sub_str(json_str + off1,"]");
            if ((k1 > 0) && (k2 > 0)) {
                mcujson_strcpy_n(token_str,json_str + off1,k2 + 1);
                token_str[k2+1] = 0;                
                mcujson_debug(token_str);
                return 0;
            }
        } else if (json_str[off1] == '{') {
            // process nested json string
            k2 = mcujson_find_sub_str(json_str + off1,"}");
            if ((k1 > 0) && (k2 > 0)) {
                mcujson_strcpy_n(token_str,json_str + off1,k2+1);
                token_str[k2+1] = 0;                
                mcujson_debug(token_str);
                return 0;
            }    
        }
        
        // none, true, false, "xx", x.x, x
        k2 = mcujson_find_sub_str(json_str + off1,",");
        off2 = off1 + k2;
        
        if (k2 > 0) {
            mcujson_debug("==============3");
            mcujson_debug(json_str + off2);
        
            mcujson_debug("debug k2.1");
            if (k2 > 0) {
            
                mcujson_debug("debug k2.2");
                mcujson_strcpy_n(token_str,json_str + off1,k2);
                token_str[k2] = 0;
                mcujson_debug(token_str);
                return 0;
            }
        } else {
            // last_one
            mcujson_debug("==============4");
            mcujson_debug(json_str + off1);
            k2 = mcujson_find_sub_str(json_str + off1,"}");
            if (k2 > 0) {
                mcujson_debug("==============5");
                mcujson_strcpy_n(token_str,json_str + off1,k2);
                token_str[k2] = 0;
                
                mcujson_debug(token_str);
                return MCUJSON_OK;
            } else {
                return -1;
            }
        }
    } else {
        return MCUJSON_ERROR_NORMAL;
    }
    
    return 0;
}

//------------------
// mcujson_get_int
//------------------
int mcujson_get_int(char* token_str,int* ptr_val){

    return 0;
}

//------------------
// mcujson_get_float
//------------------
int mcujson_get_float(char* token_str,float* ptr_val){

    return 0;
}

//------------------
// mcujson_get_bool
//------------------
int mcujson_get_bool(char* token_str,int* ptr_val){


    return 0;
}



//-----------------------------------
// mcujson_stream_state_machine
//-----------------------------------
int mcujson_stream_state_machine(char c, char* p){
    static int status_code = 0;
    static char temp_buf[4];
    static int len_index = 0;
    static int json_len = 0;
    static int json_len_index = 0;
    
    int i;
    
     
    switch(status_code){
        case 0:
            mcujson_debug("mcujson_stream_state_machine 0");        
            if (c == '*') {
                status_code = 1;
                len_index = 0;
            break;
            }
        case 1:
            mcujson_debug("mcujson_stream_state_machine 1");        
            if (c == '*') {
                break;
            } else if ((c <= '9') && (c >= '0')){
                temp_buf[len_index] = c;
                len_index++;
                if (len_index >= MCUJSON_JSON_MAX_LEN) {
                    mcujson_debug("mcujson_stream_state_machine error 2"); 
                    status_code = 0;                
                }
                
            } else if (c == '\r') {
                temp_buf[len_index] = 0;
                mcujson_debug("mcujson_stream_state_machine  0x0d found");
                mcujson_debug(temp_buf);
                mcujson_debug("mcujson_stream_state_machine  0x0d found 2");                
                i = mcujson_str2int(temp_buf, &json_len);
                
                mcujson_debug("mcujson_stream_state_machine  0x0d found 3");                
                json_len_index = 0;
                if (i == 0) {
                    status_code = 2;
                } else {
                    mcujson_debug("mcujson_stream_state_machine error 3"); 
                    status_code = 0;
                    break;
                }
            }
            break;
        
        case 2:
            mcujson_debug("mcujson_stream_state_machine 2");        
            if (c == '\n'){
                status_code = 3;
                break;
            } else {
                mcujson_debug("mcujson_stream_state_machine error 3");            
                status_code = 0;
            }
            break;

        case 3:
            mcujson_debug("mcujson_stream_state_machine 3");
            if (c == '{'){
                status_code = 4;
                p[json_len_index] = '{';
                json_len_index++;
                break;
                
            } else {
                mcujson_debug("mcujson_stream_state_machine error 4");            
                status_code = 0;
            }        
            break;
        
        case 4:
            mcujson_debug("mcujson_stream_state_machine 4");        
            p[json_len_index] = c;
            json_len_index++;
            
            mcujson_debug(p);
            if (json_len_index >= json_len) {
                    p[json_len_index] = 0;
                    status_code = 0;
                    mcujson_debug("mcujson_stream_state_machine done decoding");
                    return 1;
            }
            break;
        
        default:
            status_code = 0;
            break;
    }
    
    return 0;
}

//---------------------
// mcujson_pow
//---------------------
int mcujson_pow(int m, int n){
    int i,j;
    j = 1;
    for (i = 0; i<n; i++)
        j *= m;
    return j;
}

//---------------------
// mcujson_str2int
//---------------------
int mcujson_str2int(char* p,int* d){
    int i = 0;
    int j = 0;
    int k = 0;
    
    mcujson_debug("mcujson_str2int called");
    mcujson_debug(p);
    
    j = mcujson_strlen(p);
    
    for (i = j-1; i>=0 ;i--){
        if ((p[i] >= '0') && (p[i] <= '9')){
            k += (p[i] - '0') * mcujson_pow(10,j-i-1);
        } else {
            return -1;
        }
    }
    *d = k;
    return 0;
}





//--------------------------
// mcujson_find_sub_str
// return :  -1: no finding
//            x: offset
//          
//--------------------------
int mcujson_find_sub_str(char* p_str1, char* p_str2){
    u32 i,j;
    u32 w_str_len1, w_str_len2;


    //mcujson_debug("mcujson_find_sub_str debug 0");

    w_str_len1 = mcujson_strlen(p_str1);
    w_str_len2 = mcujson_strlen(p_str2);

    //printf("w1:%d,w2:%d\r\n",w_str_len1,w_str_len2);

    if (w_str_len1 < w_str_len2){
        //mcujson_debug("mcujson_find_sub_str debug 1");
        return -1;
    }
        
    for(i = 0; i < (w_str_len1 - w_str_len2) + 1; i++)
    {
        //printf("i:%d,%c\r\n",i,p_str1[i]);
        if (p_str1[i] == p_str2[0])
        {
            j = 0;
            while( (j< w_str_len2) && (p_str1[i+j] == p_str2[j]) )
            {
                j++;
            }

            if (j >= w_str_len2)
                return  i;

        }
    }

    return -1;
}

//--------------------------
// mcujson_find_x_char
// return : -1: no finding
//           n: offset      
//--------------------------
int mcujson_find_x_char(char* p_str1,u32 n,char c){
    u32 i,j;
    i = 0;
    j = 0;
    while( (i<n)  && (p_str1[j] != 0) )
    {
        if (p_str1[j] == c)
            i++;
        j++;
    }
    if (i>=n)
        return j;
    else
        return -1;
}

//--------------------------
// mcujson_strcpy_n
//--------------------------
void mcujson_strcpy_n(char* desc,char* src,u32 n){
    u32 i,j;
    i = 0;
    j = 0;
    for(i=0; i<n; i++){
        desc[i] = src[i];
    }
}

//--------------------------
// mcujson_lstrip_space
//--------------------------
int mcujson_lstrip_char(char* p,char c){
    u32 i = 0;   
    while(p[i] == c)
        i++;
    return i;
}

//--------------------------
// mcujson_strlen
//--------------------------
int mcujson_strlen(char* p){
    u32 i = 0;   
    while(p[i] != 0)
        i++;
    return i;
}


//--------------------------
// mcujson_add_json_header
//--------------------------
int mcujson_add_json_header(char* p1,char* p2){
    u32 i,j;
    char s1[10];
    char s2[10];
    i = mcujson_strlen(p1);
    mcujson_int2str(i,s1);
    j = mcujson_strlen(s1);
    
    s2[0] = '*';
    mcujson_strcpy_n(s2 + 1,s1,j);
    s2[j+1] = '\r';
    s2[j+2] = '\n';
    s2[j+3] = 0;
    mcujson_strcpy_n(p2,s2,3+j);
    mcujson_strcpy_n(p2+3+j,p1,i);
    p2[3+j+i] = 0;
    
    mcujson_debug("mcujson_add_json_header:");
    mcujson_debug(p2);
    
    return 0;
}

void mcujson_int2str(int i, char* p){
    sprintf(p,"%d",i);
}



//--------------------------------------------
// utils.c  -- end
//--------------------------------------------





//--------------------------------------------
// mc.c  -- begin
//--------------------------------------------
u8 mc_buff[256];



//-----------------------
// mc_checksum
//-----------------------
u8 mc_checksum(u8* p,u8 n)
{
    int i;
    u8 chksum = 0;
    for(i=0; i<n; i++)
    {
        chksum += p[i];
    }
    return chksum;
}


#define MC_ERR_ALL                400
#define MC_ERR_UART               401
#define MC_ERR_PKG_HEAD           402
#define MC_ERR_PKG_CHKSUM         403
#define MC_ERR_PKG_RESP           404


//-----------------------
// mc_set_errorinfo 
//-----------------------
void mc_set_errorinfo(u16 flag)
{
    // d_ram[PLC_API_ERR_OFF] |= flag;
    set_m(flag,1);
    set_m(MC_ERR_ALL,1);

    
}

//-----------------------
// mc_clr_errorinfo 
//-----------------------
void mc_clr_errorinfo()
{
    
    set_m(MC_ERR_ALL,0);
    set_m(MC_ERR_UART,0);
    set_m(MC_ERR_PKG_HEAD,0);
    set_m(MC_ERR_PKG_CHKSUM,0);
    set_m(MC_ERR_PKG_RESP,0);

}

//-----------------------
// mc_CHECKONLINE 
//-----------------------
u16 mc_CHECKONLINE(void)
{
    
    int iret;

    if (serial_init(mc_uart_port) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }

    mc_buff[0] = 0x3a;
    mc_buff[1] = 0x55;
    mc_buff[2] = mc_checksum(mc_buff,2);

    if (uart_tx_bytes(mc_buff,3) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        serial_close();
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }

    if (uart_rx_bytes(mc_buff,8) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        serial_close();
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }

    // check header
    if (mc_buff[0] != 0xa3)
    {
        mc_set_errorinfo(MC_ERR_PKG_HEAD);
        iret = 0;
    }

    // check checksum
    else if (mc_checksum(mc_buff,7) != mc_buff[7])
    {
        mc_set_errorinfo(MC_ERR_PKG_CHKSUM);
        iret = 0;
    }


    // check response
    else if (mc_buff[1] != 0xaa)
    {
        mc_set_errorinfo(MC_ERR_PKG_RESP);
        iret = 0;
    }

    
    iret = 1;

    d_ram[PLC_API_OUT_OFF] = iret;

    mc_clr_errorinfo();
    serial_close();
    return iret;

}


//-----------------------
// mc_CLOSELOCNOTI 
//-----------------------
u16 mc_CLOSELOCNOTI(void)
{
    int iret;

    if (serial_init(mc_uart_port) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }

    mc_buff[0] = 0xd4;
    mc_buff[1] = 0x55;
    mc_buff[2] = mc_checksum(mc_buff,2);

    if (uart_tx_bytes(mc_buff,3) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        serial_close();
        d_ram[PLC_API_OUT_OFF] = 0;

        return 0;
    }

    
    iret = 1;
    d_ram[PLC_API_OUT_OFF] = iret;

    mc_clr_errorinfo();
    serial_close();
    return iret;

}
//-----------------------
// mc_STOPALL
//-----------------------
u16 mc_STOPALL(void)
{
    int iret;

    if (serial_init(mc_uart_port) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        return 0;
    }

    mc_buff[0] = 0xd4;
    mc_buff[1] = 0xfc;

    mc_buff[2] = 0xff;
    mc_buff[3] = 0x4a;
    mc_buff[4] = 0x7f;


    if (uart_tx_bytes(mc_buff,5) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        serial_close();
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }

    
    iret = 1;

    d_ram[PLC_API_OUT_OFF] = iret;

    mc_clr_errorinfo();
    serial_close();
    return iret;
}

//-----------------------
// mc_STOPALLE
//-----------------------
u16 mc_STOPALLE(void)
{
    int iret;

    if (serial_init(mc_uart_port) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        d_ram[PLC_API_OUT_OFF] = 0;

        return 0;
    }

    mc_buff[0] = 0xd4;
    mc_buff[1] = 0xfc;

    mc_buff[2] = 0xff;
    mc_buff[3] = 0x49;
    mc_buff[4] = 0x7e;


    if (uart_tx_bytes(mc_buff,5) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        serial_close();
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }

    
    iret = 1;

    d_ram[PLC_API_OUT_OFF] = iret;

    mc_clr_errorinfo();
    serial_close();
    return iret;

}

//-----------------------
// mc_CLRLOC
//-----------------------
u16 mc_CLRLOC(void)
{

    int iret;

    if (serial_init(mc_uart_port) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }

    mc_buff[0] = 0x3a;
    mc_buff[1] = 0xd3;

    mc_buff[2] = mc_checksum(mc_buff,2);


    if (uart_tx_bytes(mc_buff,3) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        serial_close();

        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }

    if (uart_rx_bytes(mc_buff,8) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        serial_close();
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }

    // check header
    if (mc_buff[0] != 0xa3)
    {
        mc_set_errorinfo(MC_ERR_PKG_HEAD);
        iret = 0;
    }

    // check checksum
    else if (mc_checksum(mc_buff,7) != mc_buff[7])
    {
        mc_set_errorinfo(MC_ERR_PKG_CHKSUM);
        iret = 0;
    }


    // check response
    else if (mc_buff[1] != 0xaa)
    {
        mc_set_errorinfo(MC_ERR_PKG_RESP);
        iret = 0;
    }

    
    iret = 1;

    d_ram[PLC_API_OUT_OFF] = iret;

    mc_clr_errorinfo();
    serial_close();
    return iret;


}

u16 mc_GETAXISSTS(void)
{

    int iret;
    
    u16 axis;

    if (serial_init(mc_uart_port) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }


    axis = d_ram[PLC_API_IN_OFF];

    mc_buff[0] = 0x3a;
    mc_buff[1] = 0xd6;
    mc_buff[2] = axis & 0xff;
    mc_buff[3] = mc_checksum(mc_buff,3);


    if (uart_tx_bytes(mc_buff,4) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        serial_close();
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }

    if (uart_rx_bytes(mc_buff,8) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        serial_close();
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }

    // check header
    if (mc_buff[0] != 0xa3)
    {
        mc_set_errorinfo(MC_ERR_PKG_HEAD);
        iret = 0;
    }

    // check checksum
    else if (mc_checksum(mc_buff,7) != mc_buff[7])
    {
        mc_set_errorinfo(MC_ERR_PKG_CHKSUM);
        iret = 0;
    }


    // check response
    else if (mc_buff[1] != 0xb6)
    {
        mc_set_errorinfo(MC_ERR_PKG_RESP);
        iret = 0;
    }
    
    d_ram[PLC_API_OUT_OFF+1] = mc_buff[2];
    
    iret = 1;

    d_ram[PLC_API_OUT_OFF] = 1;
    mc_clr_errorinfo();
    serial_close();
    return iret;

}
u16 mc_GETPARAM(void)
{
    int iret;
    int i;

    if (serial_init(mc_uart_port) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }

    mc_buff[0] = 0x3a;
    mc_buff[1] = 0xd5;
    mc_buff[2] = 0xff;
    mc_buff[3] = mc_checksum(mc_buff,4);


    if (uart_tx_bytes(mc_buff,4) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        serial_close();
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }

    if (uart_rx_bytes(mc_buff,64) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        serial_close();
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }

    for(i=0; i<8; i++)
    {
        // check header
        if (mc_buff[i*8] != 0xa3)
        {
            mc_set_errorinfo(MC_ERR_PKG_HEAD);
            iret = 0;
        }

        // check checksum
        else if (mc_checksum(mc_buff+i*8,7) != mc_buff[i*8+7])
        {
            mc_set_errorinfo(MC_ERR_PKG_CHKSUM);
            iret = 0;
        }

        // check response
        //else if (mc_buff[1] != 0xb6)
        //{
        //  mc_set_errorinfo(MC_ERR_PKG_RESP);
        //  iret = 0;
        //}
        
        d_ram[i*2+PLC_API_OUT_OFF+1] = (mc_buff[i*8+3]<<8) + mc_buff[i*8+4];
        d_ram[i*2+PLC_API_OUT_OFF+2] = (mc_buff[i*8+5] << 8) + mc_buff[i*8+6];

    }
    
    iret = 1;
    d_ram[PLC_API_OUT_OFF] = 1;
    mc_clr_errorinfo();
    serial_close();
    return iret;

}

// u16 mc_SETPARAM(u16 axis,u16 low_speed,u16 speed,u16 accel_time)
u16 mc_SETPARAM(void)
{

    int iret;
    u32 p1,p2,p3;
    u16 axis;

    
    axis = d_ram[PLC_API_IN_OFF + 0];
    p1 = (d_ram[PLC_API_IN_OFF + 1] << 16) + d_ram[PLC_API_IN_OFF + 2];
    p2 = (d_ram[PLC_API_IN_OFF + 3] << 16) + d_ram[PLC_API_IN_OFF + 4];
    p3 = (d_ram[PLC_API_IN_OFF + 5] << 16) + d_ram[PLC_API_IN_OFF + 6];



    if (serial_init(mc_uart_port) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }

    mc_buff[0] = 0x3a;
    mc_buff[1] = 0xda;
    mc_buff[2] = (u8)(axis);

    mc_buff[3] = (u8)((p1 >> 24) & 0xff);
    mc_buff[4] = (u8)((p1 >> 16) & 0xff);
    mc_buff[5] = (u8)((p1 >>  8) & 0xff);
    mc_buff[6] = (u8)((p1 >>  0) & 0xff);

    mc_buff[7] = (u8)((p2 >> 24) & 0xff);
    mc_buff[8] = (u8)((p2 >> 24) & 0xff);
    mc_buff[9] = (u8)((p2 >> 24) & 0xff);
    mc_buff[10] = (u8)((p2 >> 24) & 0xff);

    mc_buff[11] = (u8)((p3 >> 24) & 0xff);
    mc_buff[12] = (u8)((p3 >> 24) & 0xff);
    mc_buff[13] = (u8)((p3 >> 24) & 0xff);
    mc_buff[14] = (u8)((p3 >> 24) & 0xff);

    
    mc_buff[15] = mc_checksum(mc_buff,15);


    if (uart_tx_bytes(mc_buff,3) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        serial_close();
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }

    if (uart_rx_bytes(mc_buff,8) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        serial_close();
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }

    // check header
    if (mc_buff[0] != 0xa3)
    {
        mc_set_errorinfo(MC_ERR_PKG_HEAD);
        iret = 0;
    }

    // check checksum
    else if (mc_checksum(mc_buff,7) != mc_buff[7])
    {
        mc_set_errorinfo(MC_ERR_PKG_CHKSUM);
        iret = 0;
    }


    // check response
    else if (mc_buff[1] != 0xaa)
    {
        mc_set_errorinfo(MC_ERR_PKG_RESP);
        iret = 0;
    }

    
    iret = 1;
    d_ram[PLC_API_OUT_OFF] = 1;
    mc_clr_errorinfo();
    serial_close();
    return iret;

}

u16 mc_MOVEREL(void)
{

    int iret;
    u16 axis;
    u16 d;
    u32 n;

    axis = d_ram[PLC_API_IN_OFF];
    d = d_ram[PLC_API_IN_OFF + 1];
    n = (d_ram[PLC_API_IN_OFF + 2] << 16) + d_ram[PLC_API_IN_OFF + 3];
    

    if (serial_init(mc_uart_port) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }

    mc_buff[0] = 0x3a;
    mc_buff[1] = 0xfa;
    mc_buff[2] = (u8)(axis & 0xff);
    mc_buff[3] = (u8)((d & 0xff));
    mc_buff[4] = (u8)((n >> 24) & 0xff);
    mc_buff[5] = (u8)((n >> 16) & 0xff);
    mc_buff[6] = (u8)((n >> 8) & 0xff);
    mc_buff[7] = (u8)((n >> 0) & 0xff);



    mc_buff[8] = mc_checksum(mc_buff,8);


    if (uart_tx_bytes(mc_buff,9) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        serial_close();
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }



    
    iret = 1;
    d_ram[PLC_API_OUT_OFF] = 1;
    mc_clr_errorinfo();
    serial_close();
    return iret;

}

u16 mc_MOVERELPRE()
{

    int iret;
    u16 axis;
    u16 d;
    u32 n;

    axis = d_ram[PLC_API_IN_OFF];
    d = d_ram[PLC_API_IN_OFF + 1];
    n = (d_ram[PLC_API_IN_OFF + 2] << 16) + d_ram[PLC_API_IN_OFF + 3];
    

    if (serial_init(mc_uart_port) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }

    mc_buff[0] = 0x3a;
    mc_buff[1] = 0x81;
    mc_buff[2] = (u8)(axis & 0xff);
    mc_buff[3] = (u8)(d & 0xff);
    mc_buff[4] = (u8)(n >> 24) & 0xff;
    mc_buff[5] = (u8)(n >> 16) & 0xff;
    mc_buff[6] = (u8)(n >> 8) & 0xff;
    mc_buff[7] = (u8)(n >> 0) & 0xff;



    mc_buff[8] = mc_checksum(mc_buff,8);


    if (uart_tx_bytes(mc_buff,9) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        serial_close();
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }



    
    iret = 1;
    d_ram[PLC_API_OUT_OFF] = 1;
    mc_clr_errorinfo();
    serial_close();
    return iret;

}


u16 mc_MOVERELMULTI(void)
{
    int iret;
    u16 axis;
    //u16 d;
    //u32 n;

    axis = d_ram[PLC_API_IN_OFF];
    //d = d_ram[PLC_API_IN_OFF];
    //n = (d_ram[PLC_API_IN_OFF] << 16) + d_ram[PLC_API_IN_OFF];
    

    if (serial_init(mc_uart_port) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }

    mc_buff[0] = 0x3a;
    mc_buff[1] = 0x82;
    mc_buff[2] = axis & 0xff;



    mc_buff[3] = mc_checksum(mc_buff,3);


    if (uart_tx_bytes(mc_buff,9) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        serial_close();
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }



    
    iret = 1;
    d_ram[PLC_API_OUT_OFF] = 1;
    mc_clr_errorinfo();
    serial_close();
    return iret;

}

u16 mc_GETLOC(void)
{

    int iret;
    int i;

    if (serial_init(mc_uart_port) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }


    mc_buff[0] = 0x3a;
    mc_buff[1] = 0xd2;
    mc_buff[2] = 0xff;
    mc_buff[3] = mc_checksum(mc_buff,4);


    if (uart_tx_bytes(mc_buff,4) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        
        serial_close();
        d_ram[PLC_API_OUT_OFF] = 0;

        return 0;
    }

    if (uart_rx_bytes(mc_buff,64) < 0)
    {
        mc_set_errorinfo(MC_ERR_UART);
        serial_close();
        d_ram[PLC_API_OUT_OFF] = 0;
        return 0;
    }

    for(i=0; i<8; i++)
    {
        // check header
        if (mc_buff[i*8] != 0xa3)
        {
            mc_set_errorinfo(MC_ERR_PKG_HEAD);
            iret = 0;
        }

        // check checksum
        else if (mc_checksum(mc_buff+i*8,7) != mc_buff[i*8+7])
        {
            mc_set_errorinfo(MC_ERR_PKG_CHKSUM);
            iret = 0;
        }

        // check response
        //else if (mc_buff[1] != 0xb6)
        //{
        //  mc_set_errorinfo(MC_ERR_PKG_RESP);
        //  iret = 0;
        //}
        
        d_ram[i*3+PLC_API_OUT_OFF+1] = mc_buff[i*8+2];
        d_ram[i*3+PLC_API_OUT_OFF+2] = (mc_buff[i*8+3]<<8) + mc_buff[i*8+4];
        d_ram[i*3+PLC_API_OUT_OFF+3] = (mc_buff[i*8+5] << 8) + mc_buff[i*8+6];

    }
    
    iret = 1;
    d_ram[PLC_API_OUT_OFF] = 1;
    mc_clr_errorinfo();
    serial_close();
    return iret;


}

u16 mc_SLP(void)
{
    u16 iret;

    u16 n;

    n = d_ram[PLC_API_IN_OFF];
    Sleep(n);


    iret = 1;
    d_ram[PLC_API_OUT_OFF] = 1;
    mc_clr_errorinfo();
    return iret;
}
//--------------------------------------------
// mc.c  -- end
//--------------------------------------------


//--------------------------------------------------
// plc.c -- start
//--------------------------------------------------

void timer_enable(u16 timer_number)
{

}

void timer_disable(u16 timer_number)
{

}


//-------------------------------
// realio_to_map
//-------------------------------
void realio_to_map(void)
{

    // 0
    if (is_key_pressed(VK_NUMPAD0))
    {
        dbg0("numpad 0 pressed");
        plc_ram[x_base] |= 0x01;
    }
    else
    {
        plc_ram[x_base] &= ~0x01;
    }


    // 1
    if (is_key_pressed(VK_NUMPAD1))
    {
        dbg0("numpad 1 pressed");
        plc_ram[x_base] |= 0x02;
    }
    else
    {
        plc_ram[x_base] &= ~0x02;
    }


    // 2
    if (is_key_pressed(VK_NUMPAD2))
    {
        dbg0("numpad 2 pressed");
        plc_ram[x_base] |= 0x04;
    }
    else
    {
        plc_ram[x_base] &= ~0x04;
    }


    // 3
    if (is_key_pressed(VK_NUMPAD3))
    {
        dbg0("numpad 3 pressed");
        plc_ram[x_base] |= 0x08;
    }
    else
    {
        plc_ram[x_base] &= ~0x08;
    }

    // 4
    if (is_key_pressed(VK_NUMPAD4))
    {
        dbg0("numpad 4 pressed");
        plc_ram[x_base] |= 0x10;
    }
    else
    {
        plc_ram[x_base] &= ~0x10;
    }

    // 5
    if (is_key_pressed(VK_NUMPAD5))
    {
        dbg0("numpad 5 pressed");
        plc_ram[x_base] |= 0x20;
    }
    else
    {
        plc_ram[x_base] &= ~0x20;
    }

    // 6
    if (is_key_pressed(VK_NUMPAD6))
    {
        dbg0("numpad 6 pressed");
        plc_ram[x_base] |= 0x40;
    }
    else
    {
        plc_ram[x_base] &= ~0x40;
    }

    // 7
    if (is_key_pressed(VK_NUMPAD7))
    {
        dbg0("numpad 7 pressed");
        plc_ram[x_base] |= 0x80;
    }
    else
    {
        plc_ram[x_base] &= ~0x80;
    }


    // 8
    if (is_key_pressed(VK_NUMPAD8))
    {
        dbg0("numpad 8 pressed");
    }


    // 9
    if (is_key_pressed(VK_NUMPAD9))
    {
        dbg0("numpad 9 pressed");
    }
    
}

//-------------------------------
// map_to_realio
//-------------------------------
void map_to_realio(void)
{
    int i;
    // printf("plc_ram[1]=%2x\r\n",plc_ram[1]);
    printf("X7-X0=%4x | Y7-Y0=%4x\r\n",plc_ram[x_base],plc_ram[y_base]);
    
    printf("D0-15:");
    for(i=0;i<16;i++)
        printf("%d,",d_ram[i]);
    printf("\r\n");

    printf("D1000-1015:");
    for(i=1000;i<1016;i++)
        printf("%d,",d_ram[i]);
    printf("\r\n");

    printf("D8000-8015=");
    for(i=8000;i<8016;i++)
        printf("%d,",d_ram[i]);
    printf("\r\n");

}


#define API_CHECKONLINE_IDX     0
#define API_CLOSELOCNOTI_IDX    1
#define API_STOPALL_IDX         2
#define API_STOPALLE_IDX        3   
#define API_CLRLOC_IDX          4
#define API_GETAXISSTS_IDX      5
#define API_GETPARAM_IDX        6   
#define API_SETPARAM_IDX        7   
#define API_MOVEREL_IDX         8
#define API_MOVERELPRE_IDX      9
#define API_MOVERELMULTI_IDX    10
#define API_GETLOC_IDX          11 
#define API_SLP_IDX             12
//-------------------------------
// map_to_realio
//-------------------------------
void hook_ext_api(u16 d_index)
{
    dbg1("hook_ext_api called, d_index=%d",d_index);


    switch(d_index)
    {
        case API_CHECKONLINE_IDX:
                mc_CHECKONLINE();
                break;

        case API_CLOSELOCNOTI_IDX:
                mc_CLOSELOCNOTI();
                break;

        case API_STOPALL_IDX:
                mc_STOPALL();
                break;

        case API_STOPALLE_IDX:
                mc_STOPALLE();
                break;

        case API_CLRLOC_IDX:
                mc_CLRLOC();
                break;

        case API_GETAXISSTS_IDX:
                mc_GETAXISSTS();
                break;

        case API_GETPARAM_IDX:
                mc_GETPARAM();
                break;

        case API_SETPARAM_IDX:
                mc_SETPARAM();
                break;

        case API_MOVEREL_IDX:
                mc_MOVEREL();
                break;

        case API_MOVERELPRE_IDX:
                mc_MOVERELPRE();
                break;

                
        case API_MOVERELMULTI_IDX:
                mc_MOVERELMULTI();
                break;

        case API_GETLOC_IDX:
                mc_GETLOC();
                break;

        case API_SLP_IDX:
                mc_SLP();
                break;

        default:
                break;
    }












}

//--------------------------------------------
// Porting Layer
//--------------------------------------------
u8 find_step(u16 addr)    
{
    static u16 temp1,temp2;
   
    temp1=addr/0x08;
    temp2=addr%0x08;
    temp2=(1<<temp2);

    if((step_status[temp1]&temp2)==temp2)
      return on;
    else
      return off;
 }



void set_step(u16 addr)    
 { static u16 temp1,temp2;
   temp1=addr/0x08;
   temp2=addr%0x08;
   temp2=(1<<temp2);
   step_status[temp1]|=temp2;
 }


void reset_step(u16 addr)    
{ 
   static u16 temp1,temp2;
   static u8 temp4;
   temp1=addr/0x08;
   temp2=addr%0x08;
   temp2=(1<<temp2);
   temp4=~temp2;
   step_status[temp1]&=temp4;
}


static void LD(u16 start_addr,u8 process_addr)  
{  
  u16 temp2;

  //dbg2("LD %x %x",start_addr,process_addr);

  p_data=all_data+start_addr+process_addr/0x10;   
  temp2=process_addr%0x10;

  if((*p_data&(1<<temp2))==(1<<temp2))
  {   
     process_value<<=1;
     process_value|=1;
  }
  else
  {
     process_value<<=1;
     process_value&=~1;
  }

}


static u8 LDP(u16 start_addr,u8 process_addr)     
{
    u16 temp2;
    
    //dbg2("LDP %x %x",start_addr,process_addr);

    p_data=all_data+start_addr+process_addr/0x10;   
    temp2=process_addr%0x10;
    if((*p_data&(1<<temp2))==(1<<temp2)) 
        return off;
    else
        return on;

}



static u8 LDF(u16 start_addr,u8 process_addr)     
{
    u16 temp2;
    
    //dbg2("LDF %x %x",start_addr,process_addr);

    p_data=all_data+start_addr+process_addr/0x10;   
    temp2=process_addr%0x10;
    if((*p_data&(1<<temp2))==(1<<temp2)) 
        return on;
    else
        return off;

}

static void LDI(u16 start_addr,u8 process_addr)
{
    u16 temp2;

    //dbg2("LDI %x %x",start_addr,process_addr);

    p_data=all_data+start_addr+process_addr/0x10;   
    temp2=process_addr%0x10;
    if((*p_data&(1<<temp2))==(1<<temp2)) 
        process_value<<=1,process_value&=-1;
    else
        process_value<<=1,process_value|=1;

 }

void AND(u16 start_addr,u8 process_addr)
{
    u16 temp2;
    dbg2("AND %x %x",start_addr,process_addr);

    p_data=all_data+start_addr+process_addr/0x10;   
    temp2=process_addr%0x10;
    if(((*p_data&(1<<temp2))==(1<<temp2))&&((process_value&0X01)==0X01)) 
        process_value|=0X01;
    else
        process_value&=~0X01; 
}

static void ANI(u16 start_addr,u8 process_addr)
{
    u16 temp2;
    dbg2("ANI %x %x",start_addr,process_addr);

    p_data=all_data+start_addr+process_addr/0x10;   
    temp2=process_addr%0x10;
    
    if((!((*p_data&(1<<temp2))==(1<<temp2)))&&((process_value&0X01)==0X01)) 
        process_value|=0X01;
    else
        process_value&=~0X01; 

}

static void OR(u16 start_addr,u8 process_addr)
{
    u16 temp2;
    
    dbg2("OR %x %x",start_addr,process_addr);

    p_data=all_data+start_addr+process_addr/0x10;   
    temp2=process_addr%0x10;
    if(((*p_data&(1<<temp2))==(1<<temp2))||((process_value&0X01)==0X01)) 
        process_value|=0X01;
    else
        process_value&=~0X01; 
 }



static void ORI(u16 start_addr,u8 process_addr)
{
    u16 temp2;
    
    dbg2("ORI %x %x",start_addr,process_addr);

    p_data=all_data+start_addr+process_addr/0x10;

    temp2=process_addr%0x10;

    if((!((*p_data&(1<<temp2))==(1<<temp2)))||((process_value&0X01)==0X01)) 
        process_value|=0X01;
    else
        process_value&=~0X01; 
 }


static void _OUT(u16 start_addr,u8 process_addr)
{

  u16 temp2;
  dbg2("_OUT %x %x",start_addr,process_addr);

  p_data=all_data+start_addr+process_addr/0x10;   
  temp2=process_addr%0x10;
   if((process_value&0X01)==0X01)
    *p_data|=(1<<temp2);
  else
     *p_data&=~(1<<temp2); 

}


void force_set(u16 start_addr,u8 process_addr)  
{
    u16 temp2;

    p_data=all_data+start_addr+process_addr/0x10;   
    temp2=process_addr%0x10;
    *p_data|=(1<<temp2);
}


static void BIT_SET(u16 start_addr,u8 process_addr) 
 {u16 temp2;
  p_data=all_data+start_addr+process_addr/0x10;   
  temp2=process_addr%0x10;
  if((process_value&0X01)==0X01)
    *p_data|=(1<<temp2);
}


void force_reset(u16 start_addr,u8 process_addr) 
 {u16 temp2;
  p_data=all_data+start_addr+process_addr/0x10;   
  temp2=process_addr%0x10;
    *p_data&=~(1<<temp2);
}        


static void RST(u16 start_addr,u8 process_addr) 
 {u16 temp2;
  p_data=all_data+start_addr+process_addr/0x10;   
  temp2=process_addr%0x10;
  if((process_value&0X01)==0X01)
    *p_data&=~(1<<temp2);
}


static void MPS(void)                     
{
    process_value<<=1;
    if((process_value&0x02)==0x02)
        process_value|=0x01;
    else
        process_value&=~0x01;
}

static void MRD(void)                   
 {
 if((process_value&0x02)==0x02)
     process_value|=0x01;
      else
     process_value&=~0x01;
    }


static void ORB(void)                 
 {u16 temp;
  temp=process_value;
  process_value>>=1;
  if(((process_value&0x01)==0x01)||((temp&0X01)==0X01))
     process_value|=0x01;
      else
     process_value&=~0x01;
}
static void ANB(void)                  
 {u16 temp;
  temp=process_value;
  process_value>>=1;
  if(((process_value&0x01)==0x01)&&((temp&0X01)==0X01))
     process_value|=0x01;
      else
     process_value&=~0x01;
}

static void INV(void)                  
 {
  if((process_value&0x01)==0x00)    
     process_value|=0x01;
      else
     process_value&=~0x01;
}
static void other_function(u8 process_addr)
 {switch(process_addr)
   { 
     case 0xF8:   ANB()               ;  break;    //块串联 ANB
     case 0xF9:   ORB()               ;  break;    //块并联 ORB
     case 0xFA:   MPS()               ;  break;    //进栈   MPS
     case 0xFB:   MRD()               ;  break;    //读栈   MRD
     case 0xFD:   INV()               ;  break;    //取反   INV
     case 0xFF:                         break;    //ADD
     default:  ;                       break;   
   }
}
static void extend_LD_M(void)   
 { 
   switch(*p_prog/0x100)
    {case 0xA8: LD(ext_m_base+0x00,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xA9: LD(ext_m_base+0x10,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAA: LD(ext_m_base+0x20,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAB: LD(ext_m_base+0x30,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAC: LD(ext_m_base+0x40,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAD: LD(ext_m_base+0x50,u8(*p_prog & 0xff)),p_prog++;             break;  
    }
}
static void extend_LDI_M(void)   
 { 
   switch(*p_prog/0x100)
    {case 0xA8: LDI(ext_m_base+0x00,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xA9: LDI(ext_m_base+0x10,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAA: LDI(ext_m_base+0x20,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAB: LDI(ext_m_base+0x30,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAC: LDI(ext_m_base+0x40,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAD: LDI(ext_m_base+0x50,u8(*p_prog & 0xff)),p_prog++;             break;  
    }
}
static void extend_OR_M(void)   
 { 
   switch(*p_prog/0x100)
    {case 0xA8: OR(ext_m_base+0x00,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xA9: OR(ext_m_base+0x10,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAA: OR(ext_m_base+0x20,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAB: OR(ext_m_base+0x30,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAC: OR(ext_m_base+0x40,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAD: OR(ext_m_base+0x50,u8(*p_prog & 0xff)),p_prog++;             break;  
    }
}
static void extend_ORI_M(void)   
 { 
   switch(*p_prog/0x100)
    {case 0xA8: ORI(ext_m_base+0x00,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xA9: ORI(ext_m_base+0x10,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAA: ORI(ext_m_base+0x20,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAB: ORI(ext_m_base+0x30,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAC: ORI(ext_m_base+0x40,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAD: ORI(ext_m_base+0x50,u8(*p_prog & 0xff)),p_prog++;             break;  
    }
}
static void extend_AND_M(void)   
 { 
   switch(*p_prog/0x100)
    {case 0xA8: AND(ext_m_base+0x00,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xA9: AND(ext_m_base+0x10,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAA: AND(ext_m_base+0x20,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAB: AND(ext_m_base+0x30,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAC: AND(ext_m_base+0x40,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAD: AND(ext_m_base+0x50,u8(*p_prog & 0xff)),p_prog++;             break;  
    }
}
static void extend_ANI_M(void)   
 { 
   switch(*p_prog/0x100)
    {case 0xA8: ANI(ext_m_base+0x00,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xA9: ANI(ext_m_base+0x10,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAA: ANI(ext_m_base+0x20,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAB: ANI(ext_m_base+0x30,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAC: ANI(ext_m_base+0x40,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAD: ANI(ext_m_base+0x50,u8(*p_prog & 0xff)),p_prog++;             break;  
    }
}

static void extend_SET_M(void)   
 { 
   switch(*p_prog/0x100)
    {case 0xA8: BIT_SET(ext_m_base+0x00,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xA9: BIT_SET(ext_m_base+0x10,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAA: BIT_SET(ext_m_base+0x20,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAB: BIT_SET(ext_m_base+0x30,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAC: BIT_SET(ext_m_base+0x40,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAD: BIT_SET(ext_m_base+0x50,u8(*p_prog & 0xff)),p_prog++;             break;  
    }
}
static void extend_RST_M(void)   
 { 
   switch(*p_prog/0x100)
    {case 0xA8: RST(ext_m_base+0x00,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xA9: RST(ext_m_base+0x10,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAA: RST(ext_m_base+0x20,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAB: RST(ext_m_base+0x30,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAC: RST(ext_m_base+0x40,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAD: RST(ext_m_base+0x50,u8(*p_prog & 0xff)),p_prog++;             break;  
    }
}
static void extend_OUT_M(void)   
 { 
   switch(*p_prog/0x100)
    {case 0xA8: _OUT(ext_m_base+0x00,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xA9: _OUT(ext_m_base+0x10,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAA: _OUT(ext_m_base+0x20,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAB: _OUT(ext_m_base+0x30,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAC: _OUT(ext_m_base+0x40,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0xAD: _OUT(ext_m_base+0x50,u8(*p_prog & 0xff)),p_prog++;             break;  
    }
}
static void extend_SET_S(void)   
 { 
   switch(*p_prog/0x100)
    {case 0x80: BIT_SET(s_base,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0x81: BIT_SET(0X0150,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0x82: BIT_SET(0X0160,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0x83: BIT_SET(0X0170,u8(*p_prog & 0xff)),p_prog++;             break;  
    }
}
static void extend_RST_S(void)   
 { 
   switch(*p_prog/0x100)
    {case 0x80: RST(s_base,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0x81: RST(0X0150,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0x82: RST(0X0160,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0x83: RST(0X0170,u8(*p_prog & 0xff)),p_prog++;             break;  
    }
}
static void extend_OUT_S(void)   
 { 
   switch(*p_prog/0x100)
    {case 0x80: _OUT(s_base+0x00,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0x81: _OUT(s_base+0x10,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0x82: _OUT(s_base+0x20,u8(*p_prog & 0xff)),p_prog++;             break;  
     case 0x83: _OUT(s_base+0x30,u8(*p_prog & 0xff)),p_prog++;             break;  
    }
}
static void RESET_T(u8 process_addr)        
 { if((process_value&0x01)==0x01)           
   {
     timer_disable(process_addr);
    }
   _OUT(0X0380,u8(*p_prog & 0xff));
}
static void RESET_C(u8 process_addr)       
 { if((process_value&0x01)==0x01)          
   { p_data=all_data+0x0500+process_addr;  
     *p_data=0;                            
     p_data=all_data+c_base+(process_addr/0x10); 
     *p_data&=~(1<<process_addr%0x10);     
    }
   _OUT(0X0370,u8(*p_prog & 0xff));
}
static void extend_RST_T(void)             
 {  
   switch(*p_prog/0x100)
    {case 0x86: RESET_T(u8(*p_prog & 0xff)),p_prog++;            break;  
     case 0x8E: RESET_C(u8(*p_prog & 0xff)),p_prog++;            break;  
    }
}

static void MOV_TO_K_H(void)       
 {  
  static u8 LL_BIT;      
  static u16 JOB_ADDR;
  // static u32 MOV_DATA_16BIT,MOV_DATA_16BIT_BACKUP,MOV_DATA_BACKUP1;  
  static u16 MOV_DATA_16BIT,MOV_DATA_16BIT_BACKUP,MOV_DATA_BACKUP1;  

   LL_BIT=mov_d_addr%0x10;                       
   JOB_ADDR=mov_d_addr/0x10;                     

   switch(*p_prog/0x100)                             
    
    {case 0x82: MOV_DATA_16BIT_BACKUP=mov_d_value&0X000F,MOV_DATA_16BIT_BACKUP<<=LL_BIT,MOV_DATA_BACKUP1=~(0X000F<<LL_BIT); break;
     case 0x84: MOV_DATA_16BIT_BACKUP=mov_d_value&0X00FF,MOV_DATA_16BIT_BACKUP<<=LL_BIT,MOV_DATA_BACKUP1=~(0X00FF<<LL_BIT); break;
     case 0x86: MOV_DATA_16BIT_BACKUP=mov_d_value&0X0FFF,MOV_DATA_16BIT_BACKUP<<=LL_BIT,MOV_DATA_BACKUP1=~(0X0FFF<<LL_BIT); break;
     case 0x88: MOV_DATA_16BIT_BACKUP=mov_d_value       ,MOV_DATA_16BIT_BACKUP<<=LL_BIT,MOV_DATA_BACKUP1=~(0XFFFF<<LL_BIT); break;
      default:     p_prog+=3;                           break;  
    }

   switch(*p_prog%0x100)
              
    {case 0x00: MOV_DATA_16BIT=all_data[s_base+JOB_ADDR]+(all_data[s_base+1+JOB_ADDR])*0X10000,
               
                MOV_DATA_16BIT&=MOV_DATA_BACKUP1,MOV_DATA_16BIT|=MOV_DATA_16BIT_BACKUP,
               
                all_data[s_base+JOB_ADDR]=MOV_DATA_16BIT,all_data[s_base+1+JOB_ADDR]=MOV_DATA_16BIT/0X10000; break;
     case 0x01: MOV_DATA_16BIT=all_data[0x0150+JOB_ADDR]+(all_data[0x0151+JOB_ADDR])*0X10000,
                MOV_DATA_16BIT&=MOV_DATA_BACKUP1,MOV_DATA_16BIT|=MOV_DATA_16BIT_BACKUP,
                all_data[0x0150+JOB_ADDR]=MOV_DATA_16BIT,all_data[0x0151+JOB_ADDR]=MOV_DATA_16BIT/0X10000; break;
     case 0x02: MOV_DATA_16BIT=all_data[0x0160+JOB_ADDR]+(all_data[0x0161+JOB_ADDR])*0X10000,
                MOV_DATA_16BIT&=MOV_DATA_BACKUP1,MOV_DATA_16BIT|=MOV_DATA_16BIT_BACKUP,
                all_data[0x0160+JOB_ADDR]=MOV_DATA_16BIT,all_data[0x0161+JOB_ADDR]=MOV_DATA_16BIT/0X10000; break;
     case 0x03: MOV_DATA_16BIT=all_data[0x0170+JOB_ADDR]+(all_data[0x0171+JOB_ADDR])*0X10000,
                MOV_DATA_16BIT&=MOV_DATA_BACKUP1,MOV_DATA_16BIT|=MOV_DATA_16BIT_BACKUP,
                all_data[0x0170+JOB_ADDR]=MOV_DATA_16BIT,all_data[0x0171+JOB_ADDR]=MOV_DATA_16BIT/0X10000; break;

     case 0x04: MOV_DATA_16BIT=all_data[x_base+JOB_ADDR]+(all_data[x_base+1+JOB_ADDR])*0X10000,
                MOV_DATA_16BIT&=MOV_DATA_BACKUP1,MOV_DATA_16BIT|=MOV_DATA_16BIT_BACKUP,
                all_data[x_base+JOB_ADDR]=MOV_DATA_16BIT,all_data[x_base+1+JOB_ADDR]=MOV_DATA_16BIT/0X10000; break;
     case 0x05: MOV_DATA_16BIT=all_data[y_base+JOB_ADDR]+(all_data[0x00C1+JOB_ADDR])*0X10000,
                MOV_DATA_16BIT&=MOV_DATA_BACKUP1,MOV_DATA_16BIT|=MOV_DATA_16BIT_BACKUP,
                all_data[y_base+JOB_ADDR]=MOV_DATA_16BIT,all_data[0x00C1+JOB_ADDR]=MOV_DATA_16BIT/0X10000; break;
                
     case 0x08: MOV_DATA_16BIT=all_data[0x0000+JOB_ADDR]+(all_data[0x0001+JOB_ADDR])*0X10000,
                MOV_DATA_16BIT&=MOV_DATA_BACKUP1,MOV_DATA_16BIT|=MOV_DATA_16BIT_BACKUP,
                all_data[0x0000+JOB_ADDR]=MOV_DATA_16BIT,all_data[0x0001+JOB_ADDR]=MOV_DATA_16BIT/0X10000; break;
     case 0x09: MOV_DATA_16BIT=all_data[0x0010+JOB_ADDR]+(all_data[0x0011+JOB_ADDR])*0X10000,
                MOV_DATA_16BIT&=MOV_DATA_BACKUP1,MOV_DATA_16BIT|=MOV_DATA_16BIT_BACKUP,
                all_data[0x0010+JOB_ADDR]=MOV_DATA_16BIT,all_data[0x0011+JOB_ADDR]=MOV_DATA_16BIT/0X10000; break;
     case 0x0A: MOV_DATA_16BIT=all_data[0x0020+JOB_ADDR]+(all_data[0x0021+JOB_ADDR])*0X10000,
                MOV_DATA_16BIT&=MOV_DATA_BACKUP1,MOV_DATA_16BIT|=MOV_DATA_16BIT_BACKUP,
                all_data[0x0020+JOB_ADDR]=MOV_DATA_16BIT,all_data[0x0021+JOB_ADDR]=MOV_DATA_16BIT/0X10000; break;
     case 0x0B: MOV_DATA_16BIT=all_data[0x0030+JOB_ADDR]+(all_data[0x0031+JOB_ADDR])*0X10000,
                MOV_DATA_16BIT&=MOV_DATA_BACKUP1,MOV_DATA_16BIT|=MOV_DATA_16BIT_BACKUP,
                all_data[0x0030+JOB_ADDR]=MOV_DATA_16BIT,all_data[0x0031+JOB_ADDR]=MOV_DATA_16BIT/0X10000; break;
     case 0x0C: MOV_DATA_16BIT=all_data[0x0040+JOB_ADDR]+(all_data[0x0041+JOB_ADDR])*0X10000,
                MOV_DATA_16BIT&=MOV_DATA_BACKUP1,MOV_DATA_16BIT|=MOV_DATA_16BIT_BACKUP,
                all_data[0x0040+JOB_ADDR]=MOV_DATA_16BIT,all_data[0x0041+JOB_ADDR]=MOV_DATA_16BIT/0X10000; break;
     case 0x0D: MOV_DATA_16BIT=all_data[0x0050+JOB_ADDR]+(all_data[0x0051+JOB_ADDR])*0X10000,
                MOV_DATA_16BIT&=MOV_DATA_BACKUP1,MOV_DATA_16BIT|=MOV_DATA_16BIT_BACKUP,
                all_data[0x0050+JOB_ADDR]=MOV_DATA_16BIT,all_data[0x0051+JOB_ADDR]=MOV_DATA_16BIT/0X10000; break;

     case 0x28: MOV_DATA_16BIT=all_data[0x0060+JOB_ADDR]+(all_data[0x0061+JOB_ADDR])*0X10000,
                MOV_DATA_16BIT&=MOV_DATA_BACKUP1,MOV_DATA_16BIT|=MOV_DATA_16BIT_BACKUP,
                all_data[0x0060+JOB_ADDR]=MOV_DATA_16BIT,all_data[0x0061+JOB_ADDR]=MOV_DATA_16BIT/0X10000; break;
     case 0x29: MOV_DATA_16BIT=all_data[0x0070+JOB_ADDR]+(all_data[0x0071+JOB_ADDR])*0X10000,
                MOV_DATA_16BIT&=MOV_DATA_BACKUP1,MOV_DATA_16BIT|=MOV_DATA_16BIT_BACKUP,
                all_data[0x0070+JOB_ADDR]=MOV_DATA_16BIT,all_data[0x0071+JOB_ADDR]=MOV_DATA_16BIT/0X10000; break;
     case 0x2A: MOV_DATA_16BIT=all_data[0x0080+JOB_ADDR]+(all_data[0x0081+JOB_ADDR])*0X10000,
                MOV_DATA_16BIT&=MOV_DATA_BACKUP1,MOV_DATA_16BIT|=MOV_DATA_16BIT_BACKUP,
                all_data[0x0080+JOB_ADDR]=MOV_DATA_16BIT,all_data[0x0081+JOB_ADDR]=MOV_DATA_16BIT/0X10000; break;
     case 0x2B: MOV_DATA_16BIT=all_data[0x0090+JOB_ADDR]+(all_data[0x0091+JOB_ADDR])*0X10000,
                MOV_DATA_16BIT&=MOV_DATA_BACKUP1,MOV_DATA_16BIT|=MOV_DATA_16BIT_BACKUP,
                all_data[0x0090+JOB_ADDR]=MOV_DATA_16BIT,all_data[0x0091+JOB_ADDR]=MOV_DATA_16BIT/0X10000; break;
     case 0x2C: MOV_DATA_16BIT=all_data[0x00A0+JOB_ADDR]+(all_data[0x00A1+JOB_ADDR])*0X10000,
                MOV_DATA_16BIT&=MOV_DATA_BACKUP1,MOV_DATA_16BIT|=MOV_DATA_16BIT_BACKUP,
                all_data[0x00A0+JOB_ADDR]=MOV_DATA_16BIT,all_data[0x00A1+JOB_ADDR]=MOV_DATA_16BIT/0X10000; break;
     case 0x2D: MOV_DATA_16BIT=all_data[0x00B0+JOB_ADDR]+(all_data[0x00B1+JOB_ADDR])*0X10000,
                MOV_DATA_16BIT&=MOV_DATA_BACKUP1,MOV_DATA_16BIT|=MOV_DATA_16BIT_BACKUP,
                all_data[0x00B0+JOB_ADDR]=MOV_DATA_16BIT,all_data[0x00B1+JOB_ADDR]=MOV_DATA_16BIT/0X10000; break;
                
                 
      default:     p_prog++;                           break;  
    }
        p_prog++;   
 }
static void MOV_K(void)   
 {static u8 LL_BIT;      
  //static u32 MOV_DATA_16BIT;
  static u16 MOV_DATA_16BIT;   

   LL_BIT=mov_d_value%0x10;                      

   switch(*p_prog%0x100)
    {case 0x00: MOV_DATA_16BIT=all_data[s_base+mov_d_value/0x10]+(all_data[s_base+1+mov_d_value/0x10])*0X10000,
                MOV_DATA_16BIT>>=LL_BIT,mov_d_value=MOV_DATA_16BIT; break;
     case 0x01: MOV_DATA_16BIT=all_data[0x0150+mov_d_value/0x10]+(all_data[0x0151+mov_d_value/0x10])*0X10000,
                MOV_DATA_16BIT>>=LL_BIT,mov_d_value=MOV_DATA_16BIT; break;
     case 0x02: MOV_DATA_16BIT=all_data[0x0160+mov_d_value/0x10]+(all_data[0x0161+mov_d_value/0x10])*0X10000,
                MOV_DATA_16BIT>>=LL_BIT,mov_d_value=MOV_DATA_16BIT; break;
     case 0x03: MOV_DATA_16BIT=all_data[0x0170+mov_d_value/0x10]+(all_data[0x0171+mov_d_value/0x10])*0X10000,
                MOV_DATA_16BIT>>=LL_BIT,mov_d_value=MOV_DATA_16BIT; break;

     case 0x04: MOV_DATA_16BIT=all_data[x_base+mov_d_value/0x10]+(all_data[x_base+1+mov_d_value/0x10])*0X10000,
                MOV_DATA_16BIT>>=LL_BIT,mov_d_value=MOV_DATA_16BIT; break;
     case 0x05: MOV_DATA_16BIT=all_data[y_base+mov_d_value/0x10]+(all_data[0x00C1+mov_d_value/0x10])*0X10000,
                MOV_DATA_16BIT>>=LL_BIT,mov_d_value=MOV_DATA_16BIT; break;
                
     case 0x08: MOV_DATA_16BIT=all_data[0x0000+mov_d_value/0x10]+(all_data[0x0001+mov_d_value/0x10])*0X10000,
                MOV_DATA_16BIT>>=LL_BIT,mov_d_value=MOV_DATA_16BIT; break;
     case 0x09: MOV_DATA_16BIT=all_data[0x0010+mov_d_value/0x10]+(all_data[0x0011+mov_d_value/0x10])*0X10000,
                MOV_DATA_16BIT>>=LL_BIT,mov_d_value=MOV_DATA_16BIT; break;
     case 0x0A: MOV_DATA_16BIT=all_data[0x0020+mov_d_value/0x10]+(all_data[0x0021+mov_d_value/0x10])*0X10000,
                MOV_DATA_16BIT>>=LL_BIT,mov_d_value=MOV_DATA_16BIT; break;
     case 0x0B: MOV_DATA_16BIT=all_data[0x0030+mov_d_value/0x10]+(all_data[0x0031+mov_d_value/0x10])*0X10000,
                MOV_DATA_16BIT>>=LL_BIT,mov_d_value=MOV_DATA_16BIT; break;
     case 0x0C: MOV_DATA_16BIT=all_data[0x0040+mov_d_value/0x10]+(all_data[0x0041+mov_d_value/0x10])*0X10000,
                MOV_DATA_16BIT>>=LL_BIT,mov_d_value=MOV_DATA_16BIT; break;
     case 0x0D: MOV_DATA_16BIT=all_data[0x0050+mov_d_value/0x10]+(all_data[0x0051+mov_d_value/0x10])*0X10000,
                MOV_DATA_16BIT>>=LL_BIT,mov_d_value=MOV_DATA_16BIT; break;

     case 0x28: MOV_DATA_16BIT=all_data[0x0060+mov_d_value/0x10]+(all_data[0x0061+mov_d_value/0x10])*0X10000,
                MOV_DATA_16BIT>>=LL_BIT,mov_d_value=MOV_DATA_16BIT; break;
     case 0x29: MOV_DATA_16BIT=all_data[0x0070+mov_d_value/0x10]+(all_data[0x0071+mov_d_value/0x10])*0X10000,
                MOV_DATA_16BIT>>=LL_BIT,mov_d_value=MOV_DATA_16BIT; break;
     case 0x2A: MOV_DATA_16BIT=all_data[0x0080+mov_d_value/0x10]+(all_data[0x0081+mov_d_value/0x10])*0X10000,
                MOV_DATA_16BIT>>=LL_BIT,mov_d_value=MOV_DATA_16BIT; break;
     case 0x2B: MOV_DATA_16BIT=all_data[0x0090+mov_d_value/0x10]+(all_data[0x0091+mov_d_value/0x10])*0X10000,
                MOV_DATA_16BIT>>=LL_BIT,mov_d_value=MOV_DATA_16BIT; break;
     case 0x2C: MOV_DATA_16BIT=all_data[0x00A0+mov_d_value/0x10]+(all_data[0x00A1+mov_d_value/0x10])*0X10000,
                MOV_DATA_16BIT>>=LL_BIT,mov_d_value=MOV_DATA_16BIT; break;
     case 0x2D: MOV_DATA_16BIT=all_data[0x00B0+mov_d_value/0x10]+(all_data[0x00B1+mov_d_value/0x10])*0X10000,
                MOV_DATA_16BIT>>=LL_BIT,mov_d_value=MOV_DATA_16BIT; break;
                         
      default:     p_prog+=2;                           break;  
    }
 }

static void K_M_MOV_D_H(void)      
 {                           

  switch(*p_prog/0x100)
    {case 0x82: MOV_K(),mov_d_value&=0X000F; break;
     case 0x84: MOV_K(),mov_d_value&=0X00FF; break;
     case 0x86: MOV_K(),mov_d_value&=0X0FFF; break;
     case 0x88: MOV_K()                   ; break;
      default:     p_prog+=3;                           break;  
    }
 }
static u16 h_cos_value(u16 l_value)
 {static u16 temp; 
  switch(*p_prog/0x100)
    {case 0x80: temp=l_value+((*p_prog%0x100)*0x100),p_prog++; break;                             
     case 0x82: temp=l_value+((*p_prog%0x100)*0x100),temp=all_data[0x0800+temp/2],p_prog++; break;
     case 0x84: temp=l_value+((*p_prog%0x100)*0x100),temp=all_data[0x0500+temp/2],p_prog++; break;
     case 0x86: temp=l_value+((*p_prog%0x100)*0x100),temp=all_data[0x2000+temp/2],p_prog++; break;
     case 0x88: temp=l_value+((*p_prog%0x100)*0x100),temp=all_data[0x2000+temp/2+1000],p_prog++; break;
    }
        return temp;
 }
static u16  cos_value(void)    
 {static u16 temp; 
   switch(*p_prog/0x100)
    {case 0x80: temp=*p_prog%0x100,p_prog++,temp=h_cos_value(temp); break;
     case 0x82: temp=*p_prog%0x100,p_prog++,temp=h_cos_value(temp); break;
     case 0x84: mov_d_value=*p_prog%0x100,p_prog++,K_M_MOV_D_H(),temp=mov_d_value,p_prog++;  break; 
     case 0x86: temp=*p_prog%0x100,p_prog++,temp=h_cos_value(temp); break;
     case 0x88: temp=*p_prog%0x100,p_prog++,temp=h_cos_value(temp); break;
    }
        return temp;
 }
static void add_assgin_value(u16 assgin_value) 
 {static u16 temp; 
  switch(*p_prog/0x100)
    {
     case 0x82: temp=assgin_value+((*p_prog%0x100)*0x100),all_data[0x0800+temp/2]=sub_add1,p_prog++; break;
     case 0x84: temp=assgin_value+((*p_prog%0x100)*0x100),all_data[0x0500+temp/2]=sub_add1,p_prog++; break;
     case 0x86: temp=assgin_value+((*p_prog%0x100)*0x100),all_data[0x2000+temp/2]=sub_add1,p_prog++; break;
     case 0x88: temp=assgin_value+((*p_prog%0x100)*0x100),all_data[0x2000+temp/2+1000]=sub_add1,p_prog++; break;
    }
 } 
static void add_target(void)    
 { u16 temp;
   switch(*p_prog/0x100)
    {case 0x82: temp=*p_prog%0x100,p_prog++,add_assgin_value(temp); break;
     
     case 0x84: mov_d_addr=*p_prog%0x100,p_prog++,mov_d_value=sub_add1,MOV_TO_K_H(); break;          
     case 0x86: temp=*p_prog%0x100,p_prog++,add_assgin_value(temp); break;
     case 0x88: temp=*p_prog%0x100,p_prog++,add_assgin_value(temp); break;
    }
 }
static void div_assign_value(u16 l_value) 
 {static u16 temp; 
  switch(*p_prog/0x100)
    {
     case 0x82: temp=l_value+((*p_prog%0x100)*0x100),all_data[0x0800+temp/2]=sub_add1,all_data[0x0801+temp/2]=sub_add2,p_prog++; break;
     case 0x84: temp=l_value+((*p_prog%0x100)*0x100),all_data[0x0500+temp/2]=sub_add1,all_data[0x0501+temp/2]=sub_add2,p_prog++; break;
     case 0x86: temp=l_value+((*p_prog%0x100)*0x100),all_data[0x2000+temp/2]=sub_add1,all_data[0x2001+temp/2]=sub_add2,p_prog++; break;
     case 0x88: temp=l_value+((*p_prog%0x100)*0x100),all_data[0x2000+temp/2+1000]=sub_add1,all_data[0x2000+temp/2+1001]=sub_add2,p_prog++; break;
    }
 }

static void mul_target(void)                                    
 { u16 temp;
   switch(*p_prog/0x100)
    {case 0x82: temp=*p_prog%0x100,p_prog++,div_assign_value(temp); break;
     
     case 0x84: mov_d_addr=*p_prog%0x100,p_prog++,mov_d_value=sub_add1,MOV_TO_K_H(); break;          
     case 0x86: temp=*p_prog%0x100,p_prog++,div_assign_value(temp); break;
     case 0x88: temp=*p_prog%0x100,p_prog++,div_assign_value(temp); break;
     }
 }

static void extend_MOV(void)       
{
  if((process_value&0X01)==0X01)  
    sub_add1=cos_value(),add_target();  
  else
      p_prog+=4;                  
}

static void extend_MOVP(void)     
 { u8  logic_1; 
    
    logic_1=find_step(p_prog-program_start_addr);  
     if(logic_1==off)                              
      { if((process_value&0x01)==0x01)                     
         { set_step(p_prog-program_start_addr);
           sub_add1=cos_value(),add_target(); 
          }
              else
               p_prog+=4;         
           }
        else
      {if(!((process_value&0x01)==0x01))                         
         reset_step(p_prog-program_start_addr);      
        p_prog+=4;        
           } 
}
static void add(void)      
 { signed short int temp1,temp2;
    if((process_value&0X01)==0X01)
     {temp1=cos_value();
      temp2=cos_value();
      sub_add1=temp1+temp2;
      add_target();
       }
}

static void INC(void)        //ADD TKZSLD
 {
    if((process_value&0X01)==0X01)
     {
      sub_add1=cos_value();
      p_prog-=2;
      sub_add1++;
      add_target();
       }
       else
       p_prog+=2;
}


static void extend_INCP(void)     
 { u8  logic_1; 
    
    logic_1=find_step(p_prog-program_start_addr);  
     if(logic_1==off)                              
      { if((process_value&0x01)==0x01)                     
         { set_step(p_prog-program_start_addr);
           INC();  
          }
              else
               p_prog+=2;         
           }
        else
      {if(!((process_value&0x01)==0x01))                         
         reset_step(p_prog-program_start_addr);      
        p_prog+=2;        
           } 
}

static void DEC(void)        //ADD TKZSLD   //逻辑运算 减1指令
 {
    if((process_value&0X01)==0X01)
     {
      sub_add1=cos_value();
      p_prog-=2;
      sub_add1--;
      add_target();
       }
       else
       p_prog+=2;       //跳过2步程序
}

static void extend_DECP(void)     
 { u8  logic_1; 
    
    logic_1=find_step(p_prog-program_start_addr);  
     if(logic_1==off)                              
      { if((process_value&0x01)==0x01)                     
         { set_step(p_prog-program_start_addr);
           DEC();  
          }
              else
               p_prog+=2;         
           }
        else
      {if(!((process_value&0x01)==0x01))                         
         reset_step(p_prog-program_start_addr);      
        p_prog+=2;        
           } 
}


static void extend_ADDP(void)     
 { u8  logic_1; 
    
    logic_1=find_step(p_prog-program_start_addr);  
     if(logic_1==off)                              
      { if((process_value&0x01)==0x01)                     
         { set_step(p_prog-program_start_addr);
           add();  
          }
              else
               p_prog+=4;         
           }
        else
      {if(!((process_value&0x01)==0x01))                         
         reset_step(p_prog-program_start_addr);      
        p_prog+=4;        
           } 
}
static void sub(void)      
 { signed short int temp1,temp2;
    if((process_value&0X01)==0X01)
     {temp1=cos_value();
      temp2=cos_value();
      sub_add1=temp1-temp2;
      add_target();
       }
}
static void extend_SUBP(void)     
 { u8  logic_1; 
    
    logic_1=find_step(p_prog-program_start_addr);  
     if(logic_1==off)                              
      { if((process_value&0x01)==0x01)                     
         { set_step(p_prog-program_start_addr);
           sub();  
          }
              else
               p_prog+=4;         
           }
        else
      {if(!((process_value&0x01)==0x01))                         
         reset_step(p_prog-program_start_addr);      
        p_prog+=4;        
           } 
}
static void mul(void)    
 { signed int temp1,temp2;
  if((process_value&0X01)==0X01)
   {
    temp1=cos_value();
    temp2=cos_value();
    sub_add1=(temp1*temp2)%0x10000;
    sub_add2=(temp1*temp2)/0x10000;
    mul_target();
      }
}
static void extend_MULP(void)     
 { u8  logic_1; 
    
    logic_1=find_step(p_prog-program_start_addr);  
     if(logic_1==off)                              
      { if((process_value&0x01)==0x01)                     
         { set_step(p_prog-program_start_addr);
           mul();  
          }
              else
               p_prog+=4;         
           }
        else
      {if(!((process_value&0x01)==0x01))                         
         reset_step(p_prog-program_start_addr);      
        p_prog+=4;        
           } 
}
static void div(void)    
 { signed short int temp1,temp2;
   if((process_value&0X01)==0X01)
   {
    temp1=cos_value();
    temp2=cos_value();
    sub_add1=temp1/temp2;
    sub_add2=temp1%temp2;
    mul_target();
       }
}
static void extend_DIVP(void)     
 { u8  logic_1; 
    
    logic_1=find_step(p_prog-program_start_addr);  
     if(logic_1==off)                              
      { if((process_value&0x01)==0x01)                     
         { set_step(p_prog-program_start_addr);
           div();  
          }
              else
               p_prog+=4;         
           }
        else
      {if(!((process_value&0x01)==0x01))                         
         reset_step(p_prog-program_start_addr);      
        p_prog+=4;        
           } 
}

static void amount(void)     
 { signed short int temp1,temp2;
    temp1=cos_value();
    temp2=cos_value();
    if(temp1==temp2) 
     process_value<<=1,process_value|=1;
      else
     process_value<<=1,process_value&=~1;
}



static void big(void)       
 { signed short int temp1,temp2;
    temp1=cos_value();
    temp2=cos_value();
    if(temp1>temp2) 
     process_value<<=1,process_value|=1;
      else
     process_value<<=1,process_value&=~1;
}


static void less(void)     
 { signed short int temp1,temp2;
    temp1=cos_value();
    temp2=cos_value();
    if(temp1<temp2) 
     process_value<<=1,process_value|=1;
      else
     process_value<<=1,process_value&=~1;
}



static void less_amount(void)      
 { signed short int temp1,temp2;
    temp1=cos_value();
    temp2=cos_value();
    if(temp1>temp2) 
     process_value<<=1,process_value|=1;
      else
     process_value<<=1,process_value&=~1;
}
    


static void big_amount(void)       
 { signed short int temp1,temp2;
    temp1=cos_value();
    temp2=cos_value();
    if(temp1<temp2) 
     process_value<<=1,process_value|=1;
      else
     process_value<<=1,process_value&=~1;
}

static void amount_and(void)     
 { signed short int temp1,temp2;
    temp1=cos_value();
    temp2=cos_value();
    if((temp1==temp2)&&((process_value&0X01)==0X01)) 
     process_value|=1;
      else
     process_value&=~1;
}

static void big_and(void)       
 { signed short int temp1,temp2;
    temp1=cos_value();
    temp2=cos_value();
    if((temp1>temp2)&&((process_value&0X01)==0X01)) 
     process_value|=1;
      else
     process_value&=~1;
  }

static void less_and(void)     
 { signed short int temp1,temp2;
    temp1=cos_value();
    temp2=cos_value();
    if((temp1<temp2)&&((process_value&0X01)==0X01)) 
     process_value|=1;
      else
     process_value&=~1;
}

static void less_amount_and(void)      
 { signed short int temp1,temp2;
    temp1=cos_value();
    temp2=cos_value();
    if((temp1>temp2)&&((process_value&0X01)==0X01)) 
     process_value|=1;
      else
     process_value&=~1;
}

static void big_amount_and(void)       
 { signed short int temp1,temp2;
    temp1=cos_value();
    temp2=cos_value();
    if((temp1<temp2)&&((process_value&0X01)==0X01)) 
     process_value|=1;
      else
     process_value&=~1;
 }



static u8 bit_value(void)    
 { u8  temp;
    switch(*p_prog/0X100)
     { case 0x80: temp=LDF(s_base,u8(*p_prog & 0xff)); break;  
       case 0x81: temp=LDF(0X0150,u8(*p_prog & 0xff)); break;  
       case 0x82: temp=LDF(0X0160,u8(*p_prog & 0xff)); break;  
       case 0x83: temp=LDF(0X0170,u8(*p_prog & 0xff)); break;  
       case 0x84: temp=LDF(x_base,u8(*p_prog & 0xff)); break;  
       case 0x85: temp=LDF(y_base,u8(*p_prog & 0xff)); break;  
       case 0x86: temp=LDF(t_base,u8(*p_prog & 0xff)); break;  
       
       case 0x88: temp=LDF(0X0000,u8(*p_prog & 0xff)); break;  
       case 0x89: temp=LDF(0X0010,u8(*p_prog & 0xff)); break;  
       case 0x8A: temp=LDF(0X0020,u8(*p_prog & 0xff)); break;  
       case 0x8B: temp=LDF(0X0030,u8(*p_prog & 0xff)); break;  
       case 0x8C: temp=LDF(0X0040,u8(*p_prog & 0xff)); break;  
       case 0x8D: temp=LDF(0X0050,u8(*p_prog & 0xff)); break;  
       case 0x8E: temp=LDF(c_base,u8(*p_prog & 0xff)); break;  
       case 0x8F: temp=LDF(0X00E0,u8(*p_prog & 0xff)); break;  

       case 0xA8: temp=LDF(0X0060,u8(*p_prog & 0xff)); break;  
       case 0xA9: temp=LDF(0X0070,u8(*p_prog & 0xff)); break;  
       case 0xAA: temp=LDF(0X0080,u8(*p_prog & 0xff)); break;  
       case 0xAB: temp=LDF(0X0090,u8(*p_prog & 0xff)); break;  
       case 0xAC: temp=LDF(0X00A0,u8(*p_prog & 0xff)); break;  
       case 0xAD: temp=LDF(0X00B0,u8(*p_prog & 0xff)); break;  

       default:  temp=2   ;                       break;  
     }
         return temp;
 }



static u8 bit_value_LDP(void)    
 { u8  temp;
    switch(*p_prog/0X100)
     { case 0x80: temp=LDP(s_base,u8(*p_prog & 0xff)); break;  
       case 0x81: temp=LDP(0X0150,u8(*p_prog & 0xff)); break;  
       case 0x82: temp=LDP(0X0160,u8(*p_prog & 0xff)); break;  
       case 0x83: temp=LDP(0X0170,u8(*p_prog & 0xff)); break;  
       case 0x84: temp=LDP(x_base,u8(*p_prog & 0xff)); break;  
       case 0x85: temp=LDP(y_base,u8(*p_prog & 0xff)); break;  
       case 0x86: temp=LDP(t_base,u8(*p_prog & 0xff)); break;  
       
       case 0x88: temp=LDP(0X0000,u8(*p_prog & 0xff)); break;  
       case 0x89: temp=LDP(0X0010,u8(*p_prog & 0xff)); break;  
       case 0x8A: temp=LDP(0X0020,u8(*p_prog & 0xff)); break;  
       case 0x8B: temp=LDP(0X0030,u8(*p_prog & 0xff)); break;  
       case 0x8C: temp=LDF(0X0040,u8(*p_prog & 0xff)); break;  
       case 0x8D: temp=LDP(0X0050,u8(*p_prog & 0xff)); break;  
       case 0x8E: temp=LDP(c_base,u8(*p_prog & 0xff)); break;  
       case 0x8F: temp=LDP(0X00E0,u8(*p_prog & 0xff)); break;  

       case 0xA8: temp=LDP(0X0060,u8(*p_prog & 0xff)); break;  
       case 0xA9: temp=LDP(0X0070,u8(*p_prog & 0xff)); break;  
       case 0xAA: temp=LDP(0X0080,u8(*p_prog & 0xff)); break;  
       case 0xAB: temp=LDP(0X0090,u8(*p_prog & 0xff)); break;  
       case 0xAC: temp=LDP(0X00A0,u8(*p_prog & 0xff)); break;  
       case 0xAD: temp=LDP(0X00B0,u8(*p_prog & 0xff)); break;  

       default:  temp=2   ;                       break;  
     }
         return temp;
 }

static void extend_LDP(void)      
 { u8  logic_1,logic_2;

    // dbg0("extend ldp 0");
    logic_1=find_step(p_prog-program_start_addr);
    
    // dbg1("logic_1=%d",logic_1);

    logic_2=bit_value();
    // dbg1("logic_2=%d",logic_2);
    if(logic_1==off)                               
    {
        // dbg0("extend_LDP 3");
        if(logic_2==on)                            
            process_value<<=1,process_value|=1,set_step(p_prog-program_start_addr);
        else
            process_value<<=1,process_value&=~1;;        
    }
    else
    {
        // dbg0("extend_LDP 5");
        if(logic_2==on)                            
            process_value<<=1,process_value&=~1;
        else
            process_value<<=1,process_value&=~1,reset_step(p_prog-program_start_addr);       
    }
    
    // dbg0("extend_LDP 5");

    p_prog++;       
}

static void extend_LDF(void)     
 { u8  logic_3,logic_4;

    logic_3=find_step(p_prog-program_start_addr);    
    logic_4=bit_value();                         
     if(logic_3==on)                               
      { if(logic_4==off)                           
         process_value<<=1,process_value|=1,reset_step(p_prog-program_start_addr);
          else
         process_value<<=1,process_value&=~1;        
           }
        else
      {if(logic_4==off)                            
         process_value<<=1,process_value&=~1;
          else
         process_value<<=1,process_value&=~1,set_step(p_prog-program_start_addr);        
           }  
         p_prog++;
}
static void extend_ANDP(void)    
 { u8  logic_1,logic_2;

    logic_1=find_step(p_prog-program_start_addr);    
    logic_2=bit_value();
     if(logic_1==off)                              
      { if(logic_2==on)                            
         logic_2=1,set_step(p_prog-program_start_addr);
          else
         logic_2=0;      
           }
        else
      {if(logic_2==on)                             
         logic_2=0;
          else
         logic_2=0,reset_step(p_prog-program_start_addr); 
           }  
        if(((process_value&0x01)==0x01)&&(logic_2==1))
           process_value|=1;
             else
           process_value&=~1;
         p_prog++;      
}

static void extend_ANDF(void)    
 { u8  logic_3,logic_4;

    logic_3=find_step(p_prog-program_start_addr);    
    logic_4=bit_value();
     if(logic_3==on)                               
      { if(logic_4==off)                           
         logic_4=1,reset_step(p_prog-program_start_addr);
          else
         logic_4=0;      
           }
        else
      {if(logic_4==off)                            
         logic_4=0;
          else
         logic_4=0,set_step(p_prog-program_start_addr);      
           }  
        if(((process_value&0x01)==0x01)&&(logic_4==1))
           process_value|=1;
             else
           process_value&=~1;
         p_prog++;
}
static void extend_ORP(void)     
 { u8  logic_1,logic_2;

    logic_1=find_step(p_prog-program_start_addr);    
    logic_2=bit_value();
     if(logic_1==off)                              
      { if(logic_2==on)                            
         logic_2=1,set_step(p_prog-program_start_addr);
          else
         logic_2=0;      
           }
        else
      {if(logic_2==on)                             
         logic_2=0;
          else
         logic_2=0,reset_step(p_prog-program_start_addr);        
           }  
        if(((process_value&0x01)==0x01)||(logic_2==1))
           process_value|=1;
             else
           process_value&=~1;
         p_prog++;      
}

static void extend_ORF(void)     
 { u8  logic_3,logic_4;

    logic_3=find_step(p_prog-program_start_addr);    
    logic_4=bit_value();
     if(logic_3==on)                               
      { if(logic_4==off)                           
         logic_4=1,reset_step(p_prog-program_start_addr);
          else
         logic_4=0;      
           }
        else
      {if(logic_4==off)                            
         logic_4=0;
          else
         logic_4=0,set_step(p_prog-program_start_addr); 
           }  
        if(((process_value&0x01)==0x01)||(logic_4==1))
           process_value|=1;
             else
           process_value&=~1;
         p_prog++;
}
static void CJ_EX(u16 value)  
 { 
   p_prog++;
   switch(*p_prog/0X100)
     { case 0x80: p_prog=prog_p_addr[value/2],p_prog++;   break;  
       default:               break;
     }
 }
static void CJ(void)
 { 
  if((process_value&0X01)==0X01)
  {switch(*p_prog/0X100)
     { case 0x88: CJ_EX(*p_prog%0X100);   break;  
     }
       }
 }
static void RET(void)
 { u8 temp;
   process_value=process[0];    
   p_prog=p_save[0];            
   for(temp=62;temp>0;temp--)
     {process[temp]=process[temp+1];    
      p_save[temp]=p_save[temp+1]; 
    }
}

static void P_MOV(void)
{ 
    u8 temp;
    for(temp=62;temp>0;temp--)
    {
        process[temp+1]=process[temp];    
        p_save[temp+1]=p_save[temp]; 
    }
    process[0]=process_value;   
    p_save[0]=p_prog;               
}

static void CALL_EX(u16 value)
{ 
   p_prog++;
   switch(*p_prog/0X100)
   { 
        case 0x80: P_MOV(),p_prog=prog_p_addr[value/2];
                    break;  
   }
}

static void CALL(void)
{ 

    if((process_value&0X01)==0X01)
    { 
        switch(*p_prog/0X100)
        { 
            case 0x88: CALL_EX(*p_prog%0X100),p_prog++;   
            break;

        }
    }
}


static void extend_function(void) 
 { switch(*p_prog) 
    {case 0x0002: p_prog++,extend_OUT_M();            break;  
     case 0x0003: p_prog++,extend_SET_M();            break;  
     case 0x0004: p_prog++,extend_RST_M();            break;  

     case 0x0005: p_prog++,extend_OUT_S();            break;  
     case 0x0006: p_prog++,extend_SET_S();            break;  
     case 0x0007: p_prog++,extend_RST_S();            break;  

     case 0x000C: p_prog++,extend_RST_T();            break;  
     case 0X0038: p_prog++,add();                     break; 
     case 0x0040: p_prog++,INC();                     break;  
     case 0x0042: p_prog++,DEC();                     break;  
     case 0X003A: p_prog++,sub();                     break;  
     case 0x003C: p_prog++,mul();                     break;  
     case 0x003E: p_prog++,div();                     break;  
     
     case 0x0010: p_prog++,CJ();                      break;  
     case 0x0012: p_prog++,CALL();                    break;  


     case 0x01C2: p_prog++,extend_LD_M();             break;  
     case 0x01C3: p_prog++,extend_LDI_M();            break;  
     case 0x01C4: p_prog++,extend_AND_M();            break;  
     case 0x01C5: p_prog++,extend_ANI_M();            break;  
     case 0x01C6: p_prog++,extend_OR_M();             break;  
     case 0x01C7: p_prog++,extend_ORI_M();            break;  

     case 0x01CA: 
         // dbg0("extend_ldp \r\n");

         p_prog++,extend_LDP();           break;  
     case 0x01CB: p_prog++,extend_LDF();              break;  
     case 0x01CC: p_prog++,extend_ANDP();             break;  
     case 0x01CD: p_prog++,extend_ANDF();             break;  
     case 0x01CE: p_prog++,extend_ORP();              break;  
     case 0x01CF: p_prog++,extend_ORF();              break;  

     case 0x0028: p_prog++,extend_MOV();              break;  

     case 0x000f: p_prog=p_prog;                      break;  

     case 0X01D0: p_prog++,amount();                  break;  
     case 0X01D2: p_prog++,big();                     break;  
     case 0X01D4: p_prog++,less();                    break; 
     case 0X01DA: p_prog++,less_amount();             break;  
     case 0X01DC: p_prog++,big_amount();              break;  
     case 0X1028: p_prog++,extend_MOVP();             break;  
     case 0X1038: p_prog++,extend_ADDP();             break;  
     case 0X103A: p_prog++,extend_SUBP();             break;  
     case 0X103C: p_prog++,extend_MULP();             break;  
     case 0X103E: p_prog++,extend_DIVP();             break;  
     
     case 0x1040: p_prog++,extend_INCP();              break;  
     case 0x1042: p_prog++,extend_DECP();              break;  

     case 0X01E0: p_prog++,amount_and();                  break;  
     case 0X01E2: p_prog++,big_and();                     break;  
     case 0X01E4: p_prog++,less_and();                    break; 
     case 0X01EA: p_prog++,less_amount_and();             break;  
     case 0X01EC: p_prog++,big_amount_and();              break;  



     case 0XF7FF: p_prog++,RET();                     break;  
         
     default:     p_prog++;                           break;  
    }
 }
static void enable_T_K(void)
 {T_value=*p_prog%0x100;   
  p_prog++;
  T_value+=(*p_prog%0x100)*0x100;  
  p_data=all_data+0x1000+T_number; 
  *p_data=T_value;       
  timer_enable(T_number);
  _OUT(0X280,T_number);
}


static void enable_T_D(void)
 {p_data=all_data+0x1000+T_number;
  *p_data=all_data[0x2000+T_value];
  timer_enable(T_number);
  _OUT(0X280,T_number);
}

static void disable_T(void)
 {
  timer_disable(T_number);
  _OUT(0X0280,T_number);     
  _OUT(t_base,T_number);     
}

static void T_given_value_K(void)         
 {
  if((process_value&0X01)==0X01)  
    enable_T_K();
      else
     disable_T(); 
}
static void T_given_value_D(void)         
 { T_value=(*p_prog%0x100)/2;
   p_prog++;
   switch(*p_prog/0x100) 
    { case 0x86: T_value+=(*p_prog%0x100)*0x80;   break;
      case 0x88: T_value+=(*p_prog%0x100)*0x80+1000;   break; 
       }
   if((process_value&0X01)==0X01)  
    enable_T_D();
      else
     disable_T();
}

static void operation_T(void)
{
    T_number=u8(*p_prog & 0xff);       
    p_prog++;                 
    switch(*p_prog/0x100) 
   { case 0x80: T_given_value_K(),p_prog++;              break;  
     case 0x86: T_given_value_D(),p_prog++;              break;  
   }    
}

static void enable_C_K(void)       
 {u16 temp_bit,*p_C_enable_coil;
  C_value=*p_prog%0x100;           
  p_prog++;
  C_value+=(*p_prog%0x100)*0x100;  
  p_data=all_data+0x0500+C_number;
  temp_bit=1<<(C_number%0x10);
  if(*p_data<C_value)              
   {p_C_enable_coil=all_data+0x0270+(C_number/0X10);   
    if(!((*p_C_enable_coil&temp_bit)==temp_bit))
     *p_data+=1;
       }
  if(*p_data<C_value)  
   { p_data=all_data+c_base+(C_number/0x10);  
     *p_data&=~(1<<(C_number%0x10));
  }
      else
    {  p_data=all_data+c_base+(C_number/0x10);  
       *p_data|=(1<<(C_number%0x10));
      }
  _OUT(0X270,C_number);
}
static void enable_C_D(void)      
 {u16 temp_bit,*p_C_enable_coil;
  C_value=all_data[0x2000+C_value];
  p_data=all_data+0x0500+C_number;
  temp_bit=1<<(C_number%0x10);
  if(*p_data<C_value)    
   { p_C_enable_coil=all_data+0x0270+(C_number/0X10);   
    if(!((*p_C_enable_coil&temp_bit)==temp_bit))
     *p_data+=1;
       }
  p_C_enable_coil=all_data+c_base+(C_number/0x10);  
  if(*p_data<C_value)  
   *p_C_enable_coil&=~temp_bit;
      else
     *p_C_enable_coil|=temp_bit;
  _OUT(0X270,C_number);
}
static void disable_C_K(void)
 {
  C_value=*p_prog%0x100;           
  p_prog++;
  C_value+=(*p_prog%0x100)*0x100;  
  p_data=all_data+0x0500+C_number;
  if(*p_data<C_value)  
   { p_data=all_data+c_base+(C_number/0x10);  
     *p_data&=~(1<<(C_number%0x10));
  }
      else
    {  p_data=all_data+c_base+(C_number/0x10);  
       *p_data|=(1<<(C_number%0x10));
      }
  _OUT(0X270,C_number);
}
static void disable_C_D(void)      
 {u16 temp_bit,*p_C_enable_coil;
  C_value=all_data[0x2000+C_value];
  p_data=all_data+0x0500+C_number;
  temp_bit=1<<(C_number%0x10);
  p_C_enable_coil=all_data+c_base+(C_number/0x10);  
  if(*p_data<C_value)  
   *p_C_enable_coil&=~temp_bit;
      else
     *p_C_enable_coil|=temp_bit;

   _OUT(0X270,C_number);
}

static void C_given_value_K(void)         
 {
  if((process_value&0X01)==0X01)          
    enable_C_K();                  
      else
     disable_C_K(); 
}
static void C_given_value_D(void)         
 { C_value=(*p_prog%0x100)/2;
   p_prog++;
   switch(*p_prog/0x100) 
    { case 0x86: C_value+=(*p_prog%0x100)*0x80;        break;
      case 0x88: C_value+=(*p_prog%0x100)*0x80+1000;   break; 
       }
   if((process_value&0X01)==0X01)        
    enable_C_D();
      else
     disable_C_D();
}
static void operation_C(void)
 {C_number=u8(*p_prog & 0xff);       
  p_prog++;               
  switch(*p_prog/0x100) 
   { case 0x80: C_given_value_K(),p_prog++;              break;  
     case 0x86: C_given_value_D(),p_prog++;              break;  
  } 
}
void find_p(void)   
 { u16 temp;
   p_prog=x+0x5c/2-1;
  for(temp=0;temp<7999;temp++)     
    { if((*p_prog/0x100)==0xB0)
         prog_p_addr[*p_prog%0x100]=p_prog;
        p_prog++;
    }
}
void RST_Y(void)
 { static u8 all_out_rst;
    if(all_out_rst>10)
       all_out_rst=0;
    if(p_all_data[0X01C4]==0x09)
      all_out_rst=0;
        else
      { if(all_out_rst==0)
          all_out_rst++,all_data[0x180/2]=0;
      }
}
u8 find_toend(void)   
 { u16 temp;
   p_prog=x+0x5c/2-2;
   temp=0;  
  do
    { p_prog++;
      temp++;
    }while((!(*p_prog==0x000f))&&(temp<7998)); 
    if(temp>7997)
      return 1;
       else       
       return 0;
}


u16 get_data_from_ext(u16 w1,u16 w2)
{
    return (w2 & 0xff) * 0x100 + (w1 & 0xff);
}

void myMOV(void)
{
    u16 op1,op2,op3,op4;
    u16 temp1,temp2;
    
    if ((process_value & 0x01) == 0x00)
    {
        p_prog += 5;
        return;
    }
    op1 = *(p_prog + 1);
    op2 = *(p_prog + 2);
    op3 = *(p_prog + 3);
    op4 = *(p_prog + 4);
    
    // dbg0("---------------- myMov -------------------");
    if (((op1 & 0xff00) == 0x8000) && ((op2 & 0xff00) == 0x8000))
    {
        //temp1 = (op2 & 0xff) * 0x100 + (op1 & 0xff);
        temp1 = get_data_from_ext(op1,op2);

        if ((op4 & 0xff00) == 0x8600) // 0 - 999            
            temp2 = get_data_from_ext(op3,op4)/2;
        else if ((op4 & 0xff00) == 0x8800) // 1000 - 7999
            temp2 = get_data_from_ext(op3,op4)/2 + 1000;
        else if ((op4 & 0xff00) == 0x8000) // 8000 - 8255
            temp2 = get_data_from_ext(op3,op4)/2 + 8000;


        d_ram[temp2] = temp1;
        //dbg2("myMOV k->d,%d,%d",temp1,temp2);

        if (temp2 == 8000)
        {
            hook_ext_api(temp1);
        }


    }
    else
    {

        if ((op2 & 0xff00) == 0x8600) // 0 - 999            
            temp1 = get_data_from_ext(op1,op2)/2;
        else if ((op2 & 0xff00) == 0x8800) // 1000 - 7999
            temp1 = get_data_from_ext(op1,op2)/2 + 1000;
        else if ((op2 & 0xff00) == 0x8000) // 8000 - 8255
            temp1 = get_data_from_ext(op1,op2)/2 + 8000;

        if ((op4 & 0xff00) == 0x8600) // 0 - 999            
            temp2 = get_data_from_ext(op3,op4)/2;
        else if ((op4 & 0xff00) == 0x8800) // 1000 - 7999
            temp2 = get_data_from_ext(op3,op4)/2 + 1000;
        else if ((op4 & 0xff00) == 0x8000) // 8000 - 8255
        {
            temp2 = get_data_from_ext(op3,op4)/2 + 8000;
        }


        d_ram[temp2] = d_ram[temp1];
        

        //dbg2("myMOV d->d,%d,%d",temp1,temp2);


    }
        
    p_prog += 5;
}

//----------------------------
// plc_init
//----------------------------
void plc_init(void)
{

    dbg0("1. plc init\r\n");
    dbg0("2. init plcram, x\r\n");
    // init
    memset((u8*)plc_ram,0,sizeof(plc_ram));
    memset((u8*)x,0x000f,sizeof(x));
    memset((u8*)step_status_buf,0x00,sizeof(step_status_buf));

    all_data = (u16*)plc_ram;
    p_all_data = (u8*)plc_ram;
    program_start_addr = x + (0x15c/2);
    step_status = step_status_buf;

}

//----------------------------
// plc_main
//----------------------------
void plc_main(void) 
 { 
    u8  temp5;
    // u8 run_keep;
    static u8  puls;



    
    // TODO, load code to x, init program space
    
    // TODO, pre-set register space

/*
    run_keep=p_all_data[0X01c4];   
    RST_Y(); 
    
    
    if(RUN_STOP==0)   //MY PCB== 0   AIR=1
    {
        if(Run_Flag==1)  //RUN_STOP
        {
            Run_Flag = 0;
            run_keep=0x09;
            dbg0("3. run_stop x\r\n");
        }
        
        
        if(run_keep==0x09)            
        {
            dbg0("4. run_keep 0x09 process, x\r\n");

            Run=0;
            force_set(0X00E0,0);     
            force_reset(0X00E0,1);  
            if(edit_prog==0x00)      
            { 
                find_p(),edit_prog=1;
                if(find_toend())
                {  
                    p_all_data[0X01C4]=9;
                    goto all_end;  
                }
            }
            
            if(puls==0x00)       
                force_set(0X00E0,2),force_reset(0X00E0,3);

            p_prog=x+0x5c/2-1;
            
            all_data[0x070C]=20;        
*/

            p_prog=program_start_addr;
            do
            {
                temp5=(*p_prog) >> 8;

                //dbg2("ins:%2x, op:%2x\r\n",temp5,(*p_prog) & 0xff);
                //printf("ins:%2x op:%2x\r\n",temp5,(*p_prog) & 0xff);

                switch(temp5) 
                    { 
                    case 0x06: operation_T();                           break;  
                    case 0x0E: operation_C();                           break;  
                    
                    case 0x20: LD(s_base,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x30: LDI(s_base,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x40: AND(s_base,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x50: ANI(s_base,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x60: OR(s_base,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x70: ORI(s_base,u8(*p_prog & 0xff)),p_prog++;            break;  
                    
                    case 0x21: LD(0X0150,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x31: LDI(0X0150,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x41: AND(0X0150,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x51: ANI(0X0150,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x61: OR(0X0150,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x71: ORI(0X0150,u8(*p_prog & 0xff)),p_prog++;            break;  
                    
                    case 0x22: LD(0X0160,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x32: LDI(0X0160,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x42: AND(0X0160,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x52: ANI(0X0160,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x62: OR(0X0160,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x72: ORI(0X0160,u8(*p_prog & 0xff)),p_prog++;            break;  
                    
                    case 0x23: LD(0X0170,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x33: LDI(0X0170,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x43: AND(0X0170,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x53: ANI(0X0170,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x63: OR(0X0170,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x73: ORI(0X0170,u8(*p_prog & 0xff)),p_prog++;            break;  
                    
                    
                    case 0x24: LD(x_base,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x34: LDI(x_base,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x44: AND(x_base,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x54: ANI(x_base,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x64: OR(x_base,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x74: ORI(x_base,u8(*p_prog & 0xff)),p_prog++;            break;  
                    
                    case 0x25: LD(y_base,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x35: LDI(y_base,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x45: AND(y_base,u8(*p_prog & 0xff)),p_prog++;            break;
                    case 0x55: ANI(y_base,u8(*p_prog & 0xff)),p_prog++;            break;
                    case 0x65: OR(y_base,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x75: ORI(y_base,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0XC5: _OUT(y_base,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0XD5: BIT_SET(y_base,u8(*p_prog & 0xff)),p_prog++;     break;  
                    case 0XE5: RST(y_base,u8(*p_prog & 0xff)),p_prog++;         break;  
                    
                    case 0x26: LD(t_base,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x36: LDI(t_base,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x46: AND(t_base,u8(*p_prog & 0xff)),p_prog++;            break;
                    case 0x56: ANI(t_base,u8(*p_prog & 0xff)),p_prog++;            break;
                    case 0x66: OR(t_base,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x76: ORI(t_base,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0XC6: _OUT(t_base,u8(*p_prog & 0xff)),p_prog++;            break;  
                    
                    case 0x28: LD(m_base,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x38: LDI(m_base,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x48: AND(m_base,u8(*p_prog & 0xff)),p_prog++;            break;
                    case 0x58: ANI(m_base,u8(*p_prog & 0xff)),p_prog++;            break;
                    case 0x68: OR(m_base,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x78: ORI(m_base,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0XC8: _OUT(m_base,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0XD8: BIT_SET(m_base,u8(*p_prog & 0xff)),p_prog++;     break;  
                    case 0XE8: RST(m_base,u8(*p_prog & 0xff)),p_prog++;         break;  
                    
                    case 0x29: LD(0X0010,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x39: LDI(0X0010,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x49: AND(0X0010,u8(*p_prog & 0xff)),p_prog++;            break;
                    case 0x59: ANI(0X0010,u8(*p_prog & 0xff)),p_prog++;            break;
                    case 0x69: OR(0X0010,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x79: ORI(0X0010,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0XC9: _OUT(0X0010,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0XD9: BIT_SET(0X0010,u8(*p_prog & 0xff)),p_prog++;     break;  
                    case 0XE9: RST(0X0010,u8(*p_prog & 0xff)),p_prog++;         break;  
                    
                    case 0x2A: LD(0X0020,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x3A: LDI(0X0020,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x4A: AND(0X0020,u8(*p_prog & 0xff)),p_prog++;            break;
                    case 0x5A: ANI(0X0020,u8(*p_prog & 0xff)),p_prog++;            break;
                    case 0x6A: OR(0X0020,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x7A: ORI(0X0020,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0XCA: _OUT(0X0020,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0XDA: BIT_SET(0X0020,u8(*p_prog & 0xff)),p_prog++;     break;  
                    case 0XEA: RST(0X0020,u8(*p_prog & 0xff)),p_prog++;         break;  
                    
                    case 0x2B: LD(0X0030,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x3B: LDI(0X0030,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x4B: AND(0X0030,u8(*p_prog & 0xff)),p_prog++;            break;
                    case 0x5B: ANI(0X0030,u8(*p_prog & 0xff)),p_prog++;            break;
                    case 0x6B: OR(0X0030,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x7B: ORI(0X0030,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0XCB: _OUT(0X0030,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0XDB: BIT_SET(0X0030,u8(*p_prog & 0xff)),p_prog++;     break;  
                    case 0XEB: RST(0X0030,u8(*p_prog & 0xff)),p_prog++;         break;  
                    
                    case 0x2C: LD(0X0040,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x3C: LDI(0X0040,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x4C: AND(0X0040,u8(*p_prog & 0xff)),p_prog++;            break;
                    case 0x5C: ANI(0X0040,u8(*p_prog & 0xff)),p_prog++;            break;
                    case 0x6C: OR(0X0040,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x7C: ORI(0X0040,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0XCC: _OUT(0X0040,u8(*p_prog & 0xff)),p_prog++;           break;  
                    case 0XDC: BIT_SET(0X0040,u8(*p_prog & 0xff)),p_prog++;        break;  
                    case 0XEC: RST(0X0040,u8(*p_prog & 0xff)),p_prog++;            break;  
                    
                    case 0x2D: LD(0X0050,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x3D: LDI(0X0050,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x4D: AND(0X0050,u8(*p_prog & 0xff)),p_prog++;            break;
                    case 0x5D: ANI(0X0050,u8(*p_prog & 0xff)),p_prog++;            break;
                    case 0x6D: OR(0X0050,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x7D: ORI(0X0050,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0XCD: _OUT(0X0050,u8(*p_prog & 0xff)),p_prog++;           break;  
                    case 0XDD: BIT_SET(0X0050,u8(*p_prog & 0xff)),p_prog++;        break;  
                    case 0XED: RST(0X0050,u8(*p_prog & 0xff)),p_prog++;            break;  
                    
                    case 0x2E: LD(c_base,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x3E: LDI(c_base,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x4E: AND(c_base,u8(*p_prog & 0xff)),p_prog++;            break;
                    case 0x5E: ANI(c_base,u8(*p_prog & 0xff)),p_prog++;            break;
                    case 0x6E: OR(c_base,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x7E: ORI(c_base,u8(*p_prog & 0xff)),p_prog++;            break;  
                    
                    case 0x2F: LD(0X00E0,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x3F: LDI(0X00E0,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0x4F: AND(0X00E0,u8(*p_prog & 0xff)),p_prog++;            break;
                    case 0x5F: ANI(0X00E0,u8(*p_prog & 0xff)),p_prog++;            break;
                    case 0x6F: OR(0X00E0,u8(*p_prog & 0xff)),p_prog++;             break;  
                    case 0x7F: ORI(0X00E0,u8(*p_prog & 0xff)),p_prog++;            break;  
                    case 0XCF: _OUT(0X00E0,u8(*p_prog & 0xff)),p_prog++;           break;  
                    case 0XDF: BIT_SET(0X00E0,u8(*p_prog & 0xff)),p_prog++;        break;  
                    case 0XEF: RST(0X00E0,u8(*p_prog & 0xff)),p_prog++;            break;  
                    
                    case 0XFF: other_function(u8(*p_prog & 0xff)),p_prog++;     break;  
                    case 0xB0: p_prog++; break;  
                    case 0x00: 
                          {

                               if ((*p_prog%0x100)==0x28)
                               {
                                   myMOV();
                                   break;
                               }
                               
                               if(((*p_prog%0x100)==0x1C)||((*p_prog%0x100)==0x0F)) 
                               goto all_end;
                      }
                          
                    default: extend_function(); break;

                }
            }while(1);


            all_end:
                
                p_prog=p_prog;

/*
            puls=0x01;       
            force_reset(0X00E0,2),force_set(0X00E0,3);          
        }
        else
        { 
            RST_Y(); 
            force_reset(0X00E0,0);   
            force_set(0X00E0,1);        
            all_data[0x070C]=0; 
            edit_prog=0;      
            puls=0;
            Run=1;    
        }
    }
    else
    {   
        RST_Y(); 
        force_reset(0X00E0,0);   
        force_set(0X00E0,1);        
        all_data[0x070C]=0; 
        edit_prog=0;      
        puls=0;
        if(Run_Flag == 0)   //ADD
        {
            Run_Flag = 1;
        }       
        Run=1;
    }   
    all_data[0X701]=0X01;       
    */
}


//--------------------------------------------------
// plc.c -- end
//--------------------------------------------------




//-------------------------------
// main
//-------------------------------

int main(int argc, char* argv[])
{
    int i = 0;
    dword rel_n;
    u16 plc_len;
    char json_token_str[64];
    //--------------------
    // 1. read config file
    //--------------------
    if (read_file_to_ram("config.json", (u8*)config_json_buf, 0x10000, &rel_n) == 0)
    {
        printf("load config.json ok, size=%d\r\n",rel_n);
        
        printf("%s\r\n",config_json_buf);


        if (mcujson_get_token(config_json_buf,"mc_uart_port",json_token_str) == 0)
        {
            printf("config.mc_uart_port:%s\r\n",json_token_str);
            strcpy(mc_uart_port,json_token_str);
        }
        else
            printf("get config.mc_uart_port fail\r\n");

    }
    else
    {
        printf("load config.json fail, size=%d\r\n",rel_n);
        return -1;
    }

    //--------------------
    // 2. init
    //--------------------


    printf("MYPLC Ver:%s\r\n",VERSION);
    plc_init();



    //--------------------
    // 3. fill plc program
    //--------------------

    if (read_file_to_ram("plc.pmw", (u8*)(x), 0x10000 + 0x15c, &rel_n) == 0)
    {
        printf("load plc.pwm ok, size=%d\r\n",rel_n);
        plc_len = u16((rel_n - 0x15c)/2);
        printf("plc_len:%d\r\n",plc_len);
    }
    else
    {
        printf("load plc.pmw fail, size=%d\r\n",rel_n);
        return -1;
    }


    //----------------------------
    // 4. load & run PLC program
    //----------------------------

    dbg1("3. load plc program from file, size=%d * word\r\n",plc_len);

    print_256bs_hex((u8*)program_start_addr,0);
    print_plc_bin(program_start_addr,plc_len);

    dbg0("4. start to execution plc program...\r\n");
    while(1)
    {
        // printf("Hello World!\n");
        // dbg1("this is debug info %d\n",i++);
        realio_to_map();

        plc_main();
        Sleep(100);
        map_to_realio();

        //dbg1("----------------- %d\n",i);
        //printf("-----------------------%d\r\n",i);
        i++;
    }
    return 0;
}

