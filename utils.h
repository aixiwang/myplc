//------------------------------------
// Copyright(c) Aixi Wang 2016
//------------------------------------
#ifndef __MCUJSON_H__
#define __MCUJSON_H__

#include "stdafx.h"
#include <windows.h>
#include <stdio.h>

typedef		unsigned char	byte;
typedef		unsigned char	uchar;
typedef     unsigned short  word;
typedef     unsigned short  ushort;
typedef		unsigned long	dword;

typedef		unsigned char	u8;
typedef     unsigned short  u16;
typedef		unsigned long	u32;


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


#define MCUJSON_ERROR_STR_TOO_LONG    -2
#define MCUJSON_ERROR_NORMAL          -1
#define MCUJSON_OK                    0


void mcujson_debug(char* p);

int 	dump_ram_to_file(char * str_filename, byte * addr, dword bytes);
int 	read_file_to_ram(char * str_filename, byte * addr, dword bytes, dword* rel_bytes);
void 	print_256bs_hex (byte * p_byte_buffer);
byte	uart_has_char(void);
byte 	uart_rx(byte* p_byte);
byte 	uart_tx(byte c);
void	delayms(dword i);
byte 	hex_to_bin(char value);
char 	hexit(unsigned char value);
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


#endif
