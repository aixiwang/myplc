#ifndef _DEBUGPRINTF_H_    
#define _DEBUGPRINTF_H_    
      
#include <Windows.h>    
//#include <tchar.h>    
      

#ifdef _DEBUG    
      
#define dbg0(fmt) {char sOut[256]; sprintf(sOut,fmt);OutputDebugString(sOut);}    
#define dbg1(fmt,var) {char sOut[256];sprintf(sOut,fmt,var);OutputDebugString(sOut);}    
#define dbg2(fmt,var1,var2) {char sOut[256];sprintf(sOut,fmt,var1,var2);OutputDebugString(sOut);}    
#define dbg3(fmt,var1,var2,var3) {char sOut[256];sprintf(sOut,fmt,var1,var2,var3);OutputDebugString(sOut);}    
      
#endif    
      
#ifndef _DEBUG    
      
#define dbg0(fmt) ;    
#define dbg1(fmt, var) ;    
#define dbg2(fmt,var1,var2) ;    
#define dbg3(fmt,var1,var2,var3) ;    
      
#endif    
      
#endif