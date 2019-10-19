//------------------------------------
// Copyright(c) Aixi Wang 2016
//------------------------------------
#include "utils.h"

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
    // puts(p);
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
        mcujson_debug("==============1\n");        
        mcujson_debug(json_str + off1);
        
        k1 = mcujson_lstrip_char(json_str + off1,' ');
        off1 += k1;

        k1 = mcujson_lstrip_char(json_str + off1,':');
        off1 += k1;
        
        k1 = mcujson_lstrip_char(json_str + off1,' ');
        off1 += k1;

        mcujson_debug("==============2\n");
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
            mcujson_debug("==============3\n");
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
            k2 = mcujson_find_sub_str(json_str + off1,"}");
            if (k2 > 0) {
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
    
    
}

//------------------
// mcujson_get_int
//------------------
int mcujson_get_int(char* token_str,int* ptr_val){

}

//------------------
// mcujson_get_float
//------------------
int mcujson_get_float(char* token_str,float* ptr_val){

}

//------------------
// mcujson_get_bool
//------------------
int mcujson_get_bool(char* token_str,int* ptr_val){


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
    
    int i,j;
    
     
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


//---------------------
// hex_to_bin
//---------------------
byte hex_to_bin(char value) {
    unsigned char result;
    result =value - '0';
    if ( result > 9 ) result -= ('A' - '9' - 1);
    return( result );

}

//---------------------
// hexit
//---------------------
char hexit(unsigned char value) {

    char result;
    result = value & 0x0F;
    result += '0';
    if (result > '9') result += ('A' - '9' - 1);
    return( result );

}


//--------------------------
// mcujson_find_sub_str
// return :  -1: no finding
//            x: offset
//          
//--------------------------
int mcujson_find_sub_str(char* p_str1, char* p_str2){
    int i,j;
    u32 w_str_len1, w_str_len2;


	mcujson_debug("mcujson_find_sub_str debug 0");

    w_str_len1 = mcujson_strlen(p_str1);
    w_str_len2 = mcujson_strlen(p_str2);

	printf("w1:%d,w2:%d\r\n",w_str_len1,w_str_len2);

    if (w_str_len1 < w_str_len2){
		mcujson_debug("mcujson_find_sub_str debug 1");
        return -1;
    }
        
    for(i = 0; i < (w_str_len1 - w_str_len2); i++)
    {
		printf("i:%d",i\r\n");
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
    int i,j;
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
    int i,j;
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
    int i = 0;   
    while(p[i] == c)
        i++;
    return i;
}

//--------------------------
// mcujson_strlen
//--------------------------
int mcujson_strlen(char* p){
    int i = 0;   
    while(p[i] != 0)
        i++;
    return i;
}


//--------------------------
// mcujson_add_json_header
//--------------------------
int mcujson_add_json_header(char* p1,char* p2){
    int i,j;
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
    
}

void mcujson_int2str(int i, char* p){
    sprintf(p,"%d",i);
}







#endif
