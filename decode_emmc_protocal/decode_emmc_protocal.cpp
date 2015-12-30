// decode_emmc_protocal.cpp : 定义控制台应用程序的入口点。
//
using namespace std;
#include "stdafx.h"
#include "windows.h"
#include <fstream>
#include <string>
using namespace std;

#define MAX_SIGNAL_LINES							64

#define EMMC_CMD_BITS_LENGTH				48
#define EMMC_RESPONSE_BITS_LENGTH		136

#define UP_ALIGN_2_BYTE(dat)					((dat+7)/8)

#define MAX_RESPONSE_MULTI						64

#define DECODE_BUFFER_SIZE						190

enum emmc_cmd_status_e{
	STATUS_WAIT_CMD_HIGH,
	STATUS_WAIT_CMD_START_BIT,
	STATUS_WAIT_CMD_PROCESS,
	STATUS_WAIT_CMD_END,
	STATUS_WAIT_RESP_START_BIT,
	STATUS_WAIT_RESP_PROCESS,
	STATUS_WAIT_RESP_END_BIT
};

typedef struct signal_information{
	wchar_t name[64];
	double last_timestamp;
	bool last_status;
	size_t signal_number;
}signal_information_t;

typedef struct debug_info{
	DWORD line_count;
	wchar_t lg_name[128];
	wchar_t product_id[128];
	DWORD sample_rate;
	/* Used for debug information, when get the first cmd0, 
	   the cmd line should not change accidentally; the program
	   will append debug information to the output file as warning */
	bool get_first_cmd0;
}debug_info_t;

typedef struct repeated_cmd_info{
	bool valid;
	unsigned char cmd_response[(136+48)/8];
	unsigned char length;
	unsigned char repeated_count;
}repeated_cmd_info_t;

/* Because the logic analyzer has an accuracy of 5ns, so to fix it is necessary */
typedef struct debug_fix_location{
	double time_stamp[DECODE_BUFFER_SIZE];
	bool line_status[DECODE_BUFFER_SIZE];
	unsigned int length;
	unsigned int rw_cmd_flg;
	unsigned int rw_cmd_index;
	unsigned int last_cmd23_arg;
	unsigned int last_cmd35_36_length;
	double rw_time_stamp1, rw_time_stamp2;
	double last_timestamp_of_cmd0;
}debug_fix_location_t;

wchar_t *lg_find_file_mask = _T("*.txt");
wchar_t *lg_valid_name = _T("Acute Logic Analyzer");
wchar_t *lg_valid_product_id = _T("Product ID");
wchar_t *lg_valid_sampling_rate = _T("Sampling Rate");
wchar_t *lg_valid_timestamp = _T("Timestamp");
wchar_t lg_valid_delimiter = _T(',');
wchar_t *lg_valid_cmd_name = _T("CMD");
wchar_t *lg_valid_low = _T("0");
wchar_t *lg_valid_high = _T("1");

wchar_t *filename_protocal = _T("protocal.dec");
wchar_t *filename_earse_interval = _T("earse_interval.dec");
wchar_t *filename_write_interval = _T("write_interval.dec");
wchar_t *filename_read_interval = _T("read_interval.dec");

u8 crc7(u8 crc, const u8 *buffer, size_t len);
int decode_emmc_protocal_file(wchar_t *filename);
int decode_emmc_protocal_file_2(wchar_t *filename);


#define __BIG_INTERVAL__				100000000
//#define __APPEND_WARRING__
#define __DUMP_PHY_ADDRESS__
//#define __RE_GENERATE_AGAIN__
//#define DEBUG_CMD
//#define CMD_PERIOD_FIX
#define __COMPRESS_REPEATED_COMMAND__
#define __DUMP_RW_TIME_INTERVAL

#ifdef DEBUG_CMD
#define debug_cmd		printf
#else
__inline void debug_cmd(__in_z __format_string const char * _Format, ...){
}
#endif

double convert_timestamp_string(wchar_t *str)
{
	double fraction = 0;
	long interger = 0;
	long multi = 0;
	bool fraction_flag = false;
	bool minus_flag = false;
	int minus_level;

	while(*str){
		switch(*str){
			case _T('.'):
				fraction_flag = true;
				minus_level = 10;
				break;
			case _T('-'):
				minus_flag = true;
				break;
			case _T(' '):
				break;			
			case _T('N'):
			case _T('n'):
				str++;
				if((*str ==  _T('S')) || (*str ==  _T('s'))){
					multi = 1;
				}else{
					printf("decode error timestamp number");
					return -1;
				}
				break;
			case _T('u'):
			case _T('U'):
				str++;
				if((*str ==  _T('S')) || (*str ==  _T('s'))){
					multi = 1000;
				}else{
					printf("decode error timestamp number");
					return -1;
				}
				break;
			case _T('M'):
			case _T('m'):
				str++;
				if((*str ==  _T('S')) || (*str ==  _T('s'))){
					multi = 1000000;
				}else{
					printf("decode error timestamp number");
					return -1;
				}
				break;
			case _T('S'):
			case _T('s'):
				multi = 1000000000;
				break;
			default:
				if((*str <= _T('9')) && (*str >= _T('0'))){
					if(fraction_flag == false){
						interger = interger * 10 + (*str - _T('0'));
					}else{
						fraction += ((double)*str - _T('0')) / minus_level;
						minus_level *= 10;
					}
				}else{
					printf("decode error timestamp number");
					return -1;
				}
				break;
		}
		str++;
	}

	return (((fraction + interger)  * ((minus_flag == true)?-1:1)) * multi);
}

void set_buffer_bit(unsigned char *buf, int pos)
{
	buf[pos/8] |= 1<<(pos%8);
}

void clr_buffer_bit(unsigned char *buf, int pos)
{
	buf[pos/8] &= ~(1<<(pos%8));
}

bool generate_data_again(debug_fix_location_t *loc, int period, unsigned char *dat, unsigned int max_length)
{
#ifndef __RE_GENERATE_AGAIN__
	return false;
#else
	unsigned int i;
	unsigned int j;
	double temp_interval;
	int dat_offset;
	unsigned  int current_length;
	int period_fix;
	int current_line_status = 0;
	unsigned char tmp_buf[EMMC_RESPONSE_BITS_LENGTH/8 - 1];
	unsigned char crc_val;
	int fraction;
	int period_fix_list[] = {0x0, 0x3, 0x5, -0x3, -0x5};

	for(int k=0; k<sizeof(period_fix_list)/sizeof(int); k++){
		dat_offset = 0;
		period_fix = period + period_fix_list[k];
		current_line_status = 0;

		debug_cmd("Retry %d times, period: %x\n", k, period_fix);

		for(i=1; i<loc->length; i++){
			temp_interval = loc->time_stamp[i] - loc->time_stamp[i-1];
			if(temp_interval < period_fix/2){
				return false;
			}
			current_length = (int)(temp_interval / period_fix);
			fraction = (int)(temp_interval - (current_length*period_fix));
			if(fraction > period_fix/2){
				current_length++;
			}else if((fraction > period_fix/3) && (current_length > 10)){
				current_length++;
			}

			debug_cmd(current_line_status?"Get %d bits set\n":"Get %d bits clr\n", current_length);

			if(current_length > (max_length*8 - dat_offset)){
				current_length = max_length*8 - dat_offset;
			}
			for(j=0; j<current_length; j++){
				if(current_line_status == 0){
					clr_buffer_bit(dat, max_length*8-1-dat_offset-j);
				}else{
					set_buffer_bit(dat, max_length*8-1-dat_offset-j);
				}
			}
			dat_offset += current_length;
			current_line_status = !current_line_status;
		}
		for(i=0; i<max_length; i++){
			tmp_buf[i] = dat[max_length-1-i];
		}
		/* re calculate crc7 */
		crc_val = crc7(0, tmp_buf, max_length-1);
		if(crc_val == (dat[0]>>1)){
			return true;
		}
	}	

	return false;
#endif
}

bool decode_data_from_buffer(debug_fix_location_t *loc, unsigned char *dat, unsigned int *offset, unsigned int *cmd_length)
{
	unsigned int i;
	unsigned int j;
	double temp_interval;
	int dat_offset;
	unsigned  int current_length;
	int period;
	int period_fix;
	bool current_line_status = 0;
	unsigned char tmp_buf[EMMC_RESPONSE_BITS_LENGTH/8];
	unsigned char crc_val;
	int fraction;
	int period_fix_list[] = {0x0, 0x5, -0x5, 0x4, -0x4, 0x3, -0x3, 0x2, -0x2, 0x1, -0x1};
	unsigned int max_length_cmd = 6;
	unsigned int max_length_response = 17;
	bool need_decode_response = false;
	bool need_decode_short_response = true;
	unsigned int loc_offset;
	int k;
	int cmd_index;
	int accuracy_period = 0x0;
	int accuracy_length = 0x0;
	int accuracy_pos;

	*cmd_length = 0;
	*offset = 0x2;

	if(loc->length < 0x3){
		*offset = loc->length;
		return false;
	}	

	/* Get period first */
	if(loc->line_status[0] != 0x0){
		debug_cmd("Get the error line_status[0]:%d\n", loc->line_status[0]);
		*offset = 0x1;
		return false;
	}

	if(loc->line_status[1] != 0x1){
		debug_cmd("Get the error line_status[1]:%d\n", loc->line_status[1]);
		*offset = 0x2;
		return false;
	}

	debug_cmd("Decode command\n");
	period = (int)(loc->time_stamp[1] - loc->time_stamp[0]);
	if((loc->time_stamp[1] - loc->time_stamp[0] - period) > 0.5){
		period++;
	}
	for(k=0; k<sizeof(period_fix_list)/sizeof(int); k++){		
		period_fix = period + period_fix_list[k];		

		for(int one_time_retry=0; one_time_retry<8; one_time_retry++){
			debug_cmd("Retry %d times, period: %d\n", k*8+one_time_retry, period_fix);
			accuracy_length = 0x0;		
			dat_offset = 0;
			current_line_status = 0x0;
			for(i=1; i<loc->length; i++){
				temp_interval = (int)(loc->time_stamp[i] - loc->time_stamp[i-1]);			
				if(temp_interval < period_fix/2){
					debug_cmd("interval error, current period: %d, interval: %d\n", period_fix, temp_interval);
					goto NEXT_PARSE_COMMAND;
				}
				current_length = (int)(temp_interval / period_fix);
				fraction = (int)(temp_interval - (current_length*period_fix));
				if(one_time_retry == 5){
					if(current_length < 10){
						if(fraction > period_fix/2){
							current_length++;
						}
					}
				}else if(one_time_retry == 6){
					if(current_length < 15){
						if(fraction > period_fix/2){
							current_length++;
						}
					}
				}else{
					if(fraction > period_fix/2){
						current_length++;
					}
				}				
			
				if((one_time_retry == 0) && (current_length >= 10)){
					current_length++;
				}else if((one_time_retry == 1) && (current_length >= 10)){
					current_length--;
				}else if((one_time_retry == 2) && (current_length >= 10)){
					current_length+=2;
				}else if((one_time_retry == 3) && (current_length >= 10)){
					current_length-=2;
				}else if((one_time_retry == 4) && (current_length >= 10)){
					current_length++;
				}				

				if(current_length == 0){
					debug_cmd("cmd length calculate error, current length: %d, fraction: %d\n", current_length, fraction);
					goto NEXT_PARSE_COMMAND;
				}
				debug_cmd(current_line_status?"Get %d bits set\n":"Get %d bits clr\n", current_length);			

				if(current_length > (max_length_cmd*8 - dat_offset)){
					current_length = max_length_cmd*8 - dat_offset;
				}
				for(j=0; j<current_length; j++){
					if(current_line_status == 0){
						clr_buffer_bit(dat, max_length_cmd*8-1-dat_offset-j);
					}else{
						set_buffer_bit(dat, max_length_cmd*8-1-dat_offset-j);
					}
				}
				dat_offset += current_length;
				if(current_line_status != loc->line_status[i-1]){
					debug_cmd("Line Status Error: %d, loc offset: %d\n", current_line_status, i);
				}
				current_line_status = !current_line_status;
				if(dat_offset >= 48){
					loc_offset = i;
					break;
				}

				/* may calculate more than one times, to get the max value */
				if((dat_offset > 30) && (dat_offset<47)){
					accuracy_length = dat_offset;
					accuracy_pos = i;
				}
			}
			if(dat_offset < 48){
				/* loc not enough to decode command anymore */
				*offset = 0xffff;
				return false;
			}
			for(i=0; i<max_length_cmd; i++){
				tmp_buf[i] = dat[max_length_cmd-1-i];
			}
			debug_cmd("command 0x%x-0x%x-0x%x-0x%x-0x%x-0x%x\n", 
				tmp_buf[0], tmp_buf[1], tmp_buf[2], tmp_buf[3], tmp_buf[4], tmp_buf[5]);
			/* re calculate crc7 */
			crc_val = crc7(0, tmp_buf, max_length_cmd-1);
			/* Start bit and end bit check */
			if(((tmp_buf[0] & 0xc0) == 0x40)  && ((tmp_buf[5] & 0x01) == 0x01)){			
				/* response length calculate to make sure the command didn't parse error */
				if((loc->time_stamp[loc_offset] - loc->time_stamp[loc_offset - 1]) < 2*period_fix){
					goto NEXT_PARSE_COMMAND;
				}			
				if(crc_val == (dat[0]>>1)){
					/* decode command over */
					*offset = loc_offset;
					*cmd_length = 48;
					need_decode_response = true;
					goto PARSE_COMMAND_OVER;
				}
			}
NEXT_PARSE_COMMAND:
			;			
		}	
		if(period_fix > 100){
			k=sizeof(period_fix_list)/sizeof(int);
			break;
		}
	}
PARSE_COMMAND_OVER:
	if(k == sizeof(period_fix_list)/sizeof(int)){
		debug_cmd("command crc check error\n");
	}

	if(need_decode_response == true){
		/* if get the command successfully, the period can be calculate */
		if(accuracy_length){
			accuracy_period = (int)((loc->time_stamp[accuracy_pos] - loc->time_stamp[0])/accuracy_length);
			if(((loc->time_stamp[accuracy_pos] - loc->time_stamp[0]) - period_fix*accuracy_length) > 0.5){
				accuracy_period++;
			}			
		}
		debug_cmd("Decode response\n");
		cmd_index = dat[5] & 0x3f;
		/* response has two type, 136bits or 48 bits */
		if((loc->time_stamp[*offset] - loc->time_stamp[*offset - 1]) > (MAX_RESPONSE_MULTI*period_fix)){
			return true;
		}
		for(k=0; k<15; k++){
			dat_offset = 0;
			current_line_status = 0x0;
			need_decode_short_response = true;
			if(k == 5){
				period_fix = period_fix-5;
			}else if((k == 6) || (k == 7) || (k == 8)){
				period_fix = accuracy_period;
			}else if(k == 9){
				period_fix = accuracy_period - 1;
			}else if(k == 12){
				period_fix = accuracy_period + 1;
			}
			debug_cmd("Retry %d times, period: %d\n", k, period_fix);
			/* valid decode command response */
			for(i=*offset+1; i<loc->length; i++){
				temp_interval = loc->time_stamp[i] - loc->time_stamp[i-1];
				/* Start bit valid check */
				if(dat_offset == 0){
					if(temp_interval < (period_fix+period_fix/2)){
						return true;
					}
				}
				current_length = (int)(temp_interval / period_fix);
				fraction = (int)(temp_interval - (current_length*period_fix));
				if(fraction > period_fix/2){
					current_length++;
				}else if((k == 1) && (current_length > 10) && (fraction > period_fix/3)){
					current_length++;
				}else if((k == 2) && (current_length > 10)){
					current_length++;
				}else if((k == 3) && (current_length > 10)){
					current_length--;
				}else if(((k == 7)||(k == 10)||(k == 13)) && (current_length > 10)){
					current_length--;
				}else if(((k == 8)||(k == 11)||(k == 14)) && (current_length > 10)){
					current_length++;
				}

				if(current_length == 0){
					debug_cmd("cmd length calculate error, current length: %d, fraction: %d\n", current_length, fraction);
					return true;
				}
				debug_cmd(current_line_status?"Get %d bits set\n":"Get %d bits clr\n", current_length);

				if(current_length > (max_length_response*8 - dat_offset)){
					current_length = max_length_response*8 - dat_offset;
				}
				for(j=0; j<current_length; j++){
					if(current_line_status == 0){
						clr_buffer_bit(&dat[max_length_cmd], max_length_response*8-1-dat_offset-j);
					}else{
						set_buffer_bit(&dat[max_length_cmd], max_length_response*8-1-dat_offset-j);
					}
				}
				dat_offset += current_length;
				if(current_line_status != loc->line_status[i-1]){
					debug_cmd("Line Status Error: %d, loc offset: %d\n", current_line_status, i);
				}				
				if((cmd_index == 0x02) || (cmd_index == 0x09) || (cmd_index == 0x0a)){
					if(dat_offset >= 136){
						for(unsigned int y=0; y<max_length_response; y++){
							tmp_buf[y] = dat[max_length_response-1-y];
						}
#if 0
						/* re calculate crc7 */
						crc_val = crc7(0, tmp_buf, max_length_response-1);
						if(crc_val == (tmp_buf[max_length_response-1]>>1)){
							*cmd_length = 48 + 136;
							*offset = i;
							return true;
						}
#else
						if(tmp_buf[0] = 0x3f){
							*cmd_length = 48 + 136;
							*offset = i;
							return true;
						}
#endif				
						break;
					}							
				}else{
					if(dat_offset >= 48){
						if(need_decode_short_response == true){
							need_decode_short_response = false;
							/* decode response 48 bits type over */						
							for(int y=0; y<6; y++){
								tmp_buf[y] = dat[max_length_cmd + max_length_response-1-y];
							}
							debug_cmd("response 0x%x-0x%x-0x%x-0x%x-0x%x-0x%x\n", 
								tmp_buf[0], tmp_buf[1], tmp_buf[2], tmp_buf[3], tmp_buf[4], tmp_buf[5]);
							if(cmd_index == 0x1){
								if((tmp_buf[0] == 0x3f) && (tmp_buf[5] == 0xff)){
									*cmd_length = 96;
									*offset = i;
									return true;
								}
							}else{
								/* re calculate crc7 */
								crc_val = crc7(0, tmp_buf, 5);
								if(crc_val == (tmp_buf[5]>>1)){
									if(cmd_index != (tmp_buf[0]&0x1f)){
										continue;
									}
									*cmd_length = 96;
									*offset = i;
									return true;
								}
							}							
						}
						break;
					}
				}				
				current_line_status = !current_line_status;				
			}
			if((cmd_index == 0x02) || (cmd_index == 0x09) || (cmd_index == 0x0a)){
				if(dat_offset < 136){
					/* loc not enough to decode command anymore */
					*offset = 0xffff;
					return false;
				}			
			}else{
				if(dat_offset < 48){
					/* loc not enough to decode command anymore */
					*offset = 0xffff;
					return false;
				}		
			}
		}
		if(k == 15){
			/* response check error, but command check is pass */
			debug_cmd("response crc check error\n");
			return true;
		}
	}

	*offset = 0x2;
	*cmd_length = 0x0;
	return false;
}

int _tmain(int argc, _TCHAR* argv[])
{
	HANDLE hFileHandle;
	WIN32_FIND_DATA find_data; 
	int status = 0;
	char filename_buf[512];
	size_t filename_len;

	hFileHandle = FindFirstFile(lg_find_file_mask, &find_data);
	if(hFileHandle == NULL){
		printf("Could not find any valid txt file\n");
		return -1;
	}
	if(hFileHandle == INVALID_HANDLE_VALUE){
		printf("Find first file fail, Err Code: 0x%x\n", GetLastError());
		return -2;
	}

	do{
		filename_len = WideCharToMultiByte(CP_ACP, NULL, find_data.cFileName, (int)wcslen(find_data.cFileName), filename_buf, sizeof(filename_buf), NULL, NULL);
		if(filename_len){
			filename_buf[filename_len] = 0;
			printf("Find %s file\n", filename_buf);
			decode_emmc_protocal_file_2(find_data.cFileName);
		}else{
			printf("Wide char to Multi byte convert fail\n");
		}
	}while(FindNextFile(hFileHandle,&find_data));

	FindClose(hFileHandle);
	return status;
}

int decode_emmc_protocal_file_2(wchar_t *filename)
{
	fstream file;
	fstream file_output;
#ifdef __DUMP_RW_TIME_INTERVAL
	fstream file_read_interval;
	fstream file_write_interval;
	fstream file_earse_interval;
#endif

	wchar_t file_output_name[512];
	wchar_t file_output_interval_dir[512];
	char file_output_buffer[512];

	string line_buffer;
	debug_info_t dbg_info;
	wchar_t tmp_buffer[512];
	size_t tmp_count;
	signal_information_t **pp_signal = NULL;
	size_t pp_signal_size = 0;	/* backup the alloc mem buffer size, use for release */
	wchar_t *parment_list[MAX_SIGNAL_LINES+1];
	size_t max_line_length;

	unsigned char emmc_cmd[UP_ALIGN_2_BYTE(EMMC_CMD_BITS_LENGTH) + UP_ALIGN_2_BYTE(EMMC_RESPONSE_BITS_LENGTH)];
	unsigned int cmd_length = 0;
	int cmd_tmp_length = 0;
	emmc_cmd_status_e emmc_status = STATUS_WAIT_CMD_HIGH;
	int cmd_line = -1;
	int cmd_peroid = 0;
	double current_timestamp;
	bool wait_cmd_high = false;
	unsigned int offset;
	bool decode_status;

	debug_fix_location_t debug_fix;

	repeated_cmd_info_t repeated_info;

	/* init the debug_fix data struct */
	memset(&debug_fix, 0x0, sizeof(debug_fix));

	/*  init the repeated command data struct */
	memset(&repeated_info, 0x0, sizeof(repeated_info));

	/* Init the debug information structure */
	memset(&dbg_info, 0x0, sizeof(dbg_info));
	/* always enable */
	dbg_info.get_first_cmd0 = true;

	/* Create the output filename */
	wmemcpy(file_output_name, filename, wcslen(filename));
	{
		wchar_t *pstr = wcsrchr(file_output_name, _T('.'));
		pstr[1] = 'd';
		pstr[2] = 'e';
		pstr[3] = 'c';
		pstr[4] = 0;
	}	

	/* Create the output directory */
	wmemcpy_s(file_output_interval_dir, sizeof(file_output_interval_dir)/sizeof(WCHAR), 
		filename, wcslen(filename));
	{
		file_output_interval_dir[wcslen(filename)] = 0x0;
		wchar_t *pstr = wcsrchr(file_output_interval_dir, _T('.'));
		pstr[0] = '\\';
		pstr[1] = 0;

		_wmkdir(file_output_interval_dir);
	}
	
	wmemcpy(file_output_name, file_output_interval_dir, wcslen(file_output_interval_dir));
	file_output_name[wcslen(file_output_interval_dir)] = 0x0;
	wcscat_s(file_output_name, sizeof(file_output_name)/sizeof(WCHAR), filename_protocal);

	file.open(filename);
	file_output.open(file_output_name, ios::out);
	if((!file) || (!file_output)){
		if(!file){
			wprintf_s(_T("Open file %s fail"), filename);
		}else{
			file.close();
		}

		if(!file_output){
			wprintf_s(_T("Open file %s fail"), file_output_name);
		}else{
			file_output.close();
		}
		
		return -1;
	}
#ifdef __DUMP_RW_TIME_INTERVAL
	/* Create dir for time interval operation */
	wmemcpy(file_output_name, file_output_interval_dir, wcslen(file_output_interval_dir));
	file_output_name[wcslen(file_output_interval_dir)] = 0x0;
	wcscat_s(file_output_name, sizeof(file_output_name)/sizeof(WCHAR), filename_earse_interval);
	file_earse_interval.open(file_output_name, ios::out);

	wmemcpy(file_output_name, file_output_interval_dir, wcslen(file_output_interval_dir));
	file_output_name[wcslen(file_output_interval_dir)] = 0x0;
	wcscat_s(file_output_name, sizeof(file_output_name)/sizeof(WCHAR), filename_read_interval);
	file_read_interval.open(file_output_name, ios::out);

	wmemcpy(file_output_name, file_output_interval_dir, wcslen(file_output_interval_dir));
	file_output_name[wcslen(file_output_interval_dir)] = 0x0;
	wcscat_s(file_output_name, sizeof(file_output_name)/sizeof(WCHAR), filename_write_interval);
	file_write_interval.open(file_output_name, ios::out);
#endif
	/* Check for Header */
	for(int i=0; i<10; i++){
		if(getline(file, line_buffer)){
			if(dbg_info.lg_name[0] == NULL){
				MultiByteToWideChar(CP_ACP, NULL, line_buffer.data(), (int)line_buffer.length(), dbg_info.lg_name, sizeof(dbg_info.lg_name)/sizeof(wchar_t));
				if(wmemcmp(lg_valid_name, dbg_info.lg_name, wcslen(lg_valid_name))){
					printf("Logic analyse product not support\n");
					goto Quit;
				}
			}else{
				tmp_count = MultiByteToWideChar(CP_ACP, NULL, line_buffer.data(), (int)line_buffer.length(), tmp_buffer, sizeof(tmp_buffer)/sizeof(wchar_t) );
				tmp_buffer[tmp_count] = 0;
				if(wmemcmp(tmp_buffer, lg_valid_timestamp, wcslen(lg_valid_timestamp))){
					continue;
				}else{
					/* Create signal information */
					tmp_count = 0;
					max_line_length = wcslen(tmp_buffer);
					for(size_t j=0; j<max_line_length; j++){
						if(tmp_buffer[j] == lg_valid_delimiter){
							if(tmp_count < MAX_SIGNAL_LINES){
								tmp_buffer[j] = 0;
								parment_list[tmp_count] = &tmp_buffer[j+1];
							}else{
								break;
							}
							tmp_count++;
						}
					}
					pp_signal = new signal_information_t *[tmp_count];
					memset(pp_signal, 0x0, sizeof(signal_information_t *) * tmp_count);

					pp_signal_size = tmp_count;

					for(size_t j=0; j<tmp_count; j++){
						/* Alloc memory */
						pp_signal[j] = new signal_information_t[1];
						memset(pp_signal[j] , 0x0, sizeof(signal_information_t));
						/* Init memory */
						pp_signal[j]->signal_number = j;
						wmemcpy(pp_signal[j]->name, parment_list[j], wcslen(parment_list[j]));

						if(wmemcmp(parment_list[j], lg_valid_cmd_name, sizeof(lg_valid_cmd_name)/sizeof(wchar_t)) == 0){
							cmd_line = (int)j;
						}
					}

					break;
				}
			}
		}else{
			/* Get the first line fail, return directly*/
			printf("Get header line fail\n");
			goto Quit;
		}
	}

	if(cmd_line == -1){
		printf("Can't find any valid %s line\n",  lg_valid_cmd_name);
		goto Quit;
	}
	
	while(getline(file, line_buffer)){
		dbg_info.line_count++;
		/* Transfer the line to wide char and calculate the length */
		tmp_count = MultiByteToWideChar(CP_ACP, NULL, line_buffer.data(), (int)line_buffer.length(), tmp_buffer, sizeof(tmp_buffer)/sizeof(wchar_t) );
		tmp_buffer[tmp_count] = 0;
		max_line_length = wcslen(tmp_buffer);

		/* decode parameter to list */
		parment_list[MAX_SIGNAL_LINES] = &tmp_buffer[0];
		for(size_t i=0, j=0; i<max_line_length; i++){
			if(tmp_buffer[i] == lg_valid_delimiter){
				if(j < MAX_SIGNAL_LINES){
					tmp_buffer[i] = 0;
					parment_list[j] = &tmp_buffer[i+1];
				}else{
					break;
				}
				j++;
			}			
		}
		current_timestamp = convert_timestamp_string(parment_list[MAX_SIGNAL_LINES]);

		if(debug_fix.length < DECODE_BUFFER_SIZE){
			/* Get the Start Bit, Must be low */
			if(debug_fix.length == 0){
				if(wait_cmd_high == false){
					/* cmd line go high */
					if(wmemcmp(parment_list[cmd_line], lg_valid_high, sizeof(lg_valid_high)/sizeof(wchar_t)) == 0){					
						/* init the cmd temporary buffer */
						pp_signal[cmd_line]->last_timestamp = current_timestamp;
						pp_signal[cmd_line]->last_status = 1;

						wait_cmd_high = true;
					}else{
						continue;
					}
				}else{
					/* cmd line go low */
					if(wmemcmp(parment_list[cmd_line], lg_valid_low, sizeof(lg_valid_low)/sizeof(wchar_t)) == 0){
						/* init the cmd temporary buffer */
						pp_signal[cmd_line]->last_timestamp = current_timestamp;
						pp_signal[cmd_line]->last_status = 0x0;
						
						debug_fix.length = 0x0;
						debug_fix.time_stamp[debug_fix.length] = current_timestamp;
						debug_fix.line_status[debug_fix.length] = 0;
						debug_fix.length++;
					}
				}				
			}else{
				/* fall edge */
				if((pp_signal[cmd_line]->last_status == 0x1) &&\
					(wmemcmp(parment_list[cmd_line], lg_valid_low, sizeof(lg_valid_low)/sizeof(wchar_t)) == 0)){
						debug_fix.time_stamp[debug_fix.length] = current_timestamp;
						debug_fix.line_status[debug_fix.length] = 0;
						debug_fix.length++;

						pp_signal[cmd_line]->last_timestamp = current_timestamp;
						pp_signal[cmd_line]->last_status = 0;
				}/* rise edge */
				else if((pp_signal[cmd_line]->last_status == 0x0) &&\
					(wmemcmp(parment_list[cmd_line], lg_valid_high, sizeof(lg_valid_high)/sizeof(wchar_t)) == 0)){
						debug_fix.time_stamp[debug_fix.length] = current_timestamp;
						debug_fix.line_status[debug_fix.length] = 1;
						debug_fix.length++;

						pp_signal[cmd_line]->last_timestamp = current_timestamp;
						pp_signal[cmd_line]->last_status = 1;
				}
			}
			if(debug_fix.length == DECODE_BUFFER_SIZE){
				/* decode command from buffer */
				memset(emmc_cmd, 0x0, (136+48)/8);
				decode_status = decode_data_from_buffer(&debug_fix, emmc_cmd, &offset, &cmd_length);
				if((offset == 0xffff) || (offset == 0x0)){
					break;
				}
#ifdef __COMPRESS_REPEATED_COMMAND__
				if(repeated_info.valid == true){
					if((cmd_length == repeated_info.length) && 
						(memcmp(repeated_info.cmd_response, emmc_cmd, sizeof(emmc_cmd)) == 0)){						
							repeated_info.repeated_count++;							
					}else{
						if(repeated_info.repeated_count != 0x1){
							sprintf_s(file_output_buffer, "CMD Repeated %d times\n", repeated_info.repeated_count);
							file_output.write(file_output_buffer, (int)strlen(file_output_buffer));							
						}
						goto VALID_COMMAND_PRINTOUT;
					}
				}else{
VALID_COMMAND_PRINTOUT:
#endif
					/* print out the command decode information */
					if(cmd_length >= 48){

						unsigned int argument;

						argument = ((unsigned)emmc_cmd[4]<<24) | ((unsigned)emmc_cmd[3]<<16) | \
							((unsigned)emmc_cmd[2]<<8) | (unsigned)emmc_cmd[1];

#ifdef __DUMP_RW_TIME_INTERVAL
						unsigned int response_argument;
						if(cmd_length == 96){
							response_argument = ((unsigned)emmc_cmd[6 + 15]<<24) | ((unsigned)emmc_cmd[6 + 14]<<16) | \
							((unsigned)emmc_cmd[6 + 13]<<8) | (unsigned)emmc_cmd[6 + 12];
						}else{
							response_argument = 0x0;
						}
						if(((emmc_cmd[5]&0x3f) != 13) || 
							((emmc_cmd[5]&0x3f) == 13) && (response_argument&0x100)){
							debug_fix.rw_time_stamp2 = debug_fix.time_stamp[0];
							if(debug_fix.rw_cmd_flg == TRUE){
								debug_fix.rw_cmd_flg = FALSE;								
								sprintf_s(file_output_buffer, "Interval: %lfms, Timestamp: %lf\n", 
									(debug_fix.rw_time_stamp2 - debug_fix.rw_time_stamp1)/1000000, 
									debug_fix.time_stamp[0]);								
								switch(debug_fix.rw_cmd_index){
								case 17:
								case 18:
									file_read_interval.write(file_output_buffer, (int)strlen(file_output_buffer));
									break;
								case 24:
								case 25:
									file_write_interval.write(file_output_buffer, (int)strlen(file_output_buffer));
									break;
								case 38:
									file_earse_interval.write(file_output_buffer, (int)strlen(file_output_buffer));
									break;
								default:
									/* Do nothing */
									break;
								}
							}
						}						
#endif
						
						/* printout the command and continue */
	#ifdef __DUMP_PHY_ADDRESS__
						if(((emmc_cmd[5]&0x3f) == 17) || ((emmc_cmd[5]&0x3f) == 18) || ((emmc_cmd[5]&0x3f) == 24) || ((emmc_cmd[5]&0x3f) == 25)){
							sprintf_s(file_output_buffer, "CMD%d, Argument: 0x%x(%d), Phy addr: 0x%lx, ", \
								emmc_cmd[5]&0x3f, argument, argument, (unsigned long)argument*512);
						}else{
							sprintf_s(file_output_buffer, "CMD%d, Argument: 0x%x(%d), ", emmc_cmd[5]&0x3f, argument,argument);
						}						
	#else
						sprintf_s(file_output_buffer, "CMD%d, Argument: %x, ", emmc_cmd[5]&0x3f, argument);
	#endif
						file_output.write(file_output_buffer, (int)strlen(file_output_buffer));

						if((emmc_cmd[5]&0x3f) == 0x0){
							debug_cmd("Info");
							debug_fix.last_timestamp_of_cmd0 = debug_fix.time_stamp[0];
						}

#ifdef __DUMP_RW_TIME_INTERVAL						
						unsigned char emmc_cmd_index = emmc_cmd[5] & 0x3f;
						if(emmc_cmd_index == 23){
							debug_fix.last_cmd23_arg = argument;
						}else if((emmc_cmd_index == 35) || (emmc_cmd_index == 36)){
							if(emmc_cmd_index == 35){
								debug_fix.last_cmd35_36_length = argument;
							}else{
								//if(debug_fix.last_cmd35_36_length != 0xffffffff)
								{
									debug_fix.last_cmd35_36_length = argument - debug_fix.last_cmd35_36_length;
								}
							}
						}else{
							if((emmc_cmd_index == 17) || (emmc_cmd_index == 18) || \
								(emmc_cmd_index == 24) || (emmc_cmd_index == 25) || \
								(emmc_cmd_index == 38)){
								debug_fix.rw_cmd_flg = true;
								debug_fix.rw_cmd_index = emmc_cmd_index;
								debug_fix.rw_time_stamp1 = debug_fix.time_stamp[0];

								if((emmc_cmd_index == 17)||(emmc_cmd_index == 24)){
									sprintf_s(file_output_buffer, "CMD%d\t, Sector:0x%x, Argument: 0x%x(%d)\t", 
										emmc_cmd_index, 0x01, argument);
								}else if(emmc_cmd_index == 38){
									const char *strEarseName[] = {"Erase", "Trim", "Unknow", "Discard"};
									sprintf_s(file_output_buffer, "CMD%d\t, %s: 0x%x\t", 
										emmc_cmd_index, (argument<=0x3)?strEarseName[argument&0x3]:strEarseName[2], 
										debug_fix.last_cmd35_36_length+1);
								}else{
									sprintf_s(file_output_buffer, "CMD%d\t, Sector:0x%x, Argument: 0x%x(%d)\t", 
										emmc_cmd_index, debug_fix.last_cmd23_arg & 0xffff, argument);
								}								

								switch(emmc_cmd_index){
								case 17:
								case 18:
									file_read_interval.write(file_output_buffer, (int)strlen(file_output_buffer));
									break;
								case 24:
								case 25:
									file_write_interval.write(file_output_buffer, (int)strlen(file_output_buffer));
									break;
								case 38:
									file_earse_interval.write(file_output_buffer, (int)strlen(file_output_buffer));
									break;
								default:
									/* Do nothing */
									break;
								}
							}else{
								debug_fix.last_cmd23_arg = 0x0;
								//debug_fix.last_cmd35_36_length = 0xffffffff;
							}							
						}						
#endif
						if(cmd_length == 48){
							sprintf_s(file_output_buffer, "No Response, Timestamp: %lf\n", debug_fix.time_stamp[0]);
						}else if(cmd_length == 96){
							unsigned int argument;

							argument = ((unsigned)emmc_cmd[6 + 15]<<24) | ((unsigned)emmc_cmd[6 + 14]<<16) | \
								((unsigned)emmc_cmd[6 + 13]<<8) | (unsigned)emmc_cmd[6 + 12];
							/* printout the command and continue */
	#ifdef __APPEND_WARRING__
							int compare_argument = 0;
							switch(emmc_cmd[5]&0x3f){
								/* check status */
							case 1:
								/* all send cid */
							case 2:
								/* send csd */
							case 9:
								/* send cid */
							case 10:
								break;
								/* set address */
							case 3:
								compare_argument = 0x500;
								break;
								/* select or deselect card */
							case 7:
								compare_argument = 0x700; //maybe 0x900
								break;
								/* read */
							case 17:
							case 18:
								/* write */
							case 24:
							case 25:
								/* erase param setting */
							case 35:
							case 36:						
								/* test bus rw */
							case 14:
							case 19:
								/* set block len */
							case 16:
								/* set block count */
							case 23:
								compare_argument = 0x900;							
								break;
							case 12:
								compare_argument = 0x900;
								break;
								/* setting extcsd */
							case 6:
								/* earse confirm */
							case 38:
								compare_argument = 0x800;							
								break;
							default:
								compare_argument = 0x900;							
								break;
							}
							if(compare_argument){
								sprintf_s(file_output_buffer, "Response%d, Argument: 0x%x%s\n", emmc_cmd[6+16]&0x3f, argument, (argument!=compare_argument)?", Warning":"");
							}else{
								sprintf_s(file_output_buffer, "Response%d, Argument: 0x%x\n", emmc_cmd[6+16]&0x3f, argument);
							}
	#else
							sprintf_s(file_output_buffer, "Response%d, Argument: 0x%x\n", emmc_cmd[6+16]&0x3f, argument);
	#endif
						}else if(cmd_length == (136+48)){
							sprintf_s(file_output_buffer, "Response:");
							file_output.write(file_output_buffer, (int)strlen(file_output_buffer));
							for(int k = 0; k<136/8; k++){
								sprintf_s(file_output_buffer, "0x%x%s", emmc_cmd[k+6], (k==16)?"\n":"-");
								if(k != 16){
									file_output.write(file_output_buffer, (int)strlen(file_output_buffer));
								}
							}
						}
						file_output.write(file_output_buffer, (int)strlen(file_output_buffer));

						/* backup command to the repeated struct */						
						repeated_info.valid = true;
						repeated_info.repeated_count = 1;
						repeated_info.length = cmd_length;
						memcpy(repeated_info.cmd_response, emmc_cmd, sizeof(emmc_cmd));	
					}else{
						repeated_info.valid = false;
						sprintf_s(file_output_buffer, "Check timestamp %lf for more information\n", debug_fix.time_stamp[0]);
						file_output.write(file_output_buffer, (int)strlen(file_output_buffer));
					}
#ifdef __COMPRESS_REPEATED_COMMAND__
				}
#endif
				/* print out the command decode information */
				debug_fix.length -= offset;
				for(unsigned int i=0; i<debug_fix.length; i++){
					debug_fix.time_stamp[i] = debug_fix.time_stamp[i+offset];
					debug_fix.line_status[i] = debug_fix.line_status[i+offset];
				}

				if(debug_fix.time_stamp[0] >= -7975064760.){
					debug_cmd("Info");
				}
			}
		}
	}
	/* Make sure the last byte was one */
	//if(debug_fix.line_status[debug_fix.length - 1] == FALSE)
	if(debug_fix.length < DECODE_BUFFER_SIZE)
	{
		debug_fix.time_stamp[debug_fix.length] = debug_fix.time_stamp[debug_fix.length - 1] + 1000000;
		debug_fix.line_status[debug_fix.length] = TRUE;
		debug_fix.length++;
	}	

	while(1){
		/* decode command from buffer */
		decode_status = decode_data_from_buffer(&debug_fix, emmc_cmd, &offset, &cmd_length);
		if((offset == 0xffff) || (offset == 0x0)){
			break;
		}
		/* print out the command decode information */
		if(cmd_length >= 48){
			unsigned int argument;

			argument = ((unsigned)emmc_cmd[4]<<24) | ((unsigned)emmc_cmd[3]<<16) | \
				((unsigned)emmc_cmd[2]<<8) | (unsigned)emmc_cmd[1];
			/* printout the command and continue */
#ifdef __DUMP_PHY_ADDRESS__
			if(((emmc_cmd[5]&0x3f) == 17) || ((emmc_cmd[5]&0x3f) == 18) || ((emmc_cmd[5]&0x3f) == 24) || ((emmc_cmd[5]&0x3f) == 25)){
				sprintf_s(file_output_buffer, "CMD%d, Argument: 0x%x(%d), Phy addr: 0x%lx, ", \
					emmc_cmd[5]&0x3f, argument, argument, (unsigned long)argument*512);
			}else{
				sprintf_s(file_output_buffer, "CMD%d, Argument: 0x%x(%d), ", emmc_cmd[5]&0x3f, argument,argument);
			}						
#else
			sprintf_s(file_output_buffer, "CMD%d, Argument: %x, ", emmc_cmd[5]&0x3f, argument);
#endif
			file_output.write(file_output_buffer, (int)strlen(file_output_buffer));

			if(cmd_length == 48){
				sprintf_s(file_output_buffer, "No Response, Timestamp: %lf\n", debug_fix.time_stamp[0]);
			}else if(cmd_length == 96){
				unsigned int argument;

				argument = ((unsigned)emmc_cmd[6 + 15]<<24) | ((unsigned)emmc_cmd[6 + 14]<<16) | \
					((unsigned)emmc_cmd[6 + 13]<<8) | (unsigned)emmc_cmd[6 + 12];
				/* printout the command and continue */
#ifdef __APPEND_WARRING__
				int compare_argument = 0;
				switch(emmc_cmd[5]&0x3f){
					/* check status */
						case 1:
							/* all send cid */
						case 2:
							/* send csd */
						case 9:
							/* send cid */
						case 10:
							break;
							/* set address */
						case 3:
							compare_argument = 0x500;
							break;
							/* select or deselect card */
						case 7:
							compare_argument = 0x700; //maybe 0x900
							break;
							/* read */
						case 17:
						case 18:
							/* write */
						case 24:
						case 25:
							/* erase param setting */
						case 35:
						case 36:						
							/* test bus rw */
						case 14:
						case 19:
							/* set block len */
						case 16:
							/* set block count */
						case 23:
							compare_argument = 0x900;							
							break;
						case 12:
							compare_argument = 0x900;
							break;
							/* setting extcsd */
						case 6:
							/* earse confirm */
						case 38:
							compare_argument = 0x800;							
							break;
						default:
							compare_argument = 0x900;							
							break;
				}
				if(compare_argument){
					sprintf_s(file_output_buffer, "Response%d, Argument: 0x%x, %s\n", emmc_cmd[6+16]&0x3f, argument, (argument!=compare_argument)?", Warning":"");
				}else{
					sprintf_s(file_output_buffer, "Response%d, Argument: 0x%x\n", emmc_cmd[6+16]&0x3f, argument);
				}
#else
				sprintf_s(file_output_buffer, "Response%d, Argument: 0x%x\n", emmc_cmd[6+16]&0x3f, argument);
#endif
			}else if(cmd_length == (136+48)){
				sprintf_s(file_output_buffer, "Response:");
				file_output.write(file_output_buffer, (int)strlen(file_output_buffer));
				for(int k = 0; k<136/8; k++){
					sprintf_s(file_output_buffer, "0x%x%s", emmc_cmd[k+6], (k==16)?"\n":"-");
					if(k != 16){
						file_output.write(file_output_buffer, (int)strlen(file_output_buffer));
					}
				}
			}
			file_output.write(file_output_buffer, (int)strlen(file_output_buffer));
		}else{
			sprintf_s(file_output_buffer, "Check timestamp %d for more information\n", debug_fix.time_stamp[0]);
			file_output.write(file_output_buffer, (int)strlen(file_output_buffer));
		}
		/* print out the command decode information */
		debug_fix.length -= offset;
		for(unsigned int i=0; i<debug_fix.length; i++){
			debug_fix.time_stamp[i] = debug_fix.time_stamp[i+offset];
			debug_fix.line_status[i] = debug_fix.line_status[i+offset];
		}
	}
Quit:	
	/* output deubg information */
	sprintf_s(file_output_buffer, "Decode %d lines\n", dbg_info.line_count);
	file_output.write(file_output_buffer, (int)strlen(file_output_buffer));

#ifdef __DUMP_RW_TIME_INTERVAL
	file_earse_interval.close();
	file_read_interval.close();
	file_write_interval.close();
#endif

	file.close();
	file_output.close();

	/* Release the memory alloc */
	if(pp_signal){
		for(size_t i=0; i<pp_signal_size; i++){
			delete pp_signal[i];
			pp_signal[i] = NULL;
		}
		delete pp_signal;
	}

	return 0;
}

int decode_emmc_protocal_file(wchar_t *filename)
{
	fstream file;
	fstream file_output;

	wchar_t file_output_name[512];
	char file_output_buffer[512];

	streamoff current_offset;

	string line_buffer;
	debug_info_t dbg_info;
	wchar_t tmp_buffer[512];
	size_t tmp_count;
	signal_information_t **pp_signal = NULL;
	size_t pp_signal_size = 0;	/* backup the alloc mem buffer size, use for release */
	wchar_t *parment_list[MAX_SIGNAL_LINES+1];
	size_t max_line_length;

	unsigned char emmc_cmd[UP_ALIGN_2_BYTE(EMMC_CMD_BITS_LENGTH)];
	unsigned char emmc_response[UP_ALIGN_2_BYTE(EMMC_RESPONSE_BITS_LENGTH)];
	int cmd_length = 0;
	int cmd_tmp_length = 0;
	emmc_cmd_status_e emmc_status = STATUS_WAIT_CMD_HIGH;
	int cmd_line = -1;
	int cmd_peroid = 0;
	double current_timestamp;
	double backup_last_timestamp;
	int crc_val;
	bool short_response_check;	

#ifdef __BIG_INTERVAL__
	double big_interval_value;
#endif

	debug_fix_location_t debug_fix;

	/* init the debug_fix data struct */
	memset(&debug_fix, 0x0, sizeof(debug_fix));

	/* Init the debug information structure */
	memset(&dbg_info, 0x0, sizeof(dbg_info));
	/* always enable */
	dbg_info.get_first_cmd0 = true;

	/* Create the output filename */
	wmemcpy(file_output_name, filename, wcslen(filename));
	wchar_t *pstr = wcsrchr(file_output_name, _T('.'));
	pstr[1] = 'd';
	pstr[2] = 'e';
	pstr[3] = 'c';
	pstr[4] = 0;

	file.open(filename);
	file_output.open(file_output_name, ios::out);
	if((!file) || (!file_output)){
		if(!file){
			wprintf_s(_T("Open file %s fail"), filename);
		}else{
			file.close();
		}

		if(!file_output){
			wprintf_s(_T("Open file %s fail"), file_output_name);
		}else{
			file_output.close();
		}
		
		return -1;
	}

	/* Check for Header */
	for(int i=0; i<10; i++){
		if(getline(file, line_buffer)){
			if(dbg_info.lg_name[0] == NULL){
				MultiByteToWideChar(CP_ACP, NULL, line_buffer.data(), (int)line_buffer.length(), dbg_info.lg_name, sizeof(dbg_info.lg_name)/sizeof(wchar_t));
				if(wmemcmp(lg_valid_name, dbg_info.lg_name, wcslen(lg_valid_name))){
					printf("Logic analyse product not support\n");
					goto Quit;
				}
			}else{
				tmp_count = MultiByteToWideChar(CP_ACP, NULL, line_buffer.data(), (int)line_buffer.length(), tmp_buffer, sizeof(tmp_buffer)/sizeof(wchar_t) );
				tmp_buffer[tmp_count] = 0;
				if(wmemcmp(tmp_buffer, lg_valid_timestamp, wcslen(lg_valid_timestamp))){
					continue;
				}else{
					/* Create signal information */
					tmp_count = 0;
					max_line_length = wcslen(tmp_buffer);
					for(size_t j=0; j<max_line_length; j++){
						if(tmp_buffer[j] == lg_valid_delimiter){
							if(tmp_count < MAX_SIGNAL_LINES){
								tmp_buffer[j] = 0;
								parment_list[tmp_count] = &tmp_buffer[j+1];
							}else{
								break;
							}
							tmp_count++;
						}
					}
					pp_signal = new signal_information_t *[tmp_count];
					memset(pp_signal, 0x0, sizeof(signal_information_t *) * tmp_count);

					pp_signal_size = tmp_count;

					for(size_t j=0; j<tmp_count; j++){
						/* Alloc memory */
						pp_signal[j] = new signal_information_t[1];
						memset(pp_signal[j] , 0x0, sizeof(signal_information_t));
						/* Init memory */
						pp_signal[j]->signal_number = j;
						wmemcpy(pp_signal[j]->name, parment_list[j], wcslen(parment_list[j]));

						if(wmemcmp(parment_list[j], lg_valid_cmd_name, sizeof(lg_valid_cmd_name)/sizeof(wchar_t)) == 0){
							cmd_line = (int)j;
						}
					}

					break;
				}
			}
		}else{
			/* Get the first line fail, return directly*/
			printf("Get header line fail\n");
			goto Quit;
		}
	}

	if(cmd_line == -1){
		printf("Can't find any valid %s line\n",  lg_valid_cmd_name);
		goto Quit;
	}
	
	while(getline(file, line_buffer)){
		dbg_info.line_count++;
		/* Transfer the line to wide char and calculate the length */
		tmp_count = MultiByteToWideChar(CP_ACP, NULL, line_buffer.data(), (int)line_buffer.length(), tmp_buffer, sizeof(tmp_buffer)/sizeof(wchar_t) );
		tmp_buffer[tmp_count] = 0;
		max_line_length = wcslen(tmp_buffer);

		/* decode parameter to list */
		parment_list[MAX_SIGNAL_LINES] = &tmp_buffer[0];
		for(size_t i=0, j=0; i<max_line_length; i++){
			if(tmp_buffer[i] == lg_valid_delimiter){
				if(j < MAX_SIGNAL_LINES){
					tmp_buffer[i] = 0;
					parment_list[j] = &tmp_buffer[i+1];
				}else{
					break;
				}
				j++;
			}			
		}
		current_timestamp = convert_timestamp_string(parment_list[MAX_SIGNAL_LINES]);		

		switch(emmc_status){
			case STATUS_WAIT_CMD_HIGH:
				/* cmd line go high */
				if(wmemcmp(parment_list[cmd_line], lg_valid_high, sizeof(lg_valid_high)/sizeof(wchar_t)) == 0){
					emmc_status = STATUS_WAIT_CMD_START_BIT;
					/* init the cmd temporary buffer */
					pp_signal[cmd_line]->last_timestamp = current_timestamp;
					pp_signal[cmd_line]->last_status = 1;
				}
				break;
			case STATUS_WAIT_CMD_START_BIT:
				/* cmd line go low */
				if(wmemcmp(parment_list[cmd_line], lg_valid_low, sizeof(lg_valid_low)/sizeof(wchar_t)) == 0){
					emmc_status = STATUS_WAIT_CMD_PROCESS;
					/* init the cmd temporary buffer */
					pp_signal[cmd_line]->last_timestamp = current_timestamp;
					pp_signal[cmd_line]->last_status = 0x0;

					/* init first cmd argument */
					cmd_length = 0;
					cmd_peroid = 0;
					memset(emmc_cmd, 0x0, sizeof(emmc_cmd)/sizeof(char));

					current_offset = 0;

					debug_fix.length = 0x0;
					debug_fix.time_stamp[debug_fix.length++] = current_timestamp;
				}
				break;
			case STATUS_WAIT_CMD_PROCESS:
				if(cmd_length == 0){
					if(wmemcmp(parment_list[cmd_line], lg_valid_high, sizeof(lg_valid_high)/sizeof(wchar_t)) == 0){
						/* Start bit, update peroid for future use */
						cmd_peroid = (int)(current_timestamp - pp_signal[cmd_line]->last_timestamp);
#ifdef CMD_PERIOD_FIX
						/* cmd_peroid fixed */
						if(cmd_peroid <= 30){
							cmd_peroid = 20;
						}
#endif
						pp_signal[cmd_line]->last_timestamp = current_timestamp;
						pp_signal[cmd_line]->last_status = 0x1;
						cmd_length++;
						clr_buffer_bit(emmc_cmd, 47);

						/* Update the next file location use */
						current_offset = file.tellg();

						debug_cmd("Get start bit, period: %d\n", cmd_peroid);

						debug_fix.time_stamp[debug_fix.length++] = current_timestamp;
					}
				}else{
					/* falling edge */
					if((pp_signal[cmd_line]->last_status == 0x1) &&\
						(wmemcmp(parment_list[cmd_line], lg_valid_low, sizeof(lg_valid_low)/sizeof(wchar_t)) == 0)){

							/* Invalid command: assume the cmd as glitch 
							   if low last time less than cmd_peroid/2 */
							if((current_timestamp - pp_signal[cmd_line]->last_timestamp) < cmd_peroid/2){
								if(current_offset != 0){
									file.seekg(current_offset);	
								}
								emmc_status = STATUS_WAIT_CMD_HIGH;
								
								if(dbg_info.get_first_cmd0 == true){
									sprintf_s(file_output_buffer, "Get glitch between %lf~%lf timestamp\n", \
										pp_signal[cmd_line]->last_timestamp, current_timestamp);
									file_output.write(file_output_buffer, (int)strlen(file_output_buffer));
								}

								continue;
							}

#ifdef __BIG_INTERVAL__
							if((current_timestamp - pp_signal[cmd_line]->last_timestamp) >= __BIG_INTERVAL__){
								big_interval_value = current_timestamp - backup_last_timestamp;									
							}else{
								big_interval_value = 0;
							}
#endif

							cmd_tmp_length = (int)((current_timestamp - pp_signal[cmd_line]->last_timestamp + cmd_peroid / 3) / cmd_peroid);
							
							/* Invalid command: length error */
							if(cmd_tmp_length == 0x0){
								if(current_offset != 0){
									file.seekg(current_offset);	
								}
								emmc_status = STATUS_WAIT_CMD_HIGH;

								if(dbg_info.get_first_cmd0 == true){
									sprintf_s(file_output_buffer, "cmd line last high time error, Timestamp:%lf\n", \
										current_timestamp);
									file_output.write(file_output_buffer, (int)strlen(file_output_buffer));
								}
								continue;
							}

							if((cmd_tmp_length + cmd_length) > 48){
								cmd_tmp_length = 48 - cmd_length;
							}
							for(int i=0; i<cmd_tmp_length; i++){
								set_buffer_bit(emmc_cmd, 47 - cmd_length);
								cmd_length++;
							}

							/* backup last timestamp, use for response check */
							if(cmd_length == 48){
								backup_last_timestamp = pp_signal[cmd_line]->last_timestamp;
							}

							debug_cmd("Get %d bits set\n", cmd_tmp_length);

							pp_signal[cmd_line]->last_timestamp = current_timestamp;
							pp_signal[cmd_line]->last_status = 0;

							debug_fix.time_stamp[debug_fix.length++] = current_timestamp;
					}
					/* rise edge */
					else if((pp_signal[cmd_line]->last_status == 0x0) &&\
						(wmemcmp(parment_list[cmd_line], lg_valid_high, sizeof(lg_valid_high)/sizeof(wchar_t)) == 0)){
							cmd_tmp_length = (int)((current_timestamp - pp_signal[cmd_line]->last_timestamp + cmd_peroid / 3) / cmd_peroid);
							/* Invalid command */
							if(cmd_tmp_length == 0x0){
								if(current_offset != 0){
									file.seekg(current_offset);	
								}
								emmc_status = STATUS_WAIT_CMD_START_BIT;

								if(dbg_info.get_first_cmd0 == true){
									sprintf_s(file_output_buffer, "cmd line last low time error, Timestamp:%lf\n", \
										current_timestamp);
									file_output.write(file_output_buffer, (int)strlen(file_output_buffer));
								}

								continue;
							}

							if((cmd_tmp_length + cmd_length) > 48){
								cmd_tmp_length = 48 - cmd_length;
							}
							for(int i=0; i<cmd_tmp_length; i++){
								clr_buffer_bit(emmc_cmd, 47 - cmd_length);
								cmd_length++;
							}

							/* Error: the last bit must be 0x01 */
							if(cmd_length == 48){
								if(current_offset != 0){
									file.seekg(current_offset);	
								}
								emmc_status = STATUS_WAIT_CMD_START_BIT;

								if(dbg_info.get_first_cmd0 == true){
									sprintf_s(file_output_buffer, "cmd end and the last bit not 1, Timestamp:%lf\n", \
										current_timestamp);
									file_output.write(file_output_buffer, (int)strlen(file_output_buffer));
								}
								continue;
							}

							debug_cmd("Get %d bits clr\n", cmd_tmp_length);

							pp_signal[cmd_line]->last_timestamp = current_timestamp;
							pp_signal[cmd_line]->last_status = 1;

							debug_fix.time_stamp[debug_fix.length++] = current_timestamp;
					}
				}
				if(cmd_length == 48){
					/* Calculate the command crc and continue to decode command */
ReCheckCommand:
					unsigned char tmp_buf[5];
					for(int i=0; i<sizeof(tmp_buf); i++){
						tmp_buf[i] = emmc_cmd[5-i];
					}
					crc_val = crc7(0, tmp_buf, sizeof(tmp_buf));
					if(crc_val != (emmc_cmd[0]>>1)){
						if(generate_data_again(&debug_fix, cmd_peroid, emmc_cmd, 6) == true){
							goto ReCheckCommand;
						}
						if(current_offset != 0){
							file.seekg(current_offset);
						}
						emmc_status = STATUS_WAIT_CMD_START_BIT;
						if(dbg_info.get_first_cmd0 == true){
							sprintf_s(file_output_buffer, "CMD%d, crc error, origin: 0x%x calculate: 0x%x, ", emmc_cmd[5]&0x3f, emmc_cmd[0]>>1, crc_val);
							file_output.write(file_output_buffer, (int)strlen(file_output_buffer));
						}
					}else{
						unsigned int argument;

						argument = ((unsigned)emmc_cmd[4]<<24) | ((unsigned)emmc_cmd[3]<<16) | \
							((unsigned)emmc_cmd[2]<<8) | (unsigned)emmc_cmd[1];
						/* printout the command and continue */
#ifdef __DUMP_PHY_ADDRESS__
						if(((emmc_cmd[5]&0x3f) == 17) || ((emmc_cmd[5]&0x3f) == 18) || ((emmc_cmd[5]&0x3f) == 24) || ((emmc_cmd[5]&0x3f) == 25)){
							sprintf_s(file_output_buffer, "CMD%d, Argument: 0x%x(%d), Phy addr: 0x%lx, CRC:0x%x, ", \
								emmc_cmd[5]&0x3f, argument,argument, (unsigned long)argument*512, crc_val);
						}else{
							sprintf_s(file_output_buffer, "CMD%d, Argument: 0x%x(%d), CRC:0x%x, ", emmc_cmd[5]&0x3f, argument,argument, crc_val);
						}						
#else
						sprintf_s(file_output_buffer, "CMD%d, Argument: %x, CRC:%x, ", emmc_cmd[5]&0x3f, argument, crc_val);
#endif
						file_output.write(file_output_buffer, (int)strlen(file_output_buffer));

						/* setting the debug flag */
						if((emmc_cmd[5]&0x3f) == 0){
							dbg_info.get_first_cmd0 = true;
						}						

						if((emmc_cmd[5]&0x3f) == 0x02){
							printf("CMD2 found\n");
						}

						/* At this point, the command process has already enter another command start bits */
						if(((current_timestamp - backup_last_timestamp) > (MAX_RESPONSE_MULTI*cmd_peroid)) || \
							((emmc_cmd[5]&0x3f) == 0)){
							emmc_status = STATUS_WAIT_CMD_PROCESS;

							if(dbg_info.get_first_cmd0 == true){
								sprintf_s(file_output_buffer, "CMD%d no response, Timestamp:%lf\n", \
									emmc_cmd[5]&0x3f, current_timestamp);
								file_output.write(file_output_buffer, (int)strlen(file_output_buffer));
							}

							/* init first cmd argument */
							cmd_length = 0;
							cmd_peroid = 0;
							
							memset(emmc_response, 0x0, sizeof(emmc_response)/sizeof(char));

							/*  Get the next command debug fix */
							debug_fix.length = 0x0;
							debug_fix.time_stamp[debug_fix.length++] = current_timestamp;
						}else{
							/* init first cmd argument */
							cmd_length = 0;
							emmc_status = STATUS_WAIT_RESP_PROCESS;

							/*  Get the next response debug fix */
							debug_fix.length = 0x0;
							debug_fix.time_stamp[debug_fix.length++] = current_timestamp;
						}

#ifdef __BIG_INTERVAL__
						if(big_interval_value){
							sprintf_s(file_output_buffer, "\t\tBig interval: %lfS\n", big_interval_value/1000000000);
							file_output.write(file_output_buffer, (int)strlen(file_output_buffer));
						}
#endif
						
						/* init the cmd temporary buffer */
						pp_signal[cmd_line]->last_timestamp = current_timestamp;
						pp_signal[cmd_line]->last_status = 0x0;						

						current_offset = 0;						
					}
				}
				break;

			case STATUS_WAIT_RESP_PROCESS:
				if(cmd_length == 0){
					if(wmemcmp(parment_list[cmd_line], lg_valid_high, sizeof(lg_valid_high)/sizeof(wchar_t)) == 0){
						/* Start bit check */
						if((cmd_peroid*1.5) > (current_timestamp - pp_signal[cmd_line]->last_timestamp)){
							if(dbg_info.get_first_cmd0 == true){
								sprintf_s(file_output_buffer, "CMD%d no response, Timestamp:%lf\n", \
									emmc_cmd[5]&0x3f, current_timestamp);
								file_output.write(file_output_buffer, (int)strlen(file_output_buffer));
							}
							
							/* Start bit, update peroid for future use */
							cmd_peroid = (int)(current_timestamp - pp_signal[cmd_line]->last_timestamp);
							pp_signal[cmd_line]->last_timestamp = current_timestamp;
							pp_signal[cmd_line]->last_status = 0x1;

							memset(emmc_cmd, 0x0, sizeof(emmc_cmd)/sizeof(char));
							cmd_length++;
							clr_buffer_bit(emmc_cmd, 47);

							/* Update the next file location use */
							current_offset = file.tellg();

							debug_cmd("Get start bit, period: %d\n", cmd_peroid);
							emmc_status = STATUS_WAIT_CMD_PROCESS;

							/*  Get the next command debug fix */
							debug_fix.length = 0x0;
							debug_fix.time_stamp[debug_fix.length++] = current_timestamp;
							continue;
						}
						cmd_tmp_length = (int)((current_timestamp - pp_signal[cmd_line]->last_timestamp + cmd_peroid / 3) / cmd_peroid);

						pp_signal[cmd_line]->last_timestamp = current_timestamp;
						pp_signal[cmd_line]->last_status = 0x1;
						
						cmd_length = 0;
						short_response_check = false;
						for(int i=0; i<cmd_tmp_length; i++){
							clr_buffer_bit(emmc_response, EMMC_RESPONSE_BITS_LENGTH - 1 - i);
							cmd_length++;
						}

						/* Update the next file location use */
						current_offset = file.tellg();
						debug_cmd("Get response start bit, period: %d\n", cmd_peroid);

						debug_fix.time_stamp[debug_fix.length++] = current_timestamp;
					}
				}else{
					/* falling edge */
					if((pp_signal[cmd_line]->last_status == 0x1) &&\
						(wmemcmp(parment_list[cmd_line], lg_valid_low, sizeof(lg_valid_low)/sizeof(wchar_t)) == 0)){
							/* Invalid command */
							if((current_timestamp - pp_signal[cmd_line]->last_timestamp) < cmd_peroid/2){
								if(current_offset != 0){
									file.seekg(current_offset);	
								}
								emmc_status = STATUS_WAIT_CMD_HIGH;

								if(dbg_info.get_first_cmd0 == true){
									sprintf_s(file_output_buffer, "Get glitch between %lf~%lf timestamp\n", \
										pp_signal[cmd_line]->last_timestamp, current_timestamp);
									file_output.write(file_output_buffer, (int)strlen(file_output_buffer));
								}
								continue;
							}

#ifdef __BIG_INTERVAL__
							if((current_timestamp - pp_signal[cmd_line]->last_timestamp) >= __BIG_INTERVAL__){
								big_interval_value = current_timestamp - backup_last_timestamp;									
							}else{
								big_interval_value = 0;
							}
#endif

							cmd_tmp_length = (int)((current_timestamp - pp_signal[cmd_line]->last_timestamp + cmd_peroid / 3) / cmd_peroid);

							/* Invalid command */
							if(cmd_tmp_length == 0x0){
								if(current_offset != 0){
									file.seekg(current_offset);	
								}
								emmc_status = STATUS_WAIT_CMD_HIGH;

								if(dbg_info.get_first_cmd0 == true){
									sprintf_s(file_output_buffer, "cmd line last high time error, Timestamp:%lf\n", \
										current_timestamp);
									file_output.write(file_output_buffer, (int)strlen(file_output_buffer));
								}
								continue;
							}

							if((cmd_tmp_length + cmd_length) > EMMC_RESPONSE_BITS_LENGTH){
								cmd_tmp_length = EMMC_RESPONSE_BITS_LENGTH - cmd_length;
							}
							for(int i=0; i<cmd_tmp_length; i++){
								set_buffer_bit(emmc_response, EMMC_RESPONSE_BITS_LENGTH - 1 - cmd_length);
								cmd_length++;
							}

							/* backup last timestamp, use for response check */
							if((cmd_length == 48) || (cmd_length == EMMC_RESPONSE_BITS_LENGTH)){
								backup_last_timestamp = pp_signal[cmd_line]->last_timestamp;
							}

							debug_cmd("Get %d bits set\n", cmd_tmp_length);

							pp_signal[cmd_line]->last_timestamp = current_timestamp;
							pp_signal[cmd_line]->last_status = 0;

							debug_fix.time_stamp[debug_fix.length++] = current_timestamp;
					}
					/* rise edge */
					else if((pp_signal[cmd_line]->last_status == 0x0) &&\
						(wmemcmp(parment_list[cmd_line], lg_valid_high, sizeof(lg_valid_high)/sizeof(wchar_t)) == 0)){
							cmd_tmp_length = (int)((current_timestamp - pp_signal[cmd_line]->last_timestamp + cmd_peroid / 3) / cmd_peroid);
							/* Invalid command */
							if(cmd_tmp_length == 0x0){
								if(current_offset != 0){
									file.seekg(current_offset);	
								}
								emmc_status = STATUS_WAIT_CMD_START_BIT;

								if(dbg_info.get_first_cmd0 == true){
									sprintf_s(file_output_buffer, "cmd line last low time error, Timestamp:%lf\n", \
										current_timestamp);
									file_output.write(file_output_buffer, (int)strlen(file_output_buffer));
								}
								continue;
							}

							if((cmd_tmp_length + cmd_length) > EMMC_RESPONSE_BITS_LENGTH){
								cmd_tmp_length = EMMC_RESPONSE_BITS_LENGTH - cmd_length;
							}
							for(int i=0; i<cmd_tmp_length; i++){
								clr_buffer_bit(emmc_response, EMMC_RESPONSE_BITS_LENGTH - 1 - cmd_length);
								cmd_length++;
							}

							/* Error: the last bit must be 0x01 */
							if(cmd_length == EMMC_RESPONSE_BITS_LENGTH){
								debug_cmd("cmd end and the last bit not 1\n");
								if(current_offset != 0){
									file.seekg(current_offset);	
								}
								emmc_status = STATUS_WAIT_CMD_START_BIT;

								if(dbg_info.get_first_cmd0 == true){
									sprintf_s(file_output_buffer, "cmd end and the last bit not 1, Timestamp:%lf\n", \
										current_timestamp);
									file_output.write(file_output_buffer, (int)strlen(file_output_buffer));
								}
								continue;
							}

							debug_cmd("Get %d bits clr\n", cmd_tmp_length);

							pp_signal[cmd_line]->last_timestamp = current_timestamp;
							pp_signal[cmd_line]->last_status = 1;

							debug_fix.time_stamp[debug_fix.length++] = current_timestamp;
					}
				}
				if(((cmd_length >= 48) && (short_response_check == false)) || \
					(cmd_length == EMMC_RESPONSE_BITS_LENGTH)){
ReCheckResponse:
					/* Calculate the command crc and continue to decode command */
					unsigned char tmp_buf[EMMC_RESPONSE_BITS_LENGTH/8 - 1];
					if(short_response_check == false){						
						for(int i=0; i<5; i++){
							tmp_buf[i] = emmc_response[16-i];
						}
						if(emmc_response[11] != 0xff){
							crc_val = crc7(0, tmp_buf, 5);
						}else{
							crc_val = 0x7f;
						}
						emmc_response[0] = emmc_response[11];
					}else{
						for(int i=0; i<(sizeof(tmp_buf)-1); i++){
							tmp_buf[i] = emmc_response[15-i];
						}
						crc_val = crc7(0, tmp_buf, sizeof(tmp_buf) - 1);
					}					
					if(crc_val != (emmc_response[0]>>1)){
						if(generate_data_again(&debug_fix, cmd_peroid, emmc_response, 6) == true){
							for(int i=0; i<6; i++){
								emmc_response[16-i] = emmc_response[5-i];
							}
							goto ReCheckResponse;
						}
						if(short_response_check == false){
							short_response_check = true;
							continue;
						}
						
						if(current_offset != 0){
							file.seekg(current_offset);
						}
						emmc_status = STATUS_WAIT_CMD_START_BIT;

						if(dbg_info.get_first_cmd0 == true){
							sprintf_s(file_output_buffer, "crc error, origin:%x calculate %x\n", \
								emmc_response[0]>>1, crc_val);
							file_output.write(file_output_buffer, (int)strlen(file_output_buffer));
						}
					}else{
						unsigned int argument;

						argument = ((unsigned)emmc_response[15]<<24) | ((unsigned)emmc_response[14]<<16) | \
							((unsigned)emmc_response[13]<<8) | (unsigned)emmc_response[12];
						/* printout the command and continue */
#ifdef __APPEND_WARRING__
						int compare_argument = 0;
						switch(emmc_cmd[5]&0x3f){
							/* check status */
						case 1:
							/* all send cid */
						case 2:
							/* send csd */
						case 9:
							/* send cid */
						case 10:
							break;
							/* set address */
						case 3:
							compare_argument = 0x500;
							break;
							/* select or deselect card */
						case 7:
							compare_argument = 0x700; //maybe 0x900
							break;
							/* read */
						case 17:
						case 18:
							/* write */
						case 24:
						case 25:
							/* erase param setting */
						case 35:
						case 36:						
							/* test bus rw */
						case 14:
						case 19:
							/* set block len */
						case 16:
							/* set block count */
						case 23:
							compare_argument = 0x900;							
							break;
						case 12:
							compare_argument = 0x900;
							break;
							/* setting extcsd */
						case 6:
							/* earse confirm */
						case 38:
							compare_argument = 0x800;							
							break;
						default:
							compare_argument = 0x900;							
							break;
						}
						if(compare_argument){
							sprintf_s(file_output_buffer, "Response%d, Argument: 0x%x, CRC:0x%x%s\n", emmc_response[16]&0x3f, argument, crc_val, (argument!=compare_argument)?", Warning":"");
						}else{
							sprintf_s(file_output_buffer, "Response%d, Argument: 0x%x, CRC:0x%x\n", emmc_response[16]&0x3f, argument, crc_val);
						}
#else
						sprintf_s(file_output_buffer, "Response%d, Argument: 0x%x, CRC:0x%x\n", emmc_response[16]&0x3f, argument, crc_val);
#endif
						file_output.write(file_output_buffer, (int)strlen(file_output_buffer));

						emmc_status = STATUS_WAIT_CMD_PROCESS;
						/* init first cmd argument */
						cmd_length = 0;
						cmd_peroid = 0;
						memset(emmc_cmd, 0x0, sizeof(emmc_cmd)/sizeof(char));
						memset(emmc_response, 0x0, sizeof(emmc_response)/sizeof(char));

						/* init the cmd temporary buffer */
						pp_signal[cmd_line]->last_timestamp = current_timestamp;
						pp_signal[cmd_line]->last_status = 0x0;						

						current_offset = 0;

						/*  Get the next command debug fix */
						debug_fix.length = 0x0;
						debug_fix.time_stamp[debug_fix.length++] = current_timestamp;

#ifdef __BIG_INTERVAL__
						if(big_interval_value){
							sprintf_s(file_output_buffer, "\t\tBig interval: %lfS\n", big_interval_value/1000000000);
							file_output.write(file_output_buffer, (int)strlen(file_output_buffer));
						}
#endif
					}
				}
				break;

			default:
				sprintf_s(file_output_buffer, "Unknow command type %d\n", emmc_status);
				file_output.write(file_output_buffer, (int)strlen(file_output_buffer));
				emmc_status = STATUS_WAIT_CMD_HIGH;
				break;
		}
	}
Quit:	
	/* output deubg information */
	sprintf_s(file_output_buffer, "Decode %d lines\n", dbg_info.line_count);
	file_output.write(file_output_buffer, (int)strlen(file_output_buffer));

	file.close();
	file_output.close();

	/* Release the memory alloc */
	if(pp_signal){
		for(size_t i=0; i<pp_signal_size; i++){
			delete pp_signal[i];
			pp_signal[i] = NULL;
		}
		delete pp_signal;
	}

	return 0;
}
