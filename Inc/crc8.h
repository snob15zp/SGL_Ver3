
	#define POLYNOMIAL 0x07  /* CRC8_CCITT -- this polynomial needs to match choice on javascript end */
	#define WIDTH  
	#define TOPBIT (1 << (WIDTH - 1))
	

	
	char get_crc8(char const message[], int nBytes);