
struct
{
	uint8_t temp_unit;
	uint8_t curr_range;
	uint8_t text_format;
	

	struct
	{
		uint8_t enabled;
	
		uint8_t use_ref;
		uint8_t calib_offset_at_boot;
	
		int16_t offset;
		int16_t gain;
	
	} isense;


	struct
	{
		uint8_t enabled;
		uint8_t balance_at; // 1=at full, 2=at alt full, 3=at both
	
		uint8_t current_level;
		uint16_t max_time;
		uint16_t time_per_mv;
	
	} balancer;
	
	struct
	{
		int8_t cha_min_temp_locurr;
		int8_t cha_min_temp_hicurr;
		int8_t cha_max_temp_locurr;
		int8_t cha_max_temp_hicurr;
		int8_t dsch_max_temp_locurr;
		int8_t dsch_max_temp_hicurr;
		int16_t hvc;
		int16_t hvc_alt;
		int16_t lvc_normal;
		int16_t lvc_min;
		int16_t zerosoc_offset;
	} limits;

} settings;