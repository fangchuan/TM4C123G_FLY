#include "AP_Baro.h"

void ap_baro_init(AP_Baro *ap_baro)
{
	ap_baro->last_update = 0;
	ap_baro->altitude = 0;
}

long ap_baro_get_last_update(AP_Baro *ap_baro)
{
	return ap_baro->last_update;
}

float ap_baro_get_altitude(AP_Baro *ap_baro)
{
	return ap_baro->altitude;
}
