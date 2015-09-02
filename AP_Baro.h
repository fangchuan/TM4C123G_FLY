#ifndef __AP_BARO__
#define __AP_BARO__

typedef struct _AP_Baro
{
	long last_update;
	float altitude;
} AP_Baro;

void ap_baro_init(AP_Baro *ap_baro);
long ap_baro_get_last_update(AP_Baro *ap_baro);
float ap_baro_get_altitude(AP_Baro *ap_baro);

#endif
