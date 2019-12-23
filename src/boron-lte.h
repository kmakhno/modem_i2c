#ifndef BORON_LTE_H
#define BORON_LTE_H

typedef struct
{
    char version[33];
    char name[20];
    char id[20];
    uint8_t armed;
    uint16_t locInt;
    uint8_t needPhoto;
    uint8_t photoInt;
    uint8_t soundAlarm;
    uint8_t crashLog;
    uint8_t enaLog;
} ov_cgf_s;

typedef struct ov_gps_utc_t
{
  uint8_t  hour;
  uint8_t  minute;
  uint8_t  seconds;
  uint8_t  year;
  uint8_t  month;
  uint8_t  day;
} ov_gps_utc_s;

typedef struct ov_gps_loc_t
{
  float latitude;
  float longitude;
  char  lat;
  char  lon;
} ov_gps_loc_s;

typedef struct ov_gps_info_t
{
  ov_gps_utc_s gps_utc;
  ov_gps_loc_s gps_loc;
  uint8_t      gps_fix;
} ov_gps_data_s;

extern ov_cgf_s _cfg;


#endif /*BORON_LTE_H*/