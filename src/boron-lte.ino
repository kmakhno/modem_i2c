/*
 * Project boron-lte
 * Description:
 * Author:
 * Date:
 */

/* e00fce68318ed7e8dc028d0a */

/* SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(SEMI_AUTOMATIC); */
#include <MQTT-TLS.h>
#include <ArduinoJson.h>
#include <google-maps-device-locator.h>
#include <HttpClient.h>

#define MODEM_ADDRESS 4
#define ONE_DAY_MILLIS (24 * 60 * 60 * 1000)
#define THREE_HOUR_MILLIS (10800)

#define GPS_PACKAGE_SIZE 18 //for sparkfun gps

/*TOPICS*/
#define OV_TOPIC_CFG_GET "ov/config/get"
#define OV_TOPIC_CFG_SET "ov/config/set"
#define OV_TOPIC_GPS_SET "ov/gps/set"
#define OV_TOPIC_GPS_GET "ov/gps/get"
#define OV_TOPIC_OPEN_ACTION "ov/open" //send to this topic when execute open action
#define OV_TOPIC_SHAKE_ACTION "ov/shake" //send to this topic when execute shake action

//Need for downloading almanac
#define GET_LSB_8BIT(x) (x & 0x00FF)
#define GET_MSB_8BIT(x) (x & 0xFF00) >> 8
#define EPO_BINARY_PACKET_LENGTH 191 //191 bytes 
#define EPO_RESPONSE_TIMEOUT 10000
#define TWELVE_HOURS_IN_MS   1036800

#define AMAZON_IOT_ROOT_CA_PEM                                          \
"-----BEGIN CERTIFICATE-----\r\n" \
"MIIDQTCCAimgAwIBAgITBmyfz5m/jAo54vB4ikPmljZbyjANBgkqhkiG9w0BAQsF\r\n" \
"ADA5MQswCQYDVQQGEwJVUzEPMA0GA1UEChMGQW1hem9uMRkwFwYDVQQDExBBbWF6\r\n" \
"b24gUm9vdCBDQSAxMB4XDTE1MDUyNjAwMDAwMFoXDTM4MDExNzAwMDAwMFowOTEL\r\n" \
"MAkGA1UEBhMCVVMxDzANBgNVBAoTBkFtYXpvbjEZMBcGA1UEAxMQQW1hem9uIFJv\r\n" \
"b3QgQ0EgMTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALJ4gHHKeNXj\r\n" \
"ca9HgFB0fW7Y14h29Jlo91ghYPl0hAEvrAIthtOgQ3pOsqTQNroBvo3bSMgHFzZM\r\n" \
"9O6II8c+6zf1tRn4SWiw3te5djgdYZ6k/oI2peVKVuRF4fn9tBb6dNqcmzU5L/qw\r\n" \
"IFAGbHrQgLKm+a/sRxmPUDgH3KKHOVj4utWp+UhnMJbulHheb4mjUcAwhmahRWa6\r\n" \
"VOujw5H5SNz/0egwLX0tdHA114gk957EWW67c4cX8jJGKLhD+rcdqsq08p8kDi1L\r\n" \
"93FcXmn/6pUCyziKrlA4b9v7LWIbxcceVOF34GfID5yHI9Y/QCB/IIDEgEw+OyQm\r\n" \
"jgSubJrIqg0CAwEAAaNCMEAwDwYDVR0TAQH/BAUwAwEB/zAOBgNVHQ8BAf8EBAMC\r\n" \
"AYYwHQYDVR0OBBYEFIQYzIU07LwMlJQuCFmcx7IQTgoIMA0GCSqGSIb3DQEBCwUA\r\n" \
"A4IBAQCY8jdaQZChGsV2USggNiMOruYou6r4lK5IpDB/G/wkjUu0yKGX9rbxenDI\r\n" \
"U5PMCCjjmCXPI6T53iHTfIUJrU6adTrCC2qJeHZERxhlbI1Bjjt/msv0tadQ1wUs\r\n" \
"N+gDS63pYaACbvXy8MWy7Vu33PqUXHeeE6V/Uq2V8viTO96LXFvKWlJbYK8U90vv\r\n" \
"o/ufQJVtMVT8QtPHRh8jrdkPSHCa2XV4cdFyQzR1bldZwgJcJmApzyMZFo6IQ6XU\r\n" \
"5MsI+yMRQ+hDKXJioaldXgjUkK642M4UwtBV8ob2xJNDd2ZhwLnoQdeXeGADbkpy\r\n" \
"rqXRfboQnoZsG4q5WTP468SQvvG5\r\n" \
"-----END CERTIFICATE----- "

const char amazonIoTRootCaPem[] = AMAZON_IOT_ROOT_CA_PEM;

#define CELINT_KEY_CRT_PEM                                              \
"-----BEGIN CERTIFICATE-----\r\n" \
"MIIDWTCCAkGgAwIBAgIUa8be6t6ZUjtNDIjQfQRpeP4BhpEwDQYJKoZIhvcNAQEL\r\n" \
"BQAwTTFLMEkGA1UECwxCQW1hem9uIFdlYiBTZXJ2aWNlcyBPPUFtYXpvbi5jb20g\r\n" \
"SW5jLiBMPVNlYXR0bGUgU1Q9V2FzaGluZ3RvbiBDPVVTMB4XDTE5MTIxMTEyMTEx\r\n" \
"OVoXDTQ5MTIzMTIzNTk1OVowHjEcMBoGA1UEAwwTQVdTIElvVCBDZXJ0aWZpY2F0\r\n" \
"ZTCCASIwDQYJKoZIhvcNAQEBBQADggEPADCCAQoCggEBALj53xPYapyaCZMoYodc\r\n" \
"GycTg4TmYYi1XYbi2aTVpSHNMku5XsWiw5Y0kyA0zU7Pi4uOH2LQKVLvOgJI/V7c\r\n" \
"RH2xn9HeNY9Sz1s9WQrsO8IUQNAgXIs++LeOdJ1AieaDAuUUzfFyot7Tu+eGV8tT\r\n" \
"zal05pnfyqkHAO72Fs3RNEyN6MNd4R/lSvT6wHXCLDV1+7qjOZMcGjuUXS2vjSKt\r\n" \
"iNwGfpCP7WwGGOXNXi4sAalMd1cDSpi7CKdh39J5gpceK29Bfw2ES8WI2Mop62YC\r\n" \
"pYIknSOWUL2S2HKvXD1ZlHrkyzWHddHD1VuLDsdTQz1B7dn5L88FdiPbq/jYeQXQ\r\n" \
"A58CAwEAAaNgMF4wHwYDVR0jBBgwFoAU/v1OmRoGuN1ivtKd6euhThKn4cMwHQYD\r\n" \
"VR0OBBYEFHb+hc8dVVWooTNjl1NDBx8kESL1MAwGA1UdEwEB/wQCMAAwDgYDVR0P\r\n" \
"AQH/BAQDAgeAMA0GCSqGSIb3DQEBCwUAA4IBAQAPvxDZXCCmsU3zf/tzIAAarfs8\r\n" \
"tl8FTOdClJIjA4PtDLlPsDZ5f6Ev9szwLtXrDP5MX1vH8ttLnCMKMOkKOOES8+zE\r\n" \
"24Lsq4FxlNBgVuBt7U6w2BK9uFHIUpBFXQS5unSKqgVX743lxdkWSTPH/Dq6DOY5\r\n" \
"8phIWdjMU42vxDUWdUOEAs7KhVAYqIc8tiTX3foWkc7UDQqKM80Ly2SmVBsowhNS\r\n" \
"yAUTyfoVMDoHF4B+KCIsZdTj0vR9mwgHdDPumjpFP9SZzCZcTGAfkRvls3tg8MW+\r\n" \
"NwIRD4uLF0nG3R+hEquvV863RF09ns12Whfbk29VH0WOwkyIe5hiLaYe/MwO\r\n" \
"-----END CERTIFICATE----- "

const char clientKeyCrtPem[] = CELINT_KEY_CRT_PEM;

#define CELINT_KEY_PEM                                                  \
"-----BEGIN RSA PRIVATE KEY-----\r\n" \
"MIIEowIBAAKCAQEAuPnfE9hqnJoJkyhih1wbJxODhOZhiLVdhuLZpNWlIc0yS7le\r\n" \
"xaLDljSTIDTNTs+Li44fYtApUu86Akj9XtxEfbGf0d41j1LPWz1ZCuw7whRA0CBc\r\n" \
"iz74t450nUCJ5oMC5RTN8XKi3tO754ZXy1PNqXTmmd/KqQcA7vYWzdE0TI3ow13h\r\n" \
"H+VK9PrAdcIsNXX7uqM5kxwaO5RdLa+NIq2I3AZ+kI/tbAYY5c1eLiwBqUx3VwNK\r\n" \
"mLsIp2Hf0nmClx4rb0F/DYRLxYjYyinrZgKlgiSdI5ZQvZLYcq9cPVmUeuTLNYd1\r\n" \
"0cPVW4sOx1NDPUHt2fkvzwV2I9ur+Nh5BdADnwIDAQABAoIBABVsKt+7gq1cUZuP\r\n" \
"Y8nVavedlO/BVq5kgs5qW1Zxv8woXZjTgRxWe1xb/mYOd3CXLMwey5fAD+kg95Do\r\n" \
"Lx/bVrtP5PiDSaStrIIemr9fGJSjj9YKyWz/AAMSJoNHxDDEH4O0Yx2Bb0drMing\r\n" \
"Ly7HJ9xmQ4ayml/1BbCnI4D2p53t5urvUZT6j+k4uskvSYBOavjoqVin+vp5sPlD\r\n" \
"uVHqY0jtt2IAImrOinuVgdz1ftAHwJfMquzndCIMdI0XAIWiS/xGRhHsiAhGaQBD\r\n" \
"4rZd/E06+IuRY78SAK8PBzQDWE4tYyf1hMBc5uBCyiZ5YbuA2qZtxpF3bgGcygCs\r\n" \
"vAW2eUECgYEA6ifw57kPkKypl8MfmqjTM/u3FBfYcADmOqC2gL0LCIaA0w7lT1ys\r\n" \
"6B/xFl0/Ka4AkMCzFWevOkmziMOo6jLMUhqK/JXXoUC25OxD4/aqvUrQjvtHqson\r\n" \
"g29LyQSDJg56/toNBEkWftcyXJWpCCKQgaAiEuT9Vjm6HpT0xakaDxECgYEAyjtt\r\n" \
"F3pB92fuzmAh55IrilCkbkpysedqW+yryk0XGCLCtUwfK1nbNFgKX1fLMLQtdN6V\r\n" \
"u0+XaFHscqnHbydyGzehUQPDRAYwJbHi9eM2FqcyZthwq3eCRpvrW4jkDAUDBBlY\r\n" \
"9fTSmZj/6Jbd1CJ/kRTtRdGcwCheAcWcLCmyR68CgYEAznoV5l+7p/l5Osfm0Hxa\r\n" \
"cRZfpCo6wtkoKz6YcAFC/2uLoZEbB9ZS6gVwlCX5kolLGlmyEQfy7lUbKVhPVOUi\r\n" \
"YqvhL0X6dMkZ/deRLi3O9UhJjCtsUOAd47p6e3GtLBvMvXoNzF+epI6ibB3UhcXk\r\n" \
"40kPgtXFxwj0ZA85oCqzPeECgYAUDSRZ6ZNU1odFCx8ReNq8UhaboOISZaGUD7lQ\r\n" \
"y6f4iqnHlA6bG8OyQvB0V54b7CF9rvwloFSg0U6iZ9cQXFbh//gMslTQD9UCR8r7\r\n" \
"GlYolNdYfylctvLB0X/aiY4i6vRLmb/KU0X2WMc25o6EFA+V8P89pvTl0JrRjihX\r\n" \
"8KJunQKBgH8nb9b3IKzvXus6OtsCxfJgA6GuHi8nOkmmqsoNnJsuW4sXWEPd7hn6\r\n" \
"FHTcs/31fGjy6JkFeTHfpmbGNY/NClLSbW5PvKgKtXJ6eruiUG/u3V1TaKm6PW+V\r\n" \
"whYDCAuP2BD5jkHdgceNBIIUxz5dnoAFZZrBxeWM/TMRgbOWk4X/\r\n" \
"-----END RSA PRIVATE KEY----- "

const char clientKeyPem[] = CELINT_KEY_PEM;

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
  uint16_t  year;
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

static ov_cgf_s _cfg; //тут будем хранить данные которые пришли с AWS
static ov_gps_data_s _gps; //будем хранить данные которые пришли от ESP

void awsCallback(char* topic, uint8_t* payload, unsigned int length);
void receiveEvent(int n);
void requestEvent(void);
void locationCallback(float i_lat, float i_lon, float i_accuracy);
unsigned int getLastAlmanacUpdateTime(void);
bool connectToAWS(void);
bool sendBssidToServer(void);

char AWS_endpoint[128] = "aiotk5j0bsbka-ats.iot.us-east-2.amazonaws.com";

//almanac section
bool downloadAlmanac(unsigned int currentPacket);
bool sendAlmanacToGps(void);

static char _buffer[180];

const uint8_t PREAMBLE_LSB = 0x04;
const uint8_t PREAMBLE_MSB = 0x24;
const uint8_t LENGTH_LSB = 0xBF;
const uint8_t LENGTH_MSB = 0x00;
const uint8_t PACK_TYPE_LSB = 0xD2;
const uint8_t PACK_TYPE_MSB = 0x02;
const uint8_t END_WORD_LSB = 0x0D;
const uint8_t END_WORD_MSB = 0x0A;

/************local function declaration**********/
static void createLastPackage(void);
static bool _sendAlmanacToGps(void);

/************global variables*****************/
unsigned long receivedTimeout = 0;
int rxChar = 0;
unsigned int deliveredPckg = 0;

//MQTT client(AWS_endpoint, 8883, awsCallback);

http_request_t request;
http_response_t response;
String host = "oversery.globmill.tech";
String datePath = "/almanac/date.txt";
String almanacPath = "/almanac/MTK14.EPO";
int port = 80;
http_header_t headers[] = {
    { "Accept" , "*/*"},
    { NULL, NULL } // NOTE: Always terminate headers will NULL
};
//uint32_t almanacLastTiemstamp = 0;


unsigned long lastSync = millis();
/*хранилище для данных принятых от ESP*/
uint8_t isArmed = 0;
uint16_t locInt = 0;
uint8_t needPhoto = 0;
uint8_t photoInt = 0;
uint8_t soundAlarm = 0;
uint8_t crashLog = 0;
uint8_t enableLogging = 0;
char cfgBuff[81];
unsigned int nn[10] = {0};
volatile int i = 0;
volatile int ind = 0;
volatile bool cfgReceived = false;


//store for gps data
uint8_t rxGpsData[GPS_PACKAGE_SIZE];
volatile bool gpsReceived = false;

//store for action
static uint8_t rxOpenAction = 0;
static bool isRxOpenAction = false;

//store for shake action
static uint8_t rxShakeAction = 0;
static bool isRxShakeAction = false;

//almanac timestamp
unsigned long updateAlmanacTimestampTime = 0;
typedef struct
{
    bool needAlmanac;
    bool updateAlmanacTimestamp;
    uint32_t lastTimestamp;
} almanac_timestamp_status_t;

almanac_timestamp_status_t needToUpdateAlmanac(void);
void checkAlmanacValidity(void);

char CMD[3];
volatile bool cfg = false;
volatile bool pht = false;
volatile bool gps = false;
volatile bool act = false;
volatile bool shk = false;
volatile bool dat = false;
volatile bool geo = false;

static volatile bool hasDataForESP = false; //этот флаг устанавливаеться когда есть конфигурационные данные для ESP 

GoogleMapsDeviceLocator locator; //for positioning by towers (and wifi)
bool isFix = false;
static float lat = 0.0;
static float lon = 0.0;

char bssidBuffer[10][13];
static int downloadedBssid = 0;
int bssidCnt = 0;
volatile bool bssidReadyToSend = false;

// setup() runs once, when the device is first turned on.
void setup() 
{
    if (millis() - lastSync > ONE_DAY_MILLIS) 
    {
        Particle.syncTime();
        lastSync = millis();
    }
  // Put initialization like pinMode and begin functions here.
    Serial.begin(9600);
    Serial1.begin(9600); //hardware UART
    while(!Serial);

    Wire.begin(MODEM_ADDRESS);
    Wire.onReceive(receiveEvent);
    Wire.onRequest(requestEvent);

    Cellular.setActiveSim(INTERNAL_SIM);
    Cellular.clearCredentials();
    
    /*if (connectToAWS())
    {
        Serial.println("Connected.");
    }
    else
    {
        Serial.println("Not connected.");
    } */
    checkAlmanacValidity();
    //locator.withSubscribe(locationCallback).withLocatePeriodic(60);

}

// loop() runs over and over again, as quickly as it can execute.
void loop() 
{
    //locator.loop();

/*     if (client.isConnected()) 
    {
        client.loop();
    } */

    if (cfgReceived)
    {
        //reset received flag
        cfgReceived = false;

        for (int jj = 0; jj < nn[3]; jj++)
        {
            isArmed = (uint8_t)cfgBuff[jj + nn[0] + nn[1] + nn[2]];
        }
        
        for (int jj = 0; jj < nn[4]; jj++)
        {
            locInt = (uint16_t)cfgBuff[jj + nn[0] + nn[1] + nn[2] + nn[3]];
        }
        
        for (int jj = 0; jj < nn[5]; jj++)
        {
            needPhoto = (uint8_t)cfgBuff[jj + nn[0] + nn[1] + nn[2] + nn[3] + nn[4]];
        }
        
        for (int jj = 0; jj < nn[6]; jj++)
        {
            photoInt = (uint8_t)cfgBuff[jj + nn[0] + nn[1] + nn[2] + nn[3] + nn[4] + nn[5]];
        }
        
        for (int jj = 0; jj < nn[7]; jj++)
        {
            soundAlarm = (uint8_t)cfgBuff[jj + nn[0] + nn[1] + nn[2] + nn[3] + nn[4] + nn[5] + nn[6]];
        }
        
        for (int jj = 0; jj < nn[8]; jj++)
        {
            crashLog = (uint8_t)cfgBuff[jj + nn[0] + nn[1] + nn[2] + nn[3] + nn[4] + nn[5] + nn[6] + nn[7]];
        }
        
        for (int jj = 0; jj < nn[9]; jj++)
        {
            enableLogging = (uint8_t)cfgBuff[jj + nn[0] + nn[1] + nn[2] + nn[3] + nn[4] + nn[5] + nn[6] + nn[7] + nn[8]];
        }
        //формируем JSON объект с текущими настройками
        char *buff = (char *)calloc(1024, sizeof(char));
        char *cfgV = (char *)calloc(nn[0] + 1, sizeof(uint8_t));
        char *devN = (char *)calloc(nn[1] + 1, sizeof(uint8_t));
        char *devI = (char *)calloc(nn[2] + 1, sizeof(uint8_t));
        memcpy(cfgV, cfgBuff, nn[0]);
        cfgV[nn[0] + 1] = '\0';
        memcpy(devN, cfgBuff + nn[0], nn[1]);
        devN[nn[1] + 1] = '\0';
        memcpy(devI, cfgBuff + nn[0] + nn[1], nn[2]);
        devI[nn[2] + 1] = '\0';
        snprintf(buff, 1024,
        "{\"ver\": \"%s\", \
        \"id\": \"%s\", \
        \"name\": \"%s\", \
        \"isArmed\": \"%d\", \
        \"locInt\": \"%d\", \
        \"nPhoto\": \"%d\", \
        \"pInt\": \"%d\", \
        \"sAlarm\": \"%d\", \
        \"cLog\": \"%d\", \
        \"enaLog\": \"%d\" \
        }", 
        cfgV,
        devI,
        devN,
        isArmed,
        locInt,
        needPhoto,
        photoInt,
        soundAlarm,
        crashLog,
        enableLogging);
        Serial.println(buff);

        //отправляем сформированный JSON объект на AWS
        //client.publish(OV_TOPIC_CFG_GET, buff);
        free(cfgV);
        free(devN);
        free(devI);
        free(buff);
    }


    //Обработка принятых GPS координат и отправка их на AWS сервер
    if (gpsReceived)
    {
        memset(&_gps, 0, sizeof(_gps));
        gpsReceived = false;
        int pos = 0;
        _gps.gps_utc.hour = rxGpsData[pos++];
        _gps.gps_utc.minute = rxGpsData[pos++];
        _gps.gps_utc.seconds = rxGpsData[pos++];
        _gps.gps_utc.day = rxGpsData[pos++];
        _gps.gps_utc.month = rxGpsData[pos++];
        _gps.gps_utc.year = rxGpsData[pos++];
        _gps.gps_utc.year += rxGpsData[pos++] << 8;

        int32_t lat_fixed = rxGpsData[pos++];
        lat_fixed += rxGpsData[pos++] << 8;
        lat_fixed += rxGpsData[pos++] << 16;
        lat_fixed += rxGpsData[pos++] << 24;

        _gps.gps_loc.latitude = (float)lat_fixed / 10000;

        int32_t lon_fixed = rxGpsData[pos++];
        lon_fixed += rxGpsData[pos++] << 8;
        lon_fixed += rxGpsData[pos++] << 16;
        lon_fixed += rxGpsData[pos++] << 24;

        _gps.gps_loc.longitude = (float)lon_fixed / 10000;

        _gps.gps_loc.lat = rxGpsData[pos++];
        _gps.gps_loc.lon = rxGpsData[pos++];
        _gps.gps_fix = rxGpsData[pos++];

        isFix = _gps.gps_fix;
        if (!isFix)
        {
            _gps.gps_loc.latitude = lat;
            _gps.gps_loc.longitude = lon;
        }
        

        char *buff = (char *)malloc(512 * sizeof(char));
        memset(buff, 0, 512 * sizeof(char));
        snprintf(buff, 512 * sizeof(char), 
        "{ \
        \"utc\":  \
        { \
        \"time\": \"%02u:%02u:%02u\", \
        \"data\": \"%02u.%02u.%02u\" \
        }, \
        \"loc\": \
        { \
        \"lat\": \"%0.2f\", \
        \"lon\": \"%0.2f\" \
        }, \
        \"fix\": \"%u\" \
        }",
        _gps.gps_utc.hour,
        _gps.gps_utc.minute,
        _gps.gps_utc.seconds,
        _gps.gps_utc.day,
        _gps.gps_utc.month,
        _gps.gps_utc.year,
        (_gps.gps_loc.lat == 'N') ? _gps.gps_loc.latitude : (!_gps.gps_fix) ? _gps.gps_loc.latitude : -1.0*_gps.gps_loc.latitude,
        (_gps.gps_loc.lon == 'E') ? _gps.gps_loc.longitude : (!_gps.gps_fix) ? _gps.gps_loc.longitude : -1.0*_gps.gps_loc.longitude,
        _gps.gps_fix);

        Serial.println(buff);

        //Send gps data to aws
        //client.publish(OV_TOPIC_GPS_SET, buff);

        free(buff);

    }

    //send to AWS an open action
    if (isRxOpenAction)
    {
        isRxOpenAction = false;
        char *openMsg = (char *)calloc(128, sizeof(uint8_t));
        snprintf(openMsg, 128, 
        "{ \
        \"open\": \"%d\" \
        }", 
        rxOpenAction);
        //client.publish(OV_TOPIC_OPEN_ACTION, openMsg);
        free(openMsg);
    }

    if (isRxShakeAction)
    {
        isRxShakeAction = false;
        char *shakeMsg = (char *)calloc(128, sizeof(uint8_t));
        snprintf(shakeMsg, 128, 
        "{ \
        \"shake\": \"%d\" \
        }", 
        rxShakeAction);
        //client.publish(OV_TOPIC_SHAKE_ACTION, shakeMsg);
        free(shakeMsg);
    }
    
     //надо уйти в слип
    //System.sleep(D3, RISING);

    //получаем дату альманаха каждые 3 часа
    if (millis() - updateAlmanacTimestampTime >= THREE_HOUR_MILLIS)
    {
        Serial.println("Hello");
        updateAlmanacTimestampTime = millis();
        checkAlmanacValidity();
    }

    if (bssidReadyToSend)
    {
        bssidReadyToSend = false;
        //send bsssid to server
        sendBssidToServer();
        bssidCnt = 0;
    }
    
}


void awsCallback(char* topic, uint8_t* payload, unsigned int length) 
{

    char *awsRxStr = (char *)calloc(length + 1, sizeof(uint8_t));
    memcpy(awsRxStr, payload, length);
    awsRxStr[length + 1] = '\0';
    if (strcmp(topic, OV_TOPIC_CFG_SET) == 0)
    {
        const unsigned int capacity = JSON_OBJECT_SIZE(10);
        DynamicJsonDocument doc(capacity);
        memset(&_cfg, 0, sizeof(_cfg));

        deserializeJson(doc, awsRxStr);
        strcpy(_cfg.version, doc["ver"]);
        Serial.println(_cfg.version);
        strcpy(_cfg.id, doc["id"]);
        Serial.println(_cfg.id);
        strcpy(_cfg.name, doc["name"]);
        Serial.println(_cfg.name);
        _cfg.armed = doc["isArmed"].as<uint8_t>();
        Serial.println(_cfg.armed);
        _cfg.locInt = doc["locInt"].as<uint16_t>();
        Serial.println(_cfg.locInt);
        _cfg.needPhoto = doc["nPhoto"].as<uint8_t>();
        Serial.println(_cfg.needPhoto);
        _cfg.photoInt = doc["pInt"].as<uint8_t>();
        Serial.println(_cfg.photoInt);
        _cfg.soundAlarm = doc["sAlarm"].as<uint8_t>();
        Serial.println(_cfg.soundAlarm);
        _cfg.crashLog = doc["cLog"].as<uint8_t>();
        Serial.println(_cfg.crashLog);
        _cfg.enaLog = doc["enaLog"].as<uint8_t>();
        Serial.println(_cfg.enaLog);
        hasDataForESP = true;

        doc.clear();
    }

    free(awsRxStr);
}

void receiveEvent(int n)
{
    //here we parse command from master(esp)
    if (!cfg && !pht && !gps && !act && !shk && !dat && !geo)
    {
        Wire.readBytesUntil('\r', CMD, sizeof(CMD));     
    }

    if (cfg)
    {
        for (int kk = 0; kk < n && Wire.available() > 0; kk++, ind++)
        {
            cfgBuff[ind] = Wire.read();
        }
        nn[i] = n;
        i++;
        if (i == 10) //отому что имеем 10 транзакций от мастера
        {
            cfgReceived = true; //нужен данный флаг для последующего формирования JSON объекта с текущими настройками для отправки на AWS.
            i = 0;
            ind = 0;
            cfg = false; //обнуляем когда получили все настройки
        }
    }
    else if (pht)
    {
        pht = false;
    }
    else if (gps)
    {
        gps = false;
        memset(rxGpsData, 0, sizeof(rxGpsData));
        for (int el = 0; el < n && Wire.available() > 0; el++)
        {
            rxGpsData[el] = Wire.read();
            //Serial.println(rxGpsData[el]);
            gpsReceived = true;

        }        
    }
    else if (act)
    {
        act = false;
        for (int i = 0; i < n && Wire.available() > 0; i++)
        {
            rxOpenAction = Wire.read();
            Serial.println(rxOpenAction);
            isRxOpenAction = true;
        }
        
    }
    else if (shk)
    {
        shk = false;
        for (int i = 0; i < n && Wire.available() > 0; i++)
        {
            rxShakeAction = Wire.read();
            isRxShakeAction = true;
        }
        
    }
    else if (geo)
    {
        geo = false;
        for (int i = 0; i < n && Wire.available(); i++)
        {
            if (bssidCnt != 10)
            {
                bssidBuffer[bssidCnt][i] = Wire.read();
            }           
        }
        //Serial.println((char *)(bssidBuffer + bssidCnt));
        //Serial.println();
        bssidCnt++;
        bssidReadyToSend = true;
    }
}

void requestEvent(void)
{
    if (CMD[0] == 0x52 && CMD[1] == 0x44 && CMD[2] == 0x59) //RDY == CFG
    {     
        Wire.write(0x41);
        cfg = true;
    }
    else if (CMD[0] == 0x50 && CMD[1] == 0x48 && CMD[2] == 0x54) //PHT
    {
        Wire.write(0x41);
        pht = true;
    }
    else if (CMD[0] == 0x47 && CMD[1] == 0x50 && CMD[2] == 0x53) //GPS
    {
        Wire.write(0x41);
        gps = true;
    }
    else if (CMD[0] == 0x41 && CMD[1] == 0x43 && CMD[2] == 0x54) //ACT
    {
        Wire.write(0x41);
        act = true;
    }
    else if (CMD[0] == 0x53 && CMD[1] == 0x48 && CMD[2] == 0x4B) //SHK
    {
        Wire.write(0x41);
        shk = true;
    }
    else if (CMD[0] == 0x44 && CMD[1] == 0x41 && CMD[2] == 0x54) //DAT
    {
        if (hasDataForESP)
        {
            Serial.println("has data.");
            uint8_t msgSize = strlen(_cfg.version) + strlen(_cfg.name) + strlen(_cfg.id) + 7;
            Serial.print("msg:");Serial.println(msgSize);
            Wire.write(msgSize);
        }
    }
    else if (CMD[0] == 0x47 && CMD[1] == 0x45 && CMD[2] == 0x54) //GET
    {
        hasDataForESP = false;
        Wire.write(_cfg.version);
        Wire.write(0xFF);
        Wire.write(_cfg.name);
        Wire.write(0xFF);
        Wire.write(_cfg.id);
        Wire.write(0xFF);
        Wire.write(_cfg.armed);
        Wire.write(0xFF);
        Wire.write((uint8_t *)&_cfg.locInt, 2);
        Wire.write(0xFF);
        Wire.write(_cfg.needPhoto);
        Wire.write(0xFF);
        Wire.write(_cfg.photoInt);
        Wire.write(0xFF);
        Wire.write(_cfg.soundAlarm);
        Wire.write(0xFF);
        Wire.write(_cfg.crashLog);
        Wire.write(0xFF);
        Wire.write(_cfg.enaLog);
        Wire.write(0xFF);
    }
    else if(CMD[0] == 0x47 && CMD[1] == 0x45 && CMD[2] == 0x4F)
    {
        Wire.write(0x41);
        geo = true;
    }
    
}


void locationCallback(float i_lat, float i_lon, float i_accuracy) 
{
    lat = i_lat;
    lon = i_lon;

    Serial.println(lat);
    Serial.println(lon);
    Serial.println(i_accuracy);
}

unsigned int getLastAlmanacUpdateTime()
{
    HttpClient http;
    unsigned int timestamp = 0;
    request.hostname = host;
    request.path = datePath;
    request.port = port;

    http.get(request, response, headers);
    delay(1000);
    timestamp = (unsigned int)response.body.toInt();

    return timestamp;
}

bool connectToAWS(void)
{
/*     unsigned long w = millis();
    if (client.enableTls(amazonIoTRootCaPem, sizeof(amazonIoTRootCaPem),clientKeyCrtPem, sizeof(clientKeyCrtPem), clientKeyPem, sizeof(clientKeyPem)) == 0)
    {
        Serial.println("tls enable");
        client.connect("ESPthing", "Administrator", "ichbindragau22");
        while(!client.isConnected() && millis() - w < 10000);
        if (!client.isConnected())
        {
            return false;   
        }
        client.subscribe(OV_TOPIC_CFG_SET);
    }
    else
    {
        return false;
    }
    
    return true; */
}

static void _createLastPackage(void)
{
    uint8_t pos = 0;
    uint8_t crc = 0;
    uint16_t currEpoSeq = 65535;
    uint8_t satSize = 60;
    uint8_t epoBinPacket[EPO_BINARY_PACKET_LENGTH] = {0};

    epoBinPacket[pos++] = PREAMBLE_LSB;
    epoBinPacket[pos++] = PREAMBLE_MSB;
    epoBinPacket[pos++] = LENGTH_LSB;
    crc ^= LENGTH_LSB;
    epoBinPacket[pos++] = LENGTH_MSB;
    crc ^= LENGTH_MSB;
    epoBinPacket[pos++] = PACK_TYPE_LSB;
    crc ^= PACK_TYPE_LSB;
    epoBinPacket[pos++] = PACK_TYPE_MSB;
    crc ^= PACK_TYPE_MSB;
    epoBinPacket[pos++] = GET_LSB_8BIT(currEpoSeq);
    crc ^= GET_LSB_8BIT(currEpoSeq);
    epoBinPacket[pos++] = GET_MSB_8BIT(currEpoSeq);
    crc ^= GET_MSB_8BIT(currEpoSeq);
    unsigned int ii = 0;
    for (; ii < 3*satSize; ii++, pos++)
    {
        epoBinPacket[pos] = 0;

    }    

    epoBinPacket[pos++] = crc;
    epoBinPacket[pos++] = END_WORD_LSB;
    epoBinPacket[pos++] = END_WORD_MSB;

                
    Serial1.write(epoBinPacket, sizeof(epoBinPacket));
    delay(1);

}


bool downloadAlmanac(unsigned int currentPacket)
{
    TCPClient client;
    bool con = false;
    unsigned int startByte = 0;
    unsigned int endByte = 0;

    startByte = currentPacket * 180;

    if (currentPacket == 597)
    {
        endByte = startByte + 59;
    }
    else
    {
        endByte = 180 * (currentPacket + 1) - 1;
    }
    Serial.println(startByte);
    Serial.println(endByte);

    con = client.connect("oversery.globmill.tech", 80);

    if (!con)
    {
        client.stop();
        Serial.println("Can't be connected.");
        return false;        
    }

    Serial.println("connected");
    client.print("GET ");
    client.print("/almanac/MTK14.EPO");
    client.println(" HTTP/1.1");
    client.print("Host: ");
    client.println("oversery.globmill.tech");
    client.println("Content-Disposition: attachment; filename=\"MTK14.EPO\"");
    client.println("Content-Type: application/octet-stream");
    client.print("Range: bytes=");
    client.print(startByte);
    client.print("-");
    client.println(endByte);
    client.println("Connection: close");
    client.println();

    client.flush(); //блокирует пока все данные не будут отправлены

    unsigned int bufferPosition = 0;
    unsigned long lastRead = millis();
    bool hasBody = false;
    bool skipResponseHeader = false;
    int statusCode = 0;
    int contentLength = 0;

    do
    {
        if (client.available())
        {
            if (!skipResponseHeader)
            {
                if (!client.find("HTTP/1.1"))
                {
                    break;
                }
                statusCode = client.parseInt();
                if (statusCode != 206)
                {
                    Serial.print("Status code: "); Serial.println(statusCode);
                    return false;
                }

                if (!client.find("Content-Length:"))
                {
                    break;
                }
                contentLength = client.parseInt();
                if (contentLength != 0)
                {
                    hasBody = true;
                    Serial.print("Content-Length: "); Serial.println(contentLength);
                }
                else
                {
                    Serial.println("Here return");
                    return false;
                }

                if (!client.find("\r\n\r\n"))
                {
                    break;
                }

                skipResponseHeader = true;                
            }
            
            if (hasBody)
            {
                char c = client.read();
                lastRead = millis();
                //необходимо записать данные в буффер
                _buffer[bufferPosition] = c;
                bufferPosition++;
            }
            
        }
        
    } while (client.connected() && millis() - lastRead < 5000);

    client.stop();
    Serial.println("Stop client.");

    return true;
}

static bool _sendAlmanacToGps(void)
{
    size_t sz = 107520;
    const uint8_t satSize = 60;
    uint16_t totalBinPkt = 0; //store the total number of EPO BIN packets
    uint8_t epoBinPacket[EPO_BINARY_PACKET_LENGTH] = {0};
   
    Serial1.write(0x04);
    Serial1.write(0x24);
    Serial1.write(0x0E);
    Serial1.write(static_cast<byte>(0x00));
    Serial1.write(0xFD);
    Serial1.write(static_cast<byte>(0x00));
    Serial1.write(static_cast<byte>(0x00)); //disable bin protocol
    Serial1.write(static_cast<byte>(0x00));
    Serial1.write(static_cast<byte>(0x00));
    Serial1.write(static_cast<byte>(0x00));
    Serial1.write(static_cast<byte>(0x00));
    Serial1.write(static_cast<byte>(0xF3)); //crc
    Serial1.write(static_cast<byte>(0x00));
    Serial1.write(static_cast<byte>(0x00));

    Serial1.println("$PMTK127*36");
    delay(50);
    Serial1.println("$PMTK253,1,9600*08"); //set baudrate $PMTK253,1,9600*08
    //Serial.println("$PMTK607*33");

    if (sz % 1920 != 0)
    {
        Serial.println("Verifiy error.");
        return false;
    }
    else
    {
        totalBinPkt = sz / (3 * satSize) + 1; //number bin packets without last packet take it into account
        
        uint16_t currEpoSeq = 0;
        uint16_t prevEpoSeq = 0;
        uint8_t pos, crc;

        for (unsigned int i = 0; i < totalBinPkt; i++)
        {
            pos = 0;
            crc = 0;

            epoBinPacket[pos++] = PREAMBLE_LSB;
            epoBinPacket[pos++] = PREAMBLE_MSB;
            epoBinPacket[pos++] = LENGTH_LSB;
            crc ^= LENGTH_LSB;
            epoBinPacket[pos++] = LENGTH_MSB;
            crc ^= LENGTH_MSB;
            epoBinPacket[pos++] = PACK_TYPE_LSB;
            crc ^= PACK_TYPE_LSB;
            epoBinPacket[pos++] = PACK_TYPE_MSB;
            crc ^= PACK_TYPE_MSB;
            epoBinPacket[pos++] = GET_LSB_8BIT(currEpoSeq);
            crc ^= GET_LSB_8BIT(currEpoSeq);
            epoBinPacket[pos++] = GET_MSB_8BIT(currEpoSeq);
            crc ^= GET_MSB_8BIT(currEpoSeq);
            //unsigned int ii = 0;
            //качаем альманах
            bool downloaded = downloadAlmanac(i);
            if (downloaded)
            {
                Serial.println("Downloaded.");
            }
            else
            {
                Serial.println("Was not downloaded.");
                return false;
            }
            delay(1000);

            if (i != totalBinPkt - 1)
            {
                for (int item = 0; item < 3*satSize; item++, pos++)
                {
                    epoBinPacket[pos] = _buffer[item];
                    crc ^= epoBinPacket[pos];
                }   
            }
            else
            {
                int e = 0;
                for (; e < satSize; e++, pos++)
                {
                    epoBinPacket[pos] = _buffer[e];
                    crc ^= epoBinPacket[pos];
                }
                
                for (; e < 2*satSize; e++, pos++)
                {
                    epoBinPacket[pos] = 0;
                }
            }

            memset(_buffer, 0, sizeof(_buffer));

            epoBinPacket[pos++] = crc;
            epoBinPacket[pos++] = END_WORD_LSB;
            epoBinPacket[pos++] = END_WORD_MSB;

                        
            Serial1.write(epoBinPacket, sizeof(epoBinPacket));

            prevEpoSeq = currEpoSeq;
            currEpoSeq++;
            while(!Serial1.available());

            receivedTimeout = millis();
            while (millis() - receivedTimeout < EPO_RESPONSE_TIMEOUT)
            {
                if (Serial1.available() > 0)    
                {
                    rxChar = Serial1.read();
                    while(!Serial1.available());
                    if (rxChar == 0x04 && Serial1.available() > 0)
                    {
                        rxChar = Serial1.read();
                        while(!Serial1.available());
                        if (rxChar == 0x24 && Serial1.available() > 0)
                        {
                            rxChar = Serial1.read();
                            while(!Serial1.available());
                            if (rxChar == 0x0C && Serial1.available() > 0)
                            {
                                rxChar = Serial1.read();
                                while(!Serial1.available());
                                if (rxChar == 0x00 && Serial1.available() > 0)
                                {
                                    rxChar = Serial1.read();
                                    while(!Serial1.available());
                                    if (rxChar == 0x02 && Serial1.available() > 0)
                                    {
                                        rxChar = Serial1.read();
                                        while(!Serial1.available());
                                        if (rxChar == 0x00 && Serial1.available() > 0)
                                        {
                                            rxChar = Serial1.read();
                                            //Serial.print("lsb "); Serial.println(rxChar);
                                            while(!Serial1.available());
                                            if (rxChar == GET_LSB_8BIT(prevEpoSeq) && Serial1.available() > 0)
                                            {
                                                rxChar = Serial1.read();
                                                //Serial.print("msb "); Serial.println(rxChar);
                                                while(!Serial1.available());
                                                if (rxChar == GET_MSB_8BIT(prevEpoSeq) && Serial1.available() > 0)
                                                {
                                                    rxChar = Serial1.read();
                                                    while(!Serial1.available());
                                                    if (rxChar == 1 && Serial1.available() > 0)
                                                    {
                                                        Serial.print("msg received "); Serial.println(prevEpoSeq);
                                                        delay(2);
                                                        deliveredPckg++;
                                                        break;
                                                    }
                                                    else if (rxChar == 0 && Serial1.available() > 0)
                                                    {
                                                        Serial.print("msg not received "); Serial.println(prevEpoSeq);
                                                        return false;
                                                    }
                                                    
                                                    
                                                }
                                                
                                            }
                                            
                                        }
                                        
                                    }
                                    
                                }
                                
                            }
                            
                        }
                        
                    }
                    
                }
                
            }
            
            

        }
        
        _createLastPackage();
        Serial.print("The packages was delivered: "); Serial.println(deliveredPckg);

        Serial1.write(0x04);
        Serial1.write(0x24);
        Serial1.write(0x0E);
        Serial1.write(static_cast<byte>(0x00));
        Serial1.write(0xFD);
        Serial1.write(static_cast<byte>(0x00));
        Serial1.write(static_cast<byte>(0x00)); //disable bin protocol
        Serial1.write(static_cast<byte>(0x00));
        Serial1.write(static_cast<byte>(0x00));
        Serial1.write(static_cast<byte>(0x00));
        Serial1.write(static_cast<byte>(0x00));
        Serial1.write(static_cast<byte>(0xF3)); //crc
        Serial1.write(static_cast<byte>(0x00));
        Serial1.write(static_cast<byte>(0x00));
        

        delay(500);
        //check epo status
        Serial1.println("$PMTK607*33");

        return true;
    }
}


bool sendAlmanacToGps(void)
{
    uint8_t attempts = 0;
    bool success = false;
    const unsigned int ATTEMPTS = 3;

    do
    {
        success = _sendAlmanacToGps();
        attempts++;
    } while (!success && attempts < ATTEMPTS);

    return (success && attempts < ATTEMPTS) ? true : false;    
}

almanac_timestamp_status_t needToUpdateAlmanac(void)
{
    almanac_timestamp_status_t status = {false};
    int almanacEepromAddress = 10;
    uint32_t almanacTimestampFromEeprom = 0;
    uint32_t delta = 0;
    //получаем сразу последний таймстамп обновления альманаха на сервере
    uint32_t almanacLastTiemstamp = getLastAlmanacUpdateTime();
    Serial.print("Timestamp from server: "); Serial.println(almanacLastTiemstamp);
    //считаем сохраненный таймстап с EEPROM
    EEPROM.get(almanacEepromAddress, almanacTimestampFromEeprom);
    Serial.print("Timestamp from EEPROM: "); Serial.println(almanacTimestampFromEeprom);
    //если он равен 0, тогда выставляем флаг на скачку AGPS и флаг на то чтобы обновить таймстамп при успешной загрузки AGPS
    //иначе сравниваем таймстамп с EEPROM и таймстам принятый с сервера и если разница больше чем 12 дней, тогда выставляем флаг на скачку AGPS и флаг на то чтобы обновить таймстамп при успешной загрузки AGPS
    //иначе ничего не скачиваем
    if (almanacTimestampFromEeprom == 0xFFFFFFFF)
    {
        status.needAlmanac = true;
        status.updateAlmanacTimestamp = true;
        status.lastTimestamp = almanacLastTiemstamp;
        Serial.println("The almanac need to update.");
        return status;
    }
    else
    {
        delta = almanacLastTiemstamp - almanacTimestampFromEeprom;
        if (delta >= TWELVE_HOURS_IN_MS)
        {
            status.needAlmanac = true;
            status.updateAlmanacTimestamp = true;
            status.lastTimestamp = almanacLastTiemstamp;
            Serial.println("The almanac need to update.");
            return status;
        }
    }

    Serial.println("The almanac doesn't need to update.");

    return status;
}

void checkAlmanacValidity(void)
{
    bool result = false;
    almanac_timestamp_status_t st = needToUpdateAlmanac();
    if (st.needAlmanac && st.updateAlmanacTimestamp)
    {
        result = sendAlmanacToGps();
        if (result)
        {
            EEPROM.put(10, st.lastTimestamp);       
            Serial.println("The almanac was downloaded.");
        }
        else
        {
            Serial.println("The almanac wasn't downloaded.");
        }
    }
}

bool sendBssidToServer(void)
{
    if (bssidCnt > 10)
    {
        bssidCnt = 10;
    }

    TCPClient client;
    bool con = false;

    con = client.connect("oversery.globmill.tech", 80);

    if (!con)
    {
        client.stop();
        Serial.println("Can't be connected.");
        return false;        
    }
//http://oversery.globmill.tech/devices/locate/wifi?bssid=id1,id2,id3
    Serial.println("connected");

    client.print("GET ");
    client.print("/devices/locate/wifi?bssid=");
    int i = 0;
    for (; i < bssidCnt; i++)
    {
        client.print((char *)(bssidBuffer + i));
        client.print(",");
        Serial.println((char *)(bssidBuffer + i));
    }
    client.println((char *)(bssidBuffer + i));
    client.println(" HTTP/1.1");
    client.print("Host: ");
    client.println("oversery.globmill.tech");
    client.println("Connection: close");
    client.println();

    client.flush(); //блокирует пока все данные не будут отправлены
    
    client.stop();
    delay(500);
}