#include <Particle.h>
#include "boron-lte.h"
#include "ble_modem.h"

extern bool hasDataForESP;


#define BUTTON 20
#define BATTERY_UPDATE_MS 2000

/* SYSTEM_THREAD(ENABLED);
SYSTEM_MODE(MANUAL); */

typedef struct 
{
    String deviceName;
} GENERIC_ACCESS_SVC_T;

typedef struct 
{
    uint8_t batteryLevel;
} BATTERY_SVC_T;

typedef struct
{
    String manufacturerNameString;
    String modelNumberString;
    String serialNumberString;
    String hardwareRevisionString;
    String firmwareRevisionString;
} DEVICE_INFORMATION_SVC_T;

typedef struct 
{
    String  deviceId;
    uint8_t armedStatus;
    uint16_t locationInterval;
    uint8_t photoStatus;
    uint16_t photoInterval;
    uint8_t soundAlarmStatus;
    uint8_t crashLogStatus;
    uint8_t enableLoggingStatus;
    uint8_t forceSoundAlarm;
} APP_CONFIG_PARAMETERS_T;

typedef struct
{
    uint8_t openStatus;
} DEVICE_ACTION_T;

typedef struct
{
    APP_CONFIG_PARAMETERS_T appConfigParams;
    BATTERY_SVC_T batterySvc;
    DEVICE_INFORMATION_SVC_T deviceInfSvc;
    DEVICE_ACTION_T deviceAction;
    GENERIC_ACCESS_SVC_T genericAccess;
} APP_PARAM_T;

const char *identificationUUIDStr = "01246d86-ffd3-11e9-9a9f-362b9e155667";
const char *configUUIDStr = "01246d87-ffd3-11e9-9a9f-362b9e155667";
//UUID strings for CUSTOM characteristics
const char *armedChUUIDStr = "01246d88-ffd3-11e9-9a9f-362b9e155667";
const char *locationIntervalChUUIDStr = "01246d89-ffd3-11e9-9a9f-362b9e155667";
const char *photoChUUIDStr = "01246d8a-ffd3-11e9-9a9f-362b9e155667";
const char *photoIntervalChUUIDStr = "01246d8b-ffd3-11e9-9a9f-362b9e155667";
const char *soundAlarmChUUIDStr = "01246d8c-ffd3-11e9-9a9f-362b9e155667";
const char *crashLogChUUIDStr = "01246d8d-ffd3-11e9-9a9f-362b9e155667";
const char *enableLoggingChUUIDStr = "01246d8e-ffd3-11e9-9a9f-362b9e155667";
const char *openStatusChUUIDStr = "01246d8f-ffd3-11e9-9a9f-362b9e155667";
const char *forceSoundAlarmChUUIDStr = "01246d90-ffd3-11e9-9a9f-362b9e155667";
const char *deviceIdChUUIDStr = "01246d91-ffd3-11e9-9a9f-362b9e155667";
//UUID for OUR CHARACTERISTICS

/**************GATT-services********************************************/
BleUuid genericAccessService(BLE_SIG_UUID_GENERIC_ACCESS_SVC);
BleUuid batteryService(BLE_SIG_UUID_BATTERY_SVC);
BleUuid deviceInformation(BLE_SIG_UUID_DEVICE_INFORMATION_SVC);
BleUuid identificationServiceUUID(identificationUUIDStr); //CUSTOM service UUID that use for identification ours application
BleUuid configService(configUUIDStr); //CUSTOM service UUID that use to notification central device about config parameters

//Characteristics SIG-UUID
BleUuid deviceNameChUUID(BLE_SIG_UUID_DEVICE_NAME_CHAR);
BleUuid batteryLevelChUUID(0x2A19);
BleUuid manufacturerNameStringChUUID(0x2A29);
BleUuid modelNumberStringChUUID(0x2A24);
BleUuid serialNumberStringChUUID(0x2A25);
BleUuid hardwareRevisionStringChUUID(0x2A27);
BleUuid firmwareRevisionStringChUUID(0x2A26);
//UUID for CUSTOM characteristics
BleUuid armedChUUID(armedChUUIDStr);
BleUuid locationIntervalChUUID(locationIntervalChUUIDStr);
BleUuid photoChUUID(photoChUUIDStr);
BleUuid photoIntervalChUUID(photoIntervalChUUIDStr);
BleUuid soundAlarmChUUID(soundAlarmChUUIDStr);
BleUuid crashLogChUUID(crashLogChUUIDStr);
BleUuid enableLoggingChUUID(enableLoggingChUUIDStr);
BleUuid openStatusChUUID(openStatusChUUIDStr);
BleUuid forceSoundAlarmChUUID(forceSoundAlarmChUUIDStr);
BleUuid deviceIdChUUID(deviceIdChUUIDStr);

void setAdvertisingData(void);
void btnHandler(void);
void btnInit(void);
uint8_t batteryLevelSimulate(void);
void initDeviceInformation(void);

//prototypes of callbacks 
void writeDeviceName(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context);
void writeArmed(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context);
void writeLocationInterval(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context);
void writePhoto(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context);
void writePhotoInterval(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context);
void writeSoundAlarm(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context);
void writeCrashLog(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context);
void writeEnableLogging(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context);
void writeForceSoundAlarm(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context);

//init Characteristics
BleCharacteristic deviceNameCh("devName", BleCharacteristicProperty::READ | BleCharacteristicProperty::WRITE, deviceNameChUUID, genericAccessService, writeDeviceName); //read/write 
BleCharacteristic deviceIdCh("devId", BleCharacteristicProperty::READ, deviceIdChUUID, configService);
BleCharacteristic batteryLevelCh("bat", BleCharacteristicProperty::NOTIFY, batteryLevelChUUID, batteryService);
BleCharacteristic manufacturerNameStringCh("manuf", BleCharacteristicProperty::READ, manufacturerNameStringChUUID, deviceInformation);
BleCharacteristic modelNumberStringCh("model", BleCharacteristicProperty::READ, modelNumberStringChUUID, deviceInformation);
BleCharacteristic serialNumberStringCh("serial", BleCharacteristicProperty::READ, serialNumberStringChUUID, deviceInformation);
BleCharacteristic hardwareRevisionStringCh("hard", BleCharacteristicProperty::READ, hardwareRevisionStringChUUID, deviceInformation);
BleCharacteristic firmwareRevisionStringCh("firm", BleCharacteristicProperty::READ, firmwareRevisionStringChUUID, deviceInformation);
BleCharacteristic armedCh("armed", BleCharacteristicProperty::WRITE | BleCharacteristicProperty::READ, armedChUUID, configService, writeArmed);
BleCharacteristic locationIntervalCh("locInt", BleCharacteristicProperty::WRITE | BleCharacteristicProperty::READ, locationIntervalChUUID, configService, writeLocationInterval);
BleCharacteristic photoCh("photo", BleCharacteristicProperty::WRITE | BleCharacteristicProperty::READ, photoChUUID, configService, writePhoto);
BleCharacteristic photoIntervalCh("photoInt", BleCharacteristicProperty::WRITE | BleCharacteristicProperty::READ, photoIntervalChUUID, configService, writePhotoInterval);
BleCharacteristic soundAlarmCh("sound", BleCharacteristicProperty::WRITE | BleCharacteristicProperty::READ, soundAlarmChUUID, configService, writeSoundAlarm);
BleCharacteristic crashLogCh("crash", BleCharacteristicProperty::WRITE | BleCharacteristicProperty::READ, crashLogChUUID, configService, writeCrashLog);
BleCharacteristic enableLoggingCh("enaLog", BleCharacteristicProperty::WRITE | BleCharacteristicProperty::READ, enableLoggingChUUID, configService, writeEnableLogging);
BleCharacteristic openStatusCh("openStat", BleCharacteristicProperty::NOTIFY | BleCharacteristicProperty::READ, openStatusChUUID, configService);
BleCharacteristic forceSoundAlarmCh("forceSound", BleCharacteristicProperty::WRITE | BleCharacteristicProperty::READ, forceSoundAlarmChUUID, configService, writeForceSoundAlarm);

static APP_PARAM_T peripheralParameters;
bool advertisingStart = false;
unsigned long batteryLevelUpdate = 0;


void ble_modem_init()
{
    RGB.control(true);
    Serial.println("Peripheral device.");
    btnInit();

//register Characteristics
    BLE.addCharacteristic(deviceNameCh);
    BLE.addCharacteristic(batteryLevelCh);
    BLE.addCharacteristic(manufacturerNameStringCh);
    BLE.addCharacteristic(modelNumberStringCh);
    BLE.addCharacteristic(serialNumberStringCh);
    BLE.addCharacteristic(hardwareRevisionStringCh);
    BLE.addCharacteristic(firmwareRevisionStringCh);
    BLE.addCharacteristic(armedCh);
    BLE.addCharacteristic(locationIntervalCh);
    BLE.addCharacteristic(photoCh);
    BLE.addCharacteristic(photoIntervalCh);
    BLE.addCharacteristic(soundAlarmCh);
    BLE.addCharacteristic(crashLogCh);
    BLE.addCharacteristic(enableLoggingCh);
    BLE.addCharacteristic(openStatusCh);
    BLE.addCharacteristic(forceSoundAlarmCh);
    BLE.addCharacteristic(deviceIdCh);

    initDeviceInformation();
}


void ble_modem_listener(void)
{
    if (advertisingStart)
    {
        setAdvertisingData();
    }
    

    if (BLE.connected())
    {
        RGB.color(0, 255, 0);        

        if (millis() - batteryLevelUpdate >= BATTERY_UPDATE_MS)
        {
            batteryLevelUpdate = millis();
            peripheralParameters.batterySvc.batteryLevel = batteryLevelSimulate(); //get level of charging
            batteryLevelCh.setValue(peripheralParameters.batterySvc.batteryLevel); //send current battery level
        }
        
    }
    else
    {
        RGB.color(0, 0, 255);
    }
}


/*******Local functions*******/
void setAdvertisingData(void)
{
    const char *deviceLocalName = "oversery";

    BleAdvertisingData  adv;

    advertisingStart = false;

    adv.appendServiceUUID(identificationServiceUUID);
    adv.appendLocalName(deviceLocalName);

    //BLE.setAdvertisingInterval(130); //advertise every 100ms
    BLE.setAdvertisingTimeout(1000); //advertise for 10sec
    BLE.advertise(&adv);
}

void btnInit()
{
    pinMode(BUTTON, INPUT_PULLUP);
    attachInterrupt(BUTTON, btnHandler, RISING);
}

void btnHandler()
{
    advertisingStart = true;
    RGB.color(255, 0, 0);
}


//calback realization
void writeDeviceName(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context)
{
    strlcpy(_cfg.name, (char *)data, len);
    hasDataForESP = true;
}

void writeArmed(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context)
{
    if (len == 0)
    {
        return;
    }

    peripheralParameters.appConfigParams.armedStatus = data[0]; //set new value of armed status
    Serial.printf("from central isArmed=%u\r\n", peripheralParameters.appConfigParams.armedStatus);
    _cfg.armed = peripheralParameters.appConfigParams.armedStatus;
    hasDataForESP = true;
    
}
void writeLocationInterval(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context)
{
    if (len == 0)
    {
        return;
    }

    memcpy(&peripheralParameters.appConfigParams.locationInterval, data, len);
    Serial.printf("from central locationInterval=%u\r\n", peripheralParameters.appConfigParams.locationInterval);
    _cfg.locInt = peripheralParameters.appConfigParams.locationInterval;
    hasDataForESP = true;
}
void writePhoto(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context)
{
    if (len == 0)
    {
        return;
    }
    
    peripheralParameters.appConfigParams.photoStatus = data[0];
    Serial.printf("from central needPhoto=%u\r\n", peripheralParameters.appConfigParams.photoStatus);
    _cfg.needPhoto = peripheralParameters.appConfigParams.photoStatus;
    hasDataForESP = true;
}
void writePhotoInterval(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context)
{
    if (len == 0)
    {   
        return;
    }
    
    memcpy(&peripheralParameters.appConfigParams.photoInterval, data, len);
    Serial.printf("from central photoInterval=%u\r\n", peripheralParameters.appConfigParams.photoInterval);
    _cfg.photoInt = peripheralParameters.appConfigParams.photoInterval;
    hasDataForESP = true;
}
void writeSoundAlarm(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context)
{
    if (len == 0)
    {
        return;
    }

    peripheralParameters.appConfigParams.soundAlarmStatus = data[0];
    Serial.printf("from central soundAlarm=%u\r\n", peripheralParameters.appConfigParams.soundAlarmStatus);  
    _cfg.soundAlarm = peripheralParameters.appConfigParams.soundAlarmStatus;
    hasDataForESP = true;
}
void writeCrashLog(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context)
{
    if (len == 0)
    {
        return;
    }

    peripheralParameters.appConfigParams.crashLogStatus = data[0];
    Serial.printf("from central crashLog=%u\r\n", peripheralParameters.appConfigParams.crashLogStatus);
    _cfg.crashLog = peripheralParameters.appConfigParams.crashLogStatus;
    hasDataForESP = true;
}
void writeEnableLogging(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context)
{
    if (len == 0)
    {
        return;
    }
    
    peripheralParameters.appConfigParams.enableLoggingStatus = data[0];
    Serial.printf("from central enableLogging=%u\r\n", peripheralParameters.appConfigParams.enableLoggingStatus);
    _cfg.enaLog = peripheralParameters.appConfigParams.enableLoggingStatus;
    hasDataForESP = true;
}

void writeForceSoundAlarm(const uint8_t* data, size_t len, const BlePeerDevice& peer, void* context)
{
    if (len == 0)
    {
        return;
    }
    
    peripheralParameters.appConfigParams.forceSoundAlarm = data[0];
    Serial.printf("from central forceSoundAlarm=%u\r\n", peripheralParameters.appConfigParams.forceSoundAlarm);
}

uint8_t batteryLevelSimulate()
{
    static uint8_t battery = 100;
    battery--;
    if (battery < 10)
    {
        battery = 100;
    }

    return battery;
}

void initDeviceInformation()
{
    String manufName = "oversery";
    String modelNum = "1.0.0";
    String serialNum = "2019";
    String hardwarRev = "particle";
    String firmwarRev = "0.0.1";
    String devId = "xp-1";

    peripheralParameters.deviceInfSvc.manufacturerNameString = manufName;
    manufacturerNameStringCh.setValue(peripheralParameters.deviceInfSvc.manufacturerNameString);

    peripheralParameters.deviceInfSvc.modelNumberString = modelNum;
    modelNumberStringCh.setValue(peripheralParameters.deviceInfSvc.modelNumberString);

    peripheralParameters.deviceInfSvc.serialNumberString = serialNum;
    serialNumberStringCh.setValue(peripheralParameters.deviceInfSvc.serialNumberString);

    peripheralParameters.deviceInfSvc.hardwareRevisionString = hardwarRev;
    hardwareRevisionStringCh.setValue(peripheralParameters.deviceInfSvc.hardwareRevisionString);

    peripheralParameters.deviceInfSvc.firmwareRevisionString = firmwarRev;
    firmwareRevisionStringCh.setValue(peripheralParameters.deviceInfSvc.firmwareRevisionString);

    peripheralParameters.appConfigParams.deviceId = devId;
    deviceIdCh.setValue(peripheralParameters.appConfigParams.deviceId);

    peripheralParameters.appConfigParams.armedStatus = 1;
    armedCh.setValue(peripheralParameters.appConfigParams.armedStatus);

    peripheralParameters.appConfigParams.locationInterval = 100;
    locationIntervalCh.setValue(peripheralParameters.appConfigParams.locationInterval);

    peripheralParameters.appConfigParams.photoStatus = 0;
    photoCh.setValue(peripheralParameters.appConfigParams.photoStatus);

    peripheralParameters.appConfigParams.photoInterval = 10;
    photoIntervalCh.setValue(peripheralParameters.appConfigParams.photoInterval);

    peripheralParameters.appConfigParams.soundAlarmStatus = 0;
    soundAlarmCh.setValue(peripheralParameters.appConfigParams.soundAlarmStatus);

    peripheralParameters.appConfigParams.crashLogStatus = 0;
    crashLogCh.setValue(peripheralParameters.appConfigParams.crashLogStatus);

    peripheralParameters.appConfigParams.enableLoggingStatus = 0;
    enableLoggingCh.setValue(peripheralParameters.appConfigParams.enableLoggingStatus);

    peripheralParameters.appConfigParams.forceSoundAlarm = 0;
    forceSoundAlarmCh.setValue(peripheralParameters.appConfigParams.forceSoundAlarm);

    peripheralParameters.deviceAction.openStatus = peripheralParameters.appConfigParams.armedStatus & peripheralParameters.appConfigParams.enableLoggingStatus;
    openStatusCh.setValue(peripheralParameters.deviceAction.openStatus);
}
