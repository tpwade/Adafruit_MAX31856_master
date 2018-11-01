#include <Adafruit_MAX31856.h>

// Use software SPI: CS, DI, DO, CLK
//Adafruit_MAX31856 maxTC_HOT  = Adafruit_MAX31856(10, 11, 12, 13);
//Adafruit_MAX31856 maxTC_COLD = Adafruit_MAX31856( 9, 11, 12, 13);
// use hardware SPI, just pass in the CS pin
//Adafruit_MAX31856 max = Adafruit_MAX31856(10);

uint8_t tc_type = MAX31856_TCTYPE_E;
int8_t tc_num = 2;
int8_t tc_cs[16] = {9, 10};
float tc_temp[16];

//Adafruit_MAX31856 maxTC[2] = { Adafruit_MAX31856(9, 11, 12, 13) , Adafruit_MAX31856(10, 11, 12, 13)};
//Adafruit_MAX31856 maxTC[8] = Adafruit_MAX31856();
Adafruit_MAX31856 maxTC = Adafruit_MAX31856(11, 12, 13);

//maxTC[0] = Adafruit_MAX31856(11, 12, 13);
//maxTC[1] = Adafruit_MAX31856(11, 12, 13);
//Adafruit_MAX31856 maxTC[0] = Adafruit_MAX31856(10, 11, 12, 13);
//Adafruit_MAX31856 maxTC[1] = Adafruit_MAX31856(10, 11, 12, 13);

//Adafruit_MAX31856 maxTC = Adafruit_MAX31856 maxTC(11,12,13);

void setup() {
    
    int8_t tc_i;

    // for communication back along USB or serial
    Serial.begin(115200);
    Serial.println("MAX31856 thermocouple test");
  
  
    for (tc_i = 0; tc_i < tc_num; tc_i++){
        maxTC.begin(tc_cs[tc_i]);
        maxTC.setThermocoupleType(tc_cs[tc_i],tc_type);
        Serial.print("Thermocouple type: ");
        switch ( maxTC.getThermocoupleType(tc_cs[tc_i]) ) {
            case MAX31856_TCTYPE_B: Serial.println("B Type"); break;
            case MAX31856_TCTYPE_E: Serial.println("E Type"); break;
            case MAX31856_TCTYPE_J: Serial.println("J Type"); break;
            case MAX31856_TCTYPE_K: Serial.println("K Type"); break;
            case MAX31856_TCTYPE_N: Serial.println("N Type"); break;
            case MAX31856_TCTYPE_R: Serial.println("R Type"); break;
            case MAX31856_TCTYPE_S: Serial.println("S Type"); break;
            case MAX31856_TCTYPE_T: Serial.println("T Type"); break;
            case MAX31856_VMODE_G8: Serial.println("Voltage x8 Gain mode"); break;
            case MAX31856_VMODE_G32: Serial.println("Voltage x8 Gain mode"); break;
            default: Serial.println("Unknown"); break;
        }
    }
  
}

void loop() {
  
    int8_t tc_i;
    uint8_t maxTC_FAULTS;
    float tcTemp;
    float cjTemp;
    
    // trigger all TC boards to make a single TC measurement
    for (tc_i = 0; tc_i < tc_num; tc_i++){
        maxTC.oneShotTemperature(tc_cs[tc_i]);
    }
    // A single conversion requires approximately 143ms in 60Hz filter mode
    // or 169ms in 50Hz filter mode to complete. This bit self clears to 0.
    delay(250); // MEME FIX autocalculate based on oversampling
    
    for (tc_i = 0; tc_i < tc_num; tc_i++){
        maxTC.readThermocoupleTemperatureFast(tc_cs[tc_i],&tcTemp,&cjTemp);
        Serial.print(" CJ / TC Temp: ");
        Serial.print(cjTemp);
        Serial.print(" / ");
        Serial.println(tcTemp);
        //Serial.print(" Thermocouple Temp: "); Serial.println(maxTC.readThermocoupleTemperatureFast(tc_cs[tc_i],&tcTemp,&cjTemp));
        
#ifdef NEVER
        maxTC_FAULTS = maxTC.readFault(tc_cs[tc_i]);
        if (maxTC_FAULTS) {
            if (maxTC_FAULTS & MAX31856_FAULT_CJRANGE) Serial.println("Cold Junction Range Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_TCRANGE) Serial.println("Thermocouple Range Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_CJHIGH)  Serial.println("Cold Junction High Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_CJLOW)   Serial.println("Cold Junction Low Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_TCHIGH)  Serial.println("Thermocouple High Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_TCLOW)   Serial.println("Thermocouple Low Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_OVUV)    Serial.println("Over/Under Voltage Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_OPEN)    Serial.println("Thermocouple Open Fault");
        }
#endif
    }
    
#ifdef NEVER
    for (tc_i = 0; tc_i < tc_num; tc_i++){
        Serial.print("Cold Junction Temp: "); Serial.println(maxTC.readCJTemperature(tc_cs[tc_i]));
        
        Serial.print(" Thermocouple Temp: "); Serial.println(maxTC.readThermocoupleTemperature(tc_cs[tc_i]));
        // Check and print any faults
        maxTC_FAULTS = maxTC.readFault(tc_cs[tc_i]);
        if (maxTC_FAULTS) {
            if (maxTC_FAULTS & MAX31856_FAULT_CJRANGE) Serial.println("Cold Junction Range Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_TCRANGE) Serial.println("Thermocouple Range Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_CJHIGH)  Serial.println("Cold Junction High Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_CJLOW)   Serial.println("Cold Junction Low Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_TCHIGH)  Serial.println("Thermocouple High Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_TCLOW)   Serial.println("Thermocouple Low Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_OVUV)    Serial.println("Over/Under Voltage Fault");
            if (maxTC_FAULTS & MAX31856_FAULT_OPEN)    Serial.println("Thermocouple Open Fault");
        }
   }
#endif
 
  
  delay(1000);
}

void initTC() {
}

void readTC() {
}
