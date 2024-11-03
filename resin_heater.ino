/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/
//Pin declarations
#define PIN_ELEMENT 3
#define PIN_R 5
#define PIN_G 6
#define PIN_THERMISTOR_RESIN A0
#define PIN_THERMISTOR_AIR A1

//Constants
#define BAUD_RATE 57600
#define BUFFER_SIZE 10
#define TEMPERATURE_LOOPUP_TABLE_SIZE 101

#define RESISTOR_RESIN 10000.0 //10kOhm
#define RESISTOR_AIR 10000.0 //10kOhm

#define TEMPERATURE_NOMINAL 25 
#define THERMISTOR_NOMINAL 10000
#define THERMISTOR_B_COEFFICIENT 3950

#define PLOT_MINIMUM 20
#define PLOT_MAXIMUM 40

#define ELEMENT_RESISTANCE 5.5 //Measured the value in ohms
#define INPUT_VOLTAGE 12.0
#define MAX_AIR_TEMPERATURE 32.0

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

int16_t temperatureLookupTable[TEMPERATURE_LOOPUP_TABLE_SIZE][2] = { //Temperature, Resistance
    {0, 31770},
    {1, 30250},
    {2, 28820},
    {3, 27450},
    {4, 26160},
    {5, 24940},
    {6, 23770},
    {7, 22670},
    {8, 21620},
    {9, 20630},
    {10, 19680},
    {11, 18780},
    {12, 17930},
    {13, 17120},
    {14, 16350},
    {15, 15620},
    {16, 14930},
    {17, 14260},
    {18, 13630},
    {19, 13040},
    {20, 12470},
    {21, 11920},
    {22, 11410},
    {23, 10910},
    {24, 10450},
    {25, 10000},
    {26, 9575},
    {27, 9170},
    {28, 8784},
    {29, 8416},
    {30, 8064},
    {31, 7730},
    {32, 7410},
    {33, 7106},
    {34, 6815},
    {35, 6538},
    {36, 6273},
    {37, 6020},
    {38, 5778},
    {39, 5548},
    {40, 5327},
    {41, 5117},
    {42, 4915},
    {43, 4723},
    {44, 4539},
    {45, 4363},
    {46, 4195},
    {47, 4034},
    {48, 3880},
    {49, 3733},
    {50, 3592},
    {51, 3457},
    {52, 3328},
    {53, 3204},
    {54, 3086},
    {55, 2972},
    {56, 2863},
    {57, 2759},
    {58, 2659},
    {59, 2564},
    {60, 2472},
    {61, 2384},
    {62, 2299},
    {63, 2218},
    {64, 2141},
    {65, 2066},
    {66, 1994},
    {67, 1926},
    {68, 1860},
    {69, 1796},
    {70, 1735},
    {71, 1677},
    {72, 1621},
    {73, 1567},
    {74, 1515},
    {75, 1465},
    {76, 1417},
    {77, 1371},
    {78, 1326},
    {79, 1284},
    {80, 1243},
    {81, 1203},
    {82, 1165},
    {83, 1128},
    {84, 1093},
    {85, 1059},
    {86, 1027},
    {87, 996},
    {88, 965},
    {89, 936},
    {90, 908},
    {91, 881},
    {92, 855},
    {93, 830},
    {94, 805},
    {95, 781},
    {96, 758},
    {97, 737},
    {98, 715},
    {99, 695},
    {100, 674}
};

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/
//Global scope variables
uint16_t resin_temp_buffer[BUFFER_SIZE];
uint16_t air_temp_buffer[BUFFER_SIZE];

uint16_t adc_thermistor_resin = 0;
uint16_t adc_thermistor_air = 0;

float resistance_thermistor_resin = 0;
float resistance_thermistor_air = 0;

float temp_lookup_thermistor_resin = 0;
float temp_lookup_thermistor_air = 0;

float temp_steinhart_thermistor_resin = 0;
float temp_steinhart_thermistor_air = 0;

float temp_target_resin = 30.0;
float temp_target_air = 30.0;
float maximum_power_limit = 22; //Max power in Watts

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

float mapNumber(float x, float in_min, float in_max, float out_min, float out_max){//Remaps a number to a given range
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void readADCs(void){ //Read 5 samples and average then store in a circular buffer
    static uint8_t index = 0;

    uint16_t adcResin = 0;
    uint16_t adcAir = 0;

    for(uint8_t i = 0; i < 5; i++){
        adcResin += analogRead(PIN_THERMISTOR_RESIN);
        adcAir += analogRead(PIN_THERMISTOR_AIR);
    }

    resin_temp_buffer[index] = adcResin / 5;
    air_temp_buffer[index] = adcAir / 5;

    index = (index + 1) % BUFFER_SIZE;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

uint16_t getADCAverageResin(void){
    uint16_t adcAvg = 0;

    for(uint8_t i = 0; i < BUFFER_SIZE; i++){
        adcAvg += resin_temp_buffer[i];
    }
    return adcAvg / BUFFER_SIZE;
} 

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

uint16_t getADCAverageAir(void){
    uint16_t adcAvg = 0;

    for(uint8_t i = 0; i < BUFFER_SIZE; i++){
        adcAvg += air_temp_buffer[i];
    }
    return adcAvg / BUFFER_SIZE;
} 

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

float adcToResistance(uint32_t adc){
    return (RESISTOR_RESIN * (float)adc)/ (1023.0 - constrain(adc, 0.0, 1022.0)); //Constrain to avoid dividing by zero. Should never reach a value of 1023 anyway
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

float resistanceToTemperatureLookup(uint16_t res){
    float temp = 0;
    for(uint16_t i = 1; i < TEMPERATURE_LOOPUP_TABLE_SIZE; i++){
        if(temperatureLookupTable[i][1] <= res){
            temp = mapNumber(res, temperatureLookupTable[i][1], temperatureLookupTable[i - 1][1], temperatureLookupTable[i][0], temperatureLookupTable[i - 1][0]);
            break;
        }
    }
    return temp;
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

float resistanceToTemperatureSteinheart(float res){
    float steinhart = log(res / THERMISTOR_NOMINAL); // ln(R/Ro)
    steinhart /= THERMISTOR_B_COEFFICIENT; // 1/B * ln(R/Ro)
    steinhart += 1.0 / (TEMPERATURE_NOMINAL + 273.15); // + (1/To)
    steinhart = 1.0 / steinhart; //Invert
    return steinhart - 273.15; //Kelvin to C
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void termistorCalcs(void){
    adc_thermistor_resin = getADCAverageResin();
    adc_thermistor_air = getADCAverageAir();

    resistance_thermistor_resin = adcToResistance(adc_thermistor_resin);
    resistance_thermistor_air = adcToResistance(adc_thermistor_air);

    temp_lookup_thermistor_resin = resistanceToTemperatureLookup(resistance_thermistor_resin);
    temp_lookup_thermistor_air = resistanceToTemperatureLookup(resistance_thermistor_air);

    temp_steinhart_thermistor_resin = resistanceToTemperatureSteinheart(resistance_thermistor_resin);
    temp_steinhart_thermistor_air = resistanceToTemperatureSteinheart(resistance_thermistor_air);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void heatingElementOn(void){
    float maxPower = (INPUT_VOLTAGE * INPUT_VOLTAGE) / ELEMENT_RESISTANCE; //Max possible power given coil resistance and input voltage
    uint8_t duty = 255.0 * (maximum_power_limit / maxPower); //Duty scaled proportional to the power limit
    analogWrite(PIN_ELEMENT, duty);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void heatingElementOff(void){
    analogWrite(PIN_ELEMENT, 0); //Off
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void printValues(void){
    Serial.println("\n-----------------------------------------------");
    Serial.println("Resin:");
    Serial.print("ADC: ");
    Serial.println(adc_thermistor_resin);
    Serial.print("Resistance: ");
    Serial.println(resistance_thermistor_resin);
    Serial.print("Temperature lookup: ");
    Serial.print(temp_lookup_thermistor_resin);
    Serial.println("°C");
    Serial.print("Temperature Steinhart: ");
    Serial.print(temp_steinhart_thermistor_resin);
    Serial.println("°C\n");

    Serial.println("Air:");
    Serial.print("ADC: ");
    Serial.println(adc_thermistor_air);
    Serial.print("Resistance: ");
    Serial.println(resistance_thermistor_air);
    Serial.print("Temperature lookup: ");
    Serial.print(temp_lookup_thermistor_air);
    Serial.println("°C");
    Serial.print("Temperature Steinhart: ");
    Serial.print(temp_steinhart_thermistor_air);
    Serial.println("°C\n");
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void printValuesPlotter(void){
    Serial.print("Min:");
    Serial.print(PLOT_MINIMUM); //Plot the min and max lines
    Serial.print("\t");
    Serial.print("Max:");
    Serial.print(PLOT_MAXIMUM);
    Serial.print("\t");

    Serial.print("Target_Resin_Temp:");
    Serial.print(temp_target_resin); //Target Resin Temp
    Serial.print("\t");
    Serial.print("Resin_Temp:");
    Serial.print(temp_lookup_thermistor_resin); //Measured Resin Temp
    Serial.print("\t");

    Serial.print("Target_Air_Temp:");
    Serial.print(temp_target_air); //Target Air Temp
    Serial.print("\t");
    Serial.print("Air_Temp:");
    Serial.println(temp_lookup_thermistor_air); //Measured Air Temp
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setup(){
    Serial.begin(BAUD_RATE);
    // Serial.print("Setup. ");
    pinMode(PIN_THERMISTOR_RESIN, INPUT);
    pinMode(PIN_THERMISTOR_AIR, INPUT);

    pinMode(PIN_R, OUTPUT);
    pinMode(PIN_G, OUTPUT);
    pinMode(PIN_ELEMENT, OUTPUT);

    digitalWrite(PIN_R, LOW);
    digitalWrite(PIN_G, LOW);
    digitalWrite(PIN_ELEMENT, LOW);

    digitalWrite(PIN_G, HIGH);
    delay(300);
    for(uint8_t i = 0; i < BUFFER_SIZE; i++){ //Fill the potentiometer buffer with readings
        readADCs();
        delay(50);
    }
    // Serial.println("Complete");
    digitalWrite(PIN_G, LOW);
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/

void loop(){
    static uint32_t prevTime = 0;
    static uint32_t prevTime2 = 0;
    uint32_t msTime = millis();
    if(msTime - prevTime > 100){
        readADCs();
        termistorCalcs();
        // Serial.println(temp_lookup_thermistor_resin); //Measured Resin Temp
        // printValues();
        // printValuesPlotter();

        if(temp_lookup_thermistor_resin < temp_target_resin && (temp_lookup_thermistor_air < MAX_AIR_TEMPERATURE)){  //If air temp is too high turn off the heating
            heatingElementOn();
        }
        else{
            heatingElementOff();
        }

        if(temp_lookup_thermistor_air < temp_target_air){
            digitalWrite(PIN_R, HIGH);
        }
        else{
            digitalWrite(PIN_R, LOW);
        }

        prevTime = msTime;
    }

    if(msTime - prevTime2 > 1000){
        printValuesPlotter();
        prevTime2 = msTime;
    }
}

/*--------------------------------------------------------------------------------------------------------------------------------------------------------*/