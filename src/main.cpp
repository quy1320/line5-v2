
#include <Arduino.h>
#include <EEPROM.h>

#include "digitalWriteFast.h"

const uint8_t analogInput[] = {
    PIN_PC2,
    PIN_PC3,
    PIN_PC0,
    PIN_PC1,
};

const uint8_t ledOutput[] = {
    PIN_PD4,
    PIN_PD3,
    PIN_PB2,
    PIN_PB1,
};

const uint8_t infraredPin[] = {
    PIN_PD6,
    PIN_PD5,
    PIN_PB0,
    PIN_PD7,
};

const uint8_t heartbeat = PIN_PB5;
const uint8_t calibratePin = PIN_PD2;
const uint8_t switchPin = PIN_PB5;

volatile uint16_t currentValue[4] = {0, 0, 0, 0};
volatile bool isDetectingLine[4];
volatile byte dataOut;
bool ledState[4]; // to ensure not all led light up same time
byte out = 0x00;

void processRoutine()
{
    // turn on the analog enable
    static uint8_t channelId = 0;
    channelId++;
    channelId %= 4;

    uint16_t value;

    switch (channelId)
    {
    case 0:
        /* code */
        digitalWriteFast(ledOutput[0], LOW);
        pinModeFast(infraredPin[0], OUTPUT);
        digitalWriteFast(infraredPin[0], HIGH);
        // delayMicroseconds(100);
        value = analogRead(analogInput[0]);
        pinModeFast(infraredPin[0], INPUT);
        currentValue[0] = value;
        break;

    case 1:
        digitalWriteFast(ledOutput[1], LOW);
        pinModeFast(infraredPin[1], OUTPUT);
        digitalWriteFast(infraredPin[1], HIGH);
        // delayMicroseconds(100);

        value = analogRead(analogInput[1]);
        pinModeFast(infraredPin[1], INPUT);
        currentValue[1] = value;
        break;

    case 2:
        digitalWriteFast(ledOutput[2], LOW);
        pinModeFast(infraredPin[2], OUTPUT);
        digitalWriteFast(infraredPin[2], HIGH);
        // delayMicroseconds(100);

        value = analogRead(analogInput[2]);
        digitalWriteFast(infraredPin[2], LOW);
        pinModeFast(infraredPin[2], INPUT);
        currentValue[2] = value;
        break;

    case 3:
        digitalWriteFast(ledOutput[3], LOW);
        pinModeFast(infraredPin[3], OUTPUT);
        digitalWriteFast(infraredPin[3], HIGH);
        // delayMicroseconds(100);

        value = analogRead(analogInput[3]);
        pinModeFast(infraredPin[3], INPUT);
        currentValue[3] = value;
        break;

    default:
        break;
    }
}

uint16_t maxValues[4];
uint16_t minValues[4];

void turnAll(uint8_t state)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        ledState[i] = state;
    }
}
void blink()
{
    for (int time = 0; time < 10; time++)
    {
        for (uint8_t i = 0; i < 4; i++)
        {
            digitalWrite(ledOutput[i], HIGH);
        }
        delay(20);
        for (uint8_t i = 0; i < 4; i++)
        {
            digitalWrite(ledOutput[i], LOW);
        }
        delay(20);
    }
}
void fastBlink()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        digitalWrite(ledOutput[i], HIGH);
    }
    delay(10);
    for (uint8_t i = 0; i < 4; i++)
    {
        digitalWrite(ledOutput[i], LOW);
        ledState[i] = LOW;
    }
}

void loadCalibratedValues()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        maxValues[i] = EEPROM.read(i * 4) + (EEPROM.read(i * 4 + 1) << 8);
        minValues[i] = EEPROM.read(i * 4 + 2) + (EEPROM.read(i * 4 + 3) << 8);
    }
    blink();
}
void storeCalibratedValues()
{
    for (uint8_t i = 0; i < 4; i++)
    {
        EEPROM.write(i * 4, maxValues[i] & 0xFF);
        EEPROM.write(i * 4 + 1, (maxValues[i] >> 8) & 0xFF);
        EEPROM.write(i * 4 + 2, minValues[i] & 0xFF);
        EEPROM.write(i * 4 + 3, (minValues[i] >> 8) & 0xFF);
    }

    blink();
}

void processLedOutput()
{
    static uint8_t channelId = 0;
    channelId++;
    channelId %= 4;

    for (int i = 0; i < 4; i++)
    {
        digitalWrite(ledOutput[i], ledState[i]);
    }

    // switch (channelId) {
    //   case 0 :
    //     digitalWrite(ledOutput[0], ledState[0]);
    //     digitalWrite(ledOutput[3], LOW);
    //     break;
    //   case 1 :
    //     digitalWrite(ledOutput[1], ledState[1]);
    //     digitalWrite(ledOutput[0], LOW);
    //     break;
    //   case 2 :
    //     digitalWrite(ledOutput[2], ledState[2]);
    //     digitalWrite(ledOutput[1], LOW);
    //     break;
    //   case 3 :
    //     digitalWrite(ledOutput[3], ledState[3]);
    //     digitalWrite(ledOutput[2], LOW);
    //     break;
    // }
}

bool isPress()
{
    return digitalRead(calibratePin) == LOW;
}
void mainProcess()
{
    // if (analogRead(calibratePin) < 500) {
    //   // do calibration
    //   turnAll(LOW);
    // }

    static unsigned long startCalibrateTimestamp = 0;

    // calibrate happen in 5 seconds, while this happen, the led will be blinking every 500ms

    if (startCalibrateTimestamp == 0 && isPress())
    {
        startCalibrateTimestamp = millis();
        digitalWrite(heartbeat, HIGH);

        for (uint8_t channelId = 0; channelId < 4; channelId++)
        {
            maxValues[channelId] = 0;
            minValues[channelId] = 1023;
        }

        return;
    }

    else if (startCalibrateTimestamp != 0 && millis() - startCalibrateTimestamp > 5000)
    {
        digitalWrite(heartbeat, LOW);
        startCalibrateTimestamp = 0;
        storeCalibratedValues();

        return;
    }
    else if (startCalibrateTimestamp != 0)
    {
        // perform the calibration here
        for (uint8_t channelId = 0; channelId < 4; channelId++)
        {
            if (currentValue[channelId] > maxValues[channelId])
            {
                maxValues[channelId] = currentValue[channelId];
                fastBlink();
            }
            if (currentValue[channelId] < minValues[channelId])
            {
                minValues[channelId] = currentValue[channelId];
                fastBlink();
            }
        }

        // blinking led
    }

    else
    {
        // do nothing
        const uint16_t SCHMITT_GAP = 10;
        for (uint8_t channelId = 0; channelId < 4; channelId++)
        {
            uint16_t midPointUp = (maxValues[channelId] + minValues[channelId]) / 2 + SCHMITT_GAP;
            uint16_t midPointDown = (maxValues[channelId] + minValues[channelId]) / 2 - SCHMITT_GAP;

            if (currentValue[channelId] > midPointUp)
            {
                ledState[channelId] = LOW;
                isDetectingLine[channelId] = false;
            }
            else if (currentValue[channelId] < midPointDown)
            {
                ledState[channelId] = HIGH;
                isDetectingLine[channelId] = true;
            }
        }

        // load to register for fast i2c operation

        bitWrite(out, 0, isDetectingLine[0]);
        bitWrite(out, 1, isDetectingLine[1]);
        bitWrite(out, 2, isDetectingLine[2]);
        bitWrite(out, 3, isDetectingLine[3]);
        bitWrite(out, 4, !digitalRead(switchPin));
        bitWrite(out, 5, HIGH);
        bitWrite(out, 6, HIGH);

        // out |= (!isDetectingLine[0]) << 0;
        // out |= (!isDetectingLine[1]) << 1;
        // out |= (!isDetectingLine[2]) << 2;
        // out |= (!isDetectingLine[3]) << 3;
        // out |= !digitalRead(switchPin) << 4;      // backward compatible
        // // out |= isDetectingLine[4] << 5;          // additional channel

        // // out |= HIGH << 6;
        // // out |= HIGH << 7;
        // out |= HIGH << 6;
        // out |= HIGH << 7;

        // one more bit for switchpin
        dataOut = out;
    }
}

void debugProcess()
{
    // run every 300ms
    static unsigned long lastDebug = 0;
    if (millis() - lastDebug < 300)
    {
        return;
    }
    lastDebug = millis();

    // print the currentValue for each channle, seperate with /t, end with newline

    // debug a channel
    const uint8_t channel = 3;
    Serial.print(currentValue[channel]);
    Serial.print("\t");
    Serial.print(maxValues[channel]);
    Serial.print("\t");
    Serial.print(minValues[channel]);
    Serial.print("\t");
    Serial.println(isDetectingLine[channel]);
}

#include "I2CSlave.h"

#define I2C_ADDR 0x24

void I2C_received(uint8_t received_data)
{
}

void I2C_requested()
{
    I2C_transmitByte(dataOut);
}

void setupI2C()
{
    // setup i2c as slave
    // address is 0x24

    // pinMode command
    // FL5_expander.pinMode(P0, INPUT_PULLUP);
    // FL5_expander.pinMode(P1, INPUT_PULLUP);
    // FL5_expander.pinMode(P2, INPUT_PULLUP);
    // FL5_expander.pinMode(P3, INPUT_PULLUP);
    // FL5_expander.pinMode(P4, INPUT);
    // FL5_expander.begin(addr);
    // digitalReadAll

    // set received/requested callbacks
    I2C_setCallbacks(I2C_received, I2C_requested);

    // init I2C
    I2C_init(I2C_ADDR);
}

void led(bool state)
{
    if (state)
    {
        pinMode(switchPin, OUTPUT);
        digitalWrite(switchPin, LOW);
    }
    else
    {
        pinMode(switchPin, INPUT_PULLUP);
    }
}

void infrared(bool state)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        pinMode(infraredPin[i], OUTPUT);
        digitalWrite(infraredPin[i], state);
    }
}
void indicator(bool state)
{
    for (uint8_t i = 0; i < 4; i++)
    {
        pinMode(ledOutput[i], OUTPUT);
        digitalWrite(ledOutput[i], state);
    }
}

void setup()
{
    // set all led ouput to OUTPUT, infraredPin to Output and input to input

    for (uint8_t i = 0; i < 4; i++)
    {
        pinMode(ledOutput[i], OUTPUT);
        pinMode(infraredPin[i], OUTPUT);
        pinMode(analogInput[i], INPUT);

        // digitalWrite(ledOutput[i], HIGH);
    }

    pinMode(calibratePin, INPUT_PULLUP);
    pinMode(switchPin, INPUT_PULLUP);

    loadCalibratedValues();
    delay(1000);

    setupI2C();
}

void loop()
{
    // pinMode(switchPin, OUTPUT);
    // digitalWrite(switchPin, LOW);
    // delay(1000);
    // digitalWrite(switchPin, HIGH);
    // delay(1000);

    // led(true);
    // delay(1000);
    // led(false);

    processRoutine();
    mainProcess();
    processLedOutput();

    // if (analogRead(calibratePin) > 500) {
    //   digitalWrite(heartbeat, HIGH);
    //   delay(100);
    //   digitalWrite(heartbeat, LOW);
    //   delay(100);
    //   // do calibration
    // }

    // random swithicng the ledoutput
    // uint8_t led = random(5);
    // digitalWrite(ledOutput[led], HIGH);
    // delay(100);
    // digitalWrite(ledOutput[led], LOW);
}
