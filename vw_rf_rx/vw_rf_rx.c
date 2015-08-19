#include "VirtualWire.h"
#include <stdio.h>
#include <stdlib.h>

char buffer[80];

const int led_pin = 13;
const int transmit_pin = 12;
const int receive_pin = 2;
const int transmit_en_pin = 3;

void setup()
{
    delay(1000);
    Serial.begin(9600);	// Debugging only
    Serial.println("setup");

    sprintf(buffer, "VW_MAX_MESSAGE_LEN=%d", VW_MAX_MESSAGE_LEN);

    Serial.println(buffer);

    // Initialise the IO and ISR
    vw_set_tx_pin(transmit_pin);
    vw_set_rx_pin(receive_pin);
    vw_set_ptt_pin(transmit_en_pin);
    vw_set_ptt_inverted(true); // Required for DR3100
    vw_setup(2000);	 // Bits per sec

    vw_rx_start();       // Start the receiver PLL running

    pinMode(led_pin, OUTPUT);
}

void loop()
{
    uint8_t buf[VW_MAX_MESSAGE_LEN];
    uint8_t buflen = VW_MAX_MESSAGE_LEN;

    if (vw_get_message(buf, &buflen)) // Non-blocking
    {
	int i;

        digitalWrite(led_pin, HIGH); // Flash a light to show received good message
	// Message with a good checksum received, dump it.
        Serial.print("Got:");
        snprintf(buffer, 7, "%s", buf);
        Serial.print(buffer);
        sprintf(buffer,  "%d", (uint8_t)buf[6]);
        Serial.print(buffer);

        //for (i = 0; i < buflen; i++)
        //{
	//    Serial.print(((char *)buf)[i]);
	//    Serial.print(' ');
	//}

	Serial.println();
        digitalWrite(led_pin, LOW);
    }
}

