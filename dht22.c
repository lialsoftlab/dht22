#include <stdlib.h>
#include <stdio.h>

#include "gpio_lib.h"

#define PB2    SUNXI_GPB(2)

#define MAX_TIMINGS 88

typedef struct {
    int humidity;
    int temperature;
} dht_sensor_data;

int main(int argc, char* argv[])
{
    dht_sensor_data data;
    
    while( !read_dht_data(PB2, &data) ) usleep(3000000);
    
    printf("{\n  \"Humd\": %0.1f,\n  \"Temp\": %0.1f\n}\n", data.humidity / 10.0, data.temperature / 10.0);
    
    return 0;
}

int read_dht_data(unsigned int pin, dht_sensor_data* data)
{
    unsigned octet[5] = {0, 0, 0, 0, 0};
    unsigned prev_counter = 0;
    unsigned counter      = 0;
    unsigned state = 1;
    int i, oc;
    int checksum = 0;
    int valid;
    
    if( !data ) return 0;
    
    if( SETUP_OK != sunxi_gpio_init() ) return 0;
      
    // initiating communication with sensor.  
    if( SETUP_OK != sunxi_gpio_set_cfgpin(pin,OUTPUT) ) return 0;    
    if( sunxi_gpio_output(pin,HIGH) ) return 0;    
    usleep(500);
    if( sunxi_gpio_output(pin,LOW) ) return 0;    
    usleep(2000);
    if( sunxi_gpio_output(pin,HIGH) ) return 0;
    
    // decoding impulses, skipping first three, since it's a handshake sequence of sensor.
    // parsing 5 octets of data. Tgap = 50us; T0 = 26~28us; T1 = 70us; so T0 < Tgap < T1
    if( SETUP_OK != sunxi_gpio_set_cfgpin(PB2,INPUT) ) return 0;    
    if( sunxi_gpio_input(PB2) == HIGH ) {
        for(i = 0; i < 3 + 16 * 2 + 16 * 2 + 8 * 2; i++) {
            counter = 0;
            if( state == 1 )
                while(sunxi_gpio_input(PB2) != LOW && ++counter < 1400) ;
            else
                while(sunxi_gpio_input(PB2) != HIGH && ++counter < 1400) ;
            state = !state;
            if( i < 3 ) continue;
            if( i % 2 == 0 ) {
                oc = (i - 3) / 16;
                octet[oc] = (octet[oc] << 1) | (counter < prev_counter ? 0 : 1);
            }
            prev_counter = counter;
        }      
    }
    
    data->humidity = (octet[0] << 8) + octet[1];
    data->temperature = !(octet[2] & 0x80) ? (octet[2] << 8) + octet[3] : -(((octet[2] & 0x7F) << 8) + octet[3]);  
        
    sunxi_gpio_cleanup();    

    // last octet is checksum
    return octet[4] == ((octet[0] + octet[1] + octet[2] + octet[3]) & 0xFF);
}