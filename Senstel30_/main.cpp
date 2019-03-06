/*
 *   Chinook ÉTS (C) 2019
 *
 *   Marc Beaulieu
 *   Alfred Morel-Quintin
 */

#include "mbed.h"

#define TIME_ACQ 20
#define TIME_DATA_OUT 100

//#define LED_DEBUG
#define RPM_KHZ_OUTPUT

// LEDs
DigitalOut led1(LED1);
DigitalOut led2(LED2);
DigitalOut led3(LED3);

// Threads
Thread ws_thread;
Thread acquisition_thread;
Thread data_out_thread;

// Inputs
AnalogIn loadcell_adc(A0);
AnalogIn torque_adc(A1);
Serial weather_station(PE_8, PE_7, 4800); // RX = PE_7, TX = PE_8
InterruptIn rpm_pin(PF_13);

// Pitch position encoder
DigitalOut pitch_clock(PE_15);
DigitalIn pitch_input(PE_7);

// Outputs
Serial pc(USBTX, USBRX, 115200);

// Flags
int flag_pc_out = false;

// structure to store sensor data
struct SensorData {
    float torque;
    float loadcell;
    float wind_direction;
    float wind_speed;
    float rpm_rotor;
    float rpm_wheels;
    int gear;
    int pitch;
    float angle_mat;
};
SensorData sensors;

// Trames de la weather station
char trame[64];
int index;
// RPM counter
uint32_t rpm_counter = 0;

// Fonctions
void main_acquisition();
void ws_acquisition();
void irq_rpm();
void main_data_out();

// Weather station thread
void ws_acquisition()
{
    while (true)
    {
        wait_ms(2); // Lessen the load on the uC
        if(weather_station.readable())
        {
            char character = weather_station.getc();
            
            if(character == '$' && index != 0)
            {
                // Process the last NMEA packet
                static char IIMWV_str[] = "$IIMWV";
                static char sentence_begin[7] = {0};
                memcpy(sentence_begin, trame, 6);
                if(!strcmp(IIMWV_str, sentence_begin))
                {
                    led1 = !led1;
                    // Extract information
                    static char wind_dir[6] = {0};
                    memcpy(wind_dir, trame + 7, 5);
                    static char wind_spd[6] = {0};
                    memcpy(wind_spd, trame + 15, 5);
                    
                    sensors.wind_direction = atof(wind_dir);
                    sensors.wind_speed = atof(wind_spd);
                }
                // Clear the packet
                index = 0;
            }
            // Put the character read into our buffer
            trame[index++] = character;
        }
    }
}

// RPM IRQ handler
void irq_rpm()
{
    ++rpm_counter;
}

// Performs the acquisition of the sensors
void main_acquisition()
{
    while(1)
    {
        // Torque
        sensors.torque = torque_adc.read();
        
        // Load Cell
        sensors.loadcell = loadcell_adc.read();
        
        // RPM
        sensors.rpm_rotor = (float)rpm_counter * 1000.0f / (float)TIME_ACQ;
#ifndef RPM_KHZ_OUTPUT
        sensors.rpm_rotor *= 60.0f / 360.0f;
#endif
        rpm_counter = 0;
        
        wait_ms(TIME_ACQ);
#ifdef LED_DEBUG
        led2 = !led2;
#endif
        
        // Pitch
        unsigned int pitch_data = 0;
        for(int i = 0; i < 12; ++i)
        {
            pitch_data <<= 1;
            pitch_clock = 0;
            wait_us(1);
            pitch_clock = 1;
            wait_us(1);
            pitch_data |= pitch_input;
        }
        sensors.pitch = pitch_data;
    }
}

// Analyzes the data and sends it out
void main_data_out()
{
    while(1)
    {
#ifdef LED_DEBUG
        led3 = !led3;
#endif
        
        // Out PC
        // We cant call printf in an interrupt, so we flag it for main.
        flag_pc_out = true;
        
        // Out CAN
        // TODO
        
        // Out LoRa
        // TODO
        
        // Out SD
        // TODO
        
        wait_ms(TIME_DATA_OUT);
    }
}

// main() runs in its own thread in the OS
int main()
{
    // Thread starts
    ws_thread.start(ws_acquisition);
    acquisition_thread.start(main_acquisition);
    data_out_thread.start(main_data_out);
    
    // RPM Interrupt on pin
    rpm_pin.rise(&irq_rpm);
    
    // Clear the screen string
    static char clear_str[] = "x[2Jx[H";
    clear_str[0] = clear_str[4] = 27; // ESC character replaces the x's
    
    while (true)
    {
        if(flag_pc_out)
        {
            flag_pc_out = false;
            
            // Clear the screen and reset the cursor pointer
            pc.printf(clear_str);
            
            // Print data to screen
            pc.printf("Torque = %f Nm\n\r", sensors.torque);
            pc.printf("Loadcell = %f N\n\r", sensors.loadcell);
            pc.printf("RPM = %f rpm\n\r", sensors.rpm_rotor);
            pc.printf("Wind direction = %f degs\n\r", sensors.wind_direction);
            pc.printf("Wind speed = %f knots\n\r", sensors.wind_speed);
            pc.printf("Pitch = %d \n\r", sensors.pitch);
        }
        wait_ms(5);
    }
}

