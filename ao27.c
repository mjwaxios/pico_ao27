/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "ao27.pio.h"

/*
  PIO Resources:
  PIO 0
    sm 0    Rx CLK PLL
    sm 1    NRZI decode (not zero unstuff)
    sm 2
    sm 3
    irq 0   Rx Sample Clock
    irq 1   NRZI clock
    irq 2
    irq 3
    irq 4   NRZI output ready for txclock
    irq 5
    irq 6
    irq 7
    IRQ 0
    IRQ 1
  
  PIO 1
    sm 0    Flag Detector
    sm 1    zero unstuffer
    sm 2
    sm 3
    irq 0
    irq 1
    irq 2  flag detector output ready for unstuff
    irq 3  flag detector
    irq 4  
    irq 5  
    irq 6
    irq 7
    IRQ 0  flag detector
    IRQ 1  SM 1 Rx Fifo not Empty

  PIO 2
    sm 0
    sm 1
    sm 2
    sm 3
    irq 0
    irq 1
    irq 2
    irq 3
    irq 4
    irq 5
    irq 6
    irq 7
    IRQ 0
    IRQ 1
*/

#define pinUARTtx     0
#define pinUARTrx     1

#define pinWSLed      2
#define pinSampleCLK  3
#define pinPLLAdd     4
#define pinPLLSub     5

#define pinNRZIdecode 6
#define pinFlag       7     // Last bit received was end of a flag char 
#define pinClk        8     // Decoded data stream clock (falling edge sample)
#define pinData       9     // Decoded data stream (sample on falling clock edge)

#define pinToCPU1     10
#define pinToCPU2     11
#define pin12         12
#define pin13         13

#define pinFromCPU    14
#define pin15         15

#define pin16         16
#define pin17         17

#define pin18         18
#define pin19         19
#define pin20         20
#define pin21         21

#define pinSwitch     22

#define pin26         26
#define pin27         27

#define pin28         28

uint flagcount = 0;
uint datacount = 0;
uint pio1unknownIRQ0 = 0;
uint pio1unknownIRQ1 = 0;

uint8_t packet[1024];
int packetlen = 0;

// --------------------------------------------------------
//  Flag IRQ Handler
// --------------------------------------------------------
void pio_irq_flag() {
  PIO pio = pio1;
  uint isr = 3;

  if (pio_interrupt_get(pio, isr)) {       
    flagcount++;   
    pio_interrupt_clear(pio, isr);
    if (packetlen > 0) {
      for(int i=0; i < packetlen; i++) 
        printf("%02X ", packet[i]);
      printf("\n");
    }
    packetlen = 0;
  } else {
    pio1unknownIRQ0++;
  }
}

// --------------------------------------------------------
//  Data IRQ handler
// --------------------------------------------------------
void pio_irq_data() {
  PIO pio = pio1;
  uint sm_data = 1;

  while (!pio_sm_is_rx_fifo_empty(pio, sm_data)) {
    uint32_t data = pio_sm_get(pio, sm_data);
    if (packetlen < 1024) {
      packet[packetlen++] = data >> 24;   // Grab the data byte and save it
    }
    datacount++;
  }
}

// --------------------------------------------------------
//  PIO 0   Setup for Rx Clock and NRZI
// --------------------------------------------------------
void setupPIO0() {
  printf("<setupPIO0> Start\n");
  PIO pio = pio0;

  // Setup Receive Clock PLL SM
  uint sm = 0;
  uint offset_pll = pio_add_program(pio, &rxclock_program);
  pio_sm_config sm_pll = rxclock_program_get_default_config(offset_pll);
  // Setup GPIO Pins
  // receive data in
  pio_sm_set_consecutive_pindirs(pio, sm, pinFromCPU, 1, false);
  sm_config_set_in_pins(&sm_pll, pinFromCPU);
  sm_config_set_in_pin_count(&sm_pll, 1);

  // Setup Output Pins
  pio_gpio_init(pio, pinPLLAdd);
  pio_gpio_init(pio, pinPLLSub);
  pio_sm_set_consecutive_pindirs(pio, sm, pinPLLAdd, 2, true);
  sm_config_set_sideset_pins(&sm_pll, pinPLLAdd);
  
  pio_sm_init(pio, sm, offset_pll, &sm_pll);
  
  // Setup NRZI Decoder SM
  sm = 1;
  uint offset_nrzi = pio_add_program(pio, &nrzi_program);
  pio_sm_config sm_nrzi = nrzi_program_get_default_config(offset_nrzi);
  // Input Pins
  pio_sm_set_consecutive_pindirs(pio, sm, pinFromCPU, 1, false);
  sm_config_set_in_pins(&sm_nrzi, pinFromCPU);
  sm_config_set_in_pin_count(&sm_nrzi, 1);

  // Output Pins
  pio_gpio_init(pio, pinSampleCLK);
  pio_gpio_init(pio, pinNRZIdecode);
  pio_sm_set_consecutive_pindirs(pio, sm, pinSampleCLK, 1, true);
  pio_sm_set_consecutive_pindirs(pio, sm, pinNRZIdecode, 1, true);
  sm_config_set_sideset_pins(&sm_nrzi, pinSampleCLK);
  sm_config_set_set_pins(&sm_nrzi, pinNRZIdecode, 1);
  pio_sm_init(pio, sm, offset_nrzi, &sm_nrzi);


    // Setup Receive Clock PLL SM
  sm = 2;
  uint offset_txclk = pio_add_program(pio, &txclock_program);
  pio_sm_config sm_tx = txclock_program_get_default_config(offset_txclk);
  // Setup GPIO Pins

  // Setup Output Pins
  pio_gpio_init(pio, pinToCPU1);
  pio_gpio_init(pio, pinToCPU2);
  pio_sm_set_consecutive_pindirs(pio, sm, pinToCPU1, 2, true);
  sm_config_set_set_pins(&sm_tx, pinToCPU1, 2);
  
  pio_sm_init(pio, sm, offset_txclk, &sm_tx);

  pio_enable_sm_mask_in_sync(pio, 0x07);
  printf("<setupPIO0> End\n");
}

// --------------------------------------------------------
//  PIO 1
// --------------------------------------------------------
void setupPIO1() {
  printf("<setupPIO1> Start\n");
  PIO pio = pio1;
  uint sm = 0;

  uint offset_flag = pio_add_program(pio, &flag_program);

  pio_sm_config sm_flag = flag_program_get_default_config(offset_flag);
  // Setup GPIO Pins
  // receive data in
  pio_sm_set_consecutive_pindirs(pio, sm, pinNRZIdecode, 1, false);
  sm_config_set_in_pins(&sm_flag, pinNRZIdecode);
  sm_config_set_in_pin_count(&sm_flag, 1);

  pio_gpio_init(pio, pinFlag);
  pio_gpio_init(pio, pinData);
  pio_sm_set_consecutive_pindirs(pio, sm, pinFlag, 1, true);
 // pio_sm_set_consecutive_pindirs(pio, sm, pinData, 1, true);
  sm_config_set_sideset_pins(&sm_flag, pinFlag);
//  sm_config_set_out_pins(&sm_flag, pinData, 1);

  // interrupt setup
  irq_set_exclusive_handler(PIO1_IRQ_0, pio_irq_flag);
  irq_set_enabled(PIO1_IRQ_0, true);
  pio_set_irq0_source_enabled(pio, pis_interrupt3, true);

  // Init
  pio_sm_init(pio, sm, offset_flag, &sm_flag);  // Init SM

    // unstuffer
  sm = 1;
  uint offset_unstuff = pio_add_program(pio, &unstuff_program);
  pio_sm_config sm_unstuff = unstuff_program_get_default_config(offset_unstuff);
  // Setup GPIO Pins
  // receive data in
  pio_sm_set_consecutive_pindirs(pio, sm, pinNRZIdecode, 1, false);
  sm_config_set_in_pins(&sm_unstuff, pinNRZIdecode);
  sm_config_set_in_pin_count(&sm_unstuff, 1);

  // Setup Output Pins
  pio_gpio_init(pio, pinClk);
  pio_gpio_init(pio, pinData);
  pio_gpio_init(pio, pin15);
  pio_gpio_init(pio, pin16);
  pio_gpio_init(pio, pin17);
  pio_sm_set_consecutive_pindirs(pio, sm, pinClk, 1, true);
  pio_sm_set_consecutive_pindirs(pio, sm, pinData, 1, true);
  pio_sm_set_consecutive_pindirs(pio, sm, pin15, 3, true);
  sm_config_set_sideset_pins(&sm_unstuff, pinClk);
  sm_config_set_jmp_pin(&sm_unstuff, pinFlag);
  sm_config_set_out_pins(&sm_unstuff, pin15, 3);
  
  pio_sm_init(pio, sm, offset_unstuff, &sm_unstuff);

  irq_set_exclusive_handler(PIO1_IRQ_1, pio_irq_data);
  irq_set_enabled(PIO1_IRQ_1, true);
  pio_set_irq1_source_enabled(pio, pis_sm1_rx_fifo_not_empty, true);

  pio_enable_sm_mask_in_sync(pio, 0x03);        // Enable SM
  pio->txf[0] = 0x0000007E;                    // Put Flag Char into FIFO

  printf("<setupPIO1> End\n");
}

// --------------------------------------------------------
//    main
// --------------------------------------------------------
   int main() {
  set_sys_clock_khz(156000, true);
  setup_default_uart();
  stdio_init_all();    
  printf("AO-27 Bench CPU Pico\n");
  
  // Setup FromCPU pin as Input with internal pull up
  gpio_init(pinFromCPU);
  gpio_set_dir(pinFromCPU, GPIO_IN);
  gpio_pull_up(pinFromCPU); 

  setupPIO0();
  setupPIO1();
  while(1) {
 //   printf("\nFlag Count: %i,  Data Count: %i, unknown0: %i, unknown1: %i\n", flagcount, datacount, pio1unknownIRQ0, pio1unknownIRQ1);
    sleep_ms(1000);
  }
}

