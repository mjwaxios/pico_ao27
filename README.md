# pico_ao27
AO-27 Bench CPU PI Pico board

Uses pi Pico 2W to communicate with the bench CPU.  Contains PIO code for RxClock PLL, NRZI decoder, zero bit stuff remover, data to bytes, and Flag Detector.

This commit has:
  - Pico 2 project setup
  - pins assigned
  - pio SM for rx clock recovery with PLL
  - pio SM for NRZI decoding (without zero bit stuffinhg removal)
  - pio SM SDLC flag detector (0x7E)
  - Interrupt to C code on flag detection,  Counts the number of flags detected
  - main loop in C prints flag count once a second

