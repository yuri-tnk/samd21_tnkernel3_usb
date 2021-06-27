   This is a port of the TNkernel v3 RTOS for the ATSAMD21G18 MCU
(Atmel/Microchip Cortex-M0P MCU) with USB CDC serial port and DMA UART0
serial port supporting.

   The USB CDC serial port uses ASF4 USB code as the reference.

   DMA UART0 serial port uses 3 DMA channels:

     ch 0 - UART 0 Rx - Cyclic (no interrupt)

     ch 1 - UART 0 Tx - Interrupt at the transfer complete

     ch 2 - Dummy & the CPU specific - uses just to force DMA controller
            to refresh write-back memory (we need it to get a fresh value
            of .DMAC_BTCNT of UART 0 Rx DMA)

   The project hardware is a SAMD21 MINI board
    (power - from USB, clock source - 32KHz external crystal osc)
