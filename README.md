# ETA_lights
LED lights for ETA.
The LEDs use the TLS3001 chip.
The Board in use is a Nucleo64 401RE.

This is written with the eclipse based IDE System workbench, Open STM32 Tools

Generated IOs and code with CubeMX

If you are unable to open the project with Sytem workbench open it with CubeMX and regenerate the code.

Important datapackages if you are making your own code.

ALL DATA must be manchesterencoded I.E
0=> (0&0)(0&1)

1=> (1&0)(1&1)
                                   
Normal operations is as follows, 

Send a sync frame. This will sync the internal clock of the LEDs to the one transmitting. 

A delay after the syncframe to allowit to propogate though the chain. More LEDs => longer delay 6ms is ok for 200LEDs in series

Send a Dataheader to tell the LEDs that data is incloming

Send data directly after, the data is 12bits for each color channel Red Green Blue. to decreese intensity send lower values.

Repeat the data transfer for as many LEDs you have.

If you encouter problems send the syncframe more often and see if it helps, the internal clock drifts a little. 

  uint32_t SyncFrame = 0x3FFF8800;        // 1111111111 1111100010 0000000000

  uint32_t SyncFrameMask = 0x20000000;    // 1000000000 0000000000 0000000000

  uint32_t ResetFrame = 0x7FFF4;           // 1111111111 11111 0100

  uint32_t ResetFrameMask = 0x40000;       // 1000000000 00000 0000

  uint32_t DataHeaderFrame = 0x7FFF2;      // 111111111111111 0010

  uint32_t DataHeaderFrameMask = 0x40000;  // 100000000000000 0000

  uint32_t DataFrameMask = 0b1000000000000;// 1000000000000  

  uint32_t DataFilled  = 0b011111111111;   //NOTE the first bit MUST be a zero
  
  
translated from datasheet of TLS3001:

The output current value can be calculated by applying the following formula:
Iout = (Vref/R)*2*13.8
Vref ≈ 0.46V
--- resistance is 620 Ω
When the output current is about 20mA Data communication protocol Give SDI
The input signal to the foot must follow the following definitions:

a.

The valid input data must be a Manchester code, and the signal transitions from high to --- to indicate "1", from --- to high.
Jump indicates "0"

b.

After the chip is powered on, a sync frame must be sent first so that the chip can detect the baud rate of the communication. 
This is a Synchronized frame .
The format is: 15'b111111111111111+4'b0001+11’b00000000000, in the transmission synchronization
After the frame, the data frame must be delayed for a while, so that it is accurate for each chip.
To the baud rate of communication, the delay time (us) is greater than: the number of connected chips ÷ communication wave rate (MHz) × 30

c.

After sending several frames of data, resend the duplicate frame and wait
1ms
After that, send it again
Step frame, in order to eliminate the accumulated error, the complex frame format is:
15’b111111111111111+4’b0100

d.

The data frame format is: 15'b111111111111111+4'b0010 (data header) + 39bit data to the first chip + 39bit data to the first chip + ...... + nth chip 39bit data

e.

The first chip is the chip that receives the data first. The data format of the chip is: 1’
B0 (identification --- ) + 12’ Bxxxxxxxxxxxx (output port 1 data) + 1’ b0 (identification 1⁄2 ) + 12’ bxxxxxxxxxxxx
(lose Out port 2 data)+ 1' b0 (identification 1⁄2 ) + 12’ bxxxxxxxxxxxx (output port 3 data),x for 1 or 0

f.

Data is sent first MSB (up to ---)

g.

SDI When the input pin is in the idle state, it must maintain the 1⁄2 level.

h.

In the same frame data transmission process, it must be sent continuously, there is no interruption in the middle, and the transmission frequency is not --- change
  
  
