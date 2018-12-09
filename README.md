# ETA_lights
LED lights for ETA.
The LEDs use the TLS3001 chip.
The Board in use is a Nucleo64 401RE.

Important datapackages if you are making your own code.
ALL DATA must be manchesterencoded 0=> (0&0)(0&1)
                                   1=> (1&0)(1&1)

   uint32_t SyncFrame = 0x3FFF8800;        // 1111111111 1111100010 0000000000
   uint32_t SyncFrameMask = 0x20000000;    // 1000000000 0000000000 0000000000

  uint32_t ResetFrame = 0x7FFF4;           // 1111111111 11111 0100
  uint32_t ResetFrameMask = 0x40000;       // 1000000000 00000 0000

  uint32_t DataHeaderFrame = 0x7FFF2;      // 111111111111111 0010
  uint32_t DataHeaderFrameMask = 0x40000;  // 100000000000000 0000
  
  uint32_t DataFrameMask = 0b1000000000000;// 1000000000000
  uint32_t DataFilled  = 0b011111111111;   //NOTE the first bit MUST be a zero
  
  
