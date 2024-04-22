void ConfigureADC()
{
    uint8_t* CTRLA = (uint8_t*)0x42002000;
    uint8_t* REFCTRL = (uint8_t*)0x42002001;
    uint8_t* SAMPCTRL = (uint8_t*)0x42002003;
    uint16_t* CTRLB = (uint16_t*)0x42002004;
    uint32_t* INPUTCTRL = (uint32_t*)0x42002010;
    uint8_t* EVCTRL = (uint8_t*)0x42002014;
    uint8_t* INTFLAG = (uint8_t*)0x42002018;
    uint8_t* STATUS = (uint8_t*)0x42002019;
    uint16_t* CALIB = (uint16_t*)0x42002028;

    *CTRLA = 1;
    do {
    } while (*STATUS < 0);
    *CALIB = (*((uint16_t*)0x806024) << 5) & 0x700 | *((uint16_t*)0x806020) >> 27 | (*((uint16_t*)0x806024) << 5) & 0xff | (*((uint16_t*)0x806024) & 7) << 5;
    *SAMPCTRL = 1;
    *REFCTRL = 3;
    *INPUTCTRL = 0x1270000;
    *CTRLB = 0x101;
    *INTFLAG = 0xF;
    *EVCTRL = 1;
    do {
    } while (*STATUS < 0);
}
