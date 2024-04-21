void ConfigureADC()
{

    uint8_t* CTRLA = (uint8_t*)0x42002000;
    uint8_t* STATUS = (uint8_t*)0x42002019;
    uint16_t* CALIB = (uint16_t*)0x42002028;

    *CTRLA = 1;

    do {
    } while (*STATUS < '\0');

  p[0x19] =
       (ushort)(*0x806024 << 5) & 0x700 |
       (ushort)(0x806024[-1] >> 0x1b) | (ushort)(*0x806024 << 5) & 0xff |
       (ushort)((*0x806024 & 7) << 5);
  puVar2[3] = 1;
  puVar2[1] = 3;
  *(undefined4 *)(puVar2 + 0x10) = DAT_00002af8;
  *(undefined2 *)(puVar2 + 4) = 0x101;
  puVar2[0x18] = 0xf;
  puVar2[0x14] = 1;
  do {
  } while ((char)puVar2[0x19] < '\0');
  return;
}
