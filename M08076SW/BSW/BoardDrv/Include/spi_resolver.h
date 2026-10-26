#ifndef USER_SPI_RESOLVER_H_
#define USER_SPI_RESOLVER_H_

#define  SAMPLE_Set     GpioDataRegs.GPASET.bit.GPIO21   = 1
#define  SAMPLE_Clear   GpioDataRegs.GPACLEAR.bit.GPIO21 = 1

#define  SPISTE_Set     GpioDataRegs.GPASET.bit.GPIO19   = 1
#define  SPISTE_Clear   GpioDataRegs.GPACLEAR.bit.GPIO19 = 1

#define  RDVEL_Set      GpioDataRegs.GPASET.bit.GPIO20   = 1
#define  RDVEL_Clear    GpioDataRegs.GPACLEAR.bit.GPIO20 = 1
#define  dummy          0xAA55
#define  Speed_TAU_1    1000// 0.001

//uint16_t  mech_angle;

unsigned char oneBitsInUShort (unsigned short);
void spi_init(void);
void spi_read(void);
void spi_fifo_init(void);

static unsigned char oneBitsInUChar[] = {
                                         //  0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F (<- n)
                                         //  =====================================================
                                             0, 1, 1, 2, 1, 2, 2, 3, 1, 2, 2, 3, 2, 3, 3, 4, // 0n
                                             1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, // 1n
                                             1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, // 2n
                                             2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, // 3n
                                             1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, // 4n
                                             2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, // 5n
                                             2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, // 6n
                                             3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, // 7n
                                             1, 2, 2, 3, 2, 3, 3, 4, 2, 3, 3, 4, 3, 4, 4, 5, // 8n
                                             2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, // 9n
                                             2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, // An
                                             3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, // Bn
                                             2, 3, 3, 4, 3, 4, 4, 5, 3, 4, 4, 5, 4, 5, 5, 6, // Cn
                                             3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, // Dn
                                             3, 4, 4, 5, 4, 5, 5, 6, 4, 5, 5, 6, 5, 6, 6, 7, // En
                                             4, 5, 5, 6, 5, 6, 6, 7, 5, 6, 6, 7, 6, 7, 7, 8, // Fn
                                         };

#endif /* USER_SPI_RESOLVER_H_ */
