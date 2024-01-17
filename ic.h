/*
 * ic.h
 *
 *  Created on: May 06, 2023
 *      Author: MUBARIS KT
 */

#ifndef INC_BQ76940_H_
#define INC_BQ76940_H_




#define BQ76940_ADDRESS  0x10

#define SYS_STAT 0x00

#define CELLBAL1 0x01
#define CELLBAL2 0x02
#define CELLBAL3 0x03

#define SYS_CTRL1 0x04
#define SYS_CTRL2 0x05

#define PROTECT1 0x06
#define PROTECT2 0x07
#define PROTECT3 0x08

#define OV_TRIP 0x09
#define CC_CFG  0x0A

#define VC1_HI  0x0C
#define VC2_HI  0x0E
#define VC3_HI  0x10
#define VC4_HI  0x12
#define VC5_HI  0x14
#define VC6_HI  0x16
#define VC7_HI  0x18
#define VC8_HI  0x1A
#define VC9_HI  0x1C
#define VC10_HI 0x1E
#define VC11_HI 0x20
#define VC12_HI 0x22
#define VC13_HI 0x24
#define VC14_HI 0x26
#define VC15_HI 0x28

#define VC1_LO  0x0D
#define VC2_LO  0x0F
#define VC3_LO  0x11
#define VC4_LO  0x13
#define VC5_LO  0x15
#define VC6_LO  0x17
#define VC7_LO  0x19
#define VC8_LO  0x1B
#define VC9_LO  0x1D
#define VC10_LO 0x1F
#define VC11_LO 0x21
#define VC12_LO 0x23
#define VC13_LO 0x25
#define VC14_LO 0x27
#define VC15_LO 0x29

#define BAT_HI 0x2A
#define BAT_LO 0x2B

#define TS1_HI 0x2C
#define TS2_HI 0x2E
#define TS3_HI 0x30

#define TS1_LO 0x2D
#define TS2_LO 0x2F
#define TS3_LO 0x31

#define CC_HI  0x32
#define CC_LO  0x33

#define ADCGAIN1 0x50
#define ADCOFFSET 0x51
#define ADCGAIN2 0x59

void GET_Cell_DATA(int[]);
float GET_Single_Cell_DATA(uint16_t);
void GET_ADC_Calib_DATA(void);

int GET_Status(void);

void Enable_Discharge(void);
void Enable_Charge(void);

void Disable_Discharge(void);
void Disable_Charge(void);

void Enable_Cell_BalanceFET(uint8_t);
void Disable_Cell_BalanceFET(uint8_t);

int check_adjacent_balancing(void);
int IsALIVE(void);

int Check_Fault(void);
void Clear_ALL_Fault(void);
void Shutdown(void);

int GET_Temp_data(int[]);
int Get_Pack_Voltage(void);
int OVP_ON (int);
int UVP_ON (int);
int Get_CC_Reading (void);

#endif
