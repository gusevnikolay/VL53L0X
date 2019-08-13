#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include "tof.h"

#define calcMacroPeriod(vcsel_period_pclks) ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

uint8_t address_list[TOF_SENSORS_COUNT] = TOF_ADRESSES_LIST; 
uint8_t current_device_addr = 0x52;

static unsigned char stop_variable;
static uint32_t measurement_timing_budget_us;
static unsigned char  readReg(unsigned char ucAddr);
static unsigned short readReg16(unsigned char ucAddr);
static void           writeReg16(unsigned char ucAddr, unsigned short usValue);
static void           writeReg(unsigned char ucAddr, unsigned char ucValue);
static void           writeRegList(unsigned char *ucList);
static int            initSensor(int);
static int            performSingleRefCalibration(uint8_t vhv_init_byte);
static int            setMeasurementTimingBudget(uint32_t budget_us);


typedef enum vcselperiodtype { VcselPeriodPreRange, VcselPeriodFinalRange } vcselPeriodType;
static int setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks);

typedef struct tagSequenceStepTimeouts
{
    uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
} SequenceStepTimeouts;

#define SEQUENCE_ENABLE_FINAL_RANGE 0x80
#define SEQUENCE_ENABLE_PRE_RANGE   0x40
#define SEQUENCE_ENABLE_TCC         0x10
#define SEQUENCE_ENABLE_DSS         0x08
#define SEQUENCE_ENABLE_MSRC        0x04

#define REG_IDENTIFICATION_MODEL_ID		                0xc0
#define REG_IDENTIFICATION_REVISION_ID		            0xc2
#define REG_SYSRANGE_START			                      0x00
#define REG_RESULT_INTERRUPT_STATUS 		              0x13
#define RESULT_RANGE_STATUS      		                  0x14
#define ALGO_PHASECAL_LIM                             0x30
#define ALGO_PHASECAL_CONFIG_TIMEOUT                  0x30
#define GLOBAL_CONFIG_VCSEL_WIDTH                     0x32
#define FINAL_RANGE_CONFIG_VALID_PHASE_LOW            0x47
#define FINAL_RANGE_CONFIG_VALID_PHASE_HIGH           0x48
#define PRE_RANGE_CONFIG_VCSEL_PERIOD                 0x50
#define PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI            0x51
#define PRE_RANGE_CONFIG_VALID_PHASE_LOW              0x56
#define PRE_RANGE_CONFIG_VALID_PHASE_HIGH             0x57
#define REG_MSRC_CONFIG_CONTROL                       0x60
#define FINAL_RANGE_CONFIG_VCSEL_PERIOD               0x70
#define FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI          0x71
#define MSRC_CONFIG_TIMEOUT_MACROP                    0x46
#define FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT   0x44
#define SYSRANGE_START                                0x00
#define SYSTEM_SEQUENCE_CONFIG                        0x01
#define SYSTEM_INTERRUPT_CONFIG_GPIO                  0x0A
#define RESULT_INTERRUPT_STATUS                       0x13
#define VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV             0x89
#define GLOBAL_CONFIG_SPAD_ENABLES_REF_0              0xB0
#define GPIO_HV_MUX_ACTIVE_HIGH                       0x84
#define SYSTEM_INTERRUPT_CLEAR                        0x0B

unsigned char ucI2CMode[] = {4, 0x88,0x00, 0x80,0x01, 0xff,0x01, 0x00,0x00};
unsigned char ucI2CMode2[] = {3, 0x00,0x01, 0xff,0x00, 0x80,0x00};
unsigned char ucSPAD0[] = {4, 0x80,0x01, 0xff,0x01, 0x00,0x00, 0xff,0x06};
unsigned char ucSPAD1[] = {5, 0xff,0x07, 0x81,0x01, 0x80,0x01, 0x94,0x6b, 0x83,0x00};
unsigned char ucSPAD2[] = {4, 0xff,0x01, 0x00,0x01, 0xff,0x00, 0x80,0x00};
unsigned char ucSPAD[] = {5, 0xff,0x01, 0x4f,0x00, 0x4e,0x2c, 0xff,0x00, 0xb6,0xb4};
unsigned char ucDefTuning[] = {80, 0xff,0x01, 0x00,0x00, 0xff,0x00, 0x09,0x00,
0x10,0x00, 0x11,0x00, 0x24,0x01, 0x25,0xff, 0x75,0x00, 0xff,0x01, 0x4e,0x2c,
0x48,0x00, 0x30,0x20, 0xff,0x00, 0x30,0x09, 0x54,0x00, 0x31,0x04, 0x32,0x03,
0x40,0x83, 0x46,0x25, 0x60,0x00, 0x27,0x00, 0x50,0x06, 0x51,0x00, 0x52,0x96,
0x56,0x08, 0x57,0x30, 0x61,0x00, 0x62,0x00, 0x64,0x00, 0x65,0x00, 0x66,0xa0,
0xff,0x01, 0x22,0x32, 0x47,0x14, 0x49,0xff, 0x4a,0x00, 0xff,0x00, 0x7a,0x0a,
0x7b,0x00, 0x78,0x21, 0xff,0x01, 0x23,0x34, 0x42,0x00, 0x44,0xff, 0x45,0x26,
0x46,0x05, 0x40,0x40, 0x0e,0x06, 0x20,0x1a, 0x43,0x40, 0xff,0x00, 0x34,0x03,
0x35,0x44, 0xff,0x01, 0x31,0x04, 0x4b,0x09, 0x4c,0x05, 0x4d,0x04, 0xff,0x00,
0x44,0x00, 0x45,0x20, 0x47,0x08, 0x48,0x28, 0x67,0x00, 0x70,0x04, 0x71,0x01,
0x72,0xfe, 0x76,0x00, 0x77,0x00, 0xff,0x01, 0x0d,0x01, 0xff,0x00, 0x80,0x01,
0x01,0xf8, 0xff,0x01, 0x8e,0x01, 0x00,0x01, 0xff,0x00, 0x80,0x00};

int tofInit()
{
	return initSensor(1); 
} 

static unsigned short readReg16(unsigned char ucAddr)
{
  uint8_t tmp[2];
	i2cRead(current_device_addr, ucAddr, tmp, 2);
	return (tmp[0]*256+tmp[1]);
} 

static unsigned char readReg(unsigned char ucAddr)
{
	 uint8_t tmp[1];
   i2cRead(current_device_addr, ucAddr, tmp, 1);
	 return tmp[0];
}

static void readMulti(unsigned char ucAddr, unsigned char *pBuf, int iCount)
{		
    i2cRead(current_device_addr, ucAddr, pBuf, iCount);
} 

static void writeMulti(unsigned char ucAddr, unsigned char *pBuf, int iCount)
{
	  i2cWrite(current_device_addr, ucAddr, pBuf, iCount);
} 

static void writeReg16(unsigned char ucAddr, unsigned short usValue)
{
	uint8_t data[2];
	data[0] =  (usValue >> 8) & 0xFF;
	data[1] =   usValue & 0xFF;
  i2cWrite(current_device_addr, ucAddr, data, 2);
} 

static void writeReg(unsigned char ucAddr, unsigned char ucValue)
{
	uint8_t data[2];
	data[0] =  ucValue;
	i2cWrite(current_device_addr, ucAddr, data, 1);
} 

static void writeRegList(unsigned char *ucList)
{
  unsigned char ucCount = *ucList++;
	while (ucCount)
	{
		writeReg(ucList[0], ucList[1]);
		ucList += 2;
		ucCount--;
	}
} 

static int getSpadInfo(unsigned char *pCount, unsigned char *pTypeIsAperture)
{
  int iTimeout;
  unsigned char ucTemp;
  #define MAX_TIMEOUT 50
  writeRegList(ucSPAD0);
  writeReg(0x83, readReg(0x83) | 0x04);
  writeRegList(ucSPAD1);
  iTimeout = 0;
  while(iTimeout < MAX_TIMEOUT)
  {
    if(readReg(0x83) != 0x00) break;
    iTimeout++;
    varDelayMs(1);
  }
  if (iTimeout == MAX_TIMEOUT)
  {
    return 0;
  }
  writeReg(0x83,0x01);
  ucTemp = readReg(0x92);
  *pCount = (ucTemp & 0x7f);
  *pTypeIsAperture = (ucTemp & 0x80);
  writeReg(0x81,0x00);
  writeReg(0xff,0x06);
  writeReg(0x83, readReg(0x83) & ~0x04);
  writeRegList(ucSPAD2); 
  return 1;
} 

static uint16_t decodeTimeout(uint16_t reg_val)
{
  return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
  return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
  uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
  return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

static uint16_t encodeTimeout(uint16_t timeout_mclks)
{
  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;
  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;
    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }
    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

static void getSequenceStepTimeouts(uint8_t enables, SequenceStepTimeouts * timeouts)
{
  timeouts->pre_range_vcsel_period_pclks    = ((readReg(PRE_RANGE_CONFIG_VCSEL_PERIOD) +1) << 1);
  timeouts->msrc_dss_tcc_mclks              = readReg(MSRC_CONFIG_TIMEOUT_MACROP) + 1;
  timeouts->msrc_dss_tcc_us                 = timeoutMclksToMicroseconds(timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);
  timeouts->pre_range_mclks                 = decodeTimeout(readReg16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  timeouts->pre_range_us                    = timeoutMclksToMicroseconds(timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);
  timeouts->final_range_vcsel_period_pclks  = ((readReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD) +1) << 1);
  timeouts->final_range_mclks               = decodeTimeout(readReg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));
  if (enables & SEQUENCE_ENABLE_PRE_RANGE){ timeouts->final_range_mclks -= timeouts->pre_range_mclks; }
  timeouts->final_range_us = timeoutMclksToMicroseconds(timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);
} 

static int setVcselPulsePeriod(vcselPeriodType type, uint8_t period_pclks)
{
  uint8_t vcsel_period_reg = encodeVcselPeriod(period_pclks);
  uint8_t enables;
  SequenceStepTimeouts timeouts;
  enables = readReg(SYSTEM_SEQUENCE_CONFIG);
  getSequenceStepTimeouts(enables, &timeouts);
  if (type == VcselPeriodPreRange)
  {
    switch (period_pclks)
    {
      case 12:
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x18);
        break;
      case 14:
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x30);
        break;
      case 16:
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x40);
        break;
      case 18:
        writeReg(PRE_RANGE_CONFIG_VALID_PHASE_HIGH, 0x50);
        break;
      default:
        return 0;
    }
    writeReg(PRE_RANGE_CONFIG_VALID_PHASE_LOW, 0x08);
    writeReg(PRE_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
    uint16_t new_pre_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.pre_range_us, period_pclks);
    writeReg16(PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(new_pre_range_timeout_mclks));
    uint16_t new_msrc_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.msrc_dss_tcc_us, period_pclks);
    writeReg(MSRC_CONFIG_TIMEOUT_MACROP, (new_msrc_timeout_mclks > 256) ? 255 : (new_msrc_timeout_mclks - 1));
  }
  else if (type == VcselPeriodFinalRange)
  {
    switch (period_pclks)
    {
      case 8:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x10);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x02);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x0C);
        writeReg(0xFF, 0x01);
        writeReg(ALGO_PHASECAL_LIM, 0x30);
        writeReg(0xFF, 0x00);
        break;

      case 10:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x28);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x09);
        writeReg(0xFF, 0x01);
        writeReg(ALGO_PHASECAL_LIM, 0x20);
        writeReg(0xFF, 0x00);
        break;

      case 12:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x38);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x08);
        writeReg(0xFF, 0x01);
        writeReg(ALGO_PHASECAL_LIM, 0x20);
        writeReg(0xFF, 0x00);
        break;

      case 14:
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_HIGH, 0x48);
        writeReg(FINAL_RANGE_CONFIG_VALID_PHASE_LOW,  0x08);
        writeReg(GLOBAL_CONFIG_VCSEL_WIDTH, 0x03);
        writeReg(ALGO_PHASECAL_CONFIG_TIMEOUT, 0x07);
        writeReg(0xFF, 0x01);
        writeReg(ALGO_PHASECAL_LIM, 0x20);
        writeReg(0xFF, 0x00);
        break;

      default:
        return 0;
    }

    writeReg(FINAL_RANGE_CONFIG_VCSEL_PERIOD, vcsel_period_reg);
    uint16_t new_final_range_timeout_mclks = timeoutMicrosecondsToMclks(timeouts.final_range_us, period_pclks);
    if (enables & SEQUENCE_ENABLE_PRE_RANGE)
    {
      new_final_range_timeout_mclks += timeouts.pre_range_mclks;
    }
    writeReg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
    encodeTimeout(new_final_range_timeout_mclks));
  }
  else
  {
    return 0;
  }
  setMeasurementTimingBudget(measurement_timing_budget_us);
  uint8_t sequence_config = readReg(SYSTEM_SEQUENCE_CONFIG);
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
  performSingleRefCalibration(0x0);
  writeReg(SYSTEM_SEQUENCE_CONFIG, sequence_config);
  return 1;
}

static int setMeasurementTimingBudget(uint32_t budget_us)
{
  uint32_t used_budget_us;
  uint32_t final_range_timeout_us;
  uint16_t final_range_timeout_mclks;
  uint8_t enables;
  SequenceStepTimeouts timeouts;
  uint16_t const StartOverhead      = 1320; // note that this is different than the value in get_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;
  uint32_t const MinTimingBudget = 20000;
  if (budget_us < MinTimingBudget) { return 0; }
  used_budget_us = StartOverhead + EndOverhead;
  enables = readReg(SYSTEM_SEQUENCE_CONFIG);
  getSequenceStepTimeouts(enables, &timeouts);
  if (enables & SEQUENCE_ENABLE_TCC)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }
  if (enables & SEQUENCE_ENABLE_DSS)
  {
    used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables & SEQUENCE_ENABLE_MSRC)
  {
    used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }
  if (enables & SEQUENCE_ENABLE_PRE_RANGE)
  {
    used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }
  if (enables & SEQUENCE_ENABLE_FINAL_RANGE)
  {
    used_budget_us += FinalRangeOverhead;
    if (used_budget_us > budget_us)
    {
      return 0;
    }
    final_range_timeout_us    = budget_us - used_budget_us;
    final_range_timeout_mclks = timeoutMicrosecondsToMclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);
    if (enables & SEQUENCE_ENABLE_PRE_RANGE)
    {
      final_range_timeout_mclks += timeouts.pre_range_mclks;
    }
    writeReg16(FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI, encodeTimeout(final_range_timeout_mclks));
    measurement_timing_budget_us = budget_us; // store for internal reuse
  }
  return 1;
}

static uint32_t getMeasurementTimingBudget(void)
{
  uint8_t enables;
  SequenceStepTimeouts timeouts;
  uint16_t const StartOverhead     = 1910; // note that this is different than the value in set_
  uint16_t const EndOverhead        = 960;
  uint16_t const MsrcOverhead       = 660;
  uint16_t const TccOverhead        = 590;
  uint16_t const DssOverhead        = 690;
  uint16_t const PreRangeOverhead   = 660;
  uint16_t const FinalRangeOverhead = 550;
  uint32_t budget_us = StartOverhead + EndOverhead;
  enables = readReg(SYSTEM_SEQUENCE_CONFIG);
  getSequenceStepTimeouts(enables, &timeouts);
  if (enables & SEQUENCE_ENABLE_TCC)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
  }
  if (enables & SEQUENCE_ENABLE_DSS)
  {
    budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
  }
  else if (enables & SEQUENCE_ENABLE_MSRC)
  {
    budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
  }
  if (enables & SEQUENCE_ENABLE_PRE_RANGE)
  {
    budget_us += (timeouts.pre_range_us + PreRangeOverhead);
  }
  if (enables & SEQUENCE_ENABLE_FINAL_RANGE)
  {
    budget_us += (timeouts.final_range_us + FinalRangeOverhead);
  }
  measurement_timing_budget_us = budget_us;
  return budget_us;
}

static int performSingleRefCalibration(uint8_t vhv_init_byte)
{
  int iTimeout;
  writeReg(SYSRANGE_START, 0x01 | vhv_init_byte); 
  iTimeout = 0;
  while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    iTimeout++;
    varDelayMs(1);
    if (iTimeout > 100) { return 0; }
  }
  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
  writeReg(SYSRANGE_START, 0x00);
  return 1;
} 

static int initSensor(int bLongRangeMode)
{
  unsigned char spad_count=0, spad_type_is_aperture=0, ref_spad_map[6];
  unsigned char ucFirstSPAD, ucSPADsEnabled;
  int i;
  writeReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
  readReg(VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01); 
  writeRegList(ucI2CMode);
  stop_variable = readReg(0x91);
  writeRegList(ucI2CMode2);
  writeReg(REG_MSRC_CONFIG_CONTROL, readReg(REG_MSRC_CONFIG_CONTROL) | 0x12);
  writeReg16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 32); // 0.25
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xFF);
  getSpadInfo(&spad_count, &spad_type_is_aperture);
  readMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
  writeRegList(ucSPAD);
  ucFirstSPAD = (spad_type_is_aperture) ? 12: 0;
  ucSPADsEnabled = 0;
  for (i=0; i<48; i++)
  {
    if (i < ucFirstSPAD || ucSPADsEnabled == spad_count)
    {
      ref_spad_map[i>>3] &= ~(1<<(i & 7));
    }
    else if (ref_spad_map[i>>3] & (1<< (i & 7)))
    {
      ucSPADsEnabled++;
    }
  } 
  writeMulti(GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);
  writeRegList(ucDefTuning); 
  if (bLongRangeMode)
  {
    writeReg16(FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT, 13); // 0.1
    setVcselPulsePeriod(VcselPeriodPreRange, 18);
    setVcselPulsePeriod(VcselPeriodFinalRange, 14);
  }
  writeReg(SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
  writeReg(GPIO_HV_MUX_ACTIVE_HIGH, readReg(GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10); // active low
  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
  measurement_timing_budget_us = getMeasurementTimingBudget();
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xe8);
  setMeasurementTimingBudget(measurement_timing_budget_us);
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0x01);
  if (!performSingleRefCalibration(0x40)) { return 0; }
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0x02);
  if (!performSingleRefCalibration(0x00)) { return 0; }
  writeReg(SYSTEM_SEQUENCE_CONFIG, 0xe8);
  return 1;
} 

uint16_t readRangeContinuousMillimeters(void)
{
  int iTimeout = 0;
  uint16_t range;
  while ((readReg(RESULT_INTERRUPT_STATUS) & 0x07) == 0)
  {
    iTimeout++;
    varDelayMs(2);
    if (iTimeout > 50)
    {
      return 0;
    }
  }
  range = readReg16(RESULT_RANGE_STATUS + 10);
  writeReg(SYSTEM_INTERRUPT_CLEAR, 0x01);
  return range;
}

int tofReadDistance(uint8_t chipId)
{
	if(chipId>=TOF_SENSORS_COUNT)return 0;
	int iTimeout;
	current_device_addr = address_list[chipId]*2;
  writeReg(0x80, 0x01);
  writeReg(0xFF, 0x01);
  writeReg(0x00, 0x00);
  writeReg(0x91, stop_variable);
  writeReg(0x00, 0x01);
  writeReg(0xFF, 0x00);
  writeReg(0x80, 0x00);
  writeReg(SYSRANGE_START, 0x01);
  iTimeout = 0;
  while (readReg(SYSRANGE_START) & 0x01)
  {
    iTimeout++;
    varDelayMs(2);
    if (iTimeout > 50)
    {
      return -1;
    }
  }
  return readRangeContinuousMillimeters();
} 

void tofInitAll(void)
{
	for(int i=0;i<TOF_SENSORS_COUNT;i++)tofSetResetState(i, 0);
	varDelayMs(20);
	for(int i=0;i<TOF_SENSORS_COUNT;i++)
	{
		
		tofSetResetState(i, 1);
		varDelayMs(1);
		current_device_addr = 0x52;
		tofInit();
		writeReg (0x8A, address_list[i]);		
	}
}
