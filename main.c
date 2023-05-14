/**
 ******************************************************************************
 * @file           : main.c
 * @author         : LorisRT
 * @brief          : MPU6050 driver + main file example
 ******************************************************************************
 * @attention
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */

#include <stdint.h>
#include <stdio.h>


/**
 * ***********************************
 * STM32F407 DISC ADDRESS DEFINITION *
 *************************************
 */
#define STM32F407_CLK	168
#define RCC_BASE_ADR	0x40023800
#define I2C1_BASE_ADR	0x40005400
#define GPIOB_BASE_ADR	0x40020400
#define GPIOD_BASE_ADR	0x40020c00
#define RCC_APB1ENR			(RCC_BASE_ADR + 0x40)
#define RCC_AHB1ENR			(RCC_BASE_ADR + 0x30)
#define RCC_CFGR			(RCC_BASE_ADR + 0x08)
#define RCC_CR				(RCC_BASE_ADR + 0x00)
#define I2C1_CR1		(I2C1_BASE_ADR + 0x00)
#define I2C1_CR2		(I2C1_BASE_ADR + 0x04)
#define I2C1_OAR1		(I2C1_BASE_ADR + 0x08)
#define I2C1_DR			(I2C1_BASE_ADR + 0x10)
#define I2C1_SR1		(I2C1_BASE_ADR + 0x14)
#define I2C1_SR2		(I2C1_BASE_ADR + 0x18)
#define I2C1_CCR		(I2C1_BASE_ADR + 0x1c)
#define I2C1_TRISE		(I2C1_BASE_ADR + 0x20)
#define GPIOB_MODER			(GPIOB_BASE_ADR + 0x00)
#define GPIOD_MODER			(GPIOD_BASE_ADR + 0x00)
#define GPIOD_OTYPER		(GPIOD_BASE_ADR + 0x04)
#define GPIOB_OTYPER		(GPIOB_BASE_ADR + 0x04)
#define GPIOD_ODR			(GPIOD_BASE_ADR + 0x14)
#define GPIOB_AFRL			(GPIOB_BASE_ADR + 0x20)
#define GPIOB_AFRH			(GPIOB_BASE_ADR + 0x24)
#define GPIOB_PUPDR			(GPIOB_BASE_ADR + 0x0c)
#define GPIOB_OSPEEDR		(GPIOB_BASE_ADR + 0x08)

#define CPACR			0xe000ed88



/**
 *****************************
 * MPU6050 ADRESS DEFINITION *
 *****************************
 */
#define MPU6050_ADR		0x68
#define WHO_AM_I		0x75
#define PWR_MGMT_1		0x6b
#define GYRO_CONFIG			0x1b
#define ACCEL_CONFIG		0x1c
#define ACCEL_XOUT_H	0x3b
#define ACCEL_XOUT_L	0x3c
#define ACCEL_YOUT_H	0x3d
#define ACCEL_YOUT_L	0x3e
#define ACCEL_ZOUT_H	0x3f
#define ACCEL_ZOUT_L	0x40
#define GYRO_XOUT_H			0x43
#define GYRO_XOUT_L			0x44
#define GYRO_YOUT_H			0x45
#define GYRO_YOUT_L			0x46
#define GYRO_ZOUT_H			0x47
#define GYRO_ZOUT_L			0x48

#define ACC_LSB		16384.0f /* [LSB/g] */
#define GYRO_LSB 	131.0f /* [LSB/°/s] */



/**
 *******************
 * ENUM DEFINITION *
 *******************
 */
typedef enum{
	MPU6050_OK = 0x00,
	MPU6050_NACK = 0x01,
	MPU6050_ADDR_FAIL = 0x02,
	MPU6050_READ_FAIL = 0x03,
	MPU6050_WRITE_FAIL = 0x04,
	MPU6050_ERROR = 0x05
} MPU6050_status_e;

typedef enum{
	SM = 0b0,
	FM = 0b1
} I2C_speedMode_e;

typedef enum{
	LOW = 0x00,
	HIGH = 0x01
} LED_status_e;

typedef enum{
	I2C_WRITE = 0x00,
	I2C_READ = 0x01,
} IC2_RW_status_e;

typedef enum{
	AF0 = 0x0,
	AF1 = 0x1,
	AF2 = 0x2,
	AF3 = 0x3,
	AF4 = 0x4,
	AF5 = 0x5,
	AF6 = 0x6,
	AF7 = 0x7,
	AF8 = 0x8,
	AF9 = 0x9,
	AF10 = 0xa,
	AF11 = 0xb,
	AF12 = 0xc,
	AF13 = 0xd,
	AF14 = 0xe,
	AF15 = 0xf,
}GPIO_AF_e;

typedef enum{
	HSI_div_1 = 0x0,
	HSI_div_2 = 0x8,
	HSI_div_4 = 0x9,
	HSI_div_8 = 0xa,
	HSI_div_16 = 0xb,
	HSI_div_64 = 0xc,
	HSI_div_128 = 0xd,
	HSI_div_256 = 0xe,
	HSI_div_512 = 0xf
} RCC_HSI_div_e;

typedef enum{
	APB1_div_1 = 0x0,
	APB1_div_2 = 0x4,
	APB1_div_4 = 0x5,
	APB1_div_8 = 0x6,
	APB1_div_16 = 0x7
} RCC_APB1_div_e;

typedef enum{
	ADDR_MISMATCHED = 0b0,
	ADDR_MATCHED = 0b1
} I2C_ADDR_status_e;

typedef enum{
	KO_START = 0b0,
	OK_START = 0b1
} I2C_SB_status_e;

typedef enum{
	ACK = 0b0,
	NACK = 0b1
} I2C_ACK_status_e;

typedef enum{
	BYTE_NOT_TRANSFERRED = 0b0,
	BYTE_TRANSFERRED = 0b1
} I2C_BTF_status_e;



/**
 ************************
 * STRUCTURE DEFINITION *
 ************************
 */
typedef struct{
	int16_t acc_x;
	int16_t acc_y;
	int16_t acc_z;
	int16_t temp;
	int16_t gyro_x;
	int16_t gyro_y;
	int16_t gyro_z;
} MPU_DataHandle_t;

typedef struct{
	int8_t dataHandle;
} I2C_ByteHandle_t;



/**
 ******************************************************
 * GLOBAL POINTER, STRUCTURE AND VARIABLE DECLARATION *
 ******************************************************
 */
static volatile uint32_t *p_RCC_APB1ENR = (uint32_t *) RCC_APB1ENR;
static volatile uint32_t *p_RCC_AHB1ENR = (uint32_t *) RCC_AHB1ENR;
static volatile uint32_t *p_GPIOB_MODER = (uint32_t *) GPIOB_MODER;
static volatile uint32_t *p_GPIOB_AFRH = (uint32_t *) GPIOB_AFRH;
static volatile uint32_t *p_GPIOB_OTYPER = (uint32_t *) GPIOB_OTYPER;
static volatile uint32_t *p_GPIOB_PUPDR = (uint32_t *) GPIOB_PUPDR;
static volatile uint32_t *p_GPIOB_OSPEEDR = (uint32_t *) GPIOB_OSPEEDR;
static volatile uint16_t *p_I2C1_CR1 = (uint16_t *) I2C1_CR1;
static volatile uint16_t *p_I2C1_CR2 = (uint16_t *) I2C1_CR2;
static volatile uint16_t *p_I2C1_DR = (uint16_t *) I2C1_DR;
static volatile uint16_t *p_I2C1_SR1 = (uint16_t *) I2C1_SR1;
static volatile uint16_t *p_I2C1_SR2 = (uint16_t *) I2C1_SR2;
static volatile uint16_t *p_I2C1_CCR = (uint16_t *) I2C1_CCR;
static volatile uint16_t *p_I2C1_TRISE = (uint16_t *) I2C1_TRISE;
static volatile uint32_t *p_CPACR = (uint32_t *) CPACR;

volatile MPU_DataHandle_t MPU_data_t;
volatile I2C_ByteHandle_t I2C_byte_t;
volatile MPU6050_status_e flag_status;

int16_t visual_ACC_Z;
int16_t visual_ACC_Y;
int16_t visual_ACC_X;



/**
 **********************
 * FUNCTION PROTOTYPE *
 **********************
 */
void enable_FPU(void);
void enable_I2C1_CLK(void);
void enable_GPIOB_CLK(void);
void enable_GPIOD_CLK(void);
static void init_MPU_struct(volatile MPU_DataHandle_t *p_MPU_data);
static void init_I2C_ByteHandle(volatile I2C_ByteHandle_t *pI2C_byte_t);
static I2C_ADDR_status_e I2C_STM_ADDR_status(void);
static I2C_ACK_status_e I2C_STM_ACK_status(void);
void I2C_init(void);
MPU6050_status_e I2C_Send_Address(uint8_t ADDR, uint8_t rw_bit);
MPU6050_status_e I2C_Master_Transmitter(uint8_t data);
MPU6050_status_e I2C_MPU6050_WriteByte(uint8_t slave_reg, uint8_t data);
MPU6050_status_e I2C_MPU6050_ReadByte(uint8_t slave_reg, volatile I2C_ByteHandle_t *pHandle);
MPU6050_status_e I2C_MPU6050_ReadBurst(uint8_t slave_reg, volatile uint8_t *buffer_array, uint8_t burst_size);



/**
 ************************
 * FUNCTION DECLARATION *
 ************************
 */
void enable_FPU(void)
{
	/* CP10 & CP11 full access for floating manipulation */
	*p_CPACR = (*p_CPACR | (0xf<<20));
}

void enable_I2C1_CLK(void)
{
	*p_RCC_APB1ENR = (*p_RCC_APB1ENR | (1<<21));
}


void enable_GPIOB_CLK(void)
{
	*p_RCC_AHB1ENR = (*p_RCC_AHB1ENR | (1<<1));
}


void enable_GPIOD_CLK(void)
{
	*p_RCC_AHB1ENR = (*p_RCC_AHB1ENR | (1<<3));
}


static inline void clear_ADDR_reg(void)
{
	/* Procedure to clear ADDR once set */
	uint16_t dummy_ReadVar;
	dummy_ReadVar = *p_I2C1_SR1;
	dummy_ReadVar = *p_I2C1_SR2;
	(void) dummy_ReadVar;
}


static void init_MPU_struct(volatile MPU_DataHandle_t *p_MPU_data)
{
	p_MPU_data->acc_x = 0;
	p_MPU_data->acc_y = 0;
	p_MPU_data->acc_z = 0;
	p_MPU_data->temp = 0;
	p_MPU_data->gyro_x = 0;
	p_MPU_data->gyro_y = 0;
	p_MPU_data->gyro_z = 0;
}


static void init_I2C_ByteHandle(volatile I2C_ByteHandle_t *pI2C_byte_t)
{
	pI2C_byte_t->dataHandle = 0;
}


static I2C_ADDR_status_e I2C_STM_ADDR_status(void)
{
	uint8_t ADDR_flag = (((uint8_t)(*p_I2C1_SR1>>1)) & (0x01));
	if(ADDR_flag == 0x00)
	{
		return ADDR_MISMATCHED;
	}
	else
	{
		return ADDR_MATCHED;
	}
}


static I2C_ACK_status_e I2C_STM_ACK_status(void)
{
	uint8_t AF_flag = (((uint8_t)(*p_I2C1_SR1>>10)) & (0x01));
	if(AF_flag == 0x00)
	{
		return ACK;
	}
	else
	{
		return NACK;
	}
}


void I2C_init(void)
{
	/**** STEPS FOLLOWED  ************
	1. Enable the I2C CLOCK and GPIO CLOCK
	2. Configure the I2C PINs for ALternate Functions
		a) Select Alternate Function in MODER Register
		b) Select Open Drain Output
		c) Select High SPEED for the PINs
		d) Select Pull-up for both the Pins
		e) Configure the Alternate Function in AFR Register
	3. Reset the I2C
	4. Program the peripheral input clock in I2C_CR2 Register in order to generate correct timings
	5. Configure the clock control registers
	6. Configure the rise time register
	7. Program the I2C_CR1 register to enable the peripheral
	*/

	*p_RCC_AHB1ENR = (*p_RCC_AHB1ENR | (0xa<<0));
	*p_RCC_APB1ENR = (*p_RCC_APB1ENR | (0x1<<21));

	*p_GPIOB_MODER = ((*p_GPIOB_MODER & ~(0xf<<16)) | (0xa<<16));
	*p_GPIOB_OTYPER = (*p_GPIOB_OTYPER | (0x3<<8));
	*p_GPIOB_OSPEEDR = ((*p_GPIOB_OSPEEDR & ~(0xf<<16)) | (0xa<<16));
	*p_GPIOB_PUPDR = ((*p_GPIOB_PUPDR & ~(0xf<<16)) | (0x5<<16));
	*p_GPIOB_AFRH = (*p_GPIOB_AFRH | (0x44<<0));

	*p_I2C1_CR1 = (*p_I2C1_CR1 | (1<<15));
	*p_I2C1_CR1 = (*p_I2C1_CR1 & ~(1<<15));

	/* Configure 100KHz I2C speed communication */
	*p_I2C1_CR2 = ((*p_I2C1_CR2 & ~(0x3f<<0)) | (0b010000<<0)); /* 16MHz for APB1 CLK value */
	*p_I2C1_CCR = (*p_I2C1_CCR & ~(1<<15));
	*p_I2C1_CCR = ((*p_I2C1_CCR & ~(0xfff<<0)) | (0x050<<0));
	*p_I2C1_TRISE = (((*p_I2C1_TRISE) & ~(0b111111<<0)) | (0b010001<<0)); /* (1000ns/62.5ns + 1) = 17 */

	*p_I2C1_CR1 = (*p_I2C1_CR1 | (1<<0));
}


MPU6050_status_e I2C_Send_Address(uint8_t ADDR, uint8_t rw_bit)
{
	/* Enable ACK and generate start condition*/
	*p_I2C1_CR1 = (*p_I2C1_CR1 | (1<<10));
	*p_I2C1_CR1 = (*p_I2C1_CR1 | (1<<8));
	while((((uint8_t)(*p_I2C1_SR1)) & (0x01)) == 0x00);

	/* Slave ADDR transmission */
	*p_I2C1_DR = ((ADDR<<1) | (rw_bit<<0));
	while((I2C_STM_ADDR_status() == ADDR_MISMATCHED) && (I2C_STM_ACK_status() != NACK)); /* Note: ADDR is not set after NACK reception */
	if(I2C_STM_ACK_status() == NACK)
	{
		return MPU6050_ADDR_FAIL;
	}
	clear_ADDR_reg();
	
	return MPU6050_OK;
}


MPU6050_status_e I2C_Master_Transmitter(uint8_t data)
{
	/* Write Data after addressing */
	*p_I2C1_DR = data;
	while(((((uint8_t)(*p_I2C1_SR1>>7)) & (0x01)) == 0x00) && (I2C_STM_ACK_status() != NACK));
	if(I2C_STM_ACK_status() == NACK)
	{
		return MPU6050_WRITE_FAIL;
	}
	
	return MPU6050_OK;
}


MPU6050_status_e I2C_MPU6050_WriteByte(uint8_t slave_reg, uint8_t data)
{
	/* Send slave ADDR with write condition and write register value */
	if(I2C_Send_Address(MPU6050_ADR, I2C_WRITE) != MPU6050_OK)
	{
		return MPU6050_ADDR_FAIL;
	}
	if(I2C_Master_Transmitter(slave_reg) != MPU6050_OK)
	{
		return MPU6050_WRITE_FAIL;
	}
	if(I2C_Master_Transmitter(data) != MPU6050_OK)
	{
		return MPU6050_WRITE_FAIL;
	}

	/* Send stop condition */
	*p_I2C1_CR1 = (*p_I2C1_CR1 | (1<<9));
	
	return MPU6050_OK;
}


MPU6050_status_e I2C_MPU6050_ReadByte(uint8_t slave_reg, volatile I2C_ByteHandle_t *pHandle)
{
	/* Send slave register ADDR to and write MPU6050 register to be read */
	if(I2C_Send_Address(MPU6050_ADR, I2C_WRITE) != MPU6050_OK)
	{
		return MPU6050_ADDR_FAIL;
	}
	if(I2C_Master_Transmitter(slave_reg) != MPU6050_OK)
	{
		return MPU6050_WRITE_FAIL;
	}

	/* Generate start condition*/
	*p_I2C1_CR1 = (*p_I2C1_CR1 | (1<<8));
	while((((uint8_t)(*p_I2C1_SR1)) & (0x01)) == 0x00);

	/* Slave ADDR transmission and read MPU6050 register */
	*p_I2C1_DR = ((MPU6050_ADR<<1) | (I2C_READ<<0));
	while((I2C_STM_ADDR_status() == ADDR_MISMATCHED) && (I2C_STM_ACK_status() != NACK)); /* Note: ADDR is not set after NACK reception */
	if(I2C_STM_ACK_status() == NACK)
	{
		return MPU6050_READ_FAIL;
	}
	/* Disable ACK & generate STOP condition*/
	*p_I2C1_CR1 = (*p_I2C1_CR1 & ~(1<<10));
	clear_ADDR_reg();
	*p_I2C1_CR1 = (*p_I2C1_CR1 | (1<<9));

	/* Read byte from data register */
	while((((uint8_t)(*p_I2C1_SR1>>6)) & (0x01)) == 0x00);
	pHandle->dataHandle = *p_I2C1_DR;

	return MPU6050_OK;
}


MPU6050_status_e I2C_MPU6050_ReadBurst(uint8_t slave_reg, volatile uint8_t *buffer_array, uint8_t burst_size)
{
	uint8_t temp_size = burst_size;
	uint8_t temp_idx = 0;

	/* Send slave register ADDR to be read */
	if(I2C_Send_Address(MPU6050_ADR, I2C_WRITE) != MPU6050_OK)
	{
		return MPU6050_ERROR;
	}
	if(I2C_Master_Transmitter(slave_reg) != MPU6050_OK)
	{
		return MPU6050_ERROR;
	}

	/* Generate start condition*/
	*p_I2C1_CR1 = (*p_I2C1_CR1 | (1<<8));
	while((((uint8_t)(*p_I2C1_SR1)) & (0x01)) == 0x00);
	/* Send slave ADDR with read bit */
	*p_I2C1_DR = ((MPU6050_ADR<<1) | (I2C_READ<<0));
	while((I2C_STM_ADDR_status() == ADDR_MISMATCHED) && (I2C_STM_ACK_status() != NACK)); /* Note: ADDR is not set after NACK reception */
	if(I2C_STM_ACK_status() == NACK)
	{
		return MPU6050_ERROR;
	}
	clear_ADDR_reg();

	while(temp_size>2U)
	{
		while((((uint8_t)(*p_I2C1_SR1>>6)) & (0x01)) == 0x00);
		*(buffer_array + temp_idx) = *p_I2C1_DR;
		--temp_size;
		++temp_idx;
	}

	while((((uint8_t)(*p_I2C1_SR1>>6)) & (0x01)) == 0x00);
	*(buffer_array + temp_idx) = *p_I2C1_DR;
	++temp_idx;

	/* Disable ACK & generate STOP condition*/
	*p_I2C1_CR1 = (*p_I2C1_CR1 & ~(1<<10));
	*p_I2C1_CR1 = (*p_I2C1_CR1 | (1<<9));

	while((((uint8_t)(*p_I2C1_SR1>>6)) & (0x01)) == 0x00);
	*(buffer_array + temp_idx) = *p_I2C1_DR;
	++temp_idx;

	return MPU6050_OK;
}



/**
 *************************
 * MAIN LOOP SOURCE CODE *
 *************************
 */
int main(void)
{
	/* ARM M4 configuration for floating point manipulation*/
	enable_FPU();

	/* Execution flag declaration */
	uint8_t temp_data = 0x00;
	MPU6050_status_e flag_device_id;
	MPU6050_status_e flag_device_init;
	MPU6050_status_e flag_getData = MPU6050_OK;

	/* Buffer declaration for data acquisition and initialisation*/
	volatile uint8_t butter_a[14];
	uint8_t buffer_size = 14;
	int i;
	for(i=0; i<buffer_size; i++)
	{
		butter_a[i] = 0;
	}

	/* Data structure initialisation */
	init_MPU_struct(&MPU_data_t);
	init_I2C_ByteHandle(&I2C_byte_t);

	/* I2C peripheral configuration */
	I2C_init();

	/* Check device ID */
	flag_device_id = I2C_MPU6050_ReadByte(WHO_AM_I, &I2C_byte_t);
	if(flag_device_id == MPU6050_OK)
	{
		/* Configure MPU6050 register and disable sleep mode */
		if(I2C_MPU6050_ReadByte(PWR_MGMT_1, &I2C_byte_t) == MPU6050_OK)
		{
			/* Disable sleep mode */
			temp_data = ((((uint8_t) I2C_byte_t.dataHandle)) & ~(1<<6));
			flag_device_init = I2C_MPU6050_WriteByte(PWR_MGMT_1, temp_data);
		}
		if(flag_device_init != MPU6050_OK)
		{
			/* TO BE MODIFIED: if communication error during initialisation, loop forever */
			for(;;);
		}

		/* Loop forever to read data from ACC, TEMP and GYRO */
		while(flag_getData == MPU6050_OK)
		{
			/* Data treatment */
			MPU_data_t.acc_x = (int16_t) ((butter_a[0]<<8) | (butter_a[1]<<0));
			MPU_data_t.acc_y = (int16_t) ((butter_a[2]<<8) | (butter_a[3]<<0));
			MPU_data_t.acc_z = (int16_t) ((butter_a[4]<<8) | (butter_a[5]<<0));
			MPU_data_t.temp = (int16_t) ((butter_a[6]<<8) | (butter_a[7]<<0));
			MPU_data_t.gyro_x = (int16_t) ((butter_a[8]<<8) | (butter_a[9]<<0));
			MPU_data_t.gyro_y = (int16_t) ((butter_a[10]<<8) | (butter_a[11]<<0));
			MPU_data_t.gyro_z = (int16_t) ((butter_a[12]<<8) | (butter_a[13]<<0));

			/**
			 * Global variable definition for STM Studio
			 * VarViewer: display accelerometers value through visual_ variable in [m/s^2]
			 */
			visual_ACC_Z = (int16_t) ((((float) MPU_data_t.acc_z)/ACC_LSB)*9.80665f);
			visual_ACC_Y = (int16_t) ((((float) MPU_data_t.acc_y)/ACC_LSB)*9.80665f);
			visual_ACC_X = (int16_t) ((((float) MPU_data_t.acc_x)/ACC_LSB)*9.80665f);

			/* Refresh data acquisition */
			flag_getData = I2C_MPU6050_ReadBurst(ACCEL_XOUT_H, butter_a, buffer_size);
		}
	}

	/**
	 * LOOP FOREVER:
	 * Should never get here, otherwise:
	 * - Device identification failed, check slave ID and WHO_AM_I value;
	 * - Error in data acquisition.
	 */
	for(;;);
}
