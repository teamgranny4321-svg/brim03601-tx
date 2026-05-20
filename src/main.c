/**
 * ============================================================
 *  TX Firmware  –  BRIM03601
 *  Target   : STM32F103C8T6  (Blue Pill)
 *  Flash    : ST-Link V2  (SWD)
 *  LoRa     : SX1278  @  433 MHz  (SPI1)
 *  Function : Tank sensor read → send to RX (BRIM036)
 *             0%  → info = 0   (tank empty)
 *            100% → info = 31  (tank full)
 *             Sleep between transmissions  (battery save)
 * ============================================================
 *
 *  STM32 SPI1 Pin Map (matches RX LoRa pins logic):
 *  ─────────────────────────────────────────────────
 *  NSS  (CS)   → PA4
 *  SCK         → PA5
 *  MISO        → PA6
 *  MOSI        → PA7
 *  RESET       → PB0
 *  DIO0 (IRQ)  → PB1
 *
 *  Sensor Input:
 *  ─────────────
 *  SENSOR_PIN  → PA0  (ADC1_IN0)
 *    0V   = 0%   (empty)
 *    3.3V = 100% (full)
 *
 *  Build with:  STM32CubeIDE  or  PlatformIO (framework=cmsis)
 *  Flash with:  ST-Link V2 via SWD (SWDIO=PA13, SWDCLK=PA14)
 * ============================================================
 */

#include "stm32f1xx.h"
#include <string.h>
#include <stdio.h>

/* ──────────────────────────────────────────────
   PIN DEFINITIONS
   ────────────────────────────────────────────── */
#define LORA_NSS_PIN      GPIO_PIN_4    // PA4
#define LORA_NSS_PORT     GPIOA

#define LORA_RESET_PIN    GPIO_PIN_0    // PB0
#define LORA_RESET_PORT   GPIOB

#define LORA_DIO0_PIN     GPIO_PIN_1    // PB1
#define LORA_DIO0_PORT    GPIOB

#define SENSOR_PIN        GPIO_PIN_0    // PA0 (ADC)
#define SENSOR_PORT       GPIOA

#define LED_PIN           GPIO_PIN_13   // PC13 (onboard LED)
#define LED_PORT          GPIOC

/* ──────────────────────────────────────────────
   LoRa SX1278 REGISTER MAP
   ────────────────────────────────────────────── */
#define REG_FIFO                0x00
#define REG_OP_MODE             0x01
#define REG_FRF_MSB             0x06
#define REG_FRF_MID             0x07
#define REG_FRF_LSB             0x08
#define REG_PA_CONFIG           0x09
#define REG_FIFO_ADDR_PTR       0x0D
#define REG_FIFO_TX_BASE_ADDR   0x0E
#define REG_IRQ_FLAGS           0x12
#define REG_PAYLOAD_LENGTH      0x22
#define REG_MODEM_CONFIG_1      0x1D
#define REG_MODEM_CONFIG_2      0x1E
#define REG_PREAMBLE_MSB        0x20
#define REG_PREAMBLE_LSB        0x21
#define REG_DIO_MAPPING_1       0x40
#define REG_VERSION             0x42

#define MODE_LONG_RANGE         0x80
#define MODE_SLEEP              0x00
#define MODE_STDBY              0x01
#define MODE_TX                 0x03
#define MODE_RXCONT             0x05

/* ──────────────────────────────────────────────
   PACKET FORMAT  (matches RX exactly)
   pkt[0]  = int8_t  info   (level: 0 or 31)
   pkt[1…] = char[]  dest   ("BRIM03601")
   ────────────────────────────────────────────── */
#define DEST_ID     "BRIM03601"
#define RECV_ID     "BRIM036"

/* TX interval & sleep config */
#define TX_INTERVAL_MS     10000UL   /* send every 10 seconds */
#define SLEEP_ENABLE       1         /* 1 = use WFI sleep between TXs */

/* ADC threshold:  0–4095
   < 200  → 0%  (empty)
   > 3800 → 100% (full)          */
#define ADC_EMPTY_THRESHOLD   200
#define ADC_FULL_THRESHOLD    3800

/* ──────────────────────────────────────────────
   GLOBALS
   ────────────────────────────────────────────── */
static SPI_HandleTypeDef hspi1;
static ADC_HandleTypeDef hadc1;
static TIM_HandleTypeDef htim2;   /* used as ms tick */

volatile uint32_t ms_tick = 0;

/* ──────────────────────────────────────────────
   FUNCTION PROTOTYPES
   ────────────────────────────────────────────── */
void SystemClock_Config(void);
void GPIO_Init(void);
void SPI1_Init(void);
void ADC1_Init(void);
void TIM2_Init(void);
void delay_ms(uint32_t ms);

/* LoRa helpers */
uint8_t  LoRa_ReadReg(uint8_t addr);
void     LoRa_WriteReg(uint8_t addr, uint8_t value);
uint8_t  LoRa_Init(void);
void     LoRa_SetFrequency(long freq);
void     LoRa_SetTxPower(int level);
uint8_t  LoRa_SendPacket(uint8_t *data, uint8_t len);
void     LoRa_Sleep(void);
void     LoRa_Standby(void);

/* Application */
uint16_t Read_ADC(void);
int8_t   Get_TankLevel(uint16_t adcVal);
void     Build_Packet(uint8_t *buf, uint8_t *len, int8_t level);
void     Enter_MCU_Sleep(uint32_t ms);


/* ══════════════════════════════════════════════
   MAIN
   ══════════════════════════════════════════════ */
int main(void)
{
    HAL_Init();
    SystemClock_Config();
    GPIO_Init();
    SPI1_Init();
    ADC1_Init();
    TIM2_Init();

    /* LED blink = boot OK */
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
    delay_ms(200);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);

    /* Init LoRa */
    while (!LoRa_Init()) {
        /* blink fast if LoRa fail */
        HAL_GPIO_TogglePin(LED_PORT, LED_PIN);
        delay_ms(200);
    }

    /* Confirm 433 MHz, same as RX */
    LoRa_SetFrequency(433E6);
    LoRa_SetTxPower(17);    /* dBm, match RX sensitivity */

    uint8_t  pkt_buf[12];
    uint8_t  pkt_len;
    uint32_t last_tx = 0;

    while (1)
    {
        uint32_t now = ms_tick;

        if ((now - last_tx) >= TX_INTERVAL_MS)
        {
            last_tx = now;

            /* 1. Read sensor */
            uint16_t adc  = Read_ADC();
            int8_t   lvl  = Get_TankLevel(adc);

            /* 2. Build packet like RX expects:
                  byte[0]  = info (level)
                  byte[1…] = "BRIM03601\0"           */
            Build_Packet(pkt_buf, &pkt_len, lvl);

            /* 3. Wake LoRa, transmit, sleep LoRa */
            LoRa_Standby();
            LoRa_SendPacket(pkt_buf, pkt_len);
            LoRa_Sleep();

            /* 4. Blink LED once = TX done */
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
            delay_ms(50);
            HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
        }

#if SLEEP_ENABLE
        /* MCU sleep until next TIM2 tick (WFI) */
        Enter_MCU_Sleep(1);   /* wake every 1 ms tick, loop checks interval */
#endif
    }
}


/* ══════════════════════════════════════════════
   APPLICATION FUNCTIONS
   ══════════════════════════════════════════════ */

/**
 * Read ADC on PA0 (0–4095)
 */
uint16_t Read_ADC(void)
{
    HAL_ADC_Start(&hadc1);
    HAL_ADC_PollForConversion(&hadc1, 10);
    uint16_t val = HAL_ADC_GetValue(&hadc1);
    HAL_ADC_Stop(&hadc1);
    return val;
}

/**
 * Convert ADC reading to tank level byte
 *   0%  → returns 0   (RX shows empty)
 *  100% → returns 31  (RX shows full)
 */
int8_t Get_TankLevel(uint16_t adcVal)
{
    if (adcVal <= ADC_EMPTY_THRESHOLD)
        return 0;   /* 0%  — tank empty, RX motor ON trigger */
    else if (adcVal >= ADC_FULL_THRESHOLD)
        return 31;  /* 100% — tank full,  RX motor OFF */
    else
        return -1;  /* intermediate: not sent (sensor noise guard) */
}

/**
 * Build LoRa packet matching RX struct:
 *   struct packet { int8_t info; char dest[10]; }
 *   Wire order: info byte FIRST, then dest string
 */
void Build_Packet(uint8_t *buf, uint8_t *len, int8_t level)
{
    buf[0] = (uint8_t)level;           /* info byte */
    const char *dest = DEST_ID;
    uint8_t i = 0;
    while (dest[i]) {
        buf[1 + i] = (uint8_t)dest[i];
        i++;
    }
    buf[1 + i] = '\0';
    *len = 1 + i + 1;  /* info + string + null = 11 bytes max */
}

/**
 * MCU low-power sleep (WFI)
 * Wakes on any interrupt (TIM2 tick used here)
 */
void Enter_MCU_Sleep(uint32_t ms)
{
    uint32_t target = ms_tick + ms;
    while (ms_tick < target) {
        __WFI();   /* Wait For Interrupt — lowest power without clock stop */
    }
}


/* ══════════════════════════════════════════════
   LoRa SX1278 DRIVER
   ══════════════════════════════════════════════ */

void LoRa_NSS_Low(void)  { HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_RESET); }
void LoRa_NSS_High(void) { HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);   }

uint8_t LoRa_ReadReg(uint8_t addr)
{
    uint8_t tx = addr & 0x7F;
    uint8_t rx = 0;
    LoRa_NSS_Low();
    HAL_SPI_Transmit(&hspi1, &tx, 1, 100);
    HAL_SPI_Receive(&hspi1, &rx, 1, 100);
    LoRa_NSS_High();
    return rx;
}

void LoRa_WriteReg(uint8_t addr, uint8_t value)
{
    uint8_t buf[2] = { (uint8_t)(addr | 0x80), value };
    LoRa_NSS_Low();
    HAL_SPI_Transmit(&hspi1, buf, 2, 100);
    LoRa_NSS_High();
}

uint8_t LoRa_Init(void)
{
    /* Hardware reset */
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_RESET);
    delay_ms(10);
    HAL_GPIO_WritePin(LORA_RESET_PORT, LORA_RESET_PIN, GPIO_PIN_SET);
    delay_ms(10);

    uint8_t ver = LoRa_ReadReg(REG_VERSION);
    if (ver != 0x12) return 0;  /* SX1278 version check */

    /* Sleep mode → enable LoRa */
    LoRa_WriteReg(REG_OP_MODE, MODE_LONG_RANGE | MODE_SLEEP);
    delay_ms(10);

    /* Frequency 433 MHz */
    LoRa_SetFrequency(433E6);

    /* FIFO base */
    LoRa_WriteReg(REG_FIFO_TX_BASE_ADDR, 0);

    /* BW=125kHz, CR=4/5, Explicit header */
    LoRa_WriteReg(REG_MODEM_CONFIG_1, 0x72);

    /* SF=7, CRC on */
    LoRa_WriteReg(REG_MODEM_CONFIG_2, 0x74);

    /* Preamble = 8 */
    LoRa_WriteReg(REG_PREAMBLE_MSB, 0x00);
    LoRa_WriteReg(REG_PREAMBLE_LSB, 0x08);

    /* Standby */
    LoRa_WriteReg(REG_OP_MODE, MODE_LONG_RANGE | MODE_STDBY);

    return 1;
}

void LoRa_SetFrequency(long freq)
{
    uint64_t frf = ((uint64_t)freq << 19) / 32000000UL;
    LoRa_WriteReg(REG_FRF_MSB, (uint8_t)(frf >> 16));
    LoRa_WriteReg(REG_FRF_MID, (uint8_t)(frf >> 8));
    LoRa_WriteReg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

void LoRa_SetTxPower(int level)
{
    if (level < 2)  level = 2;
    if (level > 17) level = 17;
    LoRa_WriteReg(REG_PA_CONFIG, 0x80 | (level - 2));
}

uint8_t LoRa_SendPacket(uint8_t *data, uint8_t len)
{
    /* Standby */
    LoRa_WriteReg(REG_OP_MODE, MODE_LONG_RANGE | MODE_STDBY);

    /* Reset FIFO ptr */
    LoRa_WriteReg(REG_FIFO_ADDR_PTR, 0x00);

    /* Write payload */
    for (uint8_t i = 0; i < len; i++)
        LoRa_WriteReg(REG_FIFO, data[i]);

    LoRa_WriteReg(REG_PAYLOAD_LENGTH, len);

    /* TX mode */
    LoRa_WriteReg(REG_OP_MODE, MODE_LONG_RANGE | MODE_TX);

    /* Wait TX done (DIO0 HIGH or IRQ flag) */
    uint32_t t = ms_tick;
    while (!(LoRa_ReadReg(REG_IRQ_FLAGS) & 0x08)) {
        if (ms_tick - t > 3000) return 0;  /* timeout 3s */
    }
    /* Clear IRQ */
    LoRa_WriteReg(REG_IRQ_FLAGS, 0xFF);
    return 1;
}

void LoRa_Sleep(void)
{
    LoRa_WriteReg(REG_OP_MODE, MODE_LONG_RANGE | MODE_SLEEP);
}

void LoRa_Standby(void)
{
    LoRa_WriteReg(REG_OP_MODE, MODE_LONG_RANGE | MODE_STDBY);
    delay_ms(10);
}


/* ══════════════════════════════════════════════
   STM32 PERIPHERAL INIT
   ══════════════════════════════════════════════ */

void SystemClock_Config(void)
{
    RCC_OscInitTypeDef osc = {0};
    RCC_ClkInitTypeDef clk = {0};

    osc.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    osc.HSEState       = RCC_HSE_ON;
    osc.PLL.PLLState   = RCC_PLL_ON;
    osc.PLL.PLLSource  = RCC_PLLSOURCE_HSE;
    osc.PLL.PLLMUL     = RCC_PLL_MUL9;  /* 8MHz × 9 = 72MHz */
    HAL_RCC_OscConfig(&osc);

    clk.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
                       | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
    clk.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    clk.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    clk.APB1CLKDivider = RCC_HCLK_DIV2;
    clk.APB2CLKDivider = RCC_HCLK_DIV1;
    HAL_RCC_ClockConfig(&clk, FLASH_LATENCY_2);
}

void GPIO_Init(void)
{
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    GPIO_InitTypeDef g = {0};

    /* LED PC13 */
    g.Pin   = LED_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LED_PORT, &g);
    HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET); /* LED off (active low) */

    /* LoRa NSS PA4 */
    g.Pin   = LORA_NSS_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(LORA_NSS_PORT, &g);
    HAL_GPIO_WritePin(LORA_NSS_PORT, LORA_NSS_PIN, GPIO_PIN_SET);

    /* LoRa RESET PB0 */
    g.Pin   = LORA_RESET_PIN;
    g.Mode  = GPIO_MODE_OUTPUT_PP;
    HAL_GPIO_Init(LORA_RESET_PORT, &g);

    /* LoRa DIO0 PB1 (input) */
    g.Pin   = LORA_DIO0_PIN;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(LORA_DIO0_PORT, &g);

    /* SPI1 pins PA5 PA6 PA7 (SCK MISO MOSI) */
    g.Pin   = GPIO_PIN_5 | GPIO_PIN_7;
    g.Mode  = GPIO_MODE_AF_PP;
    g.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &g);

    g.Pin   = GPIO_PIN_6;
    g.Mode  = GPIO_MODE_INPUT;
    g.Pull  = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &g);
}

void SPI1_Init(void)
{
    __HAL_RCC_SPI1_CLK_ENABLE();

    hspi1.Instance               = SPI1;
    hspi1.Init.Mode              = SPI_MODE_MASTER;
    hspi1.Init.Direction         = SPI_DIRECTION_2LINES;
    hspi1.Init.DataSize          = SPI_DATASIZE_8BIT;
    hspi1.Init.CLKPolarity       = SPI_POLARITY_LOW;
    hspi1.Init.CLKPhase          = SPI_PHASE_1EDGE;
    hspi1.Init.NSS               = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16; /* 72MHz/16 = 4.5MHz */
    hspi1.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    hspi1.Init.TIMode            = SPI_TIMODE_DISABLE;
    hspi1.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLE;
    HAL_SPI_Init(&hspi1);
}

void ADC1_Init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();

    hadc1.Instance                   = ADC1;
    hadc1.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc1.Init.ContinuousConvMode    = DISABLE;
    hadc1.Init.DiscontinuousConvMode = DISABLE;
    hadc1.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc1.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc1.Init.NbrOfConversion       = 1;
    HAL_ADC_Init(&hadc1);

    ADC_ChannelConfTypeDef ch = {0};
    ch.Channel      = ADC_CHANNEL_0;   /* PA0 */
    ch.Rank         = ADC_REGULAR_RANK_1;
    ch.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
    HAL_ADC_ConfigChannel(&hadc1, &ch);

    HAL_ADCEx_Calibration_Start(&hadc1);
}

void TIM2_Init(void)
{
    __HAL_RCC_TIM2_CLK_ENABLE();

    htim2.Instance               = TIM2;
    htim2.Init.Prescaler         = 71;       /* 72MHz / 72 = 1MHz */
    htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
    htim2.Init.Period            = 999;      /* 1MHz / 1000 = 1kHz = 1ms */
    htim2.Init.ClockDivision     = TIM_CLOCKDIVISION_DIV1;
    htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
    HAL_TIM_Base_Init(&htim2);
    HAL_TIM_Base_Start_IT(&htim2);

    HAL_NVIC_SetPriority(TIM2_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM2_IRQn);
}

void delay_ms(uint32_t ms)
{
    uint32_t start = ms_tick;
    while ((ms_tick - start) < ms);
}

/* TIM2 IRQ → ms_tick increment */
void TIM2_IRQHandler(void)
{
    if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_UPDATE)) {
        __HAL_TIM_CLEAR_FLAG(&htim2, TIM_FLAG_UPDATE);
        ms_tick++;
    }
}

void HAL_MspInit(void) {
    __HAL_RCC_AFIO_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_AFIO_REMAP_SWJ_NOJTAG();  /* free PB3/PB4, keep SWD */
}
