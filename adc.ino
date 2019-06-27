#include "libmaple/dma.h"
#include <SPI.h>

static const uint8_t LED = PC13;
static const uint32_t SAMPLE_FREQ = 50000;
static const uint8_t adc_pins[] = {PA0,PA1};
static const uint8_t num_adc_pins = sizeof(adc_pins)/sizeof(adc_pins[0]);

volatile bool transfer_complete = false;
volatile uint16_t next_sample_time = 0;
volatile uint32_t dma_isr_count = 0;
volatile uint32_t timer_isr_count = 0;
uint16_t adc_buffer[num_adc_pins];

volatile uint16_t count = 0;
uint16_t buf_a0[1000];
uint16_t buf_a1[1000];
uint64_t res = 1;

HardwareTimer timer(2);

void set_adc(adc_dev* dev);
void set_adc_channels(adc_dev* dev, const uint8_t* pins, uint8_t length);
void set_dma(uint16_t* buf, uint16_t bufLen, uint32_t dmaFlags, voidFuncPtr func);
void set_timer(HardwareTimer* timer, voidFuncPtr func);

void setup()
{
  Serial.begin(115200);
  pinMode(LED, OUTPUT);
  pinMode(PA15, OUTPUT);
  SPI.begin();
  SPI.setClockDivider(SPI_CLOCK_DIV16);
  digitalWrite(PA15,HIGH);
  for(uint8_t i = 0; i < num_adc_pins; i++)
  {
    pinMode(adc_pins[i], INPUT_ANALOG);
  }
  
  /*for (uint8_t i = 0; i<999; i++)
  {
    buf_a0[i] = 0;
    buf_a1[i] = 0;
  }*/
  
  delay(5000);    // wait for the serial console window
  
  // set the dma first
  set_dma(adc_buffer, num_adc_pins, (DMA_CCR_MINC | DMA_CCR_CIRC | DMA_TRNS_CMPLT), DMA1_CH1_isr);
  set_adc(ADC1);
  start_conversion(ADC1);          // start the first adc conversion
  set_timer(&timer, timer_isr);    // start the timer for the next conversion
  
}

void loop()
{

  
  
  static uint32_t dma_count;
  static uint32_t timer_count;
  byte SPIReceive;
  // Store these values as they will update while we are trying to print them
  dma_count = dma_isr_count;
  timer_count = timer_isr_count;
  
  Serial.print(dma_count);
  Serial.print(':');
  Serial.print(timer_count);
  
  for(uint8_t i = 0; i < num_adc_pins; i++)
  {
    Serial.print(',');
    Serial.print(adc_buffer[i]);
  }
  
  Serial.println();
  Serial.print("result:");
  Serial.print(res);
  Serial.println();

  
  digitalWrite(LED, !digitalRead(LED));
  delay(1000);
}

/*
    Interrupt handlers! Every sample complete will hook this function!
*/
static void DMA1_CH1_isr() {
  if(count == 999)
  {
    count = 0;
  }
  buf_a0[count] = adc_buffer[0];
  buf_a1[count] = adc_buffer[1];
  ++count;
  ++dma_isr_count;
  /*digitalWrite(PA15,LOW);
  SPI.transfer(adc_buffer[0]);
  digitalWrite(PA15,HIGH);*/
}

static void timer_isr()
{
  ++timer_isr_count;
  timer.setCompare(TIMER_CH1, ++next_sample_time);  // set next sample time
  transfer_complete = false;
  start_conversion(ADC1);
}

void set_adc(adc_dev* dev)
{
  adc_calibrate(dev);
  adc_set_sample_rate(dev, ADC_SMPR_1_5);
  set_adc_channels(dev, adc_pins, num_adc_pins);
  dev->regs->CR2 |= ADC_CR2_DMA;     //enable ADC DMA transfer
  dev->regs->CR1 |= ADC_CR1_SCAN;    //Set the ADC in Scan Mode
}

void set_adc_channels(adc_dev* dev, const uint8_t* pins, const uint8_t num_pins)
{
  
  uint8_t channels[num_pins];
  uint32_t records[3] = {0,0,0};

  //convert from pins to channels
  for (uint8_t i = 0; i < num_pins; i++)
  { 
    channels[i] = PIN_MAP[pins[i]].adc_channel;
  }
  
  //write the length
  records[2] |= (num_pins - 1) << 20;
  
  //i goes through records, j goes through variables.
  for (uint8_t i = 0, j = 0; i < num_pins; i++)
  {
      if ((i != 0) && (i%6 == 0))
      {
        j++;
      }
      records[j] |= (channels[i] << ((i%6)*5));
  }
  
  //update the registers inside with the scan sequence.
  dev->regs->SQR1 = records[2];
  dev->regs->SQR2 = records[1];
  dev->regs->SQR3 = records[0];
}

void start_conversion(adc_dev* dev)
{
  dev->regs->CR2 |= ADC_CR2_SWSTART;
}

void set_dma(uint16_t* buf, uint16_t bufLen, uint32_t dmaFlags, voidFuncPtr func) {
  dma_init(DMA1);
  
  if (func != NULL) {
    dma_attach_interrupt(DMA1, DMA_CH1, func);
  }
  
  dma_setup_transfer(
    DMA1,
    DMA_CH1,
    &ADC1->regs->DR,
    DMA_SIZE_16BITS,
    buf,
    DMA_SIZE_16BITS,
    dmaFlags
  );
  dma_set_num_transfers(DMA1, DMA_CH1, bufLen);
  dma_enable(DMA1, DMA_CH1);
}

void set_timer(HardwareTimer* timer, voidFuncPtr func)
{
  timer->pause();
  timer->setPrescaleFactor(72000000u / SAMPLE_FREQ);
  timer->setOverflow(65535);
  
  timer->setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
  timer->setCompare(TIMER_CH1, 1);  
  next_sample_time = 1;             

  if (func != NULL)
  {
    timer->attachInterrupt(TIMER_CH1, func);
  }
  
  timer->refresh();
  timer->resume();
}
