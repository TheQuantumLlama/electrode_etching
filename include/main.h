/* Defines and prtotypes
 * Michael Barnard
 */

/********************************************* Defines ********************************************/
#define SYSCLK 16E6 // System clock speed
#define ADC_PIN_OFFSET 14 // Pin offset for all ADC channels, sequential 0-5
#define ADC_NEAR_0 20 // Anything below this value will be taken as a 0 V reading

/**************************************** Public Functions ****************************************/
void Setup_ADC();
void Setup_timer0();
void Setup_timer1();
int8_t Process_Buffer(uint8_t * buffer, uint8_t etching_lvl);
uint8_t Check_Duty_Cycle(uint8_t * waveform, uint8_t inactive_lvl);
uint8_t Read_ADC_Channel(uint8_t channel);
uint8_t Empty_Buffer(uint8_t * buffer, uint8_t len);
uint8_t Buffer_To_Serial(uint8_t * buffer, uint8_t len);
