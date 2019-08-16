/* Basic reading and decision making for 4 ADC channels
 * Michael Barnard
 */

/************************************ Dependencies and Externs ************************************/
#include "Arduino.h"
#include "main.h"


/********************************************* Defines ********************************************/
#define NUM_CHANNELS 4 // How many wires are you sharpening simultaneously?

#define ETCH_FREQ 10L // Hz, frequency of the etching waveform
// Freq to sample the ADC at (must be <256)
#define SAMP_FREQ ((F_CPU / ((ETCH_FREQ * 100) * 64)) - 1) // = SYSCLK / (Fs * prescaler) - 1


/*************************************** Public Variables *****************************************/
/* Storage variables */
// Current level that etching happens at
uint8_t etching_avg = 0;

/* ADC buffers and flags */
// TODO: Move all of these variables into a proper datastructure
// Choose ADC channel
uint8_t ADC_channel = 0;
// Ping-pong buffers for every channel being used, enough space to hold 1.5 waveform periods
uint8_t waveform_buffer[2][150] = { 0 };
uint8_t buff_cnt =  0;
// Controls the switching between ping and pong buffers
bool write_buffer_offset = 0;
bool read_buffer_offset = 0;
// Signifies when buffer needs to be processed
volatile bool buffer_needs_processing = false;


/**************************************** Public Functions ****************************************/
void setup() {
  cli(); // Disable interrupts

  Serial.begin(115200);
  Serial.print("Hello World\n");

  // Set pins as outputs
  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
  pinMode(7, OUTPUT);
  digitalWrite(7, LOW);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);

  Setup_timer0();
  
  Empty_Buffer(waveform_buffer[0], 150);
  Empty_Buffer(waveform_buffer[1], 150);

  sei(); // Enable interrupts

  // Record number of points added together
  uint16_t num_pts = 0;
  // 32-bit variable to hold the sum of all points from 3 waveforms
  uint32_t waveform_sum = 0;
  
  // Add together three whole waveforms
  for(int i = 0; i < 3; i++) {
    Serial.print("Find etch lvl\n");

    // Wait for a buffer to be ready
    while(!buffer_needs_processing);

    Serial.print("Buf full\n");

    // Add buffer into etching average
    uint8_t k = 0;
    while(waveform_buffer[read_buffer_offset][k] > 0) {
      waveform_sum += waveform_buffer[read_buffer_offset][k];
      k++;
    }

    buffer_needs_processing = false;
  }
  // Calculate average current level that etching occurs at
  etching_avg = waveform_sum / (uint32_t)num_pts;

  Serial.print("Etch lvl found ");
  Serial.println(etching_avg);
}

void loop() {
  // Wait until a buffer needs to be processed
  if(buffer_needs_processing) {

    // Check if the electrode is finished etching
    int8_t finished_etching =
      Process_Buffer(waveform_buffer[read_buffer_offset], etching_avg);
    
    if(finished_etching == 1) {
      // TODO: Replace this with logic to swap to a new channel and etch the next electrode
      digitalWrite(LED_BUILTIN, HIGH);
      Serial.print("Etching finished on ");
      Serial.println(ADC_channel);

      while(true);
    }

    buffer_needs_processing = false;
  }
}

/* Set timer0 interrupt at 10kHz
 */
void Setup_timer0() {
  TCCR0A = 0; // Set entire TCCR0A register to 0
  TCCR0B = 0; // Same for TCCR0B
  TCNT0  = 0; // Initialize counter value to 0
  // Set compare match register
  OCR0A = SAMP_FREQ; // Must be <256
  TCCR0A |= (1 << WGM01); // Turn on CTC mode
  TCCR0B |= (1 << CS01) | (1 << CS00); // Set CS01 and CS00 bits for 64 prescaler
  TIMSK0 |= (1 << OCIE0A); // Enable timer compare interrupt
}

/* For a given buffer, determines if the current should be cut off
 * Inputs: buffer - ptr to the specific buffer to be checked
 *         etching_lvl - current level that etching occurs
 */
int8_t Process_Buffer(uint8_t * buffer, uint8_t etching_lvl) {
  // Values greater than 1.25x etching avg are considered electrolysis and count towards the
  //   duty cycle
  uint8_t etching_upper_limit = etching_lvl + (etching_lvl >> 2);

  uint8_t duty_cycle = Check_Duty_Cycle(buffer, etching_upper_limit);

  // TODO: Error checking
  if(duty_cycle < 95) {
    return 0; // If less than 95% duty, continue applying current
  } else {
    return 1; // If more than 95% duty, cut off current
  }
}

/* Calculates the duty cycle of points above 1.25x the etching average
 * Inputs: waveform - pointer to the waveform buffer to be processed
 *         inactive_lvl - below this level is the inactive part of the duty cycle
 * Output: duty_cycle - number of points in the active part of the cycle
 */
uint8_t Check_Duty_Cycle(uint8_t * waveform, uint8_t inactive_lvl) {
  // How many values were present in the waveform, this should always be between 99 and 101
  uint8_t waveform_cnt = 0;

  // Count of points in the active part of the cycle
  uint8_t duty_cycle = 0;

  while(waveform > 0) {
    if(waveform[waveform_cnt++] > inactive_lvl) {
      duty_cycle++;
    }
  }

  // TODO: Create error checking w/ waveform_cnt
  return duty_cycle;
}

/* Fill buffer with 0s
 */
uint8_t Empty_Buffer(uint8_t * buffer, uint8_t len) {
  for(uint8_t i = 0; i < len; i++) {
    buffer[i] = 0;
  }

  return 0;
}

/* Write the entire buffer into the serial port
 */
uint8_t Buffer_To_Serial(uint8_t * buffer, uint8_t len) {
  for(uint8_t i = 0; i < len; i++) {
    Serial.print(buffer[i]);
    Serial.print('\n');
  }

  return 0;
}

/******************************************* Interrupts *******************************************/
/* timer0 interrupt at 1kHz triggers an ADC reading of all 4 channels
 * ADC chanels are pins 14-17
 */
ISR(TIMER0_COMPA_vect) {
  // Serial.write("ISR\n");
  digitalWrite(8, HIGH); // Turn on digital pin for external timing

  // Read ADC of current channel (TODO: ~13 cycles by datasheet but builtin is much longer)
  // 10 bit ADC, analogRead returns a 16 bit number
  //   shift upper 2 bits into the lower 8 (lose 2 LSBs) and cast to 8 bits
  //   removing upper 8 bits (all 0s)
  uint8_t tmp = analogRead(ADC_channel) >> 2;
  // Serial.println(tmp);

  // Determine what to do with the ADC value
  if(tmp > ADC_NEAR_0) { // Only save value if it's nonzero
    waveform_buffer[write_buffer_offset][buff_cnt] = tmp;
    buff_cnt++;

  } else if(!buffer_needs_processing) {
    // (tmp < ADC_NEAR_0) condition implicit, check if counterpart buffer is waiting to be
    //   processed, only proceed if it is not

    buffer_needs_processing = true; // Set process flag for whatever buffer was just finished
    write_buffer_offset = !write_buffer_offset; // Switch to filling the unused buffer
    read_buffer_offset = !read_buffer_offset; // Switch to reading the filled buffer
    buff_cnt = 0; // Reset buffer counter

    Serial.print("ISR: Buf swp\n");

  } else {
    // If previous buffer has not been processed, reset the buffer counter and fill the same
    //   buffer again

    buff_cnt = 0;
  }

  digitalWrite(8, LOW); // Finish timing
}
