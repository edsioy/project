const int led0 = 9;  // LED0 connected to D9 (controlled by Timer1) (red)
const int led1 = 5;  // LED1 connected to D5 (controlled by Timer0) (blue)
const int led2 = 3;  // LED2 connected to D3 (controlled by Timer2) (green)
const int blinkInterval = 100; // LED blinks at 100ms intervals
const int potpin = A0; // Potentiometer connected to A0, for controlling LED brightness
const int pulseDuration = 25000; // Default unit: µs

int frequency = 100;
int dutuCycle = 50;
int brightness = 0; // Brightness level

unsigned long onTime = 5000; // Default unit: µs
unsigned long offTime = 5000;
unsigned long previousBlinkTime = 0; // Time of last LED state change
unsigned long previousPWMTime = 0;   // Time for brightness control

bool toggle = LOW;
bool increasing = true;

void setup() {
  pinMode(led0, OUTPUT);
  pinMode(led1, OUTPUT);
  pinMode(led2, OUTPUT);

  Serial.begin(115200);

  setupTimer0PWM();  // Initialize Timer0 (controls D5)
  setupTimer1PWM();  // Initialize Timer1 (controls D9)
  setupTimer2PWM();  // Initialize Timer2 (controls D3)
}

void loop() {
  unsigned long currentMicros = micros();
  // Potentiometer controls brightness (affects only the duty cycle for Timer1 and Timer2)
  
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n'); 
    command.trim();
    processing(command);
  }

  // Output to the serial plotter (ensure each line contains only numerical values)
  
  // (Additional logic can be added here)
}

// Set up Timer0 (8-bit) to generate a 100Hz PWM signal for D5
void setupTimer0PWM() {
    // Configure Timer0 for Fast PWM mode
    // Initialize registers
    TCCR0A = 0;
    TCCR0B = 0;
    TCNT0  = 0;
    
    TCCR0A |= (1 << WGM00) | (1 << WGM01);
    TCCR0B |= (1 << WGM02);

    // Set OC0B for non-inverting PWM output: COM0B1 = 1
    TCCR0A |= (1 << COM0B1);

    // Set prescaler to 1024: for Timer0, 1024 corresponds to CS02 = 1, CS00 = 1
    TCCR0B |= (1 << CS02) | (1 << CS00);
    
    OCR0A = 155;  // Set PWM frequency to 100Hz
    OCR0B = 78;   // Set duty cycle to 50%
}

// Set up Timer1 (16-bit) to generate a 100Hz PWM signal for D9
void setupTimer1PWM() {
  // 1. Turn off Timer1 (reset registers)
  TCCR1A = 0;
  TCCR1B = 0;
  
  // 2. Set Fast PWM mode (WGM13:WGM10 = 1110, i.e., ICR1 is used as TOP)
  //    COM1A1:COM1A0 = 10 -> Non-inverting output (OC1A high period = first half of compare match)
  TCCR1A = (1 << COM1A1) | (1 << WGM11);
  TCCR1B = (1 << WGM13) | (1 << WGM12);

  // 3. Set ICR1 (TOP)
  ICR1 = 19999;  // Corresponds to 100Hz

  // 4. Set OCR1A (duty cycle)
  OCR1A = 9999;  // Approximately 50% duty cycle
  
  // 5. Set prescaler to 8 (CS11 = 1)
  TCCR1B |= (1 << CS11);
}

// Set up Timer2 (8-bit) to generate a 100Hz PWM signal for D3
void setupTimer2PWM() {
  /* PWM frequency formula: f_PWM = f_clk / (N * (1 + TOP))
   * where f_clk = 16MHz and N is the prescaler value.
   *
   * To achieve approximately 100Hz, choose prescaler N = 1024, then:
   *   TOP = (16,000,000 / (1024 * 100)) - 1 ≈ 155
   *
   * Note:
   *   When using OCR2A as TOP, the OC2A output (Arduino digital pin 11) cannot be used for PWM,
   *   only OC2B (Arduino digital pin 3) can be used.
   *   Therefore, this routine outputs a 100Hz PWM signal with 50% duty cycle on digital pin 3.
   *
   * To achieve 50% duty cycle, set OCR2B = (TOP+1)/2 - 1 = (156/2 - 1) = 77
   */
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2  = 0;
    // Set Fast PWM mode, TOP = OCR2A (Mode 7: WGM22 = 1, WGM21 = 1, WGM20 = 1)
    TCCR2A |= (1 << WGM20) | (1 << WGM21);
    TCCR2B |= (1 << WGM22);

    // Set OC2B for non-inverting PWM output (COM2B1 = 1, COM2B0 = 0)
    TCCR2A |= (1 << COM2B1);

    // Set prescaler to 1024 (corresponds to CS22 = 1, CS21 = 1, CS20 = 1)
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20);
    
    OCR2A = 155;  // Set PWM frequency
    OCR2B = 77;   // Default 50% duty cycle

    // Why only pin 3 works:
    // In Fast PWM mode using OCR2A as TOP, the PWM function on OC2A is disabled,
    // so only OC2B (Arduino pin 3) can output PWM.
    // Waveform difference:
    // Although both CTC mode (toggle on compare match) and Fast PWM mode (continuous update from 0 to TOP)
    // can produce a ~100Hz square wave with 50% duty cycle, their internal mechanisms differ.
}

// Update PWM parameters for the specified channel
void updatePWMParameters(uint8_t channel, int freq, int duty) {
  if (channel == 0) { // Red LED, controlled by Timer1; prescaler value set externally (default is 8 or adjusted)
    unsigned long top = (16000000UL / (getTimer1Prescaler() * (unsigned long)freq)) - 1;
    ICR1 = top;
    unsigned int comp = ((top + 1) * duty) / 100;
    if (comp > top) comp = top;
    OCR1A = comp;
  } else if (channel == 1) { // Blue LED, controlled by Timer0; prescaler value set externally (default is 1024 or adjusted)
    unsigned int top = (16000000UL / (getTimer0Prescaler() * (unsigned long)freq)) - 1;
    OCR0A = top;
    unsigned int comp = ((top + 1) * duty) / 100;
    if (comp > top) comp = top;
    OCR0B = comp;
  } else if (channel == 2) { // Green LED, controlled by Timer2; prescaler value set externally (default is 1024 or adjusted)
    unsigned int top = (16000000UL / (getTimer2Prescaler() * (unsigned long)freq)) - 1;
    OCR2A = top;
    unsigned int comp = ((top + 1) * duty) / 100;
    if (comp > top) comp = top;
    OCR2B = comp;
  }
}

// ======================
// The following are the setTimerXPrescaler and getTimerXPrescaler functions.
// They directly manipulate the timer registers to set the prescaler
// and provide functions to query the current prescaler value.

void setTimer0Prescaler(int prescaler) {
  TCCR0B &= ~((1 << CS02) | (1 << CS01) | (1 << CS00));
  switch(prescaler) {
    case 1:   TCCR0B |= (1 << CS00); break;
    case 8:   TCCR0B |= (1 << CS01); break;
    case 64:  TCCR0B |= (1 << CS01) | (1 << CS00); break;
    case 256: TCCR0B |= (1 << CS02); break;
    case 1024:TCCR0B |= (1 << CS02) | (1 << CS00); break;
  }
}

void setTimer1Prescaler(int prescaler) {
  TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
  switch(prescaler) {
    case 1:   TCCR1B |= (1 << CS10); break;
    case 8:   TCCR1B |= (1 << CS11); break;
    case 64:  TCCR1B |= (1 << CS11) | (1 << CS10); break;
    case 256: TCCR1B |= (1 << CS12); break;
    case 1024:TCCR1B |= (1 << CS12) | (1 << CS10); break;
  }
}

void setTimer2Prescaler(int prescaler) {
  // Clear the prescaler bits in TCCR2B
  TCCR2B &= ~((1 << CS22) | (1 << CS21) | (1 << CS20));
  
  switch(prescaler) {
    case 1:
      TCCR2B |= (1 << CS20); // 001: No prescaling
      break;
    case 8:
      TCCR2B |= (1 << CS21); // 010: Divide by 8
      break;
    case 32:
      TCCR2B |= (1 << CS21) | (1 << CS20); // 011: Divide by 32
      break;
    case 64:
      TCCR2B |= (1 << CS22); // 100: Divide by 64
      break;
    case 128:
      TCCR2B |= (1 << CS22) | (1 << CS20); // 101: Divide by 128
      break;
    case 256:
      TCCR2B |= (1 << CS22) | (1 << CS21); // 110: Divide by 256
      break;
    case 1024:
      TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // 111: Divide by 1024
      break;
    default:
      Serial.println("Error: Unsupported prescaler value for Timer2");
      return;
  }
}

int getTimer0Prescaler() {
  // Determine the prescaler based on the CS bits in TCCR0B
  if (TCCR0B & (1 << CS02)) {
    if (TCCR0B & (1 << CS00)) return 1024;
    else return 256;
  } else if (TCCR0B & (1 << CS01)) {
    if (TCCR0B & (1 << CS00)) return 64;
    else return 8;
  } else {
    if (TCCR0B & (1 << CS00)) return 1;
    else return 0;
  }
}

int getTimer1Prescaler() {
  // Determine the prescaler based on the CS bits in TCCR1B
  if (TCCR1B & (1 << CS12)) {
    if (TCCR1B & (1 << CS10)) return 1024;
    else return 256;
  } else if (TCCR1B & (1 << CS11)) {
    if (TCCR1B & (1 << CS10)) return 64;
    else return 8;
  } else {
    if (TCCR1B & (1 << CS10)) return 1;
    else return 0;
  }
}

int getTimer2Prescaler() {
  // Read the CS22, CS21, and CS20 bits from TCCR2B
  uint8_t cs = TCCR2B & ((1 << CS22) | (1 << CS21) | (1 << CS20));
  int prescaler;
  switch(cs) {
    case (1 << CS20): // 001
      prescaler = 1;
      break;
    case (1 << CS21): // 010
      prescaler = 8;
      break;
    case ((1 << CS21) | (1 << CS20)): // 011
      prescaler = 32;
      break;
    case (1 << CS22): // 100
      prescaler = 64;
      break;
    case ((1 << CS22) | (1 << CS20)): // 101
      prescaler = 128;
      break;
    case ((1 << CS22) | (1 << CS21)): // 110
      prescaler = 256;
      break;
    case ((1 << CS22) | (1 << CS21) | (1 << CS20)): // 111
      prescaler = 1024;
      break;
    default:
      prescaler = 0;
      break;
  }
  Serial.print("getTimer2Prescaler() returns: ");
  Serial.println(prescaler);
  return prescaler;
}

// ======================
// Check and update timer settings:
// Based on the input frequency, determine whether the calculated TOP exceeds the timer's maximum count
// (8-bit: 255, 16-bit: 65535), and choose an appropriate prescaler from the available options.
// If a suitable prescaler is found, adjust automatically; otherwise, print an error and return false.
// This function is called before updatePWMParameters to check all three timers.
bool checkAndUpdateTimer(int channel, int freq, int duty) {
  unsigned long maxCount;
  int newPrescaler = 0;
  int currentDefault = 0;
  int prescalers[5] = {1, 8, 64, 256, 1024};
  int prescalersFORtimer2[7] = {1, 8, 32, 64, 128, 256, 1024};
  float req;
  
  if (channel == 0) { // Timer1, 16-bit, maximum count 65535
    maxCount = 65535UL;
    currentDefault = getTimer0Prescaler();  // Default Timer1 prescaler is 8
    req = 16000000.0 / ((maxCount + 1) * freq); // Minimum required prescaler value
    bool found = false;
    for (int i = 0; i < 5; i++) {
      if (prescalers[i] >= req) {
        newPrescaler = prescalers[i];
        found = true;
        break;
      }
    }
    if (!found) {
      Serial.println("Error: Frequency too low for Timer1");
      return false;
    }
    if (newPrescaler != currentDefault) {
      setTimer1Prescaler(newPrescaler);
    }
  } else if (channel == 1) { // Timer0, 8-bit, maximum count 255
    maxCount = 255UL;
    currentDefault = getTimer1Prescaler();  // Default Timer0 prescaler is 1024
    req = 16000000.0 / ((maxCount + 1) * freq);
    bool found = false;
    for (int i = 0; i < 5; i++) {
      if (prescalers[i] >= req) {
        newPrescaler = prescalers[i];
        found = true;
        break;
      }
    }
    if (!found) {
      Serial.println("Error: Frequency too low for Timer0");
      return false;
    }
    if (newPrescaler != currentDefault) {
      setTimer0Prescaler(newPrescaler);
    }
  } else if (channel == 2) { // Timer2, 8-bit, maximum count 255
    maxCount = 255UL;
    currentDefault = getTimer2Prescaler();  // Default Timer2 prescaler is 1024
    req = 16000000.0 / ((maxCount + 1) * freq);
    bool found = false;
    for (int i = 0; i < 7; i++) {
      if (prescalersFORtimer2[i] >= req) {
        newPrescaler = prescalersFORtimer2[i];
        found = true;
        break;
      }
    }
    if (!found) {
      Serial.println("Error: Frequency too low for Timer2");
      return false;
    }
    if (newPrescaler != currentDefault) {
      setTimer2Prescaler(newPrescaler);
    }
  }
  // If the frequency is suitable, update the PWM parameters
  updatePWMParameters(channel, freq, duty);
  return true;
}

void microsBlink(unsigned long currentTime) {
  if (toggle == LOW && (currentTime - previousBlinkTime >= offTime)) {
    previousBlinkTime = currentTime;
    toggle = HIGH;
    OCR1A = map(brightness, 0, 255, 1000, ICR1);
    OCR1B = map(brightness, 0, 255, 1000, ICR1);
    OCR2B = map(brightness, 0, 255, 0, OCR2A);
  } else if (toggle == HIGH && (currentTime - previousBlinkTime >= onTime)) {
    previousBlinkTime = currentTime;
    toggle = LOW;
    OCR1A = 1;
    OCR1B = 1;
    OCR2B = 1;
  }
  if (currentTime - previousBlinkTime >= (onTime + offTime)) {
    previousBlinkTime = 0;
  }
}

void controlBrightness(unsigned long currentTime) {
  // Not controlling Timer0 (D5) because Timer0 is primarily used for millis() and delay() functions,
  // and changing it might affect those functions, so it remains unchanged.
  
  int potValue = analogRead(potpin);
  brightness = map(potValue, 0, 1023, 0, 255);
  
  // Update duty cycle for Timer1's PWM output
  // For Fast PWM mode (TOP = ICR1), duty cycle = OCR1A / (ICR1 + 1)
  OCR1A = map(brightness, 0, 255, 0, ICR1);
  OCR1B = map(brightness, 0, 255, 0, ICR1);
  
  // Update duty cycle for Timer2's PWM output
  // Here OCR2A is used as TOP, so duty cycle = OCR2B / (OCR2A + 1)
  OCR2B = map(brightness, 0, 255, 0, OCR2A);
}

// Process serial commands
void processing(String command) {
  if (command.startsWith("SET=")) {
    int firstComma = command.indexOf(",");
    int secondComma = command.indexOf(",", firstComma + 1);
    if (firstComma == -1 || secondComma == -1) {
      Serial.println("Error: Invalid format. Use SET=channel,freq,duty");
      return;
    }
    int channel = command.substring(4, firstComma).toInt();
    int freq = command.substring(firstComma + 1, secondComma).toInt();
    int duty = command.substring(secondComma + 1).toInt();
    
    // Parameter range check
    if (channel < 0 || channel > 2) {
      Serial.println("Error: Channel must be 0 (Red), 1 (Blue), or 2 (Green)");
      return;
    }
    if (freq < 1) freq = 1;
    if (freq > 10000) freq = 10000;
    if (duty < 1) duty = 1;
    if (duty > 99) duty = 99;  
    if (!checkAndUpdateTimer(channel, freq, duty)) {
      // Error message already printed in checkAndUpdateTimer
      return;
    }
  }
}
