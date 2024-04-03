//Assignment 2
//Author: David Valente

//This ensures that only one core of the ESP32 is used
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif


void IRAM_ATTR rising_edge_2();
void IRAM_ATTR rising_edge_3();

// Initialise pins 

//Task1
#define DIGITAL_SIGNAL    16 // Output pin for Task1. Connect to pin 16
//Task2
#define SIGNAL2           22 // Input square wave for Task2 between 333Hz & 1000Hz. Connect to pin 22
//Task3
#define SIGNAL3           19 // Input square wave for Task3 between 500Hz and 1000Hz. Connect to pin 19
//Task4
#define POTENTIOMETER     32 // Output pin for potentiometer. Connect to pin 32 
#define LAST10            10 // Holds the last 10 values of the analogue reading of the potentiometer
#define LED1              15 // Output pin for red LED. Connect to pin 15
//Task6
#define BUTTON            23 // Input pin for button. Connect to pin 23
#define LED2              21 // Output pin for orange LED. Connect to pin 21


//Variables

//Task2
volatile float measured_frequency_2 = 0;
volatile unsigned long lastRiseTime2 = 0;
//Task3
volatile float measured_frequency_3 = 0;
volatile unsigned long lastRiseTime3 = 0;
//Task4
volatile float average = 0.0;
//Task6
unsigned long debounce = 60;    
unsigned long last_debounce = 0;  
volatile bool led_state = LOW;


void setup() {
  Serial.begin(115200);             //begin serial monitor with a baud rate of 115200 
  //Task1
  pinMode(DIGITAL_SIGNAL,OUTPUT);
  //Task2
  pinMode(SIGNAL2, INPUT_PULLUP);
  //Task3
  pinMode(SIGNAL3, INPUT_PULLUP);
  //Task4
  pinMode(POTENTIOMETER, INPUT);
  pinMode(LED1, OUTPUT);
  //Task6
  pinMode(BUTTON,INPUT_PULLDOWN);       
  pinMode(LED2, OUTPUT);
  
  //Interrupts
  attachInterrupt(digitalPinToInterrupt(SIGNAL2), rising_edge_2, RISING);
  attachInterrupt(digitalPinToInterrupt(SIGNAL3), rising_edge_3, RISING);

  //Creating tasks
  xTaskCreate(Task1, "Task1", 1024, NULL, 2, NULL);
  xTaskCreate(Task2, "Task2", 1024, NULL, 3, NULL);
  xTaskCreate(Task3, "Task3", 1024, NULL, 2, NULL);
  xTaskCreate(Task4, "Task4", 1024, NULL, 2, NULL);
  xTaskCreate(Task5, "Task5", 1024, NULL, 2, NULL);
  xTaskCreate(Task6, "Task6", 2048, NULL, 1, NULL);  
  xTaskCreate(Task7_CPUWork, "CPUWork", 1024, NULL, 1, NULL);
}

void Task1(void *pvParameters){
  while(1){
    digitalWrite(DIGITAL_SIGNAL  , HIGH);
    delayMicroseconds(180);
    digitalWrite(DIGITAL_SIGNAL  , LOW);
    delayMicroseconds(40);
    digitalWrite(DIGITAL_SIGNAL  , HIGH);
    delayMicroseconds(530);
    digitalWrite(DIGITAL_SIGNAL  , LOW);
    delayMicroseconds(250);
    vTaskDelay(3/portTICK_RATE_MS);
  }
}
// Task 2 - measure frequency within range of 333Hz & 1KHz
void Task2(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms in tick period
  xLastWakeTime = xTaskGetTickCount();

  while(1) {
    // Check if the frequency is within the desired range
    if(measured_frequency_2 >= 333 && measured_frequency_2 <= 1000) {
      Serial.print("Task 2 Measured Frequency @ 20ms: ");
      Serial.println(measured_frequency_2, 2); // Print with 2 decimal places
    } else {
      // Handle out-of-range frequency
      Serial.println("Task 2 Frequency ## OUT OF RANGE ## @ 20ms: ");
    }
    // Wait for the next cycle
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Task 3 - measure frequency within range of 500Hz & 1KHz
void Task3(void *pvParameters){
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(8); // 8ms in tick period
  xLastWakeTime = xTaskGetTickCount();

  while(1) {
    // Check if the frequency is within the desired range
    if(measured_frequency_3 >= 500 && measured_frequency_3 <= 1000) {
      Serial.print("Task 3 Measured Frequency @ 8ms: ");
      Serial.println(measured_frequency_3, 2); // Print with 2 decimal places
    } else {
      // Handle out-of-range frequency
      Serial.println(" | Task 3 Frequency ## OUT OF RANGE ## @ 8ms");
    }
    // Wait for the next cycle
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
void Task4(void *pvParameters) {
    int readValues[LAST10] = {0}; // Array to store recent readings
    int readIndex = 0; // Current index in the readings array
    long sum = 0; // Sum of the readings for calculating the average
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // Sampling frequency - 20ms

    xLastWakeTime = xTaskGetTickCount();
    const int maxAnalogValue = 4095; // Maximum value for the ADC
    const int errorThreshold = maxAnalogValue / 2; // Error threshold is half the maximum

    while (1) {
        // Read the current value from the analog pin
        int currentValue = analogRead(POTENTIOMETER);

        // Update the total sum by subtracting the oldest value and adding the new value
        sum = sum - readValues[readIndex] + currentValue;

        // Store the current value in the array
        readValues[readIndex] = currentValue;

        // Update the index for the next read
        readIndex = (readIndex + 1) % LAST10;

        // Update the running average
        average = sum / (float)LAST10;

        // Check if the running average exceeds the error threshold
        if (average > errorThreshold) {
            digitalWrite (LED1, HIGH); // Turn on the LED to indicate error
        } else {
            digitalWrite (LED1, LOW); // Turn off the LED
        }

        // Print the running average every sample cycle
        Serial.print("Running Average: ");
        Serial.println(average, 2); // Print with 2 decimal places

        // Delay until the next sample period
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void Task5(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(200); // Logging period of 200ms

    while (1) {
        // Scale and bound the frequencies measured by Task 2 and Task 3
        int scaledFrequency1 = (int)(((measured_frequency_2 - 333) / (1000.0 - 333)) * 99);
        int scaledFrequency2 = (int)(((measured_frequency_3 - 500) / (1000.0 - 500)) * 99);

        // Ensure the frequencies are within 0 to 99
        scaledFrequency1 = scaledFrequency1 < 0 ? 0 : scaledFrequency1 > 99 ? 99 : scaledFrequency1;
        scaledFrequency2 = scaledFrequency2 < 0 ? 0 : scaledFrequency2 > 99 ? 99 : scaledFrequency2;

        // Directly print the scaled frequencies to the serial port
        Serial.println();
        Serial.print("Scaled frequencies: ");
        Serial.print(scaledFrequency1);
        Serial.print(",");
        Serial.println(scaledFrequency2);


        // Delay until the next log period
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}


void Task6(void *pvParameters) {

    int lastButtonState = HIGH;  // the previous reading from the input pin
    int buttonState;             // the current reading from the input pin

    while (1) {
        // Read the state of the switch into a local variable:
        int reading = digitalRead(BUTTON);

        // Check if the pushbutton is pressed.
        // If the current state is different from the last state,
        // reset the debouncing timer
        if (reading != lastButtonState) {
            last_debounce = millis();
        }
        if ((millis() - last_debounce) > debounce) {
            // whatever the reading is at, it's been there for longer than the debounce
            // delay, so take it as the actual current state:

            // if the button state has changed:
            if (reading != buttonState) {
                buttonState = reading;

                // only toggle the LED if the new button state is HIGH
                if (buttonState == HIGH) {
                    led_state = !led_state;
                    digitalWrite(LED2, led_state);
                }
            }
        }
        // Save the reading. Next time through the loop, it'll be the lastButtonState:
        lastButtonState = reading;

        // Small delay to prevent bouncing
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void Task7_CPUWork(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms period

    while (1) {
        // Capture start time
        unsigned long startTime = micros();

        CPU_work(2); // Call CPU_work to busy the CPU for approximately 2ms

        // Capture end time and calculate duration
        unsigned long endTime = micros();
        unsigned long duration = endTime - startTime;

        // Print the duration that CPU_work took
        Serial.println();
        Serial.print("CPU_work duration: ");
        Serial.print(duration);
        Serial.println(" microseconds");

        // Wait for the next cycle, ensuring the task runs with a period of 20ms
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}



void CPU_work(int time) {
    // Calibrated loop count per millisecond (example value, adjust based on your calibration)
    volatile long loopCountPerMs = 33850; // Based on 16MHz processor

    long loops = time * loopCountPerMs;

    for(long i = 0; i < loops; i++) {
        __asm__("nop"); // Assembly instruction for "no operation"
    }
}

// ISR for detecting rising edges on the square wave pin
void IRAM_ATTR rising_edge_2() {
  unsigned long currentTime = micros();
  if(lastRiseTime2 > 0) {
    // Calculate frequency: F = 1 / T
    unsigned long period = currentTime - lastRiseTime2;
    measured_frequency_2 = 1000000.0 / period; // Convert period from µs to seconds
  }
  lastRiseTime2 = currentTime;
}
void IRAM_ATTR rising_edge_3() {
  unsigned long currentTime = micros();
  if(lastRiseTime3 > 0) {
    // Calculate frequency: F = 1 / T
    unsigned long period = currentTime - lastRiseTime3;
    measured_frequency_3 = 1000000.0 / period; // Convert period from µs to seconds
  }
  lastRiseTime3 = currentTime;
}

void loop(){
  // Leave empty for FreeRTOS
}
