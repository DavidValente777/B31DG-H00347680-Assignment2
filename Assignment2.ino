// Function Prototypes
void Task1_SignalGeneration(void *pvParameters);
void Task2_Measurefrequency(void *pvParameters);
void Task3_Measurefrequency(void *pvParameters);
void Task4_SampleAnalogueInput(void *pvParameters);
void Task5_LogFrequencies(void *pvParameters);
void Task6_ControlLED(void *pvParameters);

void IRAM_ATTR handle_rising_edge_task2();
void IRAM_ATTR handle_rising_edge_task3();

// Pin Definitions
      
#define PUSHBUTTON_PIN    23 // Push button is connected to GPIO Pin 23
#define LED_PIN_1         15 // Red LED is connected to GPIO Pin 15 for potentiometer Task4
#define LED_PIN_2         21 // Orange LED is connected to GPIO Pin 21 for pushButton Task6
#define POTENTIOMETER_PIN 32 // Potentiometer is connected to GPIO Pin 32
#define SQUARE_WAVE_PIN_1 22 // Pin connected to the square wave signal on GPIO Pin 22 (Task2 between 333Hz & 1000Hz)
#define SQUARE_WAVE_PIN_2 19 // Pin connected to the square wave signal on GPIO Pin 19 (Task3 between 500Hz and 1000Hz)
#define SAMPLE_SIZE       10// Total number of samples for calulating the running average of analogue input
#define SIGNAL_OUTPUT_PIN 16 //Output pin for Task1


//Task 6 Variables
volatile bool ledState = LOW;
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

// Global Variables for Frequency Measurement
volatile unsigned long lastRiseTime2 = 0;


volatile unsigned long lastRiseTime3 = 0;
// Task 2 Variable
volatile float measured_frequency_1 = 0;
// Task 3 Variables
volatile float measured_frequency_2 = 0;
// Task 4 Variables
volatile float runningAverage = 0.0;

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  pinMode(POTENTIOMETER_PIN, INPUT);
  pinMode(SQUARE_WAVE_PIN_1, INPUT_PULLUP);
  pinMode(SQUARE_WAVE_PIN_2, INPUT_PULLUP);
  pinMode(PUSHBUTTON_PIN, INPUT_PULLDOWN); // Set the pushbutton pin as input with internal pull-up
  pinMode(SIGNAL_OUTPUT_PIN, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(SQUARE_WAVE_PIN_1), handle_rising_edge_task2, RISING);
  attachInterrupt(digitalPinToInterrupt(SQUARE_WAVE_PIN_2), handle_rising_edge_task3, RISING);

  /*
   * Create FreeRTOS task for measuring frequency
   * xTaskCreate(Task function, "Task name", Stack size, Task input parameter, Priority, Task handle);
  */

  xTaskCreate(Task1_SignalGeneration, "Task1_Signalgeneration", 1000, NULL, 2, NULL);
  xTaskCreate(Task2_Measurefrequency, "Task2_Measurefrequency", 1000, NULL, 3, NULL);
  xTaskCreate(Task3_Measurefrequency, "Task3_Measurefrequency", 1000, NULL, 2, NULL);
  xTaskCreate(Task4_SampleAnalogueInput, "Task4_SampleAnalogInput", 1000, NULL, 2, NULL);
  xTaskCreate(Task5_LogFrequencies, "LogInformation", 1000, NULL, 2, NULL);
  xTaskCreate(Task6_ControlLED, "Task6_ControlLED", 2048, NULL, 1, NULL);  
  xTaskCreate(Task7_CPUWork, "CPUWork", 1000, NULL, 1, NULL);
}

void Task1_SignalGeneration(void *pvParameters){
  while(1){
    digitalWrite(SIGNAL_OUTPUT_PIN, HIGH);
    delayMicroseconds(180);
    digitalWrite(SIGNAL_OUTPUT_PIN, LOW);
    delayMicroseconds(40);
    digitalWrite(SIGNAL_OUTPUT_PIN, HIGH);
    delayMicroseconds(530);
    digitalWrite(SIGNAL_OUTPUT_PIN, LOW);
    delayMicroseconds(250);
    vTaskDelay(3/portTICK_RATE_MS);
  }

}
// Task 2 - measure frequency within range of 333Hz & 1KHz
void Task2_Measurefrequency(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms in tick period
  xLastWakeTime = xTaskGetTickCount();

  while(1) {
    // Check if the frequency is within the desired range
    if(measured_frequency_1 >= 333 && measured_frequency_1 <= 1000) {
      Serial.print("Task 2 Measured Frequency @ 20ms: ");
      Serial.println(measured_frequency_1, 2); // Print with 2 decimal places
    } else {
      // Handle out-of-range frequency
      Serial.println("Task 2 Frequency ## OUT OF RANGE ## @ 20ms: ");
    }
    // Wait for the next cycle
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}

// Task 3 - measure frequency within range of 500Hz & 1KHz
void Task3_Measurefrequency(void *pvParameters){
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(8); // 8ms in tick period
  xLastWakeTime = xTaskGetTickCount();

  while(1) {
    // Check if the frequency is within the desired range
    if(measured_frequency_2 >= 500 && measured_frequency_2 <= 1000) {
      Serial.print("Task 3 Measured Frequency @ 8ms: ");
      Serial.println(measured_frequency_2, 2); // Print with 2 decimal places
    } else {
      // Handle out-of-range frequency
      Serial.println(" | Task 3 Frequency ## OUT OF RANGE ## @ 8ms");
    }
    // Wait for the next cycle
    vTaskDelayUntil(&xLastWakeTime, xFrequency);
  }
}
void Task4_SampleAnalogueInput(void *pvParameters) {
    int readValues[SAMPLE_SIZE] = {0}; // Array to store recent readings
    int readIndex = 0; // Current index in the readings array
    long sum = 0; // Sum of the readings for calculating the average
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // Sampling frequency - 20ms

    xLastWakeTime = xTaskGetTickCount();
    const int maxAnalogValue = 4095; // Maximum value for the ADC
    const int errorThreshold = maxAnalogValue / 2; // Error threshold is half the maximum

    while (1) {
        // Read the current value from the analog pin
        int currentValue = analogRead(POTENTIOMETER_PIN);

        // Update the total sum by subtracting the oldest value and adding the new value
        sum = sum - readValues[readIndex] + currentValue;

        // Store the current value in the array
        readValues[readIndex] = currentValue;

        // Update the index for the next read
        readIndex = (readIndex + 1) % SAMPLE_SIZE;

        // Update the running average
        runningAverage = sum / (float)SAMPLE_SIZE;

        // Check if the running average exceeds the error threshold
        if (runningAverage > errorThreshold) {
            digitalWrite(LED_PIN_1, HIGH); // Turn on the LED to indicate error
        } else {
            digitalWrite(LED_PIN_1, LOW); // Turn off the LED
        }

        // Print the running average every sample cycle
        Serial.print("Running Average: ");
        Serial.println(runningAverage, 2); // Print with 2 decimal places

        // Delay until the next sample period
        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void Task5_LogFrequencies(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(200); // Logging period of 200ms

    while (1) {
        // Scale and bound the frequencies measured by Task 2 and Task 3
        int scaledFrequency1 = (int)(((measured_frequency_1 - 333) / (1000.0 - 333)) * 99);
        int scaledFrequency2 = (int)(((measured_frequency_2 - 500) / (1000.0 - 500)) * 99);

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


void Task6_ControlLED(void *pvParameters) {

    int lastButtonState = HIGH;  // the previous reading from the input pin
    int buttonState;             // the current reading from the input pin

    while (1) {
        // Read the state of the switch into a local variable:
        int reading = digitalRead(PUSHBUTTON_PIN);

        // Check if the pushbutton is pressed.
        // If the current state is different from the last state,
        // reset the debouncing timer
        if (reading != lastButtonState) {
            lastDebounceTime = millis();
        }
        if ((millis() - lastDebounceTime) > debounceDelay) {
            // whatever the reading is at, it's been there for longer than the debounce
            // delay, so take it as the actual current state:

            // if the button state has changed:
            if (reading != buttonState) {
                buttonState = reading;

                // only toggle the LED if the new button state is HIGH
                if (buttonState == HIGH) {
                    ledState = !ledState;
                    digitalWrite(LED_PIN_2, ledState);
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
void IRAM_ATTR handle_rising_edge_task2() {
  unsigned long currentTime = micros();
  if(lastRiseTime2 > 0) {
    // Calculate frequency: F = 1 / T
    unsigned long period = currentTime - lastRiseTime2;
    measured_frequency_1 = 1000000.0 / period; // Convert period from µs to seconds
  }
  lastRiseTime2 = currentTime;
}
void IRAM_ATTR handle_rising_edge_task3() {
  unsigned long currentTime = micros();
  if(lastRiseTime3 > 0) {
    // Calculate frequency: F = 1 / T
    unsigned long period = currentTime - lastRiseTime3;
    measured_frequency_2 = 1000000.0 / period; // Convert period from µs to seconds
  }
  lastRiseTime3 = currentTime;
}

void loop(){
  // Leave empty for FreeRTOS
}
