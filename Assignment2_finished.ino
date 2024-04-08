//Assignment 2
//Author: David Valente

//This ensures that only one core of the ESP32 is used
#if CONFIG_FREERTOS_UNICORE
static const BaseType_t app_cpu = 0;
#else
static const BaseType_t app_cpu = 1;
#endif

//Interrupt Service Routines (ISRs): In the provided code, the functions rising_edge_2() and rising_edge_3() are interrupt service routines (ISRs).
//These functions are executed in response to external events (rising edges on specific pins). 
//Since ISRs need to respond quickly to events, placing them in IRAM can reduce their latency and improve their responsiveness.

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
struct Task2_data{
volatile float measured_frequency_2 = 0;
volatile float measured_frequency_3 = 0;
};
Task2_data task2_data;
SemaphoreHandle_t task2_dataSemaphore;

volatile unsigned long lastRiseTime2 = 0;


//Task3

volatile unsigned long lastRiseTime3 = 0;
//Task4
volatile float average = 0.0;
//Task6
unsigned long debounce = 50;    
unsigned long last_debounce = 0;  
volatile bool led_state = LOW;


void setup() {
  Serial.begin(115200);             //begin serial monitor with a baud rate of 115200 

  //Task1
  pinMode(DIGITAL_SIGNAL,OUTPUT);
  //Task2
  pinMode(SIGNAL2, INPUT_PULLUP);
  task2_dataSemaphore = xSemaphoreCreateMutex();
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

  //Preemptive scheduler
  //Creating tasks
  xTaskCreate(Task1, "Task1", 2048, NULL, 4, NULL);
  xTaskCreate(Task2, "Task2", 2048, NULL, 2, NULL);
  xTaskCreate(Task3, "Task3", 2048, NULL, 3, NULL);
  xTaskCreate(Task4, "Task4", 2048, NULL, 2, NULL);
  xTaskCreate(Task5, "Task5", 2048, NULL, 1, NULL);
  xTaskCreate(Task6, "Task6", 2048, NULL, 2, NULL);  
  xTaskCreate(Task7, "Task7", 2048, NULL, 2, NULL);
}

void Task1(void *pvParameters){
  while(1){
    digitalWrite(DIGITAL_SIGNAL, HIGH);
    delayMicroseconds(180);
    digitalWrite(DIGITAL_SIGNAL, LOW);
    delayMicroseconds(40);
    digitalWrite(DIGITAL_SIGNAL, HIGH);
    delayMicroseconds(530);
    digitalWrite(DIGITAL_SIGNAL, LOW);
    delayMicroseconds(250);
    vTaskDelay(3/portTICK_RATE_MS);
  }
}

void Task2(void *pvParameters) {
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms tick period
  xLastWakeTime = xTaskGetTickCount();

  while(1) {
    
    xSemaphoreTake (task2_dataSemaphore,portMAX_DELAY);
    float final_measured_frequency_2  = task2_data.measured_frequency_2;
    xSemaphoreGive (task2_dataSemaphore);

    if(final_measured_frequency_2 >= 333 && final_measured_frequency_2 <= 1000) {     //checks if the frequency is between 333Hz and 1000Hz
     // Serial.print("Task 2 Measured Frequency(Hz): ");
      //Serial.println(final_measured_frequency_2, 2); 
    } 
    else {
      //Serial.println("Task 2 Frequency invalid ");
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);  // Wait for the next cycle
  }
}


void Task3(void *pvParameters){
  TickType_t xLastWakeTime;
  const TickType_t xFrequency = pdMS_TO_TICKS(8); // 8ms tick period
  xLastWakeTime = xTaskGetTickCount();

  while(1) {
    xSemaphoreTake (task2_dataSemaphore,portMAX_DELAY);
    volatile float final_measured_frequency_3  = task2_data.measured_frequency_3;
    xSemaphoreGive (task2_dataSemaphore);

    if(final_measured_frequency_3 >= 500 && final_measured_frequency_3 <= 1000) {   //checks if the frequency is between 500Hz and 1000Hz
      //Serial.print("Task 3 Measured Frequency(Hz): ");
      //Serial.println(final_measured_frequency_3, 2); 
    } 
    else {
      //Serial.println("Task 3 Frequency invalid ");
    }
    vTaskDelayUntil(&xLastWakeTime, xFrequency);      // Wait for the next cycle
  }
}


void Task4(void *pvParameters) {
    long sum = 0; //adds all the readings
    int readValues[LAST10] = {0}; //stores last 10 readings in an array
    int reading = 0; //current readings index
    
    TickType_t xLastWakeTime;
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // Sampling frequency - 20ms
    xLastWakeTime = xTaskGetTickCount();

    const int max_potentiometer_value = 4095; // Maximum value of the potentiometer. 2^12
    const int error = max_potentiometer_value / 2; 

    while (1) {
        int current_val = analogRead(POTENTIOMETER);
        sum = sum - readValues[reading] + current_val;
        readValues[reading] = current_val;
        reading = (reading + 1) % LAST10;
        average = sum / (float)LAST10;
        if (average > error) {
            digitalWrite (LED1, HIGH); 
        } 
        else {
            digitalWrite (LED1, LOW); 
        }
        //Serial.print("Average (last 10 readings): ");
        //Serial.println(average, 2); 

        vTaskDelayUntil(&xLastWakeTime, xFrequency);
    }
}

void Task5(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(200); //period of 200ms

    while (1) {
          xSemaphoreTake (task2_dataSemaphore,portMAX_DELAY);
          int frequency_2_scaled = (int)(((task2_data.measured_frequency_2 - 333) / (1000.0 - 333)) * 99);
          int frequency_3_scaled = (int)(((task2_data.measured_frequency_3 - 500) / (1000.0 - 500)) * 99);
          xSemaphoreGive (task2_dataSemaphore);
        //scales the frequencies between their thresholds
        
      
        frequency_2_scaled = frequency_2_scaled < 0 ? 0 : frequency_2_scaled > 99 ? 99 : frequency_2_scaled;
        frequency_3_scaled = frequency_3_scaled < 0 ? 0 : frequency_3_scaled > 99 ? 99 : frequency_3_scaled;

        //print scaled frequencies to serial monitor
        Serial.println();
        Serial.print("Scaled frequencies: ");
        Serial.print(frequency_2_scaled);
        Serial.print(",");
        Serial.println(frequency_3_scaled);
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);    // waits until next log period
    }
}


void Task6(void *pvParameters) {
    int previous_button_state = HIGH;  
    int button_state;             

    while (1) {
        int reading = digitalRead(BUTTON);    //read the state of the button

        if (reading != previous_button_state) {
            last_debounce = millis();
        }
        if ((millis() - last_debounce) > debounce) {    //update current state of button

            // if the button state has changed:
            if (reading != button_state) {
                button_state = reading;
                if (button_state == HIGH) {           //turn ON LED if its state is HIGH
                    led_state = !led_state;
                    digitalWrite(LED2, led_state);
                }
            }
        }
        previous_button_state = reading;      //update reading
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void Task7(void *pvParameters) {
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(20); // 20ms period

    while (1) {
        // Capture start time
        unsigned long start = micros();   //timestamps the start time

        CPU_work(2); // Call CPU_work to busy the CPU for 2ms or 2000 microseconds

        unsigned long end = micros();
        unsigned long duration = end - start;
        //print to serial monitor how long it took
        //Serial.println();
        //Serial.print("CPU_work time: ");
        //Serial.print(duration);
       // Serial.println(" microseconds");
        
        vTaskDelayUntil(&xLastWakeTime, xFrequency);    // Wait for the next cycle, ensuring the task runs with a period of 20ms
    }
}


void CPU_work(int time) {
    volatile long iterations_per_millisecond = 33850; //calibrate this value        //34060 or 33850
    long cycles = time * iterations_per_millisecond;

    for(long i = 0; i < cycles; i++) {
        __asm__("nop"); // Assembly instruction for "no operation"
    }
}


void IRAM_ATTR rising_edge_2() {              //this method is used to detect the rising edges for the signal in Task2  Instruction RAM attribute
  unsigned long currentTime = micros();       //this is placed in the instruction RAM instead of flash memory (faster)
  if(lastRiseTime2 > 0) {
    unsigned long period = currentTime - lastRiseTime2;

    xSemaphoreTake (task2_dataSemaphore,portMAX_DELAY);
    task2_data.measured_frequency_2 = 1000000.0 / period;
    xSemaphoreGive (task2_dataSemaphore);
  }
  lastRiseTime2 = currentTime;
}

void IRAM_ATTR rising_edge_3() {            //this method is used to detect the rising edges for the signal in Task3
  unsigned long currentTime = micros();
  if(lastRiseTime3 > 0) {
    unsigned long period = currentTime - lastRiseTime3;
    xSemaphoreTake (task2_dataSemaphore,portMAX_DELAY);
    task2_data.measured_frequency_3 = 1000000.0 / period;
    xSemaphoreGive (task2_dataSemaphore);
  }
  lastRiseTime3 = currentTime;
}

void loop(){
}
