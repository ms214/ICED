#define PIN_LED 7
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  count = toggle = 0;
  digitalWrite(PIN_LED, toggle); // turn on LED.
}

void loop() {
  if(count == 0){
    delay(1000);
  }
  while(count<10){
    count = count +1 ;
    toggle = toggle_state(toggle);
    digitalWrite(PIN_LED, toggle);
    delay(100);
  }
  digitalWrite(PIN_LED, 1);
  while(1){} // infinite loop
}

int toggle_state(int toggle) {
  if(toggle == 1){
    toggle = 0;
  }else{
    toggle = 1;
  }
  return toggle;
}
