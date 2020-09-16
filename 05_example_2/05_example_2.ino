#define PIN_LED 7
unsigned int count, toggle;

void setup() {
  pinMode(PIN_LED, OUTPUT);
  Serial.begin(115200); // Initialize serial port
  while (!Serial) {
    ; // wait for serial port to connect.
  }
  Serial.println("Hello World!");
  count = toggle = 0;
  //digitalWrite(PIN_LED, 1); // turn off LED.
}

void loop() {
  count = count +1 ;
  Serial.println(count);
  if(count <= 12 && count > 1){
    toggle = toggle_state(toggle);
    digitalWrite(PIN_LED, toggle);
    delay(100);
  }else if(count > 12){
    digitalWrite(PIN_LED, 0);
    while(1){} // infinite loop
  }else{
    digitalWrite(PIN_LED, 1);
    delay(1000);
  }
}

int toggle_state(int toggle) {
  if(toggle == 1){
    toggle = 0;
  }else{
    toggle = 1;
  }
  return toggle;
}
