#include <Adafruit_NeoPixel.h>

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 2 // Popular NeoPixel ring size

#define LED00_PIN 0
#define LED28_PIN 28
#define RGB_PIN 18


// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, RGB_PIN, NEO_GRB + NEO_KHZ800);


// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pins LEDxx_PIN as an output.
  pinMode(LED00_PIN, OUTPUT);
  pinMode(LED28_PIN, OUTPUT);
  Serial.begin(115200);

  pixels.begin(); // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear(); // Set all pixel colors to 'off'

  Serial.println("Setup done");
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED28_PIN, LOW);   // change state of the LED by setting the pin to the LOW voltage level
  digitalWrite(LED00_PIN, HIGH);  // change state of the LED by setting the pin to the HIGH voltage level
  pixels.setPixelColor(0, pixels.Color(0, 0, 0));
  pixels.setPixelColor(1, pixels.Color(0, 0, 150));
  pixels.show();   // Send the updated pixel colors to the hardware.

  delay(200);                      // wait for a second
  
  digitalWrite(LED28_PIN, HIGH);  // change state of the LED by setting the pin to the HIGH voltage level
  digitalWrite(LED00_PIN, LOW);   // change state of the LED by setting the pin to the LOW voltage level
  pixels.setPixelColor(0, pixels.Color(150, 0, 0));
  pixels.setPixelColor(1, pixels.Color(0, 0, 0));
  pixels.show();   // Send the updated pixel colors to the hardware.
  
  delay(200);                      // wait for a second
  
}

