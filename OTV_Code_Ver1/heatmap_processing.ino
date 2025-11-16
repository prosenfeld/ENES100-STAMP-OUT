/***************************************************************************
  This is a library for the AMG88xx GridEYE 8x8 IR camera

  This sketch tries to read the pixels from the sensor

  Designed specifically to work with the Adafruit AMG88 breakout
  ----> http://www.adafruit.com/products/3538

  These sensors use I2C to communicate. The device's I2C address is 0x69

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Dean Miller for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/

#include <Wire.h>
#include <Adafruit_AMG88xx.h>

Adafruit_AMG88xx amg;

float pixels[AMG88xx_PIXEL_ARRAY_SIZE];
float heatmap[8][8];
// this gets updated dynicammly for ambient; but putting a theoretical value to start.
float threshold = 40;


void setup() {
    Serial.begin(9600);
    Serial.println(F("AMG88xx pixels"));

    bool status;
    
    // default settings
    status = amg.begin();
    if (!status) {
        Serial.println("Could not find a valid AMG88xx sensor, check wiring!");
        while (1);
    }
    
    Serial.println("-- Pixels Test --");

    Serial.println();

    delay(100); // let sensor boot up
}


void loop() { 
    //read all the pixels
    amg.readPixels(pixels);
    // convert that data to a matrix
    arrayToMatrix8x8(pixels,heatmap);
    // dynamically adjust our threshold based on ambient based on the top of the frame
    threshold = computeThreshold(heatmap);
    // Print results
    Serial.print("Corner fires: ");
    Serial.print(countCorners());
    Serial.println();
    Serial.print("Total fires: ");
    Serial.print(countCorners() + 1);
    Serial.println();


    // Serial.print("[");
    // for(int i=1; i<=AMG88xx_PIXEL_ARRAY_SIZE; i++){
    //   Serial.print(pixels[i-1]);
    //   Serial.print(", ");
    //   if( i%8 == 0 ) Serial.println();
    // }
    // Serial.println("]");
    // Serial.println();

    //delay a second
    delay(1000);
}

// Count how many corners are above our threshold
int countCorners() {
  int count = 0;

  // top-left region A4:C6  → rows 2–4, cols 0–2
  if (regionHasValueAbove(heatmap, 2, 4, 0, 2, threshold)) count++;

  // top-right region F4:H6 → rows 2–4, cols 5–7
  if (regionHasValueAbove(heatmap, 2, 4, 5, 7, threshold)) count++;

  // bottom-left region A8:C9 → rows 6–7, cols 0–2
  if (regionHasValueAbove(heatmap, 6, 7, 0, 2, threshold)) count++;

  // bottom-right region F8:H9 → rows 6–7, cols 5–7
  if (regionHasValueAbove(heatmap, 6, 7, 5, 7, threshold)) count++;

  return count;
}

// average the top two rows and add 8 to adjust the threshold for ambient.
float computeThreshold(float arr[8][8]) {
    float sum = 0;
    int count = 0;

    for (int r = 0; r < 2; r++) {        // first two rows: 0 and 1
        for (int c = 0; c < 8; c++) {
            sum += arr[r][c];
            count++;
        }
    }

    return sum / count + 8;
}

// check for values above the threshold
bool regionHasValueAbove(float arr[8][8], int r1, int r2, int c1, int c2, float th) {
  for (int r = r1; r <= r2; r++) {
    for (int c = c1; c <= c2; c++) {
      if (arr[r][c] > th) return true;
    }
  }
  return false;
}

// array of length 64 -> 8x8 matrix 
void arrayToMatrix8x8(float inArray[64], float outMatrix[8][8]) {
    int index = 0;
    for (int r = 0; r < 8; r++) {
        for (int c = 0; c < 8; c++) {
            outMatrix[r][c] = inArray[index++];
        }
    }
}

