#include <Enes100.h>

#include "HUSKYLENS.h"
#include "SoftwareSerial.h"

HUSKYLENS huskylens;
SoftwareSerial mySerial(10, 9); // RX, TX
//HUSKYLENS green line >> Pin 10; blue line >> Pin 11
void printResult(HUSKYLENSResult result);

void setup() {
    Serial.begin(115200);
    mySerial.begin(9600);

    Enes100.begin("STAMP Out!", FIRE, 24, 1116, 4, 2);

    while (!huskylens.begin(mySerial))
    {
        Serial.println(F("Begin failed!"));
        Serial.println(F("1.Please recheck the \"Protocol Type\" in HUSKYLENS (General Settings>>Protocol Type>>Serial 9600)"));
        Serial.println(F("2.Please recheck the connection."));
        delay(100);
    }
}

void loop() {
    // if (!huskylens.request()) Serial.println(F("Fail to request data from HUSKYLENS, recheck the connection!"));
    // else if(!huskylens.isLearned()) Serial.println(F("Nothing learned, press learn button on HUSKYLENS to learn one!"));
    // else if(!huskylens.available()) Serial.println(F("No block or arrow appears on the screen!"));
    // else
    // {
    //     Serial.println(F("###########"));

    Serial.println(huskylens.available());
    delay(100);
        // while (huskylens.available())
        // {
        //     HUSKYLENSResult result = huskylens.read();
        //     // Serial.println(result.ID);

        //     if(result.ID == 1){
        //       Enes100.mission(TOPOGRAPHY, TOP_B);
        //     } else if(result.ID == 2){
        //        Enes100.mission(TOPOGRAPHY, TOP_C);
        //     } else if (result.ID == 3){
        //       Enes100.mission(TOPOGRAPHY, TOP_A);
        //     }
            
        //     printResult(result);


        // }    
    // }
}

void printResult(HUSKYLENSResult result){
    if (result.command == COMMAND_RETURN_BLOCK){
        Serial.println(String()+F("Block:xCenter=")+result.xCenter+F(",yCenter=")+result.yCenter+F(",width=")+result.width+F(",height=")+result.height+F(",ID=")+result.ID);
    }

}
