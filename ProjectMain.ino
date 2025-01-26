#include "KFaccn.h"
#include "KFheight.h"

KFaccn accnFilter(0.1, 0.5, 1.0);   // Q, R, P
KFheight heightFilter(0.1, 0.5, 0.01); // Q, R, dt


void setup() {
    Serial.begin(115200);
}

void loop() {
    float mAccn = getAccnFromSensor();
    float mHeight = getHeightFromSensor();

    accnFilter.update(mAccn);
    float pAccn = accnFilter.getState();

    heightFilter.predict(pAccn);
    heightFilter.update(mHeight);
    float pHeight = heightFilter.getHeight();

    Serial.print("Measured Acceleration: ");
    Serial.print(mAccn);
    Serial.println("Predicted Acceleration: ");
    Serial.print(pAccn);
    Serial.println("Measured Height: ");
    Serial.print(mHeight);
    Serial.println("Predicted Height: ");
    Serial.print(pHeight);

    delay(10);
}

float getAccnFromSensor() {
    return 9.8;
}

float getHeightFromSensor() {
    return 100.0;
}
