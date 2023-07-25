#include <cstdio>     // printf()
#include <wiringPi.h> // wiringPiSetupGpio() pinMode() pwmSetMode() pwmSetRange() pwmSetClock() pwmWrite() delay()

int main() {
    int pin = 13;

    printf("Raspberry Pi wiringPi test program\n");
    wiringPiSetupGpio();
    pinMode(pin, PWM_OUTPUT);
    pwmSetMode(PWM_MODE_MS);

    pwmSetClock(192);
    pwmSetRange(2000);

    pwmWrite(pin, 110);
    delay(1000);
    pwmWrite(pin, 145);
    delay(1000);
    pwmWrite(pin, 180);
    delay(1000);
    pwmWrite(pin, 145);
    return 0;
}
