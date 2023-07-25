#include <cstdio>     // printf()
#include <wiringPi.h> // wiringPiSetupGpio() pinMode() pwmSetMode() pwmSetRange() pwmSetClock() pwmWrite() delay()

int main() {
    int pin = 18;

    printf("Raspberry Pi wiringPi test program\n");
    wiringPiSetupGpio();
    pinMode(pin, PWM_OUTPUT);
    pwmSetMode(PWM_MODE_MS);

    pwmSetClock(192);
    pwmSetRange(2000);

    pwmWrite(pin, 10);
    delay(100);
    pwmWrite(pin, -10);
    delay(100);
    pwmWrite(pin, 0);
    return 0;
}
