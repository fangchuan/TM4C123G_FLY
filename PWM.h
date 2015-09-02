#define Moto_PwmMax 4699
extern long PWM;
void PWM_Init(void);
void PWM_Set(unsigned long motor1,unsigned long motor2,unsigned long motor3,unsigned long motor4);
void Moto_PwmRflash(void);