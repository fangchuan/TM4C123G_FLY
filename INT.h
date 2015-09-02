typedef struct _RC_S{
	long ch1;
	long ch2;
	long ch3;
	long ch4;
}RC_S;
extern RC_S RC;
void PortAIntHandler(void);
void ReceiveInit(void);