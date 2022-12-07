// Define the values for the reflectence of each piece.
#define B_Reflect 1023 
#define W_Reflect  1000
#define A_Reflect  900
#define S_Reflect 750

#define B_ID  10
#define W_ID 20
#define A_ID  30
#define S_ID  40

void Queue_Part(int Reflect_Val);
void ADC_Init(void);

volatile int isActive = 0; // Specifies if the part has entered the first optical sensor or if it is leaving.
volatile int Part_REF_Max = 0;

link *head;
link *tail;
link *newLink;
link *rtnLink;