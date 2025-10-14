extern void runTcm();
extern void initTcm();

typedef struct {
    float X;
    float Y;
    float Z;
} XYZ;

typedef struct {
    float gain[3][3];   // 3x3 gain matrix
    float offset[3];    // 3x1 offset vector
    float cubic[3];     // 3x1 cubic correction
} CubicAccelerometer;

typedef struct {
    float softIron[3][3];  // 3x3 soft-iron correction matrix
    float hardIron[3];     // 3x1 hard-iron offset
} CubicMagnetometer;

typedef struct {
    XYZ Acc;
    XYZ Mag;
    float Temp;
    float Batt;
} Sensors;

typedef struct {
    float rollRad;
	float pitchRad;
	float yawRad;
} RPY;

typedef struct {
    float North;
	float East;
} Velocity;

typedef struct {
    int TMO;
    int TMR;
    float TMA;
    float TMB;
    float TMC;
    float TMD;
} TempCalCoef;

typedef struct {
    char version[12];
    int serialNum;
    Sensors raw;
    Sensors scaled;
    TempCalCoef tempCal;
    CubicMagnetometer magCal;
    CubicAccelerometer accCal;
	RPY orientation;
	float headingDeg;
	Velocity current;
} TcmInfo;

typedef struct {
	Sensors rawSum;
	int sampleCount;
} TcmAverage;
