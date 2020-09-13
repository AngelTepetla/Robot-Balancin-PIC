

// ---------------------------------  MODULO LCD I2C  ----------------------------------
#define MPU_SDA PIN_B0                             
#define MPU_SCL PIN_B1        

#define int8_t int8
#define int16_t int16
#define uint8_t unsigned int8

// 180/PI
#define A_DEG 57.2957
#define A_RAD 0.0174532
#define REBOTE 250

// Motores
#define PWM_MS_TEST 5
#define D_MINIMA_MOTOR 0
#define D_MAXIMA_MOTOR 597 // (PPR2 + 1) * 4
#define LIMITE_INTEGRAL 400

// fc = 2Hz
#define FILTRO_C1 0.92 // 0.90660
#define FILTRO_C2 0.08 // 0.09337

// mgl sen 0
//#define MASA 0.857
//#define LONGITUD 0.115
//#define GRAVEDAD 9.81
#define MGL 0.9668
#define GYRO_SCALE 131.0 // lsb / (degrees / second)
#define PERIODO_LECTURA 20

#define OFFSET_ASCII 48
