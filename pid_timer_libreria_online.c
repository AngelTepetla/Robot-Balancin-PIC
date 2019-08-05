
#include <18F4550.h>

#build(reset=0x02000,interrupt=0x02008)
#org 0x0000,0x1FFF {}
// cristal de 8Mhz, CPU a 48M
#fuses HSPLL, PLL2, CPUDIV1, NOWDT, NOPROTECT, NOLVP, NODEBUG, NOVREGEN, USBDIV
#use delay(clock = 48000000)  
// -------------------------------------------------------------------------------------


// ---------------------------------  MODULO LCD I2C  ----------------------------------
#define MPU_SDA PIN_B0                             
#define MPU_SCL PIN_B1        
#use I2C(master, sda=MPU_SDA, scl=MPU_SCL, Fast) 
// ---------------------------------------------------------------------------------------

// cambiar el baudrate segun sea necesario
#use rs232(STREAM=UART, baud = 115200, parity = N, xmit = PIN_C6, rcv = PIN_C7, bits = 8, stop = 1)   //Comunicación serial a 9600bps, 1 bit de paro, sin paridad

#include <math.h>
#include "MPU6050.c"

// 180/PI
#define A_DEG 57.2957
#define A_RAD 0.0174532
#define REBOTE 250


//520
#define MINIMA_MOTOR 0
#define MAXIMA_MOTOR 597 // (PPR2 + 1) * 4
#define LIMITE_INTEGRAL 400


#define GYRO_SCALE 131.0 // lsb / (degrees / second)
#define PERIODO_LECTURA 20

// fc = 2Hz
#define FILTRO_C1 0.92
#define FILTRO_C2 0.08


volatile float angX;
unsigned int8 accX_H, accZ_H, accX_L, accZ_L, giroY_H, giroY_L;
signed int16 accX, accZ, giroG;

float giroY;
volatile float inclinacion_acc;
volatile float inclinacion = 0, inclinacion_old = 0, inclinacion_anterior;

// offset del giroscopio, se obtiene de manera externa
signed int16 OFFSETGY = 13;


volatile double integral, I = 0;
volatile float error, derivada, error_anterior = 0;
volatile float velo = 0;


volatile float P = 0, D = 0, auxx = 0, timing = 0;
volatile signed int16 salida = 0;


// variables a modificar para el PID
volatile float kp = 88, ki = 53, kd = 0.65, factor = 0.85, setpoint = 6.50;
volatile int16 min_motor = 300;
// factor es numero entre 0 y 1, para restringir la 'velocidad' maxima de los motores
// min_motor es el valor PWM donde apenas y se activan los motores


volatile float angle = 0;

char stringy[30];
char recibe = 0;

//
// Estructura:  [identificador][decena][unidad][.][decima][centesima]
//      para cambiar Kd:     p12.34        kd = 12.34
//                   ki:     i00.34        ki = 0.34
// no almacena nada en eeprom
#int_rda
void serial_isr(){

   //signed long ausy = 0;
   gets(stringy);                      // Obtenemos dato desde buffer USART...
   //ausy = atol(stringy);
   
   recibe = stringy[0];
   delay_ms(100);
   
   int8 a1, b1, c1, d1;
   int16 DD = 0, UU = 0, uu1 = 0, dd1 = 0;
   int16 auxU = 0;
   
   a1 = stringy[1];
   b1 = stringy[2];
   c1 = stringy[4];
   d1 = stringy[5];
   
   
   
   switch(recibe){
      
      case 'p':
     
         DD = a1 -48;
         UU = b1 -48;  // stringy 3 es el punto decimal  p45.00
         //   punto   ......   //
         uu1 = c1 -48;
         dd1 = d1 -48;
         
         auxU = (DD * 1000) + (UU * 100) + (uu1 * 10) + dd1;
         kp = (float)auxU; 
         kp = kp / 10;
         
         printf("\nNueva Kp: %2.2f (DD_UU_._uu_dd / 10) \n", kp);
         break;         
      
      
      case 'i':
             
         DD = a1 -48;
         UU = b1 -48;  // stringy 3 es el punto decimal  p45.00
         //   punto   ......   //
         uu1 = c1 -48;
         dd1 = d1 -48;
         
         auxU = (DD * 1000) + (UU * 100) + (uu1 * 10) + dd1;
         ki = (float)auxU; 
         ki = ki / 100;
         
         printf("\nNueva Ki: %2.2f\n", ki);
         break;
      
      case 'd':
         
         DD = a1 -48;
         UU = b1 -48;  // stringy 3 es el punto decimal  p45.00
         //   punto   ......   //
         uu1 = c1 -48;
         dd1 = d1 -48;
         
         auxU = (DD * 1000) + (UU * 100) + (uu1 * 10) + dd1;
         kd = (float)auxU; 
         kd = kd / 100;
         
         printf("\nNueva Kd: %2.2f\n", kd);
         break;
         
      case 'f':
         
         DD = a1 -48;
         UU = b1 -48;  // stringy 3 es el punto decimal  p45.00
         //   punto   ......   //
         uu1 = c1 -48;
         dd1 = d1 -48;
         
         auxU = (DD * 1000) + (UU * 100) + (uu1 * 10) + dd1;
         factor = (float)(auxU);
         factor = factor / 100;
         
         printf("\nNuevo factor: %2.2f\n", factor);
         break;
         
      case 'm':
     
         DD = a1 -48;
         UU = b1 -48;  // stringy 3 es el punto decimal  p45.00
         uu1 = c1 -48;
         dd1 = d1 -48;
         
         auxU = (DD * 1000) + (UU * 100) + (uu1 * 10) + dd1;
         min_motor = auxU;
         
         printf("\nNueva minima motor: %ld\n", min_motor);
         break;
      
      case 's':
     
         DD = a1 -48;
         UU = b1 -48;  // stringy 3 es el punto decimal  p45.00
         //   punto   ......   //
         uu1 = c1 -48;
         dd1 = d1 -48;
         
         auxU = (UU * 100) + (uu1 * 10) + dd1;
         setpoint = (float)auxU;
         setpoint = setpoint / 10;
         
         if(DD == 1){
            setpoint = -setpoint;
         }
         else{
            //setpoint = setpoint;
         }
         
         printf("\nNuevo setpoint: %2.2f (offset 00 | 1-, 0+)\n", setpoint);
         break;         
      
      
      case 'e':
         printf("\n\nSP: %f angulo: %f error: %f P: %f I: %f D: %f salida: %ld auxx: %4.2f\n", setpoint, angX, error, P, I, D, salida, auxx);
         printf("min_motor: %ld \n", min_motor);
         break;
      
      case 'c':
         printf("\nerror: %f P: %f I: %f D: %f salida: %ld int: %f der: %f timing: %f \n", error, P, I, D, salida, integral, derivada, timing);
         break;
         
      case 'r':
         integral = 0;
         derivada = 0;
         error = 0;
         
         printf("\nReset variables integral, derivada, error \n");
         break;
  
      default:
         //printf("p i d e c \n");
         break;
            
   } // switch
      
  
  printf("%3.2f %3.2f %3.2f \n", kp, ki, kd);
  
  clear_interrupt(INT_RDA);      // clears the timer0 interrupt flag


}

float angular(void){
   
   accX_H = Mpu6050_Read(MPU6050_RA_ACCEL_XOUT_H);
   accZ_H = Mpu6050_Read(MPU6050_RA_ACCEL_ZOUT_H);
   giroY_H = Mpu6050_Read(MPU6050_RA_GYRO_YOUT_H);
            
   accX_L = Mpu6050_Read(MPU6050_RA_ACCEL_XOUT_L);  
   accZ_L = Mpu6050_Read(MPU6050_RA_ACCEL_ZOUT_L);
   giroY_L = Mpu6050_Read(MPU6050_RA_GYRO_YOUT_L);
            
   accX = make16(accX_H, accX_L);
   accZ = make16(accZ_H, accZ_L);
   giroG = make16(giroY_H, giroY_L);
                 
   giroY = (float) (giroG / -GYRO_SCALE);

   inclinacion_acc = (atan2(accX, accZ)+PI) * A_DEG; // angulo de inclinacion del acelerometro
   inclinacion_old = FILTRO_C1 * ( inclinacion_old + giroY * 0.0218) + (FILTRO_C2 * inclinacion_acc);
   angX = inclinacion_old -180; // angulo
   
   return angX;
}

// Control motores
void motores(signed int16 velo, int16 minimaMotor, float angulo){

   int direccion = 1;
   int16 velocidad;
   char dir = ' ';
    
   if(velo < 0){
      direccion = 2;
      
      signed int16 compa = minimaMotor * -1;
      if(velo < compa ){
         velocidad = -1 * velo;
      }
      else{
         velocidad = minimaMotor;
      }
      dir = 'R';
      output_low(PIN_B5);
      output_high(PIN_B6);

   }
   
   else{
   
      if(velo > minimaMotor){
         velocidad = velo;
      }
      else{
         velocidad = minimaMotor;
      }
      
      dir = 'L';
      output_low(PIN_B6);
      output_high(PIN_B5);
      
   }
   
   if(angulo > 45 || angulo < -45){
      velocidad = 0;
      set_pwm2_duty(0);
   }
   else{
      auxx = (float)(velocidad);
      auxx = auxx * factor;
      velocidad = (int16)(auxx);
      set_pwm2_duty(velocidad);   
   }
   
} // funcion motores

// PID YO 25/07/19 14:21
void PID(float anguloIn, int16 valor_timer){
   
   float timing = (float)(valor_timer * 0.001);
   
   // calcula valores para el PID
   error = setpoint - anguloIn;
   derivada = (anguloIn - inclinacion_anterior) / timing;
   integral = integral + (error * ki * timing);
   
   /*
   if(error > 1.0){
      integral = 0;
   }
   */
   
   if(integral > MAXIMA_MOTOR){  integral = MAXIMA_MOTOR;  }
   else if( integral < -MAXIMA_MOTOR){  integral = -MAXIMA_MOTOR;  }
      
   // Calcula PID
   P = error * kp;
   I = integral;
   D = derivada * kd;
      
   salida = (int16) ( P + I - D);
   //salida = (int16) ( kp * error + integral + kd * derivada );
   if (salida > MAXIMA_MOTOR){
      salida = MAXIMA_MOTOR;
   }
   else if (salida < -MAXIMA_MOTOR){
      salida = -MAXIMA_MOTOR;
   }
     
   // Pone el valor de la salida, en los motores
   motores(salida, min_motor, anguloIn);
   error_anterior = error;
   inclinacion_anterior = anguloIn;
        
} // funcion PID


#int_timer0 
void TIMER0_isr(void){

   // lee el angulo de inclinacion
   angle = angular();
   
   // llama a la funcion de calculo de PID, y activa los motores
   PID(angle, 22); // 21.8ms overflow
   
   
} 


void main(void){ 

   set_tris_a(0xFF);
   set_tris_b(0b11101110);         //Configura puerto B  1= entrada  0= salida
   set_tris_c(0xBF);                      // Configuramos puerto c (PINES DE COMUNICACION SERIAL Y PIN DE ENTRADA BOTON EN RC2)
   
   
   setup_timer_0(RTCC_INTERNAL|RTCC_DIV_4);      //21,8 ms overflow // tiempo de muestreo de 21.8ms, casi 22
   setup_timer_2(T2_DIV_BY_16, 149, 1);      //200 us overflow, 200 us interrupt, PWM 5Khz

   setup_ccp2(CCP_PWM);
   set_pwm2_duty(0);

   enable_interrupts(INT_TIMER0);
   enable_interrupts(INT_RDA);
   enable_interrupts(GLOBAL);
 
   
   delay_ms(100);
   Mpu6050_Init();
      
   int8 x;
   x = Mpu6050_Read(MPU6050_RA_WHO_AM_I);
                                                       
   if(x != 0x68){
      //printf ("\nConnection ERR!!!");
      return;
   } // capacitor ceramico de 0.1uF, puente H, a 12V capacitor de almenos 470uF
   
   int8 primero = 0, segundo = 0;
   
   primero = OFFSETGY >> 8;
   segundo = OFFSETGY & 255;
   // offset giroscopio Y
   Mpu6050_Write(0x15, primero);  // offset gyH
   Mpu6050_Write(0x16, segundo);  // offset gyL
   
   printf("\nArranca! \n");

   while( 1 ) { 

   } 
  

  
} // main  
