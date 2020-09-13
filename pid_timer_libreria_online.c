
#include <18F4550.h>
#include <math.h>
#include "constantes.h"

// ----------------------------  REMAPEO PARA BOOTLOADER  ------------------------------
#build(reset=0x02000, interrupt=0x02008)
#org 0x0000, 0x1FFF {}
// -------------------------------------------------------------------------------------

#fuses HSPLL, NOWDT, NOPROTECT, NOLVP, NODEBUG, USBDIV, PLL5, CPUDIV1, VREGEN
#use delay(clock = 48000000)

// ---------------------------------  MODULO LCD I2C  ----------------------------------
#use I2C(master, sda = MPU_SDA, scl = MPU_SCL, Fast) 
#include "MPU6050.c"
// -----------------------------------    TIMER 1   ------------------------------------
#use TIMER(TIMER = 1, TICK = 500us, BITS = 16, NOISR)  

// -----------------------------------    SERIAL    ------------------------------------
#use RS232(STREAM = UART, baud = 115200, parity = N, xmit = PIN_C6, rcv = PIN_C7, bits = 8, stop = 1)   //Comunicación serial a 9600bps, 1 bit de paro, sin paridad


// ----------------------------------    G_PID    -----------------------------------
// Factor es numero entre 0 y 1, para restringir la 'velocidad' maxima de los motores
// Minima_Motor es el valor PWM donde apenas y se activan los motores
// Setpoint es el angulo deseado respecto al suelo, idealmente debe ser 0, depende de si el balancin naturalmente se inclina hacia cierto lado
volatile float Kp = 88, Ki = 53, Kd = 0.65, Factor = 0.85, Setpoint = 6.50;
signed long Minima_Motor = 300;


// --------------------------------    GYRO/ACC    ----------------------------------
signed long OFFSETGY = 13; // offset del giroscopio, se obtiene de manera externa

volatile float angX;
int8 accX_H, accZ_H, accX_L, accZ_L, giroY_H, giroY_L;
signed long accX, accZ, giroG;


// -----------------------------------    PID    ------------------------------------
volatile double Integral, I = 0;
volatile float Error, Error_anterior = 0, Derivada;

volatile float P = 0, D = 0, Angulo_X = 0;
volatile float Inclinacion_Actual = 0, Inclinacion_Anterior = 0;
volatile signed long Salida = 0;


char stringy[30];
char recibe = 0;

// Estructura:  [identificador][decena][unidad][.][decima][centesima]
// ejemplo:                   p45.12
//      para cambiar Kd:     p12.34        Kd = 12.34
//                   Ki:     i00.34        Ki = 0.34
// no almacena nada en eeprom

#int_rda
void serial_isr()
{
   gets(stringy);                      // Obtenemos dato desde buffer USART...   
   

   delay_ms(100);
   
   int8_t a1, b1, c1, d1;
   signed long DD = 0, UU = 0, uu1 = 0, dd1 = 0;
   signed long auxU = 0;

   recibe = stringy[0];
   a1 = stringy[1];
   b1 = stringy[2];
   // stringy 3 es el punto decimal (XX.XX)
   c1 = stringy[4];
   d1 = stringy[5];

   delay_ms(1);
   DD = a1 - OFFSET_ASCII;
   UU = b1 - OFFSET_ASCII;  // stringy 3 es el punto decimal  p45.00
   uu1 = c1 - OFFSET_ASCII;
   dd1 = d1 - OFFSET_ASCII;
   auxU = (DD * 1000) + (UU * 100) + (uu1 * 10) + dd1;

   switch(recibe)
   {
      case 'p':
         Kp = (float) auxU; 
         Kp = Kp / 10;
         
         printf("\nNueva Kp: %2.2f\n", Kp);
         break;         
      
      case 'i':
         Ki = (float) auxU; 
         Ki = Ki / 100;
         
         printf("\nNueva Ki: %2.2f\n", Ki);
         break;
      
      case 'd':
         Kd = (float) auxU; 
         Kd = Kd / 100;
         
         printf("\nNueva Kd: %2.2f\n", Kd);
         break;
         
      case 'f':
         Factor = (float)(auxU);
         Factor = Factor / 100;
         
         printf("\nNuevo Factor: %2.2f\n", Factor);
         break;
         
      case 'm':
         auxU = (DD * 1000) + (UU * 100) + (uu1 * 10) + dd1;
         Minima_Motor = auxU;
         
         printf("\nNueva minima motor: %ld\n", Minima_Motor);
         break;
      
      case 's':
         auxU = (UU * 100) + (uu1 * 10) + dd1;

         Setpoint = (float) auxU;
         Setpoint = Setpoint / 10;
         
         if(DD == 1)
         {
            Setpoint = -Setpoint;
         }
         else
         {
            //Setpoint = Setpoint;
         }
         
         printf("\nNuevo Setpoint: %2.2f (offset 00 | 1-, 0+)\n", Setpoint);
         break;         
      
      case 'e':
         printf("\n\nSP: %f angulo: %f Error: %f P: %f I: %f D: %f Salida: %ld\n", Setpoint, angX, Error, P, I, D, Salida);
         printf("Minima_Motor: %ld \n", Minima_Motor);
         break;
      
      case 'c':
         printf("\nError: %f P: %f I: %f D: %f Salida: %ld int: %f der: %f\n", Error, P, I, D, Salida, Integral, Derivada);
         break;
         
      case 'r':
         Integral = 0;
         Derivada = 0;
         Error = 0;
         
         printf("\nReset variables Integral, Derivada, Error\n");
         break;
  
      default:
         //printf("p i d e c \n");
         break;
            
   } // switch
      
  
  printf("%3.2f %3.2f %3.2f \n", Kp, Ki, Kd);
  clear_interrupt(INT_RDA);      // clears the interrupt flag
}


float Leer_Angulo(void)
{
   float inclinacion_acelerometro;
   float inclinacion_giroscopio;

   accX_H = Mpu6050_Read(MPU6050_RA_ACCEL_XOUT_H);
   accZ_H = Mpu6050_Read(MPU6050_RA_ACCEL_ZOUT_H);
   giroY_H = Mpu6050_Read(MPU6050_RA_GYRO_YOUT_H);
            
   accX_L = Mpu6050_Read(MPU6050_RA_ACCEL_XOUT_L);  
   accZ_L = Mpu6050_Read(MPU6050_RA_ACCEL_ZOUT_L);
   giroY_L = Mpu6050_Read(MPU6050_RA_GYRO_YOUT_L);
            
   accX = make16(accX_H, accX_L);
   accZ = make16(accZ_H, accZ_L);
   giroG = make16(giroY_H, giroY_L);

   inclinacion_acelerometro = ( atan2(accX, accZ) + PI ) * A_DEG; // angulo de inclinacion del acelerometro
   inclinacion_giroscopio = (float) (giroG / -GYRO_SCALE); // angulo de inclinacion del giroscopio

   Inclinacion_Actual = FILTRO_C1 * ( Inclinacion_Actual + inclinacion_giroscopio * 0.0218) + (FILTRO_C2 * inclinacion_acelerometro); // filtro complementario

   angX = Inclinacion_Actual - 180; // angulo sobre el eje X
   return angX;
}


void Control_Motores(signed long salida, signed long velocidad_minima, float angulo)
{
   signed long vdelocidad = velocidad_minima;
   float velocidad_escalada = 0;
    
   if(salida < 0)  // si la salida es negativa, se hace cambio de direccion
   {
      salida = -1 * salida;  // se elimina el signo negativo
      output_low(PIN_B5);
      output_high(PIN_B6);
   }
   else
   {
      output_low(PIN_B6);
      output_high(PIN_B5);
   }

   if(salida > velocidad_minima )
   {
      vdelocidad = salida;
   }

   if(angulo > 45 || angulo < -45) // si el balancin se cae, se apagan los motores
   {
      vdelocidad = 0;
   }
   else
   {
      velocidad_escalada = (float)(vdelocidad); // se escala la valocidad maxima, segun el Factor (0 a 100%)
      velocidad_escalada = velocidad_escalada * Factor; // **esto podria no ser necesario
      vdelocidad = (signed long)(velocidad_escalada);  
   }
   set_pwm2_duty(vdelocidad);
} // funcion motores


void PID(float angulo_entrada, unsigned long valor_timer)
{
   float segundos_transcurridos;
   segundos_transcurridos = (float)(valor_timer * 0.001);
   
   // calcula valores para el PID
   Error = Setpoint - angulo_entrada;
   Derivada = (angulo_entrada - Inclinacion_Anterior) / segundos_transcurridos;
   Integral = Integral + (Error * Ki * segundos_transcurridos);
   
   /*
   if(Error > 1.0)
   {
      Integral = 0;
   }
   */
   
   if( Integral > D_MAXIMA_MOTOR )
   {
      Integral = D_MAXIMA_MOTOR;
   }
   else if( Integral < -D_MAXIMA_MOTOR )
   {
      Integral = -D_MAXIMA_MOTOR;
   }
      
   P = Error * Kp;
   I = Integral;
   D = Derivada * Kd;
      
   Salida = (unsigned long) ( P + I - D);
   
   if (Salida > D_MAXIMA_MOTOR)
   {
      Salida = D_MAXIMA_MOTOR;
   }
   else if (Salida < -D_MAXIMA_MOTOR)
   {
      Salida = -D_MAXIMA_MOTOR;
   }
     
   // Pone el valor de la Salida, en los motores
   Control_Motores(Salida, Minima_Motor, angulo_entrada);
   Error_anterior = Error;
   Inclinacion_Anterior = angulo_entrada;
        
}


#int_timer0 
void TIMER0_isr(void)
{
   Angulo_X = Leer_Angulo(); // lee el angulo de inclinacion
   
   // llama a la funcion de calculo de PID, y activa los motores
   PID(Angulo_X, 22); // 21.8ms overflow 
} 


void main(void)
{ 
   int8_t x = 0;
   int8_t byte_alto = 0, byte_bajo = 0;

   set_tris_a(0xFF);
   set_tris_b(0b11101110);         //Configura puerto B  1= entrada  0= Salida
   set_tris_c(0xBF);                      // Configuramos puerto c (PINES DE COMUNICACION SERIAL Y PIN DE ENTRADA BOTON EN RC2)
   
   setup_timer_0(RTCC_INTERNAL|RTCC_DIV_4);      //21,8 ms overflow // tiempo de muestreo de 21.8ms, casi 22
   setup_timer_2(T2_DIV_BY_16, 149, 1);      //200 us overflow, 200 us interrupt, PWM 5Khz

   setup_ccp2(CCP_PWM);
   set_pwm2_duty(0);

   enable_interrupts(INT_TIMER0);
   enable_interrupts(INT_RDA);
   enable_interrupts(GLOBAL);
 
   // capacitor ceramico de 0.1uF, puente H, a 12V capacitor de almenos 470uF
   delay_ms(100);
   Mpu6050_Init();

   x = Mpu6050_Read(MPU6050_RA_WHO_AM_I);
   byte_alto = OFFSETGY >> 8;
   byte_bajo = OFFSETGY & 255;
                                                    
   if(x != 0x68)
   {
      //printf ("\nMPU Conexion Error!");
      return;
   } 
   
   // offset giroscopio Y
   Mpu6050_Write(0x15, byte_alto);  // offset gyH
   Mpu6050_Write(0x16, byte_bajo);  // offset gyL
   
   printf("\nArranca! \n");

   while( 1 ) 
   { 
      //
   } 
  
  
} // main  
