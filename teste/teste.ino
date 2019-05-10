  
#include <avr/pgmspace.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>

/* ****************************************************************** */
/* *** Definições diversas ****************************************** */

// Descomente para inverter a rotação (somente) do Motor A
// => Necessário com os motores montados simetricamente

#define SSPEED        115200   // Velocidade da interface serial

#define FULL_BAT        8000   // Valor em mV para bat. completamente carregada
#define DEAD_BAT        6000   // Valor em mV para bat. esgotada ( recarregar )

/* ****************************************************************** */
/* Limitações de PWM e velocidade para uso no controlador PID ******* */

#define PWM_MIN         0x0F   // PWM mín. p/ garantir movimento das duas rodas
#define PWM_MAX         0x9F   // PWM máx. para que os motores tenham aprox. 5V
#define SPD_MIN           50   // Vel. mín. em mm/s ( Condição: PWM > PWM_MIN )
#define SPD_MAX          500   // Vel. máx. em mm/s ( Condição: PWM < PWM_MAX )

/* ****************************************************************** */
/* Estas são as conexões de hardware mapeadas aos pinos do Arduino ** */

#define LED         4      // Led conectado a uma saída digital

#define RADIO_CE    7      // Pino CE do módulo de rádio
#define RADIO_CS    8      // Pino CS do módulo do rádio
#define RADIO_A0   A4      // Bit 0 do end. do rádio (LOW = ligado)
#define RADIO_A1   A5      // Bit 1 do end. do rádio (LOW = ligado)

#define IRQ_ENC_A   2      // Pino de interrupção do Encoder A
#define IRQ_ENC_B   3      // Pino de interrupção do Encoder B
#define IRQ_RADIO   5      // Pino de interrupção do Rádio

#define HBRID_EN    6      // Habilita a ponte H (High)
#define MTR_AIN1   A2      // Bit 0 - Controle da ponte H do Motor A
#define MTR_AIN2   A3      // Bit 1 - Controle da ponte H do Motor A
#define MTR_BIN1   A1      // Bit 0 - Controle da ponte H do Motor B
#define MTR_BIN2   A0      // Bit 1 - Controle da ponte H do Motor B
#define MTR_PWMA    9      // Sinal de PWM para controle  do Motor A
#define MTR_PWMB   10      // Sinal de PWM para controle  do Motor B

#define VOLT_BAT   A7      // Tensão da bateria -> Vcc/10


//Macros
// Transforma caractere ascii em um número de 4 bits em hexadecimal.
#define asc2hex(a) (((a) < 'a') ? ( (a) - '0' )  : ( ((a) - 'a')+10) )

/* ****************************************************************** */
/* Mapeamentos de teste ********************************************* */

#define TimeBlink 2000    //Valor em Millisegundos.

/* ******************************************************************* */
/* Definições de estruturas de dados ( funcionais, status e controle ) */


typedef struct {
    uint32_t last_10ms   = 0;   // Controle das tarefas executadas a cada 10ms
    uint32_t last_100ms  = 0;   // Controle das tarefas executadas a cada 100ms
    uint32_t last_1000ms = 0;   // Controle das tarefas executadas a cada 1000ms
    uint32_t last_1min   = 0;   // Controle das tarefas executadas a cada 1 minuto.
    
} TasksTCtr;

typedef union {
    struct {
        uint8_t  pwm_motor_B;        // (bits 0-7)   8 bits: Valor do PWM do Motor B
        uint8_t  pwm_motor_A;        // (bits 8-15)  8 bits: Valor do PWM do Motor A
        uint8_t  dir_motor_B : 2,    // (bits 16-17) 2 bits: BIN1 e BIN2  da ponte H
                 dir_motor_A : 2,    // (bits 18-19) 2 bits: AIN1 e AIN2  da ponte H
                 ign1_4b     : 4;    // (bits 20-23) 4 bits não utilizados (padding)
        uint8_t  ign2_8b;            // (bits 24-31) 8 bits não utilizados (padding)
    } config;
    uint32_t status = 0;             // Leitura/Escrita simuntânea do conjunto de variáveis.
} TMotCtrl;


/* ******************************************************************* */
/* *** Variáveis globais e instanciações ***************************** */


TasksTCtr tasks;		    //Contagem de tempo para execução de tarefas
uint16_t count_enc_a = 0;
uint16_t count_enc_b = 0;

TMotCtrl motor;

/* ******************************************************************* */
/* *** Protótipos das funções **************************************** */
//
// Obs: Este bloco não é necessário para compilação mas é útil como
//      referência durante o processo de desenvolvimento.

void     tasks_10ms       ( void );
void     tasks_100ms      ( void );
void     tasks_1000ms     ( void );
void     tasks_1min       ( void );
void     blinka           ( void );
uint8_t  get_node_addr    ( void );
uint16_t get_volt_bat     ( void );
void     encoderA         ( void );
void     encoderB         ( void );
void     set_motor_status ( uint32_t );
uint32_t get_motor_status ( void );
bool     is_motor_locked  ( uint8_t );
uint8_t  set_pwm_max      ( void );

/* ******************************************************************* */
/* *** SETUP ********************************************************* */

void setup() {

    Serial.begin(115200);               // Inicialização da com. serial

    // Inicialização do pino do LED
    pinMode(LED, OUTPUT);               // Pino do LED como saída digital
    digitalWrite(LED, LOW);
    
    analogReference(INTERNAL);          // Referência dos ADCs -> 1.1V

    pinMode(RADIO_A0, INPUT_PULLUP);    
    pinMode(RADIO_A1, INPUT_PULLUP);

    pinMode(IRQ_ENC_A, INPUT);
    pinMode(IRQ_ENC_B, INPUT);

    pinMode(HBRID_EN, OUTPUT);         // Habilita a ponte H (High)
    pinMode(MTR_AIN1, OUTPUT);         // Bit 0 - Controle da ponte H do Motor A
    pinMode(MTR_AIN2, OUTPUT);         // Bit 1 - Controle da ponte H do Motor A
    pinMode(MTR_BIN1, OUTPUT);         // Bit 0 - Controle da ponte H do Motor B
    pinMode(MTR_BIN2, OUTPUT);         // Bit 1 - Controle da ponte H do Motor B
    pinMode(MTR_PWMA, OUTPUT);         // Sinal de PWM para controle  do Motor A
    pinMode(MTR_PWMB, OUTPUT);         // Sinal de PWM para controle  do Motor B

    attachInterrupt(0, encoderA, RISING);  
    attachInterrupt(1, encoderB, RISING);  

    
    SPI.begin();                        // Inicializa a interface SPI

}


/* ******************************************************************* */
/* *** LOOP PRINCIPAL ************************************************ */

void loop() {

}


/* ******************************************************************* */
/* ******* TASKS ***************************************************** */

//Tarefas que devem ser executadas em intervalos de 10ms
void tasks_10ms( void ) {

     if( (millis() - tasks.last_10ms) > 1 ){
        tasks.last_10ms = millis();

    }
}

//Tarefas que devem ser executadas em intervalos de 100ms
void tasks_100ms( void ) {

    if( (millis() - tasks.last_100ms) > 100 ){
        tasks.last_100ms = millis();       
    }
}

//Tarefas que devem ser executadas em intervalos de 1000ms
void tasks_1000ms( void ) {

  if( (millis() - tasks.last_1000ms) > 1000 ){
    tasks.last_1000ms = millis();
    }
    
}

void tasks_1min ( void ) {

  if (millis() = tasks.last_1min) > 60000 ) {
    tasks.last_1min = millis();
  }
  
}

/* ****************************************************************** */
/* Acha o enderecamento do radio ************************************ */

uint8_t get_node_addr( void ){
   //Modo tosco que foi o primeiro que eu pensei.
   if( (RADIO_A0 == HIGH) && (RADIO_A1 == HIGH) )
    return 0;
   if( (RADIO_A0 == HIGH) && (RADIO_A1 == HIGH) )
    return 1;
   if( (RADIO_A0 == HIGH) && (RADIO_A1 == HIGH) )
    return 2;
   if( (RADIO_A0 == HIGH) && (RADIO_A1 == HIGH) )
    return 3;
   
   //Modo elegante usando bits.
   uint8_t address = 0xFF;
   address = address << 1;
   address = address | digitalRead(RADIO_A0);
   address = address << 1;
   address |= digitalRead(RADIO_A1);
   return ~address;

}

/* ****************************************************************** */
/* Pisca a luz de TimeBlink em Timeblink  *************************** */

void blinka (void) {
  uint32_t blinker;
  uint32_t aux32b;
  if ((millis() - aux32b) > TimeBlink){
    aux32b = millis();
    // Escreve "bit 0" de "blinker" para o LED 
    digitalWrite(LED, bitRead(blinker, 0));
        
    // Serial.write(blinker);  // Escreve dados binários na porta serial
    // Serial.print(blinker);  // Imprime dados na porta serial como texto ASCII
    //aux = blinker%1000;
    Serial.println(blinker);   // Idem ao "print", adicionando EOL ao string  
    blinker++;
  }

}

/* ****************************************************************** */
/* Le a tensao da bateria ******************************************* */

//Retorna o valor em millivolts da bateria.
uint16_t get_volt_bat ( void ){
  //Solução inicial.
  int16_t sensorValue = analogRead(VOLT_BAT);
  uint16_t volt = ((sensorValue*1.1/1023.0)*10000.0);
  return volt;

  //Outra solução.
//  uint8_t  n   = 10;
//  uint32_t adc =  0;
//
//  for (int i = 0; i < n; i++){
//      adc += analogRead(VOLT_BAT);
//      delayMicroseconds(100);
//  }
//  
//  uint16_t volt = ((adc*1.1/1023.0)*1000.0);
//  
//  return volt;
}

/* ****************************************************************** */
/* ENCODERS  ******************************************************** */

//Imprime e conta.
void encoderA() {
  Serial.print ("Contagem Encoder A: ");
  Serial.println(count_enc_a++);
}


void encoderB() {
  Serial.print ("Contagem Encoder B: ");
  Serial.println(count_enc_b++);
}


/* ****************************************************************** */
/* Ponte H  ********************************************************* */
//Seta o status do motor, no caso ajusta a ponte H, pwm...
void set_motor_status( uint32_t state) {
    
    motor.status = state;
    
    // Desabilita a ponte H.
    digitalWrite(HBRID_EN, LOW);
    
    //Ajustas os pinos da ponte H conforme o que foi passado no int state para a struct motor.
    //O bit mais sgnificativo é o 1 e o menos o 2.
    digitalWrite(MTR_AIN1, bitRead(motor.config.dir_motor_A, 1));
    digitalWrite(MTR_AIN2, bitRead(motor.config.dir_motor_A, 0));
    digitalWrite(MTR_BIN1, bitRead(motor.config.dir_motor_B, 1));
    digitalWrite(MTR_BIN2, bitRead(motor.config.dir_motor_B, 0));

    //Verifica se pwm_motor esta na faixa PWM_MIN <= PWM_MOTOR <= PWM_MAX, e, caso nao esteja o matem nessa faixa.
    if(motor.config.pwm_motor_A < PWM_MIN)
      motor.config.pwm_motor_A = PWM_MIN;
    else if (motor.config.pwm_motor_A > PWM_MAX)
      motor.config.pwm_motor_A = PWM_MAX;
      
    if(motor.config.pwm_motor_B < PWM_MIN)
      motor.config.pwm_motor_B = PWM_MIN;
    else if (motor.config.pwm_motor_B > PWM_MAX)
      motor.config.pwm_motor_B = PWM_MAX;
    
    //Ajusta o PWM de cada motor, conforme foi passado no int state e depois para a struct motor. 
    analogWrite(MTR_PWMA, motor.config.pwm_motor_A); 
    analogWrite(MTR_PWMB, motor.config.pwm_motor_B);

    // Habilita a ponte H novamente.
    digitalWrite(HBRID_EN, HIGH);    
}

//Retorna qual o status do motor (de forma "unica").
uint32_t get_motor_status( void ){
    return motor.status;  
}

//Verifica se está em freio elétrico.
bool is_motor_locked( uint8_t ident){

  //Shifta a variavei para que os ultimos 4 bits que são inutilizaveis sejam obrigatoria mente zeros.
  ident <<= 4;
  
  //Cria duas mascaras que checam se o robo está indo para frente ou para trás(não importa a ordem).
  uint8_t mask1 = 00000101;
  uint8_t mask2 = 00001010;

  //Caso ele seja igual a uma das mascaras ele está andando, caso contrario está freiado.
  if ((ident == mask1) || (ident == mask2))
    return 0;
  return 1;
}


 uint8_t set_pwm_max( void ){
  
 }
