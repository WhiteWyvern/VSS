
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
    
} TasksTCtr;

/* ******************************************************************* */
/* *** Variáveis globais e instanciações ***************************** */


TasksTCtr tasks;		// Contagem de tempo para execução de tarefas


/* ******************************************************************* */
/* *** Protótipos das funções **************************************** */
//
// Obs: Este bloco não é necessário para compilação mas é útil como
//      referência durante o processo de desenvolvimento.

void     tasks_10ms   ( void );
void     tasks_100ms  ( void );
void     tasks_1000ms ( void );
void     blinka       ( void );
uint8_t  get_node_addr( void );
uint16_t get_volt_bat ( void );

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

    //Serial.println(get_node_addr());

    

    SPI.begin();                        // Inicializa a interface SPI

}


/* ******************************************************************* */
/* *** LOOP PRINCIPAL ************************************************ */

void loop() {
    
    //tasks_10ms();                // Tarefas executadas a cada 10ms
    //tasks_100ms();               // Tarefas executadas a cada 100ms
    tasks_1000ms();              // Tarefas executadas a cada 1000ms
    //blinka();

}


/* ******************************************************************* */
/* *** FUNÇÕES (implementações) ************************************** */

/* *********************************************************************
 * Tarefas que devem ser executadas em intervalos de 10ms
 */
void tasks_10ms( void ) {

     if( (millis() - tasks.last_10ms) > 10 ){
        tasks.last_10ms = millis();


    }
}

/* *********************************************************************
 * Tarefas que devem ser executadas em intervalos de 100ms
 */
void tasks_100ms( void ) {

    if( (millis() - tasks.last_100ms) > 100 ){
        tasks.last_100ms = millis();
               
    }
}

/* *********************************************************************
 * Tarefas que devem ser executadas em intervalos de 1000ms
 */
void tasks_1000ms( void ) {

  if( (millis() - tasks.last_1000ms) > 1000 ){
    tasks.last_1000ms = millis();

     Serial.println(get_node_addr());
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
   address |= digitalRead(RADIO_A0);
   address = address << 1;
   address |= digitalRead(RADIO_A1);
   return ~address;

}

/* ****************************************************************** */
/* Blinker ********************* ************************************ */

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
