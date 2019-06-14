//NUMERO MAGICO: 2678018048.
#include <avr/pgmspace.h>
#include <RF24Network.h>
#include <RF24.h>
#include <SPI.h>

// ------------------------------------------------------------------- //
// ------------------------ Definições diversas ---------------------- //

#define SSPEED      115200 // Velocidade da interface serial
#define FULL_BAT    8000   // Valor em mV para bat. completamente carregada
#define DEAD_BAT    6000   // Valor em mV para bat. esgotada ( recarregar )
#define TimeBlink   2000   //Valor em Millisegundos em que o LED deve piscar.

// ------------------------------------------------------------------- //
// ----------------------- Definições das rodas ---------------------- //

#define WHEEL_TICKS 48     // Número de furos por roda
#define WHEEL_DIAM  60     // Diâmetro da cada roda, em mm
#define WHEELS_SPC  60     // Espaçamento entre rodas, em mm

// ------------------------------------------------------------------- //
// --- Limitações de PWM e velocidade para uso no controlador PID ---- //

#define PWM_MIN     0x0F    // PWM mín. p/ garantir movimento das duas rodas
#define PWM_MAX     0x9F    // PWM máx. para que os motores tenham aprox. 5V
#define SPD_MIN     50      // Vel. mín. em mm/s ( Condição: PWM > PWM_MIN )
#define SPD_MAX     500     // Vel. máx. em mm/s ( Condição: PWM < PWM_MAX )

// ------------------------------------------------------------------- //
// ---------------------- Definições para o radio -------------------- //

#define BASE_ADDRESS   00   // Base tem o endereço 0 (em octal)

#define TAM_BUFFER     16   // Buffer de SW para o rádio
#define BASE_ADDRESS   00   // Base tem o endereço 0 (em octal)
#define NETW_CHANNEL  100   // Canal padrão de operação do rádio

// ------------------------------------------------------------------- //
// ---------------- DEFINES DE PINOS DO ARDUINO ---------------------- //

#define LED         4       // Led conectado a uma saída digital

#define RADIO_CE    7       // Pino CE do módulo de rádio
#define RADIO_CS    8       // Pino CS do módulo do rádio
#define RADIO_A0    A4      // Bit 0 do end. do rádio (LOW = ligado)
#define RADIO_A1    A5      // Bit 1 do end. do rádio (LOW = ligado)

#define IRQ_ENC_A   2       // Pino de interrupção do Encoder A
#define IRQ_ENC_B   3       // Pino de interrupção do Encoder B
#define IRQ_RADIO   5       // Pino de interrupção do Rádio

#define HBRID_EN    6       // Habilita a ponte H (High)
#define MTR_AIN1    A2      // Bit 0 - Controle da ponte H do Motor A
#define MTR_AIN2    A3      // Bit 1 - Controle da ponte H do Motor A
#define MTR_BIN1    A1      // Bit 0 - Controle da ponte H do Motor B
#define MTR_BIN2    A0      // Bit 1 - Controle da ponte H do Motor B
#define MTR_PWMA    9       // Sinal de PWM para controle  do Motor A
#define MTR_PWMB    10      // Sinal de PWM para controle  do Motor B

#define VOLT_BAT    A7      // Tensão da bateria -> Vcc/10

//Macros
// Transforma caractere ascii em um número de 4 bits em hexadecimal.
#define asc2hex(a) (((a) < 'a') ? ( (a) - '0' )  : ( ((a) - 'a')+10) )

// ------------------------------------------------------------------- //
// ---------------------- ESTRUTURAS DE DADOS ------------------------ //

//Struct dos tipos de mensagens.
typedef struct {
  uint32_t last_10ms   = 0;      // Controle das tarefas executadas a cada 10ms
  uint32_t last_100ms  = 0;      // Controle das tarefas executadas a cada 100ms
  uint32_t last_1min   = 0;      // Controle das tarefas executadas a cada 1min
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
  uint32_t status = 0;           // Leitura/Escrita simuntânea do conjunto de variáveis.
} TMotCtrl;

typedef union {
    struct {
        uint8_t data1;
        uint8_t data2; 
    };
    int16_t data;
} Data;

typedef union  {
    struct {
        uint8_t id  :4,
                pad :4;
        char    chr;
        Data    data;
    } conf;
    uint32_t  stats = 0;
} TRadioMsg;

// ------------------------------------------------------------------- //
//------------------------- VARIAVEIS GLOBAIS ------------------------ //

//Calcula uma constante de quando o robo anda por furo na roda.
const uint32_t dist_ticks = 3.14*WHEEL_DIAM/WHEEL_TICKS;
const byte     canais[2]  = {0x00, 0xFF}; 

TasksTCtr tasks;                           //Contagem de tempo para execução de tarefas.
uint16_t count_enc_a = 0;
uint16_t count_enc_b = 0;

TMotCtrl motor;                            //Variavel global que contra o motor.

//Parte da inicializacao do radio.
RF24        radio   (RADIO_CE, RADIO_CS);  // Instância do rádio.
RF24Network network (radio);               // Instância da rede.

bool is_rotating;						               // Define se o robô está ou não girando.

uint8_t netw_channel;                      // Canal de rádio da rede

uint16_t  base_node, this_node;            // End. do nó remoto (base) e deste nó

uint8_t  recv;
uint32_t buffer;
uint32_t message;

TRadioMsg msg;                             // Mensagem que está sendo executada.
TRadioMsg oldMsg;                          // Ultima mensagem executada.

// ------------------------------------------------------------------- //
// ---------------------- SUMARIO DAS FUNCOES ------------------------ //

// Obs: Este bloco não é necessário para compilação mas é útil como
//      referência durante o processo de desenvolvimento.

void     blinka                 ( void );

void     tasks_10ms             ( void );
void     tasks_100ms            ( void );
void     tasks_1min             ( void );

void     encoderA               ( void );
void     encoderB               ( void );

void     task_radio_Rx          ( void );
void     task_radio_Tx          ( void );

void 	   messageM               ( void );
void 	   messageR               ( void );
void 	   messageS               ( void );
void 	   messageP               ( void );

void     set_rotation           ( int16_t );
void     set_motor_status       ( uint32_t );
void     set_speed              ( uint8_t, uint16_t );

bool     is_motor_locked        ( uint8_t );

uint8_t  get_node_addr          ( void );
uint8_t  set_pwm_max            ( void );

uint16_t get_volt_bat           ( void );
uint32_t get_motor_status       ( void );


// ------------------------------------------------------------------- //
// ---------------------------- MAIN --------------------------------- //

void setup() {

  Serial.begin(115200);              // Inicialização da com. serial

  base_node = BASE_ADDRESS;          // Endereço do nó base

  // Inicialização do pino do LED
  pinMode(LED, OUTPUT);              // Pino do LED como saída digital
  digitalWrite(LED, LOW);
  
  analogReference(INTERNAL);         // Referência dos ADCs -> 1.1V

  pinMode(RADIO_A0, INPUT_PULLUP);   //Inicialização do radio.
  pinMode(RADIO_A1, INPUT_PULLUP);

  pinMode(IRQ_ENC_A, INPUT);         //Pino para interrupção no encoder A.
  pinMode(IRQ_ENC_B, INPUT);         //Pino para interrupção no encoder B.
  pinMode(IRQ_RADIO, INPUT);         //Pino para interrupção para o radio.

  pinMode(HBRID_EN, OUTPUT);         // Habilita a ponte H (High)
  pinMode(MTR_AIN1, OUTPUT);         // Bit 0 - Controle da ponte H do Motor A
  pinMode(MTR_AIN2, OUTPUT);         // Bit 1 - Controle da ponte H do Motor A
  pinMode(MTR_BIN1, OUTPUT);         // Bit 0 - Controle da ponte H do Motor B
  pinMode(MTR_BIN2, OUTPUT);         // Bit 1 - Controle da ponte H do Motor B
  pinMode(MTR_PWMA, OUTPUT);         // Sinal de PWM para controle  do Motor A
  pinMode(MTR_PWMB, OUTPUT);         // Sinal de PWM para controle  do Motor B

  attachInterrupt(0, encoderA, RISING);  
  attachInterrupt(1, encoderB, RISING);  

  SPI.begin();                        // Inicializa a interface SPI.

  // Inicializaçoes do radio.
  radio.begin();                      // Inicializa o modulo de radio.
  netw_channel = NETW_CHANNEL;
  radio.setChannel(netw_channel);     // Canal da rede de rádio ( 0 - 125 );
  radio.setPALevel(RF24_PA_MAX);      // Potência da transmissão em 0dB ( 1mW )
  radio.setCRCLength(RF24_CRC_16);    // Comprimento do CRC: 8 ou 16 bits
  radio.enableDynamicPayloads();      // Habilita mensagens de tamanho dinâmico
  radio.setRetries(4,10);             // Reenvios (em HW): 4 * 250us = 1ms ; count: 10
  radio.setAutoAck(true);             // Autoack habilitado (feito em HW)
  radio.maskIRQ(1,1,0);               // Interrupção somente quando recebe pacotes

  // Inicialização da rede
  network.begin(this_node);

  is_rotating = 0;
  count_enc_a = 0;
  count_enc_b = 0;

  motor.status = 2678018048;

  set_motor_status(2678018048);

  // inicialização de rádio com RF24
  radio.begin();
  radio.setPALevel(RF24_PA_MAX);
  radio.setDataRate(RF24_2MBPS);
  radio.setChannel(netw_channel);
  radio.openWritingPipe(canais[0]);
  radio.openReadingPipe(1, canais[1]);
  radio.startListening();
}

//Main loop, o que o robo fica fazendo "pra sempre".
void loop() {
	tasks_10ms();
	tasks_100ms();
	tasks_1min();
}


// ------------------------------------------------------------------- //
// ------------------------------ TASKS ------------------------------ //

//Ainda nao defini como sera cada task, mas acho que essa forma ainda seja a mais eficiente.

// Tarefas que devem ser executadas em intervalos de 10ms 
void tasks_10ms( void ) {

    if( (millis() - tasks.last_10ms) > 1 ){
      tasks.last_10ms = millis();

    }
}

// Tarefas que devem ser executadas em intervalos de 100ms
void tasks_100ms( void ) {

    if( (millis() - tasks.last_100ms) > 100 ){
        tasks.last_100ms = millis();

      radio.startListening();
        if (radio.available())  {
          radio.read(&buffer, sizeof(uint32_t));
          set_motor_status(buffer);
        }
    }

    get_speed();
}

// Tarefas que devem ser executadas em intervalos de 1m
void tasks_1min( void ) {

  if( (millis() - tasks.last_1min) > 60000 ){
    tasks.last_1min = millis();

    // Caso a bateria esteja fraca, desliga o robo.
    if (get_volt_bat() < DEAD_BAT){
    	digitalWrite(MTR_AIN1, bitRead(motor.config.dir_motor_A, 0));
  		digitalWrite(MTR_AIN2, bitRead(motor.config.dir_motor_A, 0));
  		digitalWrite(MTR_BIN1, bitRead(motor.config.dir_motor_B, 0));
 		digitalWrite(MTR_BIN2, bitRead(motor.config.dir_motor_B, 0));
    }

  }
}

// ------------------------------------------------------------------- //
// ------------------------------ LED -------------------------------- //

void blinka (void) {
  uint32_t blinker;
  uint32_t aux32b;
  if ((millis() - aux32b) > TimeBlink){
    aux32b = millis();
    // Escreve "bit 0" de "blinker" para o LED 
    digitalWrite(LED, bitRead(blinker, 0));
        
    Serial.println(blinker);   // Idem ao "print", adicionando EOL ao string  
    blinker++;
  }

}

// ------------------------------------------------------------------- //
// ------------------------------ BATERIA ---------------------------- // 

//Retorna a tensao na bateria, transforma analogico em digital e retorna 
//um valor analogico, novamente.
uint16_t get_volt_bat ( void ){
  int16_t sensorValue = analogRead(VOLT_BAT);
  uint16_t volt = ((sensorValue*1.1/1023.0)*10000.0);
 // Serial.print("voltagem: ");
  //Serial.println(volt);
  return volt;
}

// ------------------------------------------------------------------- //
// ------------------------------ ENCODERS  -------------------------- //

//Tentei manter as mensagens de interrupção dos encoders o mais simples possivel.
//Caso seja necessario testar o funcionamente via serial basta:
//descomentar a primeira linha e substituir a segunda por Serial.print(count_enc_X++)

void encoderA() {
  //Serial.print ("Contagem Encoder A: ");
  //Serial.println(count_enc_a++);
  count_enc_a++;
}


void encoderB() {
  //Serial.print ("Contagem Encoder B: ");
  //Serial.println(count_enc_b++);
  count_enc_b++;
}
// ------------------------------------------------------------------- //
// ------------------------------ Ponte H ---------------------------- //  

//Parte da ponte H e do ajuste dos PWM estao todas feitas.
//Talvez de para mudar a forma como ele checa o freio eletrico (verificar a corretude, tambem).

//Seta o status do motor, no caso ajusta a ponte H, pwm...
void set_motor_status( uint32_t state) {
    
  motor.status = state;
  
  // Desabilita a ponte H.
  digitalWrite(HBRID_EN, LOW);
  
  //Ajustas os pinos da ponte H conforme o que foi passado no int state para a struct motor.
  //O bit mais sgnificativo é o 1 e o menos o 2.
  digitalWrite(MTR_AIN1, bitRead(motor.config.dir_motor_A, 1));
  digitalWrite(MTR_AIN2, bitRead(motor.config.dir_motor_A, 0));
  digitalWrite(MTR_BIN1, bitRead(motor.config.dir_motor_B, 0));
  digitalWrite(MTR_BIN2, bitRead(motor.config.dir_motor_B, 1));

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
bool is_motor_locked( uint8_t mtr){
    
    if( mtr ) 
        return !(bitRead(motor.config.dir_motor_B,0) ^ bitRead(motor.config.dir_motor_B,1));
    else 
    	return !(bitRead(motor.config.dir_motor_A,0) ^ bitRead(motor.config.dir_motor_A,1));

}

uint8_t set_pwm_max( void ){
  uint8_t pwm;
  uint16_t tension = get_volt_bat();
  
  //O PWM tem que ser max caso ele esteja em 5v ou menos, sendo assim:
  if (tension < PWM_MAX)
    return PWM_MAX;

  //Com o PWM entre 5v e 8v, devemos achar a "porcentagem" em que ele deve
  //ficar ligado, dado um periodo.
  pwm = PWM_MAX*16000/tension;

  return pwm;
 }

// ------------------------------------------------------------------- //
// ---------------------------- MOTORES ------------------------------ //

//Nao consegui terminar devido a falta de tempo e de entendimento do enunciado proposto
//O Alexandre usa uma struct de PID (proportional, integral, and derivative), mas eu acho
//Que nao e necessario fazer assim, verificar mais tarde.

//Seta as velocidades dos motores.
void set_speed ( uint8_t speed1, uint16_t speed2) {

	if (speed1 == 0) {
		uint16_t newSpeed = ((speed2*10) * PWM_MAX) / SPD_MAX;

		motor.config.pwm_motor_A = (uint8_t) newSpeed;
		motor.config.pwm_motor_B = (uint8_t) newSpeed;

		set_motor_status(motor);
	
	} else {	
	
		uint8_t newSpeed = ((speed1*10) * PWM_MAX) / SPD_MAX;

		motor.config.pwm_motor_A = newSpeed;
		motor.config.pwm_motor_B = newSpeed;

		set_motor_status(motor);
	}

}

//rotaciona o robo.
void set_rotation ( int16_t rotation ) {

  uint16_t speed = 0 ;
  uint16_t deg   = (rotation & 0x3FF)%360;
  uint16_t tks   = (deg * 10) / 75;          // Furos do motor que serão contados (7,5° por furo)

  // Ajusta ângulo para múltiplo de 7,5°
  deg = (tks * 75) / 10;

  //Gira para a esquerda.
  if (rotation > 0){
    is_rotating = 1;
  //set_speed();
  //Gira para a direita.
  } else if (rotation < 0) {
    is_rotating = 1;
    //set_speed();
 
  //Nao gira.
  } else 
    is_rotating = 0;

}

// ------------------------------------------------------------------- //
// ----------------------------- RADIO ------------------------------- //

//Estou usando o Radio do Alexandre, mas estou comentando o funcionamento deve
//Para que fique mais facil o entendimento, sera possivel fazer as devidas mudancas
//Mais para frente.

//Acha e devolve de forma "fisica" o ID do robo que sera utilizado pelo radio.
uint8_t get_node_addr( void ){
   if( (digitalRead(RADIO_A0) == HIGH) && (digitalRead(RADIO_A1) == HIGH) )
    return 0;
   if( (digitalRead(RADIO_A0) == HIGH) && (digitalRead(RADIO_A1) == LOW) )
    return 1;
   if( (digitalRead(RADIO_A0) == LOW) && (digitalRead(RADIO_A1) == HIGH) )
    return 2;
   if( (digitalRead(RADIO_A0) == LOW) && (digitalRead(RADIO_A1) == LOW) )
    return 3;
}

// ------------------------------------------------------------------- //
// ------------------------------ AUX -------------------------------- //

//Função auxiliar para chegar se andou determinada distancia.
bool notDist (uint32_t A, uint32_t B) {

	if ( (A - encoderA < 0 ) && (B - encoderB < 0))
		return 1;
	return 0; 
}

// ------------------------------------------------------------------- //
// ---------------------------- MENSAGENS ---------------------------- //


void messageM (uint8_t pad, uint8_t dist, uint8_t speed) {

	// Caso o pad seja para a "velocidade negativa", inverte a ponte H.
	if (pad == '-') {
		  digitalWrite(MTR_AIN1, !bitRead(motor.config.dir_motor_A, 1));
  		digitalWrite(MTR_AIN2, !bitRead(motor.config.dir_motor_A, 0));
  		digitalWrite(MTR_BIN1, !bitRead(motor.config.dir_motor_B, 0));
  		digitalWrite(MTR_BIN2, !bitRead(motor.config.dir_motor_B, 1));
	}

	// Variaveis para ajustar a distancia.
	uint32_t x = ((uint32_t)dist/dist_ticks);
	uint32_t auxDistA = encoderA + x;
	uint32_t auxDistB = encoderB + x;

	// A velocidade que ele ira se mover.

	set_speed(speed, 0);

	// Enquanto não tiver terminado de andar aquela distancia, continua no estado atual.
	while (notDist(auxDistA, auxDistB))

	// Coloca a mensagem que acabou de executar como se fosse a "ultima".
	oldMsg = msg;
}

void messageS (uint8_t pad, uint16_t speed) {

	// Caso o pad seja para a "velocidade negativa", inverte a ponte H.
	if (pad == '-') {
		digitalWrite(MTR_AIN1, !bitRead(motor.config.dir_motor_A, 1));
  		digitalWrite(MTR_AIN2, !bitRead(motor.config.dir_motor_A, 0));
  		digitalWrite(MTR_BIN1, !bitRead(motor.config.dir_motor_B, 0));
  		digitalWrite(MTR_BIN2, !bitRead(motor.config.dir_motor_B, 1));
	}

	set_speed(0, speed);

	oldMsg = msg;
}

void messageR (uint16_t graus) {
	set_rotation(graus);
	oldMsg = msg;
}

void messageP () {
	
	while(!is_motor_locked()){
		digitalWrite(HBRID_EN, LOW);

		digitalWrite(MTR_AIN1, bitRead(motor.config.dir_motor_A, 0));
		digitalWrite(MTR_AIN2, bitRead(motor.config.dir_motor_A, 0));
		digitalWrite(MTR_BIN1, bitRead(motor.config.dir_motor_B, 0));
		digitalWrite(MTR_BIN2, bitRead(motor.config.dir_motor_B, 0));

		digitalWrite(HBRID_EN, HIGH);
	}

	oldMsg = msg;

}

void read_message (TRadioMsg message) {

	uint8_t ID = message.id;
	uint8_t pad;
	switch(ID) {
		case 'M':
			pad  = message.pad;
			messageM(pad, message.data.data1,  message.data.data2);
		break;

		case 'S':
			pad = message.pad;
			messageS(pad, massage.data);
		break;

		case 'R':
			messageR(message.data);
		break;

		case 'P':
			messageP();
		break;

		default:
			read_message(oldMsg);
		break;
	}
}
