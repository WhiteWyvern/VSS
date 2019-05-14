  
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

#define BASE_ADDRESS      00   // Base tem o endereço 0 (em octal)

#define TAM_BUFFER        16   // Buffer de SW para o rádio
#define BASE_ADDRESS      00   // Base tem o endereço 0 (em octal)
#define NETW_CHANNEL     100   // Canal padrão de operação do rádio

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

typedef struct {
    uint8_t  type;              // 1 byte  - Tipo da mensagem
    uint32_t data;              // 4 bytes - Dado da mensagem
} TRadioMsg;

typedef struct {
    uint8_t   head;              // Índice do primeiro elemento da fila
    uint8_t   tail;              // Índice do último elemento da fila
    uint8_t   tam;               // Número de mensagens na fila
    uint32_t  last_t;            // Última transmissão ou recepção
    TRadioMsg msg[TAM_BUFFER];   // Fila de mensagens
} TRadioBuf;

// ------------------------------------------------------------------- //
//------------------------- VARIAVEIS GLOBAIS ------------------------ //

TasksTCtr tasks;		                       //Contagem de tempo para execução de tarefas.
uint16_t count_enc_a = 0;
uint16_t count_enc_b = 0;

TMotCtrl motor;                            //Variavel global que contra o motor.

//Parte da inicializacao do radio.
RF24        radio   (RADIO_CE, RADIO_CS);  // Instância do rádio.
RF24Network network (radio);               // Instância da rede.
TRadioBuf   rx_buffer, tx_buffer;          // Buffers para mensagens (Rx & Tx). 

bool rotation_FLAG = 0;


// ------------------------------------------------------------------- //
// ---------------------- SUMARIO DAS FUNCOES ------------------------ //

// Obs: Este bloco não é necessário para compilação mas é útil como
//      referência durante o processo de desenvolvimento.

void     tasks_10ms             ( void );
void     tasks_100ms            ( void );
void     tasks_1min             ( void );
void     blinka                 ( void );
void     encoderA               ( void );
void     encoderB               ( void );
void     task_radio_Rx          ( void );
void     task_radio_Tx          ( void );
void     set_rotation           ( int16_t );
void     set_motor_status       ( uint32_t );
void     set_speed              ( uint32_t );
void     flush_radio_buffer     ( TRadioBuf& );
bool     is_rotating            ( void ); 
bool     is_motor_locked        ( uint8_t );
bool     is_radio_buffer_full   ( TRadioBuf& );
bool     is_radio_buffer_empty  ( TRadioBuf& );
uint8_t  get_node_addr          ( void );
uint8_t  set_pwm_max            ( void );
int8_t   write_msg_radio_buffer ( TRadioBuf&, TRadioMsg& );
int8_t   read_msg_radio_buffer  ( TRadioBuf&, TRadioMsg& );
uint16_t get_volt_bat           ( void );
uint32_t get_motor_status       ( void );
uint32_t get_speed              ( void );


// ------------------------------------------------------------------- //
// ---------------------------- MAIN --------------------------------- //

void setup() {

  Serial.begin(115200);             // Inicialização da com. serial

  base_node = BASE_ADDRESS;           // Endereço do nó base

  // Inicialização do pino do LED
  pinMode(LED, OUTPUT);             // Pino do LED como saída digital
  digitalWrite(LED, LOW);
  
  analogReference(INTERNAL);        // Referência dos ADCs -> 1.1V

  pinMode(RADIO_A0, INPUT_PULLUP);  //Inicialização do radio.
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

  // Inicialização dos buffers
  flush_radio_buffer( &rx_buffer );
  flush_radio_buffer( &tx_buffer );

}

//Main loop, o que o robo fica fazendo "pra sempre".
void loop() {

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
    }
}

// Tarefas que devem ser executadas em intervalos de 1000ms
void tasks_1min( void ) {

  if( (millis() - tasks.last_1min) > 60000 ){
    tasks.last_1min = millis();

    // Caso a bateria esteja fraca, desliga o robo.
    if (get_volt_bat() < 6000){

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
        
    //Serial.write(blinker);   // Escreve dados binários na porta serial
    //Serial.print(blinker);   // Imprime dados na porta serial como texto ASCII
    //aux = blinker%1000;
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
  return volt;
}

// ------------------------------------------------------------------- //
// ------------------------------ ENCODERS  -------------------------- //

//Tentei manter as mensagens de interrupção dos encoders o mais simples possivel.
//Caso seja necessario testar o funcionamente via serial basta:
//descomentar a primeira linha e substituir a segunda por Serial.print(count_enc_X++)

void encoderA() {
  //Serial.print ("Contagem Encoder A: ");
  count_enc_a++;
}


void encoderB() {
  //Serial.print ("Contagem Encoder B: ");
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

  ident = ident >> 4;
  
  uint8_t mask1 = 00000101;
  uint8_t mask2 = 00001010;

  if ((ident == mask1) || (ident == mask2))
    return 0;
  return 1;
}

uint8_t set_pwm_max( void ){
  uint8_t pwm;
  uint16_t tension = uint16_t get_volt_bat();
  
  //O PWM tem que ser max caso ele esteja em 5v ou menos, sendo assim:
  if (tension < PWM_MAX)
    return PWM_MAX;

  //Com o PWM entre 5v e 8v, devemos achar a "porcentagem" em que ele deve
  //ficar ligado, dado um periodo.
  pwm = PWM_MAX/tension;

  return pwm;
 }

// ------------------------------------------------------------------- //
// ---------------------------- MOTORES ------------------------------ //

//Nao consegui terminar devido a falta de tempo e de entendimento do enunciado proposto
//O Alexandre usa uma struct de PID (proportional, integral, and derivative), mas eu acho
//Que nao e necessario fazer assim, verificar mais tarde.

//Seta as velocidades dos motores.
void set_speed ( uint32_t data ) {

}

//Devolve o valor da velocidade atual.
uint32_t get_speed ( void ) {

  uint16_t speedA;
  uint16_t speedB;
  uint32_t speed  = 0;
  uint16_t timeMM = 0;

  //Calcula uma constante de quando o robo anda por furo na roda.
  uint32_t dist_ticks = 3.14*WHEEL_DIAM/WHEEL_TICKS;

  //Faz a verificacao e os ajustes para acertar a velocidade.
  if (!is_motor_locked()){
    //verificar se em dado tempo encoder passa por speedA e speedB furos.
    speedA = count_enc_a;
    speedB = count_enc_b;
	  
    //Pega um tempo para calcular a velocidade do robo, nao e a forma mais eficiente,
    //pois o tempo e insuficiente
    timeMM = millis();	  
    delay(100);
    timeMM = millis() - time;
	  
    //Calcula a velocidade em cada motor.
    speedA = ((count_enc_a - speed)*dist_ticks)/timeMM;
    speedB = ((count_enc_b - speed)*dist_ticks)/timeMM;


    //Coloca as velocidades dos dois motores em uma mesma variavel
    //0-15 bits: B e 16-31 bits: A 
    speed = speedA;
    speed = speed << 16;
    speed += speedB;

  } else
  	//Caso o motor eseteja travado, a velocidade e zero.
  	return speed;
}

//rotaciona o robo.
void set_rotation ( int16_t rotation ) {

  uint32_t speed = 0 ;
  uint16_t deg   = (uint16_t)(rotation & 0x3FF)%360;
  uint8_t tks    = (uint8_t) ((deg * 10) / 75);          // Furos do motor que serão contados (7,5° por furo)

  // Ajusta ângulo para múltiplo de 7,5°
  deg = (uint16_t)((tks * 75) / 10);

  //Gira para a esquerda.
  if (rotation > 0){
    rotation_FLAG = 1;
 	set_speed();
  //Gira para a direita.
  } else if (rotation < 0) {
    rotation_FLAG = 1
    set_speed();
 
  //Nao gira.
  } else 
    rotation_FLAG = 0;

  if (rotation_FLAG)
    set_speed(speed);
}

bool is_rotating( void ) {
    return rotation_FLAG;
}

// ------------------------------------------------------------------- //
// ----------------------------- RADIO ------------------------------- //

//Estou usando o Radio do Alexandre, mas estou comentando o funcionamento deve
//Para que fique mais facil o entendimento, sera possivel fazer as devidas mudancas
//Mais para frente.

//Acha e devolve de forma "fisica" o ID do robo que sera utilizado pelo radio.
uint8_t get_node_addr( void ){
   if( (RADIO_A0 == HIGH) && (RADIO_A1 == HIGH) )
    return 0;
   if( (RADIO_A0 == HIGH) && (RADIO_A1 == HIGH) )
    return 1;
   if( (RADIO_A0 == HIGH) && (RADIO_A1 == HIGH) )
    return 2;
   if( (RADIO_A0 == HIGH) && (RADIO_A1 == HIGH) )
    return 3;
}

//Le mensagem do buffer do hardware e as armazena em um buffer (rx_buffer).
void task_radio_Rx( void ) {

  if( !network.available() ) 
    return;
  
  TRadioMsg msg;
  RF24NetworkHeader header;
  
  while( network.available() && !is_radio_buffer_full(&rx_buffer) ) {
    network.read(header, &msg.data, sizeof(msg.data));
    msg.type  = header.type;
    write_msg_radio_buffer(&rx_buffer, &msg);
  }
}

//Encaminha as mensagens que foram postas no buffer para serem transmitidas.
void task_radio_Tx( void ) {
    TRadioMsg msg;
    while ( !is_radio_buffer_empty(&tx_buffer) ){    
      read_msg_radio_buffer( &tx_buffer, &msg );
      RF24NetworkHeader header(base_node, msg.type);
      network.write(header, &msg.data, sizeof(msg.data));
    }
}

//"Executa" a mensagem dada o tipo dela (A, C, S, R, M, V, Z ou N).
void dispatch_msgs( void ){
  if( is_radio_buffer_empty(&rx_buffer) ) return;
        
  TRadioMsg msg;
  read_msg_radio_buffer( &rx_buffer, &msg );

  switch ( msg.type ){

    // Leitura do endereço do nó
    case 'A':   get_node_addr( true ); 
    break;

    // Ajusta curso (rotação, sentido e velocidade)
    case 'C':   if( is_rotating() )
      write_msg_radio_buffer( &rx_buffer, &msg );
      else set_speed( msg.data );
    break;

    // Ajuste individual das vels. dos motores
    case 'S':   if( is_rotating() )
        write_msg_radio_buffer( &rx_buffer, &msg );
      else set_speed( msg.data );
    break;

    // Rotação do robô
    case 'R' :  if( is_rotating() )
      write_msg_radio_buffer( &rx_buffer, &msg );
      else set_rotation( msg.data ); 
      Serial.println(msg.data, HEX);
    break;

    // Leitura/Config. direta dos params. dos motores
    case 'M' :  if( is_rotating() )
      write_msg_radio_buffer( &rx_buffer, &msg );
      else if( msg.data > 0xFFFFF ) 
        get_motor_status( true );
      else {
        mov.config.ctr_pid_en = 0;
        set_motor_status( msg.data );
      }
    break;

      // Leitura da tensão da bateria
    case 'V':   get_volt_bat( true ); 
    break;

    // Reinicio o Arduino por SW
    case 'Z': soft_reset();
    break;
        
    // Altera o canal de rádio do robô
    case 'N': set_network_channel( (uint8_t)msg.data );
    break;

    // Mensagens com tipos desconhecidos são devolvidas à base
    default: write_msg_radio_buffer( &tx_buffer, &msg );
  }

}

//Coloca uma mensagem no buffer interno do sistema.
int8_t write_msg_radio_buffer( TRadioBuf&, TRadioMsg& ){
  //Verifica se o buffer esta cheio.
  if( buf->tam == TAM_BUFFER ) 
    return -1;
  
  //Coloca a mensagem no final do buffer.
  buf->msg[buf->tail].type = msg->type;
  buf->msg[buf->tail].data = msg->data;
  
  //Verifica se a tail nao esta no final do buffer.
  if( ++buf->tail == TAM_BUFFER ) 
    buf->tail = 0;
  
  //Arruma o tamanho do buffer.
  buf->tam++;

  return ( TAM_BUFFER - buf->tam );        // Retorna: espaço restante
}

int8_t read_msg_radio_buffer( TRadioBuf&, TRadioMsg& ){
  
  //Verifica se o buffer nao esta vazio.
  if( buf->tam == 0 ) 
    return -1;           // Erro, buffer vazio
  
  //A mensagem que sera executada recebe a mensagem que esta no comeco do buffer.
  msg->type = buf->msg[buf->head].type;
  msg->data = buf->msg[buf->head].data;
  
  //verifica se a head nao esta no final do buffer.
  if( ++buf->head == TAM_BUFFER ) 
    buf->head = 0;
  
  //Diminui o tamanho do buffer.
  buf->tam--;
  
  return buf->tam;                         // Retorna: espaço ocupado
}

//Reinicia o buffer, setando tudo em 0.
void flush_radio_buffer( TRadioBuf& ){
  buf->head   = 0;
  buf->tail   = 0;
  buf->tam    = 0;
  buf->last_t = 0;
}

//Verifica se o buffer esta cheio.
bool is_radio_buffer_full( TRadioBuf& ){
  if (buf->tam == TAM_BUFFER)
    return 1;
  return 0;
}

//verifica se o buffer esta vazio
bool is_radio_buffer_empty( TRadioBuf& ){
  if (buf->tam == 0)
    return 1;
  return 0;
}
