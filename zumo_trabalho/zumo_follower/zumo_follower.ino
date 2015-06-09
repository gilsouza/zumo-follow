#include <QTRSensors.h>
#include <ZumoReflectanceSensorArray.h>
#include <ZumoMotors.h>
#include <ZumoBuzzer.h>
#include <Pushbutton.h>

#define SENSOR_THRESHOLD 300
// Macro definida para dizer se "linha" ou pedaço dela deve ser evitada.
#define ABOVE_LINE(sensor)((sensor) > SENSOR_THRESHOLD)
#define TURN_SPEED 200
#define SPEED 200 
#define DISTANCE_OBJECT 10.0

#define MELODY_LENGTH 95

/*
 * Estes arrays tem um total de 285 bytes de RAM. Nao atingem o limite do ATmega.
 * Os arrays para notas e duracao das notas possuem o mesmo tamanho
 */

// Notas para melodia da musica do mario
unsigned char note[MELODY_LENGTH] = 
{
  NOTE_E(5), SILENT_NOTE, NOTE_E(5), SILENT_NOTE, NOTE_E(5), SILENT_NOTE, NOTE_C(5), NOTE_E(5),
  NOTE_G(5), SILENT_NOTE, NOTE_G(4), SILENT_NOTE,

  NOTE_C(5), NOTE_G(4), SILENT_NOTE, NOTE_E(4), NOTE_A(4), NOTE_B(4), NOTE_B_FLAT(4), NOTE_A(4), NOTE_G(4),
  NOTE_E(5), NOTE_G(5), NOTE_A(5), NOTE_F(5), NOTE_G(5), SILENT_NOTE, NOTE_E(5), NOTE_C(5), NOTE_D(5), NOTE_B(4),

  NOTE_C(5), NOTE_G(4), SILENT_NOTE, NOTE_E(4), NOTE_A(4), NOTE_B(4), NOTE_B_FLAT(4), NOTE_A(4), NOTE_G(4),
  NOTE_E(5), NOTE_G(5), NOTE_A(5), NOTE_F(5), NOTE_G(5), SILENT_NOTE, NOTE_E(5), NOTE_C(5), NOTE_D(5), NOTE_B(4),

  SILENT_NOTE, NOTE_G(5), NOTE_F_SHARP(5), NOTE_F(5), NOTE_D_SHARP(5), NOTE_E(5), SILENT_NOTE,
  NOTE_G_SHARP(4), NOTE_A(4), NOTE_C(5), SILENT_NOTE, NOTE_A(4), NOTE_C(5), NOTE_D(5),

  SILENT_NOTE, NOTE_G(5), NOTE_F_SHARP(5), NOTE_F(5), NOTE_D_SHARP(5), NOTE_E(5), SILENT_NOTE,
  NOTE_C(6), SILENT_NOTE, NOTE_C(6), SILENT_NOTE, NOTE_C(6),

  SILENT_NOTE, NOTE_G(5), NOTE_F_SHARP(5), NOTE_F(5), NOTE_D_SHARP(5), NOTE_E(5), SILENT_NOTE,
  NOTE_G_SHARP(4), NOTE_A(4), NOTE_C(5), SILENT_NOTE, NOTE_A(4), NOTE_C(5), NOTE_D(5),

  SILENT_NOTE, NOTE_E_FLAT(5), SILENT_NOTE, NOTE_D(5), NOTE_C(5)
};

// Duracao para cada nota da musica do mario
unsigned int duration[MELODY_LENGTH] =
{
  100, 25, 125, 125, 125, 125, 125, 250, 250, 250, 250, 250,

  375, 125, 250, 375, 250, 250, 125, 250, 167, 167, 167, 250, 125, 125,
  125, 250, 125, 125, 375,

  375, 125, 250, 375, 250, 250, 125, 250, 167, 167, 167, 250, 125, 125,
  125, 250, 125, 125, 375,

  250, 125, 125, 125, 250, 125, 125, 125, 125, 125, 125, 125, 125, 125,

  250, 125, 125, 125, 250, 125, 125, 200, 50, 100, 25, 500,

  250, 125, 125, 125, 250, 125, 125, 125, 125, 125, 125, 125, 125, 125,

  250, 250, 125, 375, 500
};

ZumoBuzzer buzzer;
// Array de sensores de refletancia que nos dirao a posicao da linha de acordo com sua leitura
ZumoReflectanceSensorArray reflectanceSensors;
ZumoMotors motors;
Pushbutton button(ZUMO_BUTTON);
int lastError = 0;
/* 
 * Flag usada para identificar linha branca ou nao.
 * Ao alterar o valor de white para 1 a macro ABOVE_LINE deve ficar (sensor) < SENSOR_THRESHOLD. 
 * Invertendo o sinal de comparacao para se adequar a sensibilidade
 */
int white = 0;
// Usado para controle do indice usado para as notas e duracoes da musica tocada.
unsigned char currentIdx;

// pinagem usada pelo ultra som.
const uint8_t trig_pin = 4;
const uint8_t echo_pin = 5;
uint32_t print_timer;

/*
 * Funcao para inicializacao de configuracao
 * Executada antes do loop
 *
 * Nela Inicia faleremos a inicializacao dos sensores de refletancia,
 * calibragem dos sensores e setup dos pinos para sensor ultra som.
 */

void setup()
{
  // buzzer indica que robo iniciou setup
  buzzer.play(">g32>>c32");
  // notas e melodias comecam do zero.
  currentIdx = 0;
  // inicia ssensores de refletancia
  reflectanceSensors.init();
  // aguarda confirmacao para inicio da calibragem
  button.waitForButton();
  
  // calibragem
  calibrate();
  // setup de pinos ultra som
  setupUltraSom();
  
  // aguarda confirmacao para inicio do robo
  button.waitForButton();
}

/*
 * Funcao contendo o loop de execucao do robo
 */
void loop()
{
  // Teste
  //getDistanceWithUltraSonic();
  solveCircuit();
  // Teste
  //aroundTheWorld();
}

void setupUltraSom()
{
  Serial.begin(9600); // habilita Comunicação Serial a uma taxa de 9600 bauds.
 
  // configuração do estado inicial dos pinos Trig e Echo.
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
  digitalWrite(trig_pin, LOW);
}


double getDistanceWithUltraSonic() {
  // garante intervalo de 0,5s (500ms) entre medições.
  millis();
  if (millis() - print_timer > 500) {
    print_timer = millis();
 
    // pulso de 5V por pelo menos 10us para iniciar medição.
    digitalWrite(trig_pin, HIGH);
    delayMicroseconds(11);
    digitalWrite(trig_pin, LOW);
 
    /* mede quanto tempo o pino de echo ficou no estado alto, ou seja,
    o tempo de propagação da onda. */
    uint32_t pulse_time = pulseIn(echo_pin, HIGH);
 
    /* a distância entre o sensor e o objeto será proporcional a velocidade
      do som no meio e a metade do tempo de propagação. Para o ar na
      temperatura ambiente Vsom = 0,0343 cm/us. */
    double distance = 0.01715 * pulse_time;
 
    // imprimimos o valor na porta serial;
    Serial.print(distance);
    Serial.println(" cm");
    
    return distance;
  }
}

/*
 * Funcao que toca a musica presente nos arrays de notas e duracoes
 */

void song()
{
    if (currentIdx < MELODY_LENGTH && !buzzer.isPlaying())
  {
    // toca a nota corrente no volume maximo
    buzzer.playNote(note[currentIdx], duration[currentIdx], 15);
    currentIdx++;
    // reinicia a musica
    if (currentIdx == MELODY_LENGTH) currentIdx = 0;
  }
}


/*
 * Funcao para seguir linha
 * Nesta funcao o robo capaz de identificar a linha
 * O quao alinhado esta e o quanto eh necessario se ajustar
 *
 * O robo decidira se deve continuar ao identificar um gap
 * E sera capaz de contornar um obstaculo curto pela direita.
 */
void solveCircuit()
{
  // declaracao de variaveis para tomada de decisao
  unsigned int position;
  unsigned int sensors[6];
  int offset_from_center;
  int power_difference;
  
  // looping para circuito
  while(1)
  {
    /* le os sensores passando como terceiro argumento o uso de linha branca ou nao 
       e retorna a posicao.
       de acordo com a documentacao sera a media ponderada conforme formula abaixo:
       
        0*value0 + 1000*value1 + 2000*value2 + ...
       --------------------------------------------
             value0  +  value1  +  value2 + ...
    */
    position = reflectanceSensors.readLine(sensors, QTR_EMITTERS_ON, white);
    
    // calcula diferenca da posicao
    offset_from_center = ((int)position) - 2500;
    // ajusta diferenca da velocidade
    power_difference = offset_from_center / 3;

    // regula velocidade maxima
    if(power_difference > SPEED)
      power_difference = SPEED;
    if(power_difference < -SPEED)
      power_difference = -SPEED;
     
     // ajusta direcao
    if(power_difference < 0)
      motors.setSpeeds(SPEED + power_difference, SPEED);
    else
      motors.setSpeeds(SPEED, SPEED - power_difference);
      
      // verifica distancia do objeto para realizar contorno
  //  if (getDistanceWithUltraSonic() <= DISTANCE_OBJECT)
//    {
        // para antes de contornar
  //    motors.setSpeeds(0,0);
        // realiza contorno pela direita
//      aroundTheWorld();
//    }
//    else
//    {
      
      // condicao de parada para o loop.
      // linha transversal encontrada no array dos sensores.
      if(ABOVE_LINE(sensors[1]) && ABOVE_LINE(sensors[2]) && ABOVE_LINE(sensors[3]) && ABOVE_LINE(sensors[4]))
      {
        motors.setSpeeds(0,0);
        //buzzer.stopPlaying();
        break;
      }
      
      // verifica gap e mantem a velocidade. 
      if(!ABOVE_LINE(sensors[0]) && !ABOVE_LINE(sensors[1]) && !ABOVE_LINE(sensors[2]) && !ABOVE_LINE(sensors[3]) && !ABOVE_LINE(sensors[4]) && !ABOVE_LINE(sensors[5]))
      {
        motors.setSpeeds(SPEED, SPEED);
      }
      else if(ABOVE_LINE(sensors[0]) || ABOVE_LINE(sensors[5]))
      {
        return;
      }
    //}
  }
}

/*
 * Funcao para contornar obstaculos curtos durante o ciruito.
 * onde R = direita, L = esquerda
 */
void aroundTheWorld()
{
  turn('R');
  motors.setSpeeds(TURN_SPEED, TURN_SPEED);
  delay(1000);
  turn('L');
  motors.setSpeeds(TURN_SPEED, TURN_SPEED);
  delay(1000);
  turn('L');
  motors.setSpeeds(TURN_SPEED, TURN_SPEED);
  delay(1000);
  turn('R');
  motors.setSpeeds(0, 0);
}

/*
 * Funcao para mudar direcao do robo
 */
void turn(char dir)
{
  switch(dir)
  {
    // esquerda ou para tras
    case 'L':
    case 'B':
      motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
      delay(700);
      motors.setSpeeds(0, 0);
    
    break;
    
    // direita
    case 'R':
      motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
      delay(700);
      motors.setSpeeds(0, 0);
      
    break;
    
    // nao muda
    case 'S':
    break;
  }
}

/*
 * Funcao para calibragem dos sensores
 * faz uso da biblioteca dos sensores de refletancia
 * ajustando os minimos e maximos como valores aceitaveis de acordo com ambiente
 */

void calibrate()
{
  // setup de pinos para sensor
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
 
  delay(1000);
  // loop para varrer linha e suas bordas
  int i;
  for(i = 0; i < 80; i++)
  {
    if ((i > 10 && i <= 30) || (i > 50 && i <= 70))
      motors.setSpeeds(-200, 200);
    else
      motors.setSpeeds(200, -200);
    reflectanceSensors.calibrate();
    
    // loop demorara no maximo 20 * 80 = 1600 ms
    delay(20);
  }
  motors.setSpeeds(0,0);

  digitalWrite(13, LOW);
  // robo acabou de calibrar
  buzzer.play(">g32>>c32");
}
