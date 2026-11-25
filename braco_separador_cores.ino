/*
 * =====================================================================
 *  Braço robótico separador de objetos por cor com saída em áudio
 * =====================================================================
 *
 *  Trabalho de Conclusão de Curso — Engenharia de Controle e Automação
 *  PUC-Campinas, 2026
 *
 *  Hardware: Arduino UNO, kit de braço MARK2 (4 servos), sensor de cor
 *            TCS3200 e módulo de áudio DFPlayer Mini.
 *
 *  Funcionamento
 *    1) O sensor lê a cor da peça a cada INTERVALO_LEITURA_MS e mostra o
 *       resultado no Monitor Serial.
 *    2) O operador aperta o botão (ou envia 's' pelo Monitor Serial).
 *    3) Cor identificada: o áudio com o nome da cor toca enquanto o braço
 *       pega a peça e a leva até o nicho correspondente.
 *    4) Cor não identificada: toca o áudio 0004 e o braço permanece na
 *       posição inicial (todos os servos em 90°).
 *
 *  Cartão SD (FAT32), pasta "mp3", arquivos em formato MP3:
 *    0001.mp3 = vermelho | 0002.mp3 = azul | 0003.mp3 = verde
 *    0004.mp3 = cor não identificada
 *
 *  Ligações
 *    Sensor TCS3200 : OUT -> 5 | S2 -> 6 | S3 -> 7
 *    Botão          : entre o pino 2 e o GND (usa o pull-up interno)
 *    Servos         : base -> 8 | cotovelo -> 9 | ombro -> 10 | garra -> 13
 *    DFPlayer Mini  : TX do módulo -> pino 3
 *                     RX do módulo -> resistor de 1 kΩ -> pino 4
 *                     VCC em 5 V | GND comum | alto-falante em SPK_1 e SPK_2
 *
 *  Bibliotecas: Servo, SoftwareSerial e DFRobotDFPlayerMini
 * =====================================================================
 */

#include <Servo.h>
#include <SoftwareSerial.h>
#include <DFRobotDFPlayerMini.h>


// =====================================================================
//  CONFIGURAÇÃO
// =====================================================================

// ---------- Pinos do sensor e do botão ----------
#define PIN_OUT   5
#define PIN_S2    6
#define PIN_S3    7
#define PIN_BOTAO 2

// ---------- Pinos dos servos ----------
const int PIN_BASE     = 8;
const int PIN_OMBRO    = 10;
const int PIN_COTOVELO = 9;
const int PIN_GARRA    = 13;

// ---------- Módulo de áudio ----------
const int PIN_AUDIO_RX = 3;    // Arduino recebe <- TX do módulo
const int PIN_AUDIO_TX = 4;    // Arduino envia  -> RX do módulo (via 1 kΩ)
const int VOLUME_AUDIO = 30;   // de 0 a 30

// Faixas da pasta /mp3 do cartão SD
const int FAIXA_VERMELHO = 1;
const int FAIXA_AZUL     = 2;
const int FAIXA_VERDE    = 3;
const int FAIXA_NAO_IDENTIFICADA = 4;

// ---------- Velocidade do movimento ----------
// Tempo, em ms, para a junta que mais se move andar 1 grau.
//   10 ms -> 100 °/s (rápido) | 20 ms -> 50 °/s | 30 ms -> 33 °/s
const int ESPERA_PASSO_MS = 20;
const int PAUSA_ETAPA_MS  = 400;   // parada entre uma etapa e outra

// ---------- Poses (ângulos dos servos, em graus) ----------
// Posição da base para pegar a peça e para cada nicho
const int BASE_PEGA     = 0;
const int BASE_AZUL     = 180;
const int BASE_VERDE    = 120;
const int BASE_VERMELHO = 60;

const int BASE_HOME = 90, OMBRO_HOME = 90, COTOVELO_HOME = 90;   // posição inicial
const int OMBRO_ALTO  = 90, COTOVELO_ALTO  = 90;    // braço recolhido (alto)
const int OMBRO_BAIXO = 60, COTOVELO_BAIXO = 120;   // braço inclinado (baixo)
const int GARRA_ABERTA = 150, GARRA_FECHADA = 90;

// ---------- Sensor de cor ----------
const unsigned long TIMEOUT_US = 30000UL;          // desiste de um pulso após 30 ms
const int N_LEITURAS = 5;                          // média de 5 pulsos por canal
const unsigned long LIMITE_VERMELHO = 100;         // ajustar conforme a calibração
const unsigned long INTERVALO_LEITURA_MS = 2500;   // tempo entre leituras


// =====================================================================
//  OBJETOS E VARIÁVEIS GLOBAIS
// =====================================================================

Servo servoBase, servoOmbro, servoCotovelo, servoGarra;
Servo* servos[4] = { &servoBase, &servoOmbro, &servoCotovelo, &servoGarra };
enum { BASE, OMBRO, COTOVELO, GARRA };   // índices do vetor servos[]
int atual[4] = { 90, 90, 90, 90 };       // ângulo atual de cada servo

SoftwareSerial serialAudio(PIN_AUDIO_RX, PIN_AUDIO_TX);
DFRobotDFPlayerMini audio;

// Resultado de classificar(): 0 = inconclusivo, 1 = azul, 2 = verde, 3 = vermelho
const char* NOME_COR[4] = {"Inconclusivo", "Azul", "Verde", "Vermelho"};
int corAtual = 0;                 // última cor lida
int corAvisada = -1;              // última cor mostrada no Monitor Serial
unsigned long ultimaLeitura = 0;  // instante (millis) da última leitura


// =====================================================================
//  PROTÓTIPOS
// =====================================================================

bool operadorAutorizou();
unsigned long lerCanal(int s2, int s3);
int classificar(unsigned long r, unsigned long g, unsigned long b);
void lerCor();
void mostrarRespostaDoModulo();
void tocarFaixa(int faixa);
void moverSuave(int base, int ombro, int cotovelo, int garra);
void separar(int baseDestino);


// =====================================================================
//  SETUP
// =====================================================================

void setup() {
  // Grava o ângulo inicial ANTES do attach(), para que o servo já parta
  // na posição de repouso em vez de dar um salto ao ser ligado.
  for (int i = 0; i < 4; i++) servos[i]->write(atual[i]);
  servoBase.attach(PIN_BASE);
  servoOmbro.attach(PIN_OMBRO);
  servoCotovelo.attach(PIN_COTOVELO);
  servoGarra.attach(PIN_GARRA);

  pinMode(PIN_OUT, INPUT);
  pinMode(PIN_S2, OUTPUT);
  pinMode(PIN_S3, OUTPUT);
  pinMode(PIN_BOTAO, INPUT_PULLUP);
  Serial.begin(9600);

  serialAudio.begin(9600);
  delay(1500);                                  // tempo para o módulo inicializar

  // begin(serial, ack = false, reset = false)
  // Com o ACK desligado o módulo não responde a cada comando, o que reduz
  // o tráfego na SoftwareSerial no instante em que o braço começa a andar.
  // Observação: com reset = false o begin() sempre retorna true, por isso
  // o retorno não é usado para confirmar a comunicação.
  audio.begin(serialAudio, false, false);
  audio.volume(VOLUME_AUDIO);
  Serial.println("Modulo de audio iniciado.");

  Serial.println("Pronto. Coloque a peca na frente do sensor.");
}


// =====================================================================
//  LOOP PRINCIPAL
// =====================================================================

void loop() {
  mostrarRespostaDoModulo();

  // 1) Lê a cor periodicamente (só acontece com o braço parado)
  if (millis() - ultimaLeitura >= INTERVALO_LEITURA_MS) {
    lerCor();
  }

  // 2) Com a autorização do operador, áudio e movimento começam juntos
  if (operadorAutorizou()) {
    Serial.print("Cor: ");
    Serial.println(NOME_COR[corAtual]);

    switch (corAtual) {
      case 1:  tocarFaixa(FAIXA_AZUL);     separar(BASE_AZUL);     break;
      case 2:  tocarFaixa(FAIXA_VERDE);    separar(BASE_VERDE);    break;
      case 3:  tocarFaixa(FAIXA_VERMELHO); separar(BASE_VERMELHO); break;
      default:
        tocarFaixa(FAIXA_NAO_IDENTIFICADA);
        Serial.println("Cor nao identificada: braco parado na posicao inicial.");
        break;
    }

    // Ao fim do ciclo a peça já saiu do sensor: exige uma leitura nova
    corAtual = 0;
    corAvisada = -1;
    ultimaLeitura = millis();
  }
}


// =====================================================================
//  LEITURA E CLASSIFICAÇÃO DA COR
// =====================================================================

// Lê os três canais do TCS3200 e atualiza corAtual.
void lerCor() {
  ultimaLeitura = millis();
  unsigned long vermelho = lerCanal(LOW,  LOW);
  unsigned long verde    = lerCanal(HIGH, HIGH);
  unsigned long azul     = lerCanal(LOW,  HIGH);
  corAtual = classificar(vermelho, verde, azul);

  // Só escreve no Monitor Serial quando a cor muda, para não poluir a saída
  if (corAtual != corAvisada) {
    corAvisada = corAtual;
    Serial.print("R: ");  Serial.print(vermelho);
    Serial.print("  G: "); Serial.print(verde);
    Serial.print("  B: "); Serial.println(azul);
    Serial.print("Cor: ");
    Serial.println(NOME_COR[corAtual]);
    Serial.println("Aperte o botao para iniciar.");
  }
}

// Seleciona o filtro (S2/S3) e devolve a largura média do pulso em µs.
// No TCS3200, quanto MENOR o pulso, MAIS intensa é aquela cor.
// Devolve 0 se nenhuma leitura for válida.
unsigned long lerCanal(int s2, int s3) {
  digitalWrite(PIN_S2, s2);
  digitalWrite(PIN_S3, s3);
  delay(15);                                    // estabilização do filtro
  unsigned long soma = 0;
  int validas = 0;
  for (int i = 0; i < N_LEITURAS; i++) {
    unsigned long t = pulseIn(PIN_OUT, LOW, TIMEOUT_US);
    if (t > 0) { soma += t; validas++; }        // 0 = tempo esgotado
  }
  if (validas == 0) return 0;
  return soma / validas;
}

// Classifica pela cor de menor pulso (cor dominante).
// Devolve 1 = azul, 2 = verde, 3 = vermelho, 0 = inconclusivo.
int classificar(unsigned long r, unsigned long g, unsigned long b) {
  if (r == 0 || g == 0 || b == 0) return 0;     // leitura inválida
  if (b < r && b < g) return 1;
  if (g < r && g < b) return 2;
  if (r < b && r < g && r < LIMITE_VERMELHO) return 3;
  return 0;
}


// =====================================================================
//  ÁUDIO
// =====================================================================

// Envia o comando e retorna imediatamente: o módulo toca sozinho, então
// o braço já começa a se mover enquanto a mensagem é falada.
void tocarFaixa(int faixa) {
  audio.playMp3Folder(faixa);           // toca /mp3/000N.mp3
}

// Mostra no Monitor Serial os erros informados pelo módulo de áudio.
void mostrarRespostaDoModulo() {
  if (!audio.available()) return;
  uint8_t tipo = audio.readType();
  int valor = audio.read();
  if (tipo == DFPlayerError) {
    Serial.print("ERRO do modulo de audio: ");
    Serial.println(valor);   // 1 = sem cartão | 3 = comunicação | 6 = faixa não encontrada
  }
}


// =====================================================================
//  AUTORIZAÇÃO DO OPERADOR
// =====================================================================

// Retorna true quando o botão é apertado (e solto) ou quando chega 's'
// pelo Monitor Serial.
bool operadorAutorizou() {
  if (digitalRead(PIN_BOTAO) == LOW) {          // botão apertado
    delay(50);                                  // debounce
    while (digitalRead(PIN_BOTAO) == LOW) { }   // espera soltar
    return true;
  }
  if (Serial.available() > 0 && Serial.read() == 's') return true;
  return false;
}


// =====================================================================
//  MOVIMENTO SUAVE DOS SERVOS
// =====================================================================

// Conversão de graus para microssegundos na MESMA escala do servo.write()
// (544 a 2400 µs). Assim os ângulos calibrados continuam válidos e é
// possível enviar frações de grau, deixando o movimento mais contínuo.
const float US_MIN = 544.0, US_MAX = 2400.0;
const int PERIODO_SERVO_MS = 20;   // o servo recebe um pulso novo a cada 20 ms

void escreverGraus(int i, float graus) {
  servos[i]->writeMicroseconds((int)(US_MIN + graus * (US_MAX - US_MIN) / 180.0 + 0.5));
}

// Leva as 4 juntas aos ângulos-alvo, todas chegando ao mesmo tempo.
// Usa um perfil em "S" (smoothstep): o movimento sai devagar, acelera e
// freia suavemente no fim, evitando trancos na partida e na parada.
// Duração total = maior deslocamento (graus) * ESPERA_PASSO_MS.
void moverSuave(int base, int ombro, int cotovelo, int garra) {
  int alvo[4] = { base, ombro, cotovelo, garra };
  int inicio[4];
  int maiorDelta = 0;
  for (int i = 0; i < 4; i++) {
    alvo[i] = constrain(alvo[i], 0, 180);
    inicio[i] = atual[i];
    int d = abs(alvo[i] - inicio[i]);
    if (d > maiorDelta) maiorDelta = d;
  }
  if (maiorDelta == 0) return;            // já está na pose

  unsigned long duracao = (unsigned long)maiorDelta * ESPERA_PASSO_MS;
  int passos = duracao / PERIODO_SERVO_MS;
  if (passos < 1) passos = 1;

  for (int k = 1; k <= passos; k++) {
    float t = (float)k / passos;          // progresso de 0 a 1
    float s = t * t * (3.0 - 2.0 * t);    // curva em S
    for (int i = 0; i < 4; i++) {
      escreverGraus(i, inicio[i] + (alvo[i] - inicio[i]) * s);
    }
    delay(PERIODO_SERVO_MS);
  }
  for (int i = 0; i < 4; i++) atual[i] = alvo[i];
}

void pausa() { delay(PAUSA_ETAPA_MS); }


// =====================================================================
//  SEQUÊNCIA DE SEPARAÇÃO (a mesma para as três cores)
// =====================================================================

void separar(int baseDestino) {
  // Durante o movimento a SoftwareSerial deixa de escutar o módulo de áudio.
  // Cada byte recebido desliga as interrupções por cerca de 1 ms, o que
  // distorce o pulso dos servos e provoca trancos.
  // Consequência: erros do módulo que cheguem durante o movimento não
  // aparecem no Monitor Serial.
  serialAudio.stopListening();

  // Vai até a peça
  moverSuave(BASE_PEGA, OMBRO_ALTO,  COTOVELO_ALTO,  GARRA_ABERTA); pausa();
  moverSuave(BASE_PEGA, OMBRO_BAIXO, COTOVELO_BAIXO, GARRA_ABERTA); pausa();
  moverSuave(BASE_PEGA, OMBRO_BAIXO, COTOVELO_BAIXO, GARRA_ABERTA);  pausa();
  moverSuave(BASE_PEGA, OMBRO_BAIXO, COTOVELO_BAIXO, GARRA_FECHADA); pausa();  // pega
  moverSuave(BASE_PEGA, OMBRO_ALTO,  COTOVELO_ALTO,  GARRA_FECHADA); pausa();  // levanta

  // Leva até o nicho
  moverSuave(baseDestino, OMBRO_ALTO,  COTOVELO_ALTO,  GARRA_FECHADA); pausa();
  moverSuave(baseDestino, OMBRO_BAIXO, COTOVELO_BAIXO, GARRA_FECHADA); pausa();
  moverSuave(baseDestino, OMBRO_BAIXO, COTOVELO_BAIXO, GARRA_ABERTA);  pausa();  // solta
  moverSuave(baseDestino, OMBRO_BAIXO, COTOVELO_BAIXO, GARRA_ABERTA); pausa();
  moverSuave(baseDestino, OMBRO_ALTO,  COTOVELO_ALTO,  GARRA_FECHADA); pausa();  // afasta

  // Volta à posição inicial
  moverSuave(BASE_HOME, OMBRO_HOME, COTOVELO_HOME, GARRA_FECHADA);

  serialAudio.listen();                 // volta a escutar o módulo
}


/*
// =====================================================================
//  ETAPA FUTURA — CINEMÁTICA INVERSA (ainda não é chamada pelo loop)
// =====================================================================
//  Para usar:
//    1) medir L2, L3 e H no braço montado;
//    2) calibrar SINAL_SERVO e OFFSET_SERVO de cada junta;
//    3) trocar as poses fixas por chamadas a irParaPonto().
// =====================================================================

const float L2 = 100.0;   // A MEDIR: comprimento do elo ombro -> cotovelo (mm)
const float L3 = 100.0;   // A MEDIR: cotovelo -> ponta da garra (mm)
const float H  = 50.0;    // A MEDIR: altura do eixo do ombro em relação à mesa (mm)

// ângulo_servo = OFFSET + SINAL * ângulo_do_modelo (graus) -> calibrar com transferidor
const float SINAL_SERVO[3]  = { 1, 1, 1 };   // base, ombro, cotovelo (+1 ou -1)
const float OFFSET_SERVO[3] = { 0, 0, 90 };  // A CALIBRAR
const bool  PARALELOGRAMO   = false;         // true: servo do cotovelo recebe q2 + q3

// Dado um ponto (x, y, z) em mm, calcula q1, q2 e q3 em radianos.
// Retorna false se o ponto estiver fora do alcance.
//   q1: rotação da base
//   q2: ângulo do ombro, medido a partir da horizontal
//   q3: ângulo do cotovelo, relativo ao elo anterior (q3 < 0 = cotovelo para cima)
bool cinematicaInversa(float x, float y, float z, float &q1, float &q2, float &q3) {
  float rho  = sqrt(x * x + y * y);
  float zeta = z - H;
  float D = (rho * rho + zeta * zeta - L2 * L2 - L3 * L3) / (2.0 * L2 * L3);
  if (D > 1.0 || D < -1.0) return false;
  q1 = atan2(y, x);
  q3 = atan2(-sqrt(1.0 - D * D), D);
  q2 = atan2(zeta, rho) - atan2(L3 * sin(q3), L2 + L3 * cos(q3));
  return true;
}

// Converte o ângulo do modelo (rad) para o ângulo do servo (0 a 180°).
// junta: 0 = base, 1 = ombro, 2 = cotovelo
int paraServo(float qRad, int junta) {
  float graus = qRad * 180.0 / PI;
  int ang = (int)round(OFFSET_SERVO[junta] + SINAL_SERVO[junta] * graus);
  return constrain(ang, 0, 180);
}

// Move a ponta da garra até o ponto (x, y, z), com a garra no ângulo dado.
bool irParaPonto(float x, float y, float z, int garra) {
  float q1, q2, q3;
  if (!cinematicaInversa(x, y, z, q1, q2, q3)) {
    Serial.println("Ponto fora do alcance");
    return false;
  }
  float qCotovelo = PARALELOGRAMO ? (q2 + q3) : q3;
  moverSuave(paraServo(q1, 0), paraServo(q2, 1), paraServo(qCotovelo, 2), garra);
  return true;
}
*/
