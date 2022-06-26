// Adicionando biblioteca LiquidCrystal para uso do LCD.
#include <LiquidCrystal.h>
#include <stdlib.h>

// Adicionando todos os pinos correspondentes ao LCD.
const int rs = 12, en = 11, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

// Funcões antigas para controle do display de 7 segmentos - 1 parte do projeto.
byte heart[8] = {
  0b00000,
  0b01010,
  0b11111,
  0b11111,
  0b11111,
  0b01110,
  0b00100,
  0b00000
};

byte smiley[8] = {
  0b00000,
  0b00000,
  0b01010,
  0b00000,
  0b00000,
  0b10001,
  0b01110,
  0b00000
};

byte frownie[8] = {
  0b00000,
  0b00000,
  0b01010,
  0b00000,
  0b00000,
  0b00000,
  0b01110,
  0b10001
};

byte armsDown[8] = {
  0b00100,
  0b01010,
  0b00100,
  0b00100,
  0b01110,
  0b10101,
  0b00100,
  0b01010
};

byte armsUp[8] = {
  0b00100,
  0b01010,
  0b00100,
  0b10101,
  0b01110,
  0b00100,
  0b00100,
  0b01010
};

//VARIÁVEIS DE CONTROLE
int contador = 0;
int temperatura = A0;
int heater = 9;
int cooler = A1;
int potenCooler = A2;
int potenKp = A3;
int potenKd = A4;
int potenKi = A5;
float temperaturaDesejada = 40;
int contadorDelaySerial = 0;

//CONTROLADOR PID
float kp = 20;
float kd = 0;
float ki = 2;
float e_ant, ui_ant, ud_ant;

void setup() {

  Serial.begin(9600);

  //Inicializando LCD.
  lcd.begin(16, 2);
  lcd.createChar(0, heart);
  lcd.createChar(1, smiley);
  lcd.createChar(2, frownie);
  lcd.createChar(3, armsDown);
  lcd.createChar(4, armsUp);
  lcd.setCursor(0, 0);

  //Configurando pinos dos nossos componentes.
  pinMode(potenCooler, INPUT);
  pinMode(potenKp, INPUT);
  pinMode(potenKd, INPUT);
  pinMode(potenKi, INPUT);
  pinMode(heater, OUTPUT);
  pinMode(cooler, OUTPUT);
}


void loop() {
  mostrarTemperatura();
  controlarCooler();
  controlarKp();
  controlarKd();
  controlarKi();
  delay(100);
  lcd.clear();
  contadorDelaySerial = contadorDelaySerial + 100;
}

//Função para leitura da temperatura e demonstração no lcd após filtragem e controle.
void mostrarTemperatura() {
  float leituraTemperatura = analogRead(temperatura);

  leituraTemperatura = simularRuido(leituraTemperatura);          //Simulação de um ruído da leitura analógica

  float leituralFiltrada = filtroPassaBaixa(leituraTemperatura);  //Implementação do filtro passa baixa da leitura do sensor
  float temperaturaAtual = leituralFiltrada * (500 / 1023.0);     //Conversão da temperatura do sensor

  //Printando informações no LCD
  lcd.setCursor(0, 0);
  lcd.print(String(temperaturaAtual) + " C" + " Kp:" + String(kp));

  lcd.setCursor(0, 1);
  lcd.print("Kd:" + String(kd) + " Ki:" + String(ki));
  
  calcularPID(temperaturaAtual);
}

float simularRuido(float leituraTemperatura) {
  float ruidoRandom = rand() % 5 + 1;
  leituraTemperatura = leituraTemperatura + ruidoRandom;
  return leituraTemperatura;
}

//Filtro passa baixa utilizado para filtrar leitura do sensor de temperatura.
float filtroPassaBaixa(float temp) {
  static float x = 0;              //Unidade estática para utilização no filtro
  float y;                         // Retorno do filtro

  y = x;
  x = 0.6875 * x + 0.3125 * temp;  //Calcula x(K+1)
  return (y);
}

//Função responsável por calcular PID
void calcularPID(float temperaturaAtual) {
  float valorHeater = controladorPID(temperaturaAtual);

  if (contadorDelaySerial == 500) {
    Serial.println("Valor do heater: " + String(valorHeater));
    contadorDelaySerial = 0;
  }

  analogWrite(heater, int(valorHeater));
}

//Controlador PID
float controladorPID(float tempAtual) {
  float e = temperaturaDesejada - tempAtual; //Calcula o erro
  
  float up = kp * e;                    //Controlador proporcional
  float ui = ui_ant + ki * e * 0.1;     //Controlador integral
  float ud = (kd * (e - e_ant)) / 0.1;  //Controlador derivativo
  float u = up + ui + ud;               //Calculo do controle 

  e_ant = e;                            //Popula variável do erro anterior a partir do erro atual
  ui_ant = ui;                          //Popula variável do controlador integral anterior a partir do controlador integral atual
  ud_ant = ud;                          //Popula variável do controlador derivativo anterior a partir do controlador derivativo atual

  u = u <= 255 ? u : 255;               //Caso o controle retorne maior que 255 que é o valor máximo, seta padrão 255
  u = u > 0 ? u : 0;                    //Caso o controle retorne menos que 0 que é o valor mínimo, seta padrão 0
  return u;
}

//Função para controle do cooler a partir da leitura analógica do potenciômetro
void controlarCooler() {
  float leituraCooler = analogRead(potenCooler);
  leituraCooler = leituraCooler / 4;
  leituraCooler = leituraCooler > 255 ? 255 : leituraCooler;
  analogWrite(cooler, leituraCooler);
}

//Função para controle do kp a partir da leitura analógica do potenciômetro
void controlarKp() {
  float leituraKp = analogRead(potenKp);
  leituraKp = leituraKp / 20;
  kp = leituraKp;
}

//Função para controle do kd a partir da leitura analógica do potenciômetro
void controlarKd() {
  float leituraKd = analogRead(potenKd);
  leituraKd = leituraKd / 20;
  kd = leituraKd;
}

//Função para controle do ki a partir da leitura analógica do potenciômetro
void controlarKi() {
  float leituraKi = analogRead(potenKi);
  leituraKi = leituraKi / 20;
  ki = leituraKi;
}
