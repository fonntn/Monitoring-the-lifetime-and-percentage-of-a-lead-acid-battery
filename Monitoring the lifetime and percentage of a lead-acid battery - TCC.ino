//106REV2 - CARREGADO NA CADEIRA DIA 24/11/18 - SÁBADO, COM 2A CHAVE PARA REDE , FILTRO DO ADS E SIRENE...

//PGM deixado na Cadeira de rodas em 19/11/2018...rodando mediçao e blynk no ESP32 sem temperaturas....

// 103revxTCC_BAT...AUT-371 2018.2 - Módulo de medição  no ESP32 e TX via serial para o NODEMCU
// Monitoração das 2 baterias recarregaveis da cadeira de rodas via IoT.
// Controle de versões
/*
 * Este SW executa no ESP32 as medições dos sinais de It, Vt, VB1 e Temp. das 2 baterias.  VB2 calculada via SW. E envia os dados para o NodeMCU via Serial.
 * Em outro SW, executado no NodeMCU, tem-se a recepção dos sinais do ESP32 via serial, alguns cálculos e transmissão via Blynk (IoT).
 * 103rev1 - versão inicial instalada na cadeira de rodas em 17/11/2018.
 *  
 * Diagrama em bocos - básico
                        |--------------------------------|
                        |        Disjuntor 50 A          |
             ___________| (backup do sensor de corrente) |------------------->>>> vai para a cadeira de rodas --------------------------------------------|
            |           |   em paralelo com o sensor     |                                                                                                |
            |           |     de corrente ACS758         |---------<<< + 5 Vcc (vem do modulo da fonte 2)                                                 |
            |           |--------------------------------|--->>>>>sinal de tensão correspondente a It -------->>Vai para módulo (ESP32)                   |
            |                                                                                                                                   |--------------------|
           +|------------>>>>> Tomada da tensão total das Baterias 1 e 2 (Vt= VBat1 + VBat2)------------------>>Vai para módulo (ESP32)         |      CADEIRA       |
        |--------|                                                                                                                              |                    |
        |  Bat1  |                                                                                                                              |        DE          |
        |--------|                                                                                                                              |                    |
           -|                                                                                                                                   |       RODAS        |
            |------------>>>>> Tomada para alimentar módulos 1 e 2 (fontes de alimentação) e tensão da VBat2 -->>Vai para módulo (ESP32)        |                    |
           +|                                                                                                                                   |                    |
        |--------|                                                                                                                              |--------------------|
        |  Bat2  |                                                                                                                                         |          
        |--------|                                                                                                                                         |       
           -|                                                                                                                                              |       
            |-----SW ----Fusivel ---GND------------------------------------------------------------------------>>Vai para módulo (ESP32)                   |       
            |                                                                                                                                              |       
            |------------------------------------------------------------------>>>> vai para a cadeira de rodas -------------------------------------------|

                                                                                 ^
            |--------|                                            |------------|     | (Blynk - via wireless)                                                                            |--------------------|
            |  ESP32 |  -------------->>>>  (Serial) ----------   |   NodeMCU  | -----                                                                                                                           |          
            |--------|                                            |------------|

*/

//Bibliotecas: DAllas e OneWire (DS18B20) + Blynk + SimpleTimer
//#include <Wire.h>


#define  CORRENTE  adc0
#define  N  600
#include <Adafruit_ADS1015.h>
#define  Simple.timer  Timer
 
// ADS1115 ........................... Sensor de tensão de 4 canais.
#include <Adafruit_ADS1015.h>
int16_t adc0, adc1, adc2, adc3;
Adafruit_ADS1115 adsA(0x48);                  //16-bit version       //Adafruit_ADS1015 ads; //12-bit version

float avg0 = 0.0f;
float avg1 = 0.0f;
float avg2 = 0.0f;
float avg3 = 0.0f;
int filtered;
int total = 45;
int vals[N];

float carga1 = (total - (filtered*0.17));
float carga = (carga1 - (filtered*0.17));
float porcentagem1 = (1-((total - carga1)/(total)));
float porcentagem = (porcentagem1 - ((carga1 - carga) / carga1));


float avg0_prel = 0.0f;
float avg1_prel = 0.0f;
float avg2_prel = 0.0f;
float avg3_prel = 0.0f;

// Chave seletora no módulo eletronico
#define SW1 26                         // Chave em D26 (posição da chave em "0": aberta, D26==1. Posição da chave em "1": fechada, D26==0)
#define SW2 5                          // chave 1-0-1
#define SW3 18                         // chave 1-0-1
int16_t a, v5a, v6a, i;                // i(iteração blynk)

//ESP32S BLE + WIFI ..........................<<<<<<<<<<<<<<<<<<<<<< trocar também o modulo no Blybk
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>



WidgetLED led2(V0);                      //LIVE >> Led para sinalizar Blynk conectado
WidgetLED led26(V26);                    // Reserva futura
//Essa versão teste  BLYNK sem serial e medição e blynk no ESP32





const int ledBi = 2;                     // Built-In led na placa ESP32. RESERVA FUTURA
float v0,v1,v2,v3,v21,v10,v11,v60;         
long tempoAtual,tempoAtualA,tempoAtualB;                         //Tempo atual para temporizar flashing leds

//Medidos pelo ADS1115
//v0:  It       (-50 a +50 A)   (medido, inicialmente, pelo ACS758)
//v1:  Vt       (24 Vcc)        (Vt=VB1 + VB2). VB1 é calculado: (VB1 = Vt-VB2)
//v2:  VB2      (12 Vcc)
//v3:  Fonte +5 (5 Vcc)         Módulo fonte 2 : alimehta o sensor de corrente ACS758

//v10: Tensão de alarme para bateria descarregada 100% e ajustada pelo Slider do Blynk) .. ................processar no NODEMCU
//v11: Health (Saúde da bateria) ......................................................................... processar no NODEMCU 
//v28 e v29 >>>> Variáveis dos valores de temperatura em graus Celsius das baterias 1 e 2 .................medição feita no NODEMCU
//Medidos pelo DS18B20 no módulo NODEMCU
//v28: Temperatura Bateria 1 (0 a 70 oC)>>> deletados
//v29: Temperatura Bateria 2 (0 a 70 oC)>>> deletados
//v60: Potência (watts)    (calculada Vt e It).. ..........................................................processar no NODEMCU

unsigned char message1[55] = "AUT371,12345,6789,abcde,fghijk,lmnop,  ";    //alterado de 55 para 65 para incluir temperaturas e outros sinais de controle....
int j;

//      AUT371,12345,6789,abcde,fghijk,lmnop, qrstu
//field:   0,    1  , 2  ,  3  ,  4   ,  5   ,  6
// message[1] a [6]: "AUT371" : senha para sinalizar início da mensagem
// message[8]:  valor binário da chave SW1 (D26)
// message[9]:
// message[10] a [14]: v0 tensão do ACS758 correspondente ao sinal da corrente It. Canal A0:ADS1115
// message1[15] = 1; //=1 para It>=0, =0 para It <0. Visa a transformar para positivo o sinal de It para a serial e trata sinal de It no módulo NODEMCU/RX (Blynk)
// message[16] a [20]: v1 tensão da Bateria 1 + bateria 2 (24 Vcc). Canal A1:ADS1115
// message[21] a [25]: v2 tensão da bateria 2 (12 Vcc). Canal A3:ADS1115
// message[26] a [30]: v3 tensão da fonte de + 5 Vcc (módulo fonte 2) que alimenta o ACS758. Medição para cálculo do desvio de zero do sinal de It.
// message[31] a [35]: v21 tensão calculada da bateria 1 (v21 = v2 - v1)
// message[36] a [40]: 
// message[41] a [45]: 
// message[46] a [50]:
// message[51] a [55]:

/*
float a11, a21, a22, a23, a2, a3, a4, a5, a6;  //variáveis da estrutura de conversão para caracteres para envio via serial
int d1, d2, d3, d4, d5;                        //variáveis da estrutura de conversão para caracteres para envio via serial

//https://www.reddit.com/r/arduino/comments/6y2lzs/returning_multiple_variables_with_a_function/
struct Data {
  int d1, d2, d3, d4, d5;    //caracteres de cada um dos dígitos da variavel que serao passadas pelo canal serial
};

Data getdat (float x) {      //exempo: V0 = 23,735 ( o canal serial enviará um caractere por vez. Torna-se necessário decompor o rferido valor para a TX.)
  a11 = ( x / 10.0);         //2,3735               dig1= int(a11)
  a21 = a11 - int(a11);      //0,3735
  a22 = a21 * 10;            //3,735                dig2 = int(a22)
  a23 = a22 - int(a22);      // 0,735
  a2 = a23 * 10;             //7,35                 dig3 = int (a2)
  a3 = a2 - int(a2);         //0,35
  a4 = a3 * 10;              //3,5                  dig4 = int(a4)
  a5 = a4 - int(a4);         //0,5
  a6 = a5 * 10;              //5                    dig5 = int(a6)
  return {
    d1 = int(a11),           //d1=2
    d2 = int(a22),           //d2=3
    //ponto decimal          // .
    d3 = int(a2),            //d3=7
    d4 = int(a4),            //d4=3
    d5 = int(a6)};           //d5=5
}
*/

//char auth[] = "your char auth";

//char ssid1[] = "your char ssid1";
//char pass1[] = "your char pass1";    //rede residencial

//char ssid2[] = "your char ssid2";             //Roteamento pelo celular do Thiago
//char pass2[] = "your char pass2";

void setup(void)
{

  Serial.begin (9600); 
  int filtered;
  int vals[N];
  int total = 45;
  int16_t adc0, adc1, adc2, adc3;
  
 
 
  
  if(v1>25){
  Blynk.virtualWrite(V0, (porcentagem1));
  }
  if(v1<25){
  Blynk.virtualWrite(V0, (porcentagem));
  }
  Serial.begin(9600);
  adsA.setGain(GAIN_TWO);          // 2x gain   +/- 2.048V  1 bit = 1mV  0.0625mV
  adsA.begin();
  pinMode(32, OUTPUT);            // led verde (VD) 
  pinMode(33, OUTPUT);            // led vermelho (VM)
  pinMode(SW1, INPUT);            // Chave selecao rede SW1 (D26)  
  pinMode(ledBi, OUTPUT);         // Led Built-in na placa ESP32 (não usado)
  pinMode(23,OUTPUT);             // Sirene ativada pelo Blynk (D23)
  pinMode(SW2, INPUT);            // Chave selecao rede 1-0-1 em D5
  pinMode(SW3, INPUT);            // Chave selecao rede 1-0-1 em D18

if((digitalRead(SW1)==1)&&(digitalRead(SW2)==1)&&(digitalRead(SW3)==1)){
    Serial.print("rede 1 - casa");
    Blynk.begin(auth, ssid1, pass1);     //residencia
    }
if((digitalRead(SW1)==0)&&(digitalRead(SW2)==1)&&(digitalRead(SW3)==1)){
    Serial.print("rede 2 - celular  Thiago");
    Blynk.begin(auth, ssid2, pass2);     //celular
    }
if((digitalRead(SW1)==0)&&(digitalRead(SW2)==0)&&(digitalRead(SW3)==1)){
    Serial.print("rede 3 - trabalho");
    Blynk.begin(auth, ssid3, pass3);     //celular
    }
if((digitalRead(SW1)==0)&&(digitalRead(SW2)==1)&&(digitalRead(SW3)==0)){
    Serial.print("rede 4 - IFRJ");
    Blynk.begin(auth, ssid4, pass4);     //celular
    }
if((digitalRead(SW1)==1)&&(digitalRead(SW2)==1)&&(digitalRead(SW3)==0)){
    Serial.print("Celular Gilmar ");
    Blynk.begin(auth, ssid5, pass5);     //celular
    }   
   Timer.setInterval(1637L, Timer_1s);
  //timer.setInterval(4973L, Timer_5s);

}
//Rotinas BLYNK
void Timer_1s() 
{
  v0=(((avg0-3200.0f)*(100000.0f)/(25600.0f))-50000.0f)/1000.0f;
  v1=(avg1)*(30000.0f)/(32000.0f)/1000.0f;
  v2=(avg2)*(15000.0f)/(32000.0f)/1000.0f;
  v3=(avg3)*(6000.0f)/(32000.0f)/1000.0f;
  if (led2.getValue()) {     //led2 (LIVE System/Blynk)>> "V0"
    led2.off();
    digitalWrite(32, HIGH);
    digitalWrite(33, LOW);
    }
  else 
    {
    led2.on();
    digitalWrite(32, LOW);
    digitalWrite(33, HIGH);
    Blynk.virtualWrite(V50,(v0));                                
    Blynk.virtualWrite(V51,(v1));
    Blynk.virtualWrite(V52,(v2));
    Blynk.virtualWrite(V53,(v3));
    Blynk.virtualWrite(V21,(v21));
    Blynk.virtualWrite(V60,(v60));
   }
}

//BLYNK_WRITE(V10) //Recebe valor do Blynk da tensão limite para bateria 100% descarregada (ex.: 10,5 Vcc)
//{
// v10 = param.asFloat(); 
//}


void loop(void)
{
     Blynk.run();
     Timer.run();
   /*
  if(millis()-tempoAtual>600){    
    tempoAtual=millis();
    }
   */
for(int n1=0; n1<25; n1++){
    adc0 = adsA.readADC_SingleEnded(0);
    adc1 = adsA.readADC_SingleEnded(1);
    adc2 = adsA.readADC_SingleEnded(2);
    adc3 = adsA.readADC_SingleEnded(3);

    avg0_prel += adc0;
    avg1_prel += adc1;
    avg2_prel += adc2;
    avg3_prel += adc3;
  }
  avg0 = avg0_prel/(25.0f);
  avg1 = avg1_prel/(25.0f);
  avg2 = avg2_prel/(25.0f);
  avg3 = avg3_prel/(25.0f);

    avg0_prel =0.0f;
    avg1_prel =0.0f;
    avg2_prel =0.0f;
    avg3_prel =0.0f;
    
   //v0 = map1(adc0, 3200, 28800, -50000, 50000);              //sinal de corrente total -50 a + 50A(-50 a + 50A/0,5 V - zero em 2,5V - 4,5V/3200 a 28800)
   v0=(((avg0-3200.0f)*(100000.0f)/(25600.0f))-50000.0f)/1000.0f;
   //v1 = map1(adc1, 0, 32000, 0, 30000);                      //sinal de tensÃ£o total das baterias 1 e 2 (0 a 24 V) >> V51 no Blynk
   v1=(avg1)*(30000.0f)/(32000.0f)/1000.0f;
   //v2 = map1(adc2, 0, 32000, 0, 15000);                       //sinal da bateria 2 + 12 Vcc (0 a 15V/0 a 2V/0 a 32000) >> V52 no Blynk       
   v2=(avg2)*(15000.0f)/(32000.0f)/1000.0f;
   //v3 = map1(adc3, 0, 32000, 0, 6000);                        //Sinal da fonte de + 5,0 Vcc (0 a 6V /0 a 2V/0 a 32000)  V53 no Blynk      
   v3=(avg3)*(6000.0f)/(32000.0f)/1000.0f;
   v21 = (v1 - v2) / 1.0f;                                      //Calculo da tensÃ£o VB1 = V(B1+B2) - VB2
  
  //correcao/compensaÃ§Ã£o da leitura de v0(It) pelas variaÃ§Ãµes da fonte +5Vcc
  //float v5 = ((((v3 / 2.0f) + 2.0f) / 2.5f) / (2.048f)) * 32767.0f;
  //v5a = int16_t(v5);
  //float v6 = v5 - 25605.0f;   //-90: ajuste de zero de It em funÃ§Ã£o de desvio da tensÃ£o lida pelo ADS para a corrente correspondente
  //v6a = int16_t(v6);
  //...............*********************.........................****************************
  //v0 = map(adc0, v6a, v5a, -50000, 50000) / 1000.0;         //sinal de corrente total -50 a + 50A(-50 a + 50A/0,5 V - zero em 2,5V - 4,5V/3200 a 28800)
  
//Cálculo da POTENCIA
     v60=v0*v1;                 //P=Vt*It (W)
          
//ALARME_LIMIAR_TENSAO Battery Health  (Se Vt >=27V (100%). Se Vt<21V (0%))
    v11=(((v10-21.0f)*(100.0f)/(6.0f))-0.0f)/1.0f;

    float value = v0;

  for(int i=N - 1; i>0; i--){
    vals[i] = vals[i-1];
  }

vals[0] = value;


long sum = 0;

for(int i = 0; i<N; i++){
sum = sum + vals[i];
}


  filtered = sum/N;

  if (v1 > 25.00)
  {
    float carga1; 
    float porcentagem1;
    }
    
  else if (v1 < 25.00);  {
    float carga;
    float carga2;
    float porcentagem;
    }

}
