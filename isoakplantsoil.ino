#include <DHT.h>
#include <DHT_U.h>
#include <EEPROM.h>

//#define   PORTA_SENSOR_UMIDADE_SOLO                 {0,1,2,3}
//#define   PORTA_SENSOR_TEMPERATURA_UMIDADE_AMBIENTE {2,3,4,5}
//#define   VALOR_UMIDADE_DO_SOLO                     {HIGH,HIGH,HIGH,HIGH}
//#define   VALOR_UMIDADE_DO_AMBIENTE                 {0,0,0,0}
//#define   VALOR_TEMPERATURA_DO_AMBIENTE             {0,0,0,0}
//#define   BOMBA                                     {10,11,12,13}

#define   QTDE_SENSORES                             1
#define   PORTA_SENSOR_UMIDADE_SOLO                 {0}
#define   PORTA_SENSOR_TEMPERATURA_UMIDADE_AMBIENTE {2}
#define   VALOR_UMIDADE_DO_SOLO                     {HIGH}
#define   VALOR_UMIDADE_DO_AMBIENTE                 {0}
#define   VALOR_TEMPERATURA_DO_AMBIENTE             {0}

//The variables bellow must to be included in a single structure in such way each 
// "module" (a composite of sensor and actuators to controls a "stufa")
#define   BOMBA                                     {10}
#define   DHTTYPE                                   DHT11
#define   MOLHADO                                   0
#define   SECO                                      1000
#define   UMIDO                                     700
#define   INTERVALO_DE_TEMPO_ENTRE_REGAS_FORTES     24l*60l*60l*1000l // (24h * 60min * 60seg * 1000)milis
#define   INTERVALO_DE_TEMPO_ENTRE_REGAS_BRANDAS    3l*60l*60l*1000l // (3h * 60min * 60seg * 1000)milis
#define   LIMPAR_EPPROM                             0 // (3h * 60min * 60seg * 1000)milis


//This structure must be changed
typedef struct Indicadores {
  long  regaForte=0;
  int   regasBrandasRemanescentes=8;
  long  umidadeMaxSolo =-1;
  long  umidadeMinSolo =-1 ;
  long  umidadeMediaSolo =-1;
  float umidadeMediaAmbiente =-1;
  float umidadeMaxAmbiente =-1;
  float umidadeMinAmbiente =-1;
  float temperaturaMediaAmbiente=-1;
  float temperaturaMaxAmbiente=-1;
  float temperaturaMinAmbiente=-1;
} IND;

int   portaSensorUmidadeSolo[]                = PORTA_SENSOR_UMIDADE_SOLO;
int   portaSensorTemperaturaUmidadeAmbiente[] = PORTA_SENSOR_TEMPERATURA_UMIDADE_AMBIENTE;
int   valorUmidadeDoSolo[]                    = VALOR_UMIDADE_DO_SOLO;
IND   *indicadores                            = (IND*) malloc( sizeof(IND) * QTDE_SENSORES );
DHT   *estruturaSensorTemperaturaUmidade[sizeof(portaSensorTemperaturaUmidadeAmbiente)/sizeof(int)];
float valorUmidadeDoAmbiente[]                = VALOR_UMIDADE_DO_AMBIENTE;
float valorTemperaturaDoAmbiente[]            = VALOR_TEMPERATURA_DO_AMBIENTE;
int   bomba[]                                 = BOMBA;
int   irrigou[]                               = {0,0,0,0};
long  horaInicial                             = 0l;

void setup(){
  if(LIMPAR_EPPROM){
    EEPROMWritelong(0, millis() + 1000l*60l*60l*24l);
    EEPROMWritelong(4, millis());
    
  }  

  Serial.begin(9600);
  Serial.println("Programa Irrigadora Iniciado");
  for(int i=0;i<sizeof(portaSensorTemperaturaUmidadeAmbiente) / sizeof(int);i++){
     estruturaSensorTemperaturaUmidade[i] =  new DHT(portaSensorTemperaturaUmidadeAmbiente[i], DHTTYPE);
     estruturaSensorTemperaturaUmidade[i] -> begin();
     pinMode(bomba[i], OUTPUT);
     digitalWrite(bomba[i], HIGH);
  indicadores[0].regaForte = EEPROMReadlong(0);
  Serial.print("Rega forte lida da EEPROM:");Serial.println(indicadores[0].regaForte);
  indicadores[0].regaForte = indicadores[0].regaForte<0
      ?INTERVALO_DE_TEMPO_ENTRE_REGAS_FORTES
      :indicadores[0].regaForte; 
  Serial.print("Valor Rega forte:");Serial.println(indicadores[0].regaForte);
  indicadores[i].regasBrandasRemanescentes = 7;
 }
 horaInicial = EEPROMReadlong(4);
 Serial.print("Hora lida da EEPROM:");Serial.println(horaInicial);
}


void loop(){
    readUmidadeDoSolo();
    readUmidadeETemperaturaAmbiente();
    calculaEstatisticas();
    mostraLeiturasNaSerial();
    acionarBombas();
//  acionarUmidificador();
//  acionarVentiladores();
    salvaTempoEeprom();
} 


void salvaTempoEeprom(){
    if (my_millis()- (60l*1000l)){
      Serial.print("Escrevendo o tempo atual na EEPROM: ");Serial.println(my_millis());
      EEPROMWritelong(4, my_millis());
    }  
}

void calculaEstatisticas(){
  for(int i=0;i<sizeof(portaSensorUmidadeSolo)/sizeof(int);i++){ 
    if(valorUmidadeDoSolo[i]> 0){
      //indicadores[i].umidadeMediaSolo = valorUmidadeDoSolo[i]>0:(indicadores[i].umidadeMediaSolo * 999 + valorUmidadeDoSolo[i])/1000?0;    
      calculaMediaPassadoForte(valorUmidadeDoSolo[i], &indicadores[i].umidadeMediaSolo);
      calculaMediaPassadoForte(valorUmidadeDoAmbiente[i], &indicadores[i].umidadeMediaAmbiente);      
      calculaMediaPassadoForte(valorTemperaturaDoAmbiente[i], &indicadores[i].temperaturaMediaAmbiente);            
      indicadores[i].umidadeMaxSolo = indicadores[i].umidadeMaxSolo>valorUmidadeDoSolo[i]?indicadores[i].umidadeMaxSolo:valorUmidadeDoSolo[i];
      indicadores[i].umidadeMaxAmbiente = indicadores[i].umidadeMaxAmbiente > valorUmidadeDoAmbiente[i]?indicadores[i].umidadeMaxAmbiente:valorUmidadeDoAmbiente[i];
      indicadores[i].temperaturaMaxAmbiente = indicadores[i].temperaturaMaxAmbiente > valorTemperaturaDoAmbiente[i]?indicadores[i].temperaturaMaxAmbiente:valorTemperaturaDoAmbiente[i];
      indicadores[i].umidadeMinSolo = indicadores[i].umidadeMinSolo<valorUmidadeDoSolo[i]?indicadores[i].umidadeMinSolo:valorUmidadeDoSolo[i];
      indicadores[i].umidadeMinAmbiente = indicadores[i].umidadeMinAmbiente < valorUmidadeDoAmbiente[i]?indicadores[i].umidadeMinAmbiente:valorUmidadeDoAmbiente[i];
      indicadores[i].temperaturaMinAmbiente = indicadores[i].temperaturaMinAmbiente < valorTemperaturaDoAmbiente[i]?indicadores[i].temperaturaMinAmbiente:valorTemperaturaDoAmbiente[i];
    }    
  }
}

void calculaMediaPassadoForte(int grandezaAtual, long *mediaGrandeza){
  static long peso = 100000l;
  peso = peso==50000?100000l:--peso;
  long media = ((long) *mediaGrandeza * peso + (long) grandezaAtual * (1000l-peso) ) /1000l;
  *mediaGrandeza = *mediaGrandeza>0 && *mediaGrandeza<1000 ? media : grandezaAtual;    
}

void calculaMediaPassadoForte(float grandezaAtual, float *mediaGrandeza){
  static long peso = 100000l;
  peso = peso==50000?100000l:--peso;
  float media = (*mediaGrandeza * peso + grandezaAtual * (1000l-peso) ) /1000l;
  *mediaGrandeza = *mediaGrandeza>0 && *mediaGrandeza<1000 ? media : grandezaAtual;  
}

void mostraLeiturasNaSerial(){
  for(int i=0;i<sizeof(portaSensorTemperaturaUmidadeAmbiente)/sizeof(int);i++){ 
      long proximaRegaBranda = indicadores[i].regaForte - (indicadores[i].regasBrandasRemanescentes * INTERVALO_DE_TEMPO_ENTRE_REGAS_BRANDAS);
      Serial.print("Modulo(");Serial.print(i);Serial.println(")");
      Serial.println("{");
      Serial.print("   Minutos ligado: ");Serial.print(convertToMinutes(my_millis()));Serial.println("],");
      Serial.print("   ProximaRegaForte[min]: ");Serial.print(convertToMinutes(indicadores[i].regaForte - my_millis()));Serial.println(",");
      Serial.print("   proximaRegaBranda[min]: ");Serial.print(convertToMinutes(proximaRegaBranda-my_millis()));Serial.println(",");
      Serial.print("   regasBrandasRemanescentes: ");Serial.print(indicadores[i].regasBrandasRemanescentes);Serial.println(",");
      Serial.print("   UmidSol/Med,Max,Min,Lim: ");Serial.print(valorUmidadeDoSolo[i]);Serial.print("%/");Serial.print(indicadores[i].umidadeMediaSolo);Serial.print("%, ");Serial.print(indicadores[i].umidadeMaxSolo);Serial.print("%, ");Serial.print(indicadores[i].umidadeMinSolo);Serial.print("%, ");Serial.print(UMIDO);Serial.println("%, ");
      Serial.print("   QtdeAguadas        : " );Serial.print(irrigou[i]);Serial.println(", ");
      Serial.print("   UmidAmb/Med,Max,Min: " );Serial.print(valorUmidadeDoAmbiente[i]);Serial.print("%/");Serial.print(indicadores[i].umidadeMediaAmbiente);Serial.print("%, ");Serial.print(indicadores[i].umidadeMaxAmbiente);Serial.print("%, ");Serial.print(indicadores[i].umidadeMinAmbiente);Serial.println("%, ");
      Serial.print("   TemAmb/Med ,Max,Min: " );Serial.print(valorTemperaturaDoAmbiente[i]);Serial.print("ºC/");Serial.print(indicadores[i].temperaturaMediaAmbiente);Serial.print("ºC,");Serial.print(indicadores[i].temperaturaMaxAmbiente);Serial.print("ºC,");Serial.print(indicadores[i].temperaturaMinAmbiente);Serial.println("ºC");
      Serial.println("}");  
  }  
}

void readUmidadeDoSolo(){
  for(int i=0;i<sizeof(portaSensorUmidadeSolo)/sizeof(int);i++){
    valorUmidadeDoSolo[i] = analogRead(portaSensorUmidadeSolo[i]);
  }
}

void readUmidadeETemperaturaAmbiente(){
  for(int i=0;i<sizeof(portaSensorTemperaturaUmidadeAmbiente)/sizeof(int);i++){
    valorUmidadeDoAmbiente[i]     = estruturaSensorTemperaturaUmidade[i]->readHumidity();
    valorTemperaturaDoAmbiente[i] = estruturaSensorTemperaturaUmidade[i]->readTemperature();
  }
}

void acionarBombas(){
  for(int i=0;i<sizeof(valorUmidadeDoSolo)/sizeof(int);i++){
      regaInteligente(i);
  }
}

void regaInteligente(int numeroBomba){
    //irriga a planta caso o momento atual seja maior que o momento previsto para a rega forte.
    if(my_millis() > indicadores[numeroBomba].regaForte){
      Serial.println(">>>>>>>>>>> Executando rega forte.");
      digitalWrite(bomba[numeroBomba], LOW);//liga bomba
      delay(15000);
      digitalWrite(bomba[numeroBomba], HIGH);//desligabomba
      ajustaHorarioDaProximaRegaForte(numeroBomba);
      Serial.print(">>>>>>>>> Setando próxima rega branda para:");Serial.println(indicadores[numeroBomba].regasBrandasRemanescentes);
      ajustarHorarioDaProximaRegaBranda(numeroBomba);
      irrigou[numeroBomba]=++irrigou[numeroBomba];
    }

    //irriga a planta caso o momento atual seja maior que o momento previsto para a rega branda
    if(
            my_millis() > indicadores[numeroBomba].regaForte - (indicadores[numeroBomba].regasBrandasRemanescentes * INTERVALO_DE_TEMPO_ENTRE_REGAS_BRANDAS) 
        &&  my_millis() < indicadores[numeroBomba].regaForte
      ){
      if(valorUmidadeDoSolo[numeroBomba] > UMIDO){
        Serial.print("Executando rega branda: ");Serial.print(my_millis());Serial.print(" > ");Serial.println(indicadores[numeroBomba].regaForte - (indicadores[numeroBomba].regasBrandasRemanescentes * INTERVALO_DE_TEMPO_ENTRE_REGAS_BRANDAS));
        digitalWrite(bomba[numeroBomba], LOW);
        delay(4500);
        digitalWrite(bomba[numeroBomba], HIGH);
      }
      readUmidadeETemperaturaAmbiente();      
      ajustarHorarioDaProximaRegaBranda(numeroBomba);
      irrigou[numeroBomba]=++irrigou[numeroBomba];
    }
 
}


void ajustaHorarioDaProximaRegaForte(int numeroBomba){
      Serial.print(">>>>>>>>> Última rega forte atual:");Serial.println(indicadores[numeroBomba].regaForte);
      indicadores[numeroBomba].regaForte = my_millis() + INTERVALO_DE_TEMPO_ENTRE_REGAS_FORTES;      
      Serial.print(">>>>>>>>> Setando próxima rega forte para:");Serial.println(indicadores[numeroBomba].regaForte);
      EEPROMWritelong(0, indicadores[numeroBomba].regaForte);  
}


void ajustarHorarioDaProximaRegaBranda(int numeroBomba){        
      indicadores[numeroBomba].regasBrandasRemanescentes = indicadores[numeroBomba].regasBrandasRemanescentes-1;
      indicadores[numeroBomba].regasBrandasRemanescentes = indicadores[numeroBomba].regasBrandasRemanescentes>0?indicadores[numeroBomba].regasBrandasRemanescentes:0;
}

void acionarUmidificador(){
}

void acionarVentiladores(){   
}

long EEPROMReadlong(long address) {
  long four  = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two   = EEPROM.read(address + 2);
  long one   = EEPROM.read(address + 3); 
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}

void EEPROMWritelong(int address, long value) {
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);
  
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
}

long my_millis(){
  return horaInicial  + millis();
}

long convertToMinutes(long timeinmillis){
  return timeinmillis/(60l*1000l); 
}

 
