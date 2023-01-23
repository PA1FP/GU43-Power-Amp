


/* GU43 Power-Amplifier software 
 * de software ondersteund de volgende zaken:
 * Controle Hoogspanning / Anodestroom / Roosterspanning / Roosterstroom Stuurrooster spanning / stuurrooster stroom.
 * controlle alle gebruikte gelijkspanningen
 * buzzer bij fouten
 * Tune en Load condensator en rolspoel auto instellingen door stappen motoren.
 * dynamische ventilator regeling door dimmer regeling met nul doorgangs detectie.
 * inrush inschakelen hoogspanningstrafo door 2 relais met serie weerstanden.
 * aparte led indicatie voor alle HF Banden.
 * error led
 * LCD / I2C Display met menu's
 * Jog-shuttle voor handmatige bediening van de drie stappenmotoren.
 * E-eprom memory voor opslaan laatste gebruikte instellingen.
 * 
 * 
 * 
 * 
 * 
 * Auteur: Frank Pereboom PA1FP
 * Versie 1.0  23-08-2021
 * 
 * 
 * 
 * 
 * 
 */
#include <LiquidCrystal_I2C.h>
#include <AccelStepper.h>



LiquidCrystal_I2C lcd(0x27, 20, 4); // I2C address 0x27, 20 column and 4 rows

// constants won't change. Used here to set a pin number :
// INPUTS:
const int rotary_a =2;            //rotary encoder kanaal A
const int rotary_b =3;            //rotary encoder kanaal B
const int on_off = 12;            //aan/uit drukknop om toestel te activeren
const int left =13;               //linker pijl toets
const int right =22;              //rechter pijltoets
const int up =23;                 //up pijltoets
const int down =24;               //down pijltoets
const int enter=25;               //enter toets
const int bandknop=26;            //band toets
const int tune=27;                //Tune om stappenmotoren af te stemmen
const int tx_on=41;               //wanneer zender op tx staat is deze input actief
const int tune_zero =43;           //tune condensator referentie stand
const int load_zero =44;           //load condensator referentie stand

//OUTPUTS:

const int coil_speed=8;           //rolspoel motor snelheid
const int motpower_on=10;         //relais om voeding stappenmotoren en rolspoel in te schakelen.
const int tien_meter=28;          //band-ledje voor front
const int twaalf_meter=29;        //band-ledje voor front
const int vijftien_meter=30;      //band-ledje voor front
const int zeventien_meter=31;     //band-ledje voor front
const int twintig_meter=32;       //band-ledje voor front
const int dertig_meter=33;        //band-ledje voor front
const int veertig_meter=34;       //band-ledje voor front
const int zestig_meter=35;        //band-ledje voor front
const int tachtig_meter=36;       //band-ledje voor front
const int honderdzestig_meter=37; //band-ledje voor front
const int alarm_led=38;           //alarm ledje bij fouten
const int main_on=39;             //Hoofdrelais om HV trafo in te schakelen
const int main_delay=40;          //Hoofdrelais veraagd inschakelen met serie weerstanden voor hv trafo
const int bypass_relay=42;        //wanneer we op TX gaan wordt deze output hoog om vucuum relais te schakelen.
const int coil_cw=45;             //rolspoel rechts om
const int coil_ccw=46;            //rolspoel links om
const int runled=47;              //actief knipperledje toggle om de 100ms
const int fan=48;                 //koel ventilator
const int buzzer=49;              // buzzer die pulserend geluid produceerd bij errors.
const int G1VL=50;                //rooster spanning regelen
const int g1_inh=51;              //g1 rooster aan zetten
const int g1_enable=52;           //g1 rooster aan zetten
const int g2_enable=53;           //g2 spanning aanzetten

//CONSTANTEN:
const int opwarmtijd=5;    //deze tijd in seconden hoe lang we gaan opwarmen

//VARIABELEN:
int opwarm_timer;    //wanneer deze tijd om is,is de buis opgewarmd
int sec_ticker, sec_ticker_security = -1;      //geeft elke seconde een tik voor diverse timers te gebruiken
int seconden_ticker; //nodig voor sec_merker
int sec_merker;      //geeft bij elke seconde een toggle
int temp_ms_ticker=0;
int ms_ticker;       //geeft elke 100ms een tik, wordt gebruikt als interrupt voor naukeurige timer.
int delay_ticker=0;  // timertje voor 2e mainrelais in te schakelen
int fan_timer=0;     // ventilator gaat na inschakelen aan
int pwm_coil=127;    //deze waarde bepaald de snelheid van de spoel
int coil_pot;        //analoge waarde van de rolspoel-potmeter
int coil_preset;     //variabele om elke band de juiste stand van de spoel te krijgen
int tune_preset;     //stand tune condensator
int load_preset;     //stand load condensator
int ok_knop_timer;   //bepalend wanneer de ok knop actie moet zijn
int ok_knop_actief;  //bepalend wanneer de ok knop actie moet zijn
bool warmingup;      //merker als de buis op is gewarmd
bool error;          //wanneer deze merker op 1 staat, geeft de buzzer een pulserende toon.
bool aan_knop=0;     //merker wanneer er een toets is ingedrukt
bool knop_flank=0;   //merker wanneer er een toets is losgelaten
bool band_flank;     //flank detectie band knopje
bool band;           //merker als band-knop is ingedrukt
int  bandkeuze;      //geeft op het display welke band er is gekozen
bool op;             //merker als up knop is ingedrukt
bool neer;           //merker als down knop wordt ingedrukt
bool links;          //merker als links wordt ingedrukt
bool links_flank;
bool rechts;         //merker als rechts wordt ingedrukt
bool rechts_flank;
bool ok;             //merker als enter wordt ingedrukt
byte band_counter;     //teller om te bepalen op welke band we zitten
bool bedrijf=0;      //merker of toestel in bedrijf is.
bool tune_zero_switch; //tune condensator referentie schakelaar
bool load_zero_switch; //load condensator referentie schakelaar
bool tune_knop;        //tune knopje
bool motor_tune_mode;
byte handmode_keuze=1;   //getal die aangeeft welke condensator of spoel we willen bedienen in handmode


//---------------------------------------------------------------------------------------------------------
void setupTimer1() {   //100ms timer activeren op interrupt
  noInterrupts();
  // Clear registers
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  // 10 Hz (16000000/((6249+1)*256))
  OCR1A = 6249;
  // CTC
  TCCR1B |= (1 << WGM12);
  // Prescaler 256
  TCCR1B |= (1 << CS12);
  // Output Compare Match A Interrupt Enable
  TIMSK1 |= (1 << OCIE1A);
  interrupts();
}


//----------------------------------------------------------------------------------------------------------
void setup() {
//INPUTS:
 
  
  
  pinMode (rotary_a, INPUT);
  pinMode (rotary_b, INPUT);
  pinMode (on_off, INPUT);
  pinMode (left, INPUT);
  pinMode (right, INPUT);
  pinMode (up, INPUT);
  pinMode (down, INPUT);
  pinMode (enter, INPUT);
  pinMode (bandknop, INPUT);
  pinMode (tune, INPUT);
  pinMode (tx_on, INPUT);
  pinMode (tune_zero, INPUT);
  pinMode (load_zero, INPUT);
  
//---------------------------------------------------
//OUTPUTS: 
 
 
  pinMode(motpower_on, OUTPUT);
  pinMode(tien_meter, OUTPUT);
  pinMode(twaalf_meter, OUTPUT);
  pinMode(vijftien_meter, OUTPUT);
  pinMode(zeventien_meter, OUTPUT);
  pinMode(twintig_meter, OUTPUT);
  pinMode(dertig_meter, OUTPUT);
  pinMode(veertig_meter, OUTPUT);
  pinMode(zestig_meter, OUTPUT);
  pinMode(tachtig_meter, OUTPUT);
  pinMode(honderdzestig_meter, OUTPUT);
  pinMode(alarm_led, OUTPUT);
  pinMode(main_on, OUTPUT);
  pinMode(main_delay, OUTPUT);
  pinMode(bypass_relay, OUTPUT);
  pinMode(coil_cw, OUTPUT);
  pinMode(coil_ccw, OUTPUT);
  pinMode(coil_speed, OUTPUT);
  pinMode(runled, OUTPUT);
  pinMode(fan, OUTPUT);
  pinMode(buzzer, OUTPUT);

band_counter = 4;            //voor het gemak naar 4, straks wordt dit naar de EEprom geschreven




#define motorInterfaceType 1        //stappendriver definieren
#define STEPPER1_DIR_PIN 4          //Tune Condensator
#define STEPPER1_STEP_PIN 5         //Tune Condensator
#define STEPPER2_DIR_PIN 6          //Load Condensator
#define STEPPER2_STEP_PIN 7         //Load Condensator

//------------------------------------------------------------------------------ 
  

  
  setupTimer1();
  Serial.begin(9600);
  lcd.init();                          // initialize the lcd
  lcd.backlight();                     //backlite aan
  lcd.clear();                         //Clean the screen 

 
  
}

//-------------------------------------------------------------------------------------------
void loop() {

coil_pot = analogRead(A8); //potmeter van rolspoel

int hoogspanning = analogRead(A0); //controle of we de juiste hoogspanning hebben.
int anodestroom = analogRead(A1); //anode stroom in beeld brengen.
int G1_current = analogRead(A2); //check stuurrooster spanning.
int G2_current = analogRead(A3); //check stuurrooster stroom.
int swr_fwd = analogRead(A4); //uitgaand vermogen meten.
int swr_refl = analogRead(A5); //retourgaand vermogen meten.
int air_temp = analogRead(A6); //gemeten uitgeblazen temperatuur koeling buis.
int gloeispanning = analogRead(A7); //controle of we de juiste gloeispanning hebben.
int vierentwintigvolt = analogRead(A13); //controle 24.
int twaalfvolt = analogRead(A14); //controle 12V.
int vijfvolt = analogRead(A15); //controle 5V.



tune_zero_switch = digitalRead (tune_zero);   //referentieschakelaar tune condensator uitlezen
load_zero_switch = digitalRead (load_zero);   //referentieschakelaar load condensator uitlezen
tune_knop = digitalRead (tune);             //tune knopje uitlezen
ok = digitalRead (enter);


if (ok == 0 )
{
 ++ ok_knop_timer;

}
else if (ok == 1)                        //als we langer als 3 sec de ok knop ingedrukt houden gaan we naar het motor_tune menu!
{
  ok_knop_timer = 0;
}

if (ok_knop_timer > 30 && motor_tune_mode == 0)
{
  lcd.clear();
  ok_knop_timer = 0;
  motor_tune_mode = 1;
  motoren_tunen();
}

lcd.setCursor(0,0); //Defining positon to write from first row,first column .
lcd.print("PA1FP HOORN- GU-43 -");
lcd.setCursor(0,3);
lcd.print("BAND:");
lcd.setCursor(8,3);
lcd.print("METER");
lcd.setCursor(5,3);
lcd.print(bandkeuze);
lcd.setCursor(0,2);
lcd.print("COIL:");
lcd.setCursor(5,2);
lcd.print(coil_pot);
lcd.setCursor(12,1);
lcd.print(coil_preset);
lcd.setCursor(0,1);
lcd.print("COIL-PRESET:");

lcd.setCursor(18,2);
lcd.print(sec_merker);



if (sec_ticker > opwarmtijd)    // tellen totdat de opwarmtijd is afgelopen, dan merker zetten dat we zijn opgewarmd.
{
  
  warmingup = 1;
  digitalWrite (fan, 1 );       //na opwarmen mag de ventilator gelijk aan, en gaat ook niet meer uit.
  opwarm_timer = 0;
}

//--------------------------------------------------------------------------------------------------------------------------------------
band = digitalRead(bandknop);                                       //wanneer het bandknopje is ingedrukt
aan_knop = digitalRead (on_off);                                    //we gaan het toestel in bedrijf zetten
if (aan_knop ==0 && knop_flank==0 && bedrijf==0 && warmingup==1)    //LET OP!! de druktoetsen schakelen naar massa bij indrukken, en worden dus 0 bij indrukken!

{
  knop_flank=1;                                 //merker zetten dat de knop is ingedrukt
}

if (aan_knop ==1 && bedrijf ==0 && knop_flank==1 && warmingup==1)
{
  
  
  sec_ticker_security = sec_ticker;
                                             //en vervolgens als we de aan knop loslaten en de flank  id
  knop_flank=0;                              //hoog dan mogen we het toestel in bedrijf nemen na het aftellen van de sec ticker
}
if (aan_knop ==0 && knop_flank==0 && bedrijf==1)

{
  
  knop_flank=1;                              //nu weer vice-versa, eerst de knop indrukken voor de flank
}

if (aan_knop ==1 && knop_flank==1 && bedrijf==1)

{
  knop_flank=0;
  bedrijf=0;                                 //en als deze wordt losgelaten zetten we alles uit.
  digitalWrite (main_on, 0);
  digitalWrite (main_delay, 0);
  delay_ticker=0;
  sec_ticker_security = -1;                  //hier zetten de waarde op -1 om de buzzer uit te zetten
}

if (bedrijf==1 && delay_ticker>1)            //als de delay_ticker waarde groter is als 100ms komt het delay relais op.  
{

 digitalWrite (main_delay, 1); 
}
if (sec_ticker_security > -1 && sec_ticker - sec_ticker_security >= 2) //een aantal seconden buzzeren voordat we de boel gaan aanzetten.
{
  digitalWrite(main_on, 1);
  bedrijf = 1;
  sec_ticker_security = -1;                  //hier zetten de waarde op -1 om de buzzer uit te zetten
  digitalWrite(buzzer, 0);
}
//------------------------------banden knopje laten bepalen op welke band we staan--------------------------------------------------

if (band ==0)
{
  band_flank =1;
  
  }
if (band ==1 && band_flank ==1)
{
  ++band_counter;                            //bij elke druk op de knop na loslaten telt de band counter er eentje bij voor de band keuze
  band_flank =0;                             //gelijk weer op nul om de volgende keer niet meer er bij op te tellen
  
}

if (band_counter > 10)
{
  band_counter = 1;
 
}

if (band_counter ==1)
{
  bandkeuze = 10;
  digitalWrite(tien_meter, 1);
  tune_preset = 3000;
  load_preset = 1500;
  coil_preset = 550;
}
else
{
 digitalWrite(tien_meter, 0);
 
}
if (band_counter ==2)
{
  bandkeuze = 12;
  digitalWrite(twaalf_meter, 1);
  tune_preset = 3000;
  load_preset = 1500;
  coil_preset = 770;
}
else
{
 digitalWrite(twaalf_meter, 0);
}
if (band_counter ==3)
{
  bandkeuze = 15;
  digitalWrite(vijftien_meter, 1);
  tune_preset = 3000;
  load_preset = 1500;
  coil_preset = 678;
}
else
{
 digitalWrite(vijftien_meter, 0);
}
if (band_counter ==4)
{
  bandkeuze = 17;
  digitalWrite(zeventien_meter, 1);
  tune_preset = 3000;
  load_preset = 1500;
  coil_preset = 480;
}
else
{
 digitalWrite(zeventien_meter, 0);
}
if (band_counter ==5)
{
  bandkeuze = 20;
  digitalWrite(twintig_meter, 1);
  tune_preset = 3000;
  load_preset = 1500;
  coil_preset = 250;
}
else
{
 digitalWrite(twintig_meter, 0); 
}
if (band_counter ==6)
{
  bandkeuze = 30;
  digitalWrite(dertig_meter, 1);
  tune_preset = 3000;
  load_preset = 1500;
  coil_preset = 275;
}
else
{
 digitalWrite(dertig_meter, 0); 
}
if (band_counter ==7)
{
  bandkeuze = 40;
  digitalWrite(veertig_meter, 1);
  tune_preset = 3000;
  load_preset = 1500;
  coil_preset = 500;
}
else
{
 digitalWrite(veertig_meter, 0); 
 
}
if (band_counter ==8)
{
  bandkeuze = 60;
  digitalWrite(zestig_meter, 1);
  tune_preset = 3000;
  load_preset = 1500;
  coil_preset = 350;
}
else
{
 digitalWrite(zestig_meter, 0); 
}
if (band_counter ==9)
{
  bandkeuze = 80;
  digitalWrite(tachtig_meter, 1);
  tune_preset = 3000;
  load_preset = 1500;
  coil_preset = 450;
}
else
{
 digitalWrite(tachtig_meter, 0); 
}
if (band_counter ==10)
{
  bandkeuze = 160;
  digitalWrite(honderdzestig_meter, 1);
  tune_preset = 3000;
  load_preset = 1500;
  coil_preset = 650;
}
else
{
 digitalWrite(honderdzestig_meter, 0); 
}
//----------------------------------------------------------------------------------------------------------------------------------------


if (tune_knop ==0)
{
  lcd.clear();
  Motoren_Nullen();                         //hier gaan we alle stappenmotoren op nul-stand zetten
}


}
//------------------------------ 100Msec. Interrupt ----------------------------------------------------------------
ISR(TIMER1_COMPA_vect) {
 tune_zero_switch = digitalRead (tune_zero);
  digitalWrite(runled, digitalRead(runled) ^ 1);     //Runledje 100Ms laten toggelen
  ++ ms_ticker;                                      // 100ms teller optellen.
  if (ms_ticker > 10)
  {
    int temp_sec_ticker = sec_ticker;
    sec_ticker = ms_ticker /10;                      //sec tellertje laten lopen voor diverse doeleinden
    if (temp_sec_ticker < sec_ticker) opwarm_timer = opwarm_timer + (sec_ticker - temp_sec_ticker);
  }
  ++ seconden_ticker;
  if (seconden_ticker > 10)
  {
    seconden_ticker = 0;
   
  }
  if (seconden_ticker > 0 & seconden_ticker <6)     //sec_merker op 1 zetten elke keer als er een seconde verder is
  {
    sec_merker =1;
  }
 else 
 {
  sec_merker=0;
 }
  
  if (bedrijf==1 && delay_ticker < 5)               //hier de teller niet onnodig hoog door laten lopen 
  {
    delay_ticker ++;                                //optellen om vertraagd het main relais in te schakelen.
  }
  if (sec_ticker_security > -1)
  {
    int buzzer_value = ms_ticker % 2 || error==1;  //als we de waarde van ms_ticker delen door modulo 2 (procent teken) komt er bij een oneven getal altijd een 1 uit,
    digitalWrite(buzzer, buzzer_value);            //en een even getal is dat altijd 0. B.V. 131 % 131 = 1 en 130 %130 =0.
                                                   //hier laten we dus een buzzer pulserend op 100ms zoemen.
  }
//coil_pot = analogRead(A8); //potmeter van rolspoel uitlezen
}

//------------------------------STAPPENMOTOREN EN ROLSPOEL IN NULSTAND ZETTEN-----------------------------------------------------------------------------------------------------


void Motoren_Nullen() {
lcd.setCursor(0,0); 
lcd.print("ALL MOTORS TO PRESET"); 

digitalWrite(motpower_on, 1); //relais voor voeding en rolspoel aanzetten.
delay(200);                   // we wachten even om de motoren te laten draaien
AccelStepper stepper1 = AccelStepper(motorInterfaceType, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);

    stepper1.setMaxSpeed(2000.0);
    stepper1.setAcceleration(100.0);
    stepper2.setMaxSpeed(2000.0);
    stepper2.setAcceleration(100.0);
   
 
  
  //stepper1.setCurrentPosition(0);
  // stepper1 laten draaien tot we de eindschakelaar zien
  while(tune_zero_switch ==0) 
  {
    tune_zero_switch = digitalRead (tune_zero); //eindschakelaar uitlezen
    stepper1.setSpeed(-2000);
    stepper1.runSpeed();
    stepper1.run();
     
  }
 delay(500);
  stepper1.setCurrentPosition(0); // positie stepper1 op nul zetten

//nu de tweede stepper

  while(load_zero_switch ==0) 
  {
    load_zero_switch = digitalRead (load_zero);//eindschakelaar uitlezen
    stepper2.setSpeed(-2000);
    stepper2.runSpeed();
    stepper2.run();
     
  }
 delay(500);
  stepper2.setCurrentPosition(0); // positie stepper2 op nul zetten

  // nu gaan we de stappenmotor draaien naar de positie die bij elke band hoort
 
  while(stepper1.currentPosition() != (coil_preset))
  
  {
  stepper1.setSpeed(3500);
  stepper1.runSpeed();
 
  }

delay(500);
 //stepper2.setCurrentPosition(0); // positie stepper2 op nul zetten

  // nu gaan we de stappenmotor draaien naar de positie die bij elke band hoort
 
  while(stepper2.currentPosition() != (coil_preset))
  
  {
  stepper2.setSpeed(3500);
  stepper2.runSpeed();
 
  }
delay(500);

//nu gaan we de rolspoel op positie zetten

analogWrite (coil_speed, 255);   //de motorsnelheid instellen

//coil_pot = analogRead(A8); //potmeter van rolspoel uitlezen
while (coil_pot < coil_preset)
{
coil_pot = analogRead(A8); //potmeter van rolspoel uitlezen
digitalWrite (coil_ccw, 0);
digitalWrite (coil_cw, 1);

if (coil_pot >= coil_preset -40 && coil_pot <= coil_preset +40)
{
  analogWrite (coil_speed, 180);   //de motorsnelheid instellen
}
if (coil_pot >= coil_preset -20 && coil_pot <= coil_preset +20) //binnen deze range gaat de motor langzamer draaien
{
  analogWrite (coil_speed, 150);   //de motorsnelheid instellen
}

}

while (coil_pot > coil_preset)
{
coil_pot = analogRead(A8); //potmeter van rolspoel uitlezen
digitalWrite (coil_cw, 0);
digitalWrite (coil_ccw, 1);
if (coil_pot >= coil_preset -40 && coil_pot <= coil_preset +40) //binnen deze range gaat de motor langzamer draaien
{
  analogWrite (coil_speed, 180);   //de motorsnelheid instellen
}
if (coil_pot >= coil_preset -20 && coil_pot <= coil_preset +20) //binnen deze range gaat de motor langzamer draaien
{
  analogWrite (coil_speed, 150);   //de motorsnelheid instellen
}

}
digitalWrite (coil_ccw, 0); //rolspoel motor uit zetten
digitalWrite (coil_cw, 0);  //rolspoel motor uit zetten
digitalWrite(motpower_on, 0); //relais voor voeding en rolspoel uitzetten.  
lcd.setCursor(5,2);
lcd.print(coil_pot);

delay(1000);

lcd.clear();
loop();   
//}






}

//-------------------------handmatig motoren laten draaien om af te stemmen-----------------------------------------

//door in het menu te selecteren op handafstemming kan je de stappenmotoren en rolspoel laten draaien om op de juiste afstem stand te komen.
//als de juiste stand is bereikt kan de betreffende standen van de spoel en condensatoren worden opgeslagen in de EEprom.
AccelStepper stepper1 = AccelStepper(motorInterfaceType, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
AccelStepper stepper2 = AccelStepper(motorInterfaceType, STEPPER2_STEP_PIN, STEPPER2_DIR_PIN);


void motoren_tunen() {


links = digitalRead (left);
rechts = digitalRead (right);
op = digitalRead (up);
neer = digitalRead (down);
ok = digitalRead (enter);
coil_pot = analogRead(A8); //potmeter van rolspoel


 
   
//stepper1.setCurrentPosition(0);
  
  
  digitalWrite(motpower_on, 1); //relais voor voeding en rolspoel aanzetten.
//lcd.clear();   //even het scherm leeg maken

lcd.setCursor(0,0); 
lcd.print("-- HANDBEDIENING --");
lcd.setCursor(0,2); 
lcd.print("TUNE:");
lcd.setCursor(0,3); 
lcd.print("LOAD:");
lcd.setCursor(5,2);
lcd.print(stepper1.currentPosition());     //Actuele positie aangeven van de stappenmotor
lcd.setCursor(5,3);
lcd.print(stepper2.currentPosition());     //Actuele positie aangeven van de stappenmotor
lcd.setCursor(11,2); 
lcd.print("COIL:");
lcd.setCursor(16,2);
lcd.print(coil_pot);
lcd.setCursor(11,3); 
lcd.print("BAND:");
lcd.setCursor(16,3);
lcd.print(bandkeuze);


if (handmode_keuze ==1)
{
lcd.setCursor(1,1); 
lcd.print("TUNE-CONDENSATOR"); 
}
if (handmode_keuze ==2)
{
 lcd.setCursor(1,1); 
lcd.print("LOAD-CONDENSATOR"); 
}
if (handmode_keuze ==3)
{
 lcd.setCursor(1,1); 
lcd.print(" ROLLER-INDUCTOR "); 
}

if (rechts ==0)
{
  rechts_flank =1;
  
  }
if (rechts ==1 && rechts_flank ==1)
{
  ++handmode_keuze;                            //bij elke druk op de knop na loslaten telt de handmode_keuze er eentje bij voor keuze welke condensator of spoel je kiest voor handbed.
  rechts_flank =0;                             //gelijk weer op nul om de volgende keer niet meer er bij op te tellen
}

if (handmode_keuze > 3)                        //keuze uit: 1= tune condensator 2=load condensator 3=rolspoel
{                                              //dus hoger als stand 3 gaan we weer naar stand 1
  handmode_keuze = 1;
}


while (op ==0 && handmode_keuze ==1)    //tune condensator bedienen
  {
    stepper1.setMaxSpeed(600.0);
    stepper1.setAcceleration(10000.10);  
    stepper1.setSpeed(-1000);           //linksom draaien
    stepper1.runSpeed();
    stepper1.run();
    op = digitalRead (up);              //blijven uitlezen of de toets is ingedrukt
  }

while (neer ==0 && handmode_keuze ==1)  //tune condensator bedienen
  {
    stepper1.setMaxSpeed(600.0);
    stepper1.setAcceleration(100.0);  
    stepper1.setSpeed(1000);             //rechtsom draaien
    stepper1.runSpeed();
    stepper1.run();
    neer = digitalRead (down);          //blijven uitlezen of de toets is ingedrukt
  }

while (op ==0 && handmode_keuze ==2)    //load condensator bedienen
  {
    stepper2.setMaxSpeed(1000.0);
    stepper2.setAcceleration(100.0);  
    stepper2.setSpeed(-1000);           //linksom draaien
    stepper2.runSpeed();
    stepper2.run();
    op = digitalRead (up);              //blijven uitlezen of de toets is ingedrukt
  }

while (neer ==0 && handmode_keuze ==2) 
  {
    stepper2.setMaxSpeed(1000.0);
    stepper2.setAcceleration(100.0);  
    stepper2.setSpeed(1000);             //rechtsom draaien
    stepper2.runSpeed();
    stepper2.run();
    neer = digitalRead (down);           //blijven uitlezen of de toets is ingedrukt
  }

while (op ==0 && handmode_keuze ==3) 
  {
    digitalWrite (coil_cw, 1);
    analogWrite (coil_speed, 180);
    op = digitalRead (up);               //blijven uitlezen of de toets is ingedrukt
    coil_pot = analogRead(A8); //potmeter van rolspoel
    lcd.setCursor(16,2);
    lcd.print(coil_pot);
  }

while (neer ==0 && handmode_keuze ==3)  //rolspoel bedienen
  { 
    digitalWrite (coil_ccw, 1);          //rechtsom draaien
    analogWrite (coil_speed, 180);      //linksom draaien
    neer = digitalRead (down);           //blijven uitlezen of de toets is ingedrukt
    coil_pot = analogRead(A8); //potmeter van rolspoel
    lcd.setCursor(16,2);
    lcd.print(coil_pot);
  }
digitalWrite (coil_cw, 0);              //draairichting uitzetten
digitalWrite (coil_ccw, 0);

if (ok == 0 )
{
 ++ ok_knop_timer;

}
else if (ok == 1)
{
  ok_knop_timer = 0;
}

if (ok_knop_timer > 30)                 //langer als 3 sec ok indrukken gaan we weer terug naar de loop en uit de tune mode
{
  lcd.clear();
   ok_knop_timer = 0;
  motor_tune_mode = 0;
 
  
}

if (motor_tune_mode == 1)
{
  motoren_tunen(); 
}
else
{
  digitalWrite(motpower_on, 0); //relais voor voeding en rolspoel uitzetten.
  loop();
}
 
}
