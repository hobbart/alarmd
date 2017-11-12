//*************************************************************************************
// alarmd_22.c
//
// Meine Alarmanlage :-)
// ---------------------
//
// Verteilte Funktionalität in Alarm-Daemon, und RFID-Daemon
// Mit 2 USB-Kameras, SMS-Versendung bei Alarm
// USB RFID-Reader hotplug-fähig, kein Alarm bei Abziehen und Anstecken
// Signalhandler, Alarmtimer, und fread() in Child-Prozess
// Alarm einschalten bei falschem RFIF-Tag
// Variablenübergabe Child-->Parent mittels shared memory
// shared memory wird bei Programmabbruch mit Ctrl+C nicht freigegeben --> memory-leak
// Eintrag der Aktivierung und Deaktivierung von Anlage und Alarm im Syslog
// Reaktivierung der Anlage nach Stromausfall bzw. System-Boot im Vorzustand (über Statusdatei)
// Programmablauf: Volle Funktionalität als default, Simulation mit Schalter -s
// mit Alarmaktivierung und Deaktivierung via SIGUSR2 für Wartungs- und Entwicklungszwecke
// ACHTUNG: Für korrekte Funktion muss das Signal SIGUSR2 an den Kind-Pzozess gesendet werden
// einstellbare Alarm-Verzögerung für Abschaltmöglichkeit bei Türeintritt mit RFID-Tag
// SMS-Versendung bei Abschaltung eines laufenden Alarms mit RFID-Tag (nach Countdown-Ablauf)
// Zuordnung von Personen zu RFID-Nummern
// V0.10 Test ob schon ein alarmd Task läuft -->Info-->exit(1)
// Option -h mit Hilfetext eingefügt
// V0.11 Konfiguration mit config-file alarmd.conf
// V0.12 (zusätzlich) konfigurierbarer Email-Versand im Alarmfall
// V0.13 einstellbare Einschalt-Verzögerung zum Verlassen der Wohnung (nach Anlagenaktivierung)
// Separate Aktivierung/Deaktivierung von SMS und/oder Email-Versand per config-file
// V0.14 BUGFIX: RFID-Tag Nummer bei der 2. Alarm-Aktivierung falsch --> korrigiert
// Prozess-Name von Kind-Prozess (RFID-Daemon) auf "rfidd" gesetzt
// Label für syslog-Einträge aus Kind-Prozess (RFID-Daemon) auf "rfidd" gesetzt
// V0.15 Ausgabe Blitzlicht, Sirene und Flurlicht-Schaltung über GPIO und Relaisplatine
// V0.16 PIR-Sensoren Abfrage mit Flankendetektion, 8sec-timeout, separate Threads und Signalisierung über Variable pir_flag
// V0.17 Geändert: rfidd als Thread, keine Signale, kein IPC-Mem, main-Endlosschleife anpassen
// V0.18 kleinere Fehler bereinigt, Sensoren wieder eingepflegt
// LED_OK, LED_ACTIVE, LED_ALARM Ausgabe mit separaten Threads über GPIO
// V0.19 BUGFIX: Syslog-Eintrag für Aufwärmphase, Ausgabe von Sensornummern statt GPIO-Pins
// Flurlichtschaltung als Impuls, CTRL-C Handler für Beendigung der Hauptschleife
// durch Setzen der Schleifenvariable auf 0, Aufräumen in Funktion cleanup() mit
// Syslog-Eintrag, GPIO und Kamera-Deaktivierung, Beendigung aller laufenden Threads
// sowie reguläre Programmbeendigung bei Abbruch mit CTRL-C oder killall-Befehl
// V0.20 bei Initialisierung der State-Machine direkt nach Zustand "Ein" springen
// V0.21 BUGFIX: bei Start Alarm-Countdown Sensor-Nummer im Syslog mit ausgeben
// bei Alarm-Abbruch mit richtigem RFID-Tag #9 Kameras und alle Relais abschalten
// in State-Machine Übergang #7 Bedingung pir_flag=1 gelöscht
// in State-Machine Übergang #11 hinzugefügt: sofortige Alarm-Aktivierung bei
// Auflegen von falschem RFID-Tag im Zustand "Anlage ein"
// in State-Machine Übergänge #5, #7 rfid_bad=1 gelöscht
// V0.22 BUGFIX: GPIO-Eingänge als Pegel-aktiv konfigurieren (nicht mehr Flanken-aktiv, da zu störempfindlich)
// 2x malige Abfrage der Sensoren mit 200ms Watezeit zwischen den Abfragen zur Vermeidung von Fehlalarm durch transiente Störimpluse
//*************************************************************************************

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <pthread.h>
#include <linux/hidraw.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/shm.h>
#include <sys/prctl.h>
#include <fcntl.h>
#include <math.h>
#include <errno.h>
#include <syslog.h>
#include <unistd.h>
#include "iniparser.h"
#include "gpiolib.h"

#define GPIO_BLITZLICHT 18
#define GPIO_SIRENE 23
#define GPIO_FLURLICHT 24
#define GPIO_RELAIS_4 25
#define GPIO_LED_OK 12
#define GPIO_LED_ACTIVE 16
#define GPIO_LED_ALARM 20

//**************************************************
//Funktionsdeklarationen
//**************************************************
void ctrl_c_handler(int signal);
void syslog_init();
void config_init();
void memory_init();
int hardware_init();
int start_cam();
int stop_cam();
int send_sms(char sms_text[]);
int send_email(char email_text[]);
void status_init();
void status_write();
void status_close();
void signal_handler(int signal);
int check_options(int argc, char * argv[]);
void check_alarmd_task();
void blitzlicht_ein();
void blitzlicht_aus();
void sirene_ein();
void sirene_aus();
void flurlicht_ein();
void flurlicht_aus();
void start_threads();
void *pir_read(void *threadid);
void *rfidd();
void *led_out();
void cleanup();
//**************************************************
//Globale Variablen
//**************************************************
//Initialisierung aus config-file
int sim; //Schalter für Simulation des Programmblaufs: 0=volle Funktionalität (default) 1=nur Simulation
const char *work_dir; //Arbeitsverzeichnis für Kameras
const char *status_file;  //Datei mit Anlagenstatus
const char *tel_nr; //Tel-Nr. für Alarm-SMS
int sw_sms; //Schalter für SMS-Versand
int watchdog_time; //Zeitdauer in sec für Syslogeinträge, dass Programm noch läuft
int alarm_verzoegerung; //Alarmverzögerung in sec
int alarm_dauer; //Alarmdauer in sec (max. 3min erlaubt)
int sensoren[256]; //Sensorenliste Zuordnung zu GPIO-Pins
int cam1_brightness; //Helligkeit Kamera 1
int cam2_brightness; //Helligkeit Kamera 2
const char *rfid_device; //RFID-Reader device
int tag_anzahl; //Anzahl bekannter RFID-Tags
int rfid_tags[256]; //bekannte RFID-Tags
char personen[256][50]; //bekannte Personen
const char *email_addr; //Email-Adresse für Alarm-Email
int sw_email; //Schalter für Email-Versand
int einschalt_verzoegerung; //Zeitdauer für Einschalt-Verzögerung in sec
int sw_blitzlicht; //Schalter für Blitzlicht
int sw_sirene; //Schalter für Sirene & Blitzlicht
int sw_flurlicht; //Schalter für Flurlicht
int sensor_anzahl; //Anzahl installierter Sensoren
int sensor_warmup; //Aufwärmzeit für PIR-Sensoren (default 60sec)
//Initialisierung hier
int run = 1; //Schleifenvariable der Hauptschleife
char config_file[20]="/etc/alarmd.conf"; //Config-File
char child_proc_name[]="rfidd"; //Name von Kind-Prozess
dictionary *config; //Struktur für alle geparsten Config-Einträge
int ret; //Rückgabewert-Wert von Funktionen
char str_buf[50]; //Buffer für diverse Strings
char text[160]; //Buffer für SMS-Text
char command[200]=""; //Kommando-Zwischenspeicher
FILE *fd_rfid, *fd_status; //Filehandles für RFID-Reader und Status-Datei
char buf[176]; //Buffer für RFID-Reader
int sensor_rfid=256; //virtuelle Sensor-Nummer für Alarmaktivierung durch falsches RFID-Tag
int i, j, res = 0;
pid_t ppid; //Prozess-ID Elternprozess
pthread_t sensor_thread[256], rfidd_thread, led_thread; //Thread-Identifier Array (opaque, IEEE POSIX 1003.1c standard)
int watchdog_zaehler = 0; //Watchdog-Zähler für Syslog-Eintrag
int countdown; //Countdown für Betreten/Verlassen der Wohnung und Alarmdauer
int led_freq=4; //LED Blinkfrequenz in Hz
int led_ok_status, led_active_status, led_alarm_status; //Zustand der LEDs: 0=aus, 1=an, 2=blinkt
int help_text_zeilen = 6; //Anzahl der Zeilen für Hilfetext
char help_text[][100]={
"Aufruf: alarmd [OPTION]",
"Version 0.22",
"",
"Optionen:",
"-s	Aktiviere Simulationsmodus",
"-h	Ausgabe Hilfetext" }; //Help-Text

//Statemachine vorheriger[0] und aktueller[1] Zustand
int anlage_status[2];    //Anlagenstatus: 0=aus, 1=ein
int anlage_countdown[2]; //Anlagen-Countdown: 0=aus, 1=ein
int alarm_status[2];     //Alarmstatus: 0=aus, 1=ein, >1 Alarmanzahl
int alarm_countdown[2];  //Alarm-Countdown: 0=aus, 1=ein

time_t alarm_zeit; //Alarmzeit: Zeitpunkt einer Alarmaktivierung
int alarm_sensor;  //aktivierter Sensor
int rfid_tag;      //dekodierter RFID-Tag
int rfid_nr;       //Laufende Nummer in Personenliste für RFID-Tag
int rfid_good;     //RFID-Tag in Liste? 0=nein, 1=ja
int rfid_bad;      //RFID-Tag nicht in Liste? 0=nein, 1=ja
int pir_flag;      //PIR-Sensor aktiviert? 0=nein, 1=ja

//**************************************************
//Hauptprogramm
//**************************************************
int main(int argc, char *argv[])
{
//Allgemeine Initialisierungen
syslog_init(); //Syslog initialisieren
syslog(LOG_NOTICE, "Alarm-Daemon gestartet"); //Syslog-Eintrag
ppid=getpid(); //Prozess-ID des Hauptprozesses ermitteln
signal(SIGINT, ctrl_c_handler); //Signalhandler für CTRL-C initialisieren
config_init(); //Variablen-Initialisierung aus Config-Datei
check_options(argc, argv); //Parameterübergabe an main auswerten
check_alarmd_task(); //Test ob schon ein alarmd-Task läuft-->Info-->exit(100)
//Anlagen-Initialisierung
led_ok_status=2; //grüne LED blinkt = Anlagen-Initialisierung
hardware_init(); //Hardware-Initialisierung (GPIO, Kameras)
status_init(); //Anlagenstatus initialisieren (Zustand vor Neustart)
start_threads(); //Threads erzeugen und starten

//   #0 Initialisierung der State-Machine
  led_ok_status=1; //grüne LED ein = Software läuft
  if(anlage_status[0]) { //falls Anlage eingeschaltet war --> wieder einschalten
  anlage_status[0]=0; //bei Zustand "Anlage aus" starten
  anlage_countdown[0]=1; //Einschalt-Countdown läuft
  countdown=0; //und ist gerade beendet
  }

  while(run) { //Hauptprogramm in Endlosschleife wird jede Sekunde 1x aufgerufen
//   #1 wenn Anlage erstmalig aktiviert wird
if(!anlage_status[0] && !anlage_countdown[0] && !alarm_status[0] && !alarm_countdown[0] && rfid_good) {
        anlage_countdown[1]=1; //neuen Anlagenzustand schreiben
        countdown=einschalt_verzoegerung; //Einschalt-Countdown starten
        rfid_good=0; //Trigger löschen
        led_ok_status=1; //grüne LED ein
        led_active_status=2; //gelbe LED blinkt
        led_alarm_status=0; //rote LED aus
        syslog(LOG_NOTICE, "Einschalt-Countdown gestartet"); //Syslog-Eintrag
     }
//   #2 Einschalt-Countdown abbrechen
if(!anlage_status[0] && anlage_countdown[0] && !alarm_status[0] && !alarm_countdown[0] && rfid_good) {
        anlage_status[1]=0; //Anlage deaktivieren
        anlage_countdown[1]=0; //Anlagen-Countdown abschalten
        rfid_good=0; //Trigger löschen
        led_ok_status=1; //grüne LED ein
        led_active_status=0; //gelbe LED aus
        led_alarm_status=0; //rote LED aus
        //countdown=0; //Zaehler rücksetzen problem mit #3
        syslog(LOG_NOTICE, "Einschalt-Countdown abgebrochen"); //Syslog-Eintrag
     }
//   #3 Einschalt-Countdown auf 0
if(!anlage_status[0] && anlage_countdown[0] && !alarm_status[0] && !alarm_countdown[0] && countdown==0) {
        anlage_status[1]=1; //Anlage aktivieren
        anlage_countdown[1]=0; //Countdown abschalten
        alarm_status[1]=0; //alle Alarme löschen
        alarm_countdown[1]=0; //Alarm-Countdown abschalten
        led_ok_status=1; //grüne LED ein
        led_active_status=1; //gelbe LED ein
        led_alarm_status=0; //rote LED aus
        syslog(LOG_NOTICE, "Anlage aktiviert, RFID-Tag: %010d, Name: %s",rfid_tag,personen[rfid_nr]); //Syslog-Eintrag
        status_write(); //aktuellen Anlagenstatus auf Disk speichern
     }
//   #4 wenn Anlage deaktiviert wird
if(anlage_status[0] && !anlage_countdown[0] && !alarm_status[0] && !alarm_countdown[0] && rfid_good) {
        anlage_status[1]=0; //Anlage deaktivieren
        rfid_good=0; //Trigger löschen
        led_ok_status=1; //grüne LED ein
        led_active_status=0; //gelbe LED aus
        led_alarm_status=0; //rote LED aus
        syslog(LOG_NOTICE, "Anlage deaktiviert, RFID-Tag: %010d, Name: %s",rfid_tag,personen[rfid_nr]); //Syslog-Eintrag
        status_write(); //aktuellen Anlagenstatus auf Disk speichern
     }
//   #5 wenn Alarm erstmalig von PIR-Sensor aktiviert wird
if(anlage_status[0] && !anlage_countdown[0] && !alarm_status[0] && !alarm_countdown[0] && pir_flag) {
        alarm_countdown[1]=1; //neuen Anlgenzustand schreiben
        countdown=alarm_verzoegerung; //Alarm-Countdown starten
        pir_flag=0; //Trigger löschen
        led_ok_status=1; //grüne LED ein
        led_active_status=1; //gelbe LED ein
        led_alarm_status=2; //rote LED blinkt
        syslog(LOG_NOTICE, "Alarm-Countdown gestartet, Sensor: %d",alarm_sensor); //Syslog-Eintrag
     }
//   #6 Alarm-Countdown abbrechen und Anlage ausschalten
if(anlage_status[0] && !anlage_countdown[0] && !alarm_status[0] && alarm_countdown[0] && rfid_good) {
        anlage_status[1]=0;  //Anlage deaktivieren
        alarm_countdown[1]=0; //Countdown abschalten
        rfid_good=0; //Trigger löschen
        led_ok_status=1; //grüne LED ein
        led_active_status=0; //gelbe LED aus
        led_alarm_status=0; //rote LED aus
        syslog(LOG_NOTICE, "Alarm-Countdown gestoppt"); //Syslog-Eintrag
     }
//   #7 Alarm-Countdown auf 0
if(anlage_status[0] && !anlage_countdown[0] && !alarm_status[0] && alarm_countdown[0] && countdown==0) {
        alarm_status[1]++; //Alarm-Status hochzählen
        alarm_countdown[1]=0; //Alarm-Countdown abschalten
        pir_flag=0; //Trigger löschen
        led_ok_status=1; //grüne LED ein
        led_active_status=1; //gelbe LED ein
        led_alarm_status=1; //rote LED ein
        countdown=alarm_dauer; //Countdown für Alarm-Dauer starten
        syslog(LOG_NOTICE, "Alarm aktiviert, Sensor: %d",alarm_sensor); //Syslog-Eintrag
        start_cam(); //Kameraaufnahme starten
        syslog(LOG_NOTICE, "Kameras gestartet"); //Syslog-Eintrag
        sprintf(text,"ALARM! Zeit: %s Sensor: %d",ctime(&alarm_zeit),alarm_sensor);
        if (sw_sms) send_sms(text); //Alarm-SMS verschicken
        if (sw_sms) syslog(LOG_NOTICE, "Alarm-SMS gesendet"); //Syslog-Eintrag
        if (sw_email) send_email(text); //Alarm-Email verschicken
        if (sw_email) syslog(LOG_NOTICE, "Alarm-Email gesendet"); //Syslog-Eintrag
        if (sw_blitzlicht) blitzlicht_ein(); //Blitzlicht einschalten
        if (sw_sirene) sirene_ein(); //Sirene enschalten
        if (sw_flurlicht) flurlicht_ein(); //Flurlicht einschalten
     }
//   #8 Alarmzeit abgelaufen Alarm wird deaktiviert
     if(anlage_status[0] && !anlage_countdown[0] && alarm_status[0] && !alarm_countdown[0] && countdown==0) {
        alarm_status[1]=0; //Alarm deaktivieren
        led_ok_status=1; //grüne LED ein
        led_active_status=1; //gelbe LED ein
        led_alarm_status=0; //rote LED aus
        syslog(LOG_NOTICE, "Alarm deaktiviert"); //Syslog-Eintrag
        stop_cam(); //Kameraaufnahme stoppen
        syslog(LOG_NOTICE, "Kameras gestoppt"); //Syslog-Eintrag
        if (sw_blitzlicht) blitzlicht_aus(); //Blitzlicht ausschalten
        if (sw_sirene) sirene_aus(); //Sirene ausschalten
        if (sw_flurlicht) flurlicht_aus(); //Flurlicht ausschalten
     }
//   #9 laufenden Alarm mit RFID-Tag abbrechen und Anlage deaktivieren
     if(anlage_status[0] && !anlage_countdown[0] && alarm_status[0] && !alarm_countdown[0] && rfid_good) {
        anlage_status[1]=0; //Anlage deaktivieren
        anlage_countdown[1]=0; //Anlagen-Countdown abschalten
        alarm_status[1]=0; //Alarm deaktivieren
        alarm_countdown[1]=0; //Alarm-Countdown deaktivieren
        rfid_good=0; //Trigger löschen
        led_ok_status=1; //grüne LED ein
        led_active_status=0; //gelbe LED aus
        led_alarm_status=0; //rote LED aus
        sprintf(text,"Alarm & Anlage mit RFID deaktiviert, Name: %s, Zeit: %s",personen[rfid_nr],ctime(&alarm_zeit));
        syslog(LOG_NOTICE, "Alarm & Anlage deaktiviert, RFID-Tag: %010d, Name: %s",rfid_tag,personen[rfid_nr]); //Syslog-Eintrag
        if (sw_sms) send_sms(text); //Deaktivierungs-SMS verschicken
        if (sw_sms) syslog(LOG_NOTICE, "Deaktivierungs-SMS gesendet"); //Syslog-Eintrag
        if (sw_email) send_email(text); //Deaktivierungs-Email verschicken
        if (sw_email) syslog(LOG_NOTICE, "Deaktivierungs-Email gesendet"); //Syslog-Eintrag
        stop_cam(); //Kameraaufnahme stoppen
        syslog(LOG_NOTICE, "Kameras gestoppt"); //Syslog-Eintrag
        if (sw_blitzlicht) blitzlicht_aus(); //Blitzlicht ausschalten
        if (sw_sirene) sirene_aus(); //Sirene ausschalten
        if (sw_flurlicht) flurlicht_aus(); //Flurlicht ausschalten
        status_write(); //aktuellen Anlagenstatus auf Disk speichern
     }
//   #10 laufender Alarm wird reaktiviert
     if(anlage_status[0] && !anlage_countdown[0] && alarm_status[0] && !alarm_countdown[0] && (rfid_bad || pir_flag)) {
        alarm_status[1]++; //Alarm-Status hochzählen
        countdown=alarm_dauer; //erneut Countdown für Alarm-Dauer starten
        rfid_bad=0; //Trigger löschen
        pir_flag=0; //Trigger löschen
        led_ok_status=1; //grüne LED ein
        led_active_status=1; //gelbe LED ein
        led_alarm_status=1; //rote LED ein
        syslog(LOG_NOTICE, "Alarm reaktiviert, Sensor: %d, Alarm-Anzahl: %d",alarm_sensor,alarm_status[1]); //Syslog-Eintrag
     }
//   #11 sofortige Alarm-Aktivierung aus Zustand "Anlage ein" mit falschem RFID-Tag
if(anlage_status[0] && !anlage_countdown[0] && !alarm_status[0] && rfid_bad) {
        alarm_status[1]++; //Alarm-Status hochzählen
        alarm_countdown[1]=0; //Alarm-Countdown abschalten
        rfid_bad=0; //Trigger löschen
        pir_flag=0; //Trigger löschen
        led_ok_status=1; //grüne LED ein
        led_active_status=1; //gelbe LED ein
        led_alarm_status=1; //rote LED ein
        countdown=alarm_dauer; //Countdown für Alarm-Dauer starten
        syslog(LOG_NOTICE, "Alarm aktiviert, Sensor: %d",alarm_sensor); //Syslog-Eintrag
        start_cam(); //Kameraaufnahme starten
        syslog(LOG_NOTICE, "Kameras gestartet"); //Syslog-Eintrag
        sprintf(text,"ALARM! Zeit: %s Sensor: %d",ctime(&alarm_zeit),alarm_sensor);
        if (sw_sms) send_sms(text); //Alarm-SMS verschicken
        if (sw_sms) syslog(LOG_NOTICE, "Alarm-SMS gesendet"); //Syslog-Eintrag
        if (sw_email) send_email(text); //Alarm-Email verschicken
        if (sw_email) syslog(LOG_NOTICE, "Alarm-Email gesendet"); //Syslog-Eintrag
        if (sw_blitzlicht) blitzlicht_ein(); //Blitzlicht einschalten
        if (sw_sirene) sirene_ein(); //Sirene enschalten
        if (sw_flurlicht) flurlicht_ein(); //Flurlicht einschalten
     }

     if(countdown>0) countdown--; //wenn Countdown gestartet ist, runterzählen
     watchdog_zaehler++; //Watchdog-Zähler hochzählen
     if(watchdog_zaehler==watchdog_time) { //wenn 1h erreicht: Syslog-Eintrag und rücksetzen
        syslog(LOG_NOTICE, "Alarm-Daemon aktiv"); //Syslog-Eintrag
        watchdog_zaehler=0; //Zähler zurücksetzen
        }
     printf("anlage=%d anl_cdwn=%d alarm=%d alm_cdwn=%d sensor=%d cdwn:%dsec\n",\
     anlage_status[1],anlage_countdown[1],alarm_status[1],alarm_countdown[1],\
     alarm_sensor,countdown);

     anlage_status[0]=anlage_status[1]; //aktuellen Anlagenstatus auf vorherigen Status speichern
     anlage_countdown[0]=anlage_countdown[1];
     alarm_status[0]=alarm_status[1];
     alarm_countdown[0]=alarm_countdown[1];
     sleep(1); //1 sec schlafen
  } //Ende Endlosschleife
cleanup(); //Programmende --> alles aufräumen
return 0;
}  /* Ende main() */


//************************************************************************************
//   Sub-Routinen, Threads und Handler
//************************************************************************************
// noch sehr einfach gehalten, keine Prüfung auf mehrere Optionen
int check_options(int argc, char *argv[]) {
   if(argc > 1) { //Wenn Parameter übergeben wurden
      //printf("parameter: %d %s\n", argc, argv[1]);
      if( !strcmp(argv[1],"-s") ) { //Option Simulationsmodus
         sim=1; //Simulationsmodus setzen
         syslog(LOG_NOTICE, "Alarm-Daemon im Simulationsmodus"); //Syslog-Eintrag
      }
      else if( !strcmp(argv[1],"-h") ) { //Option Hilfetext
         for(i=0; i<help_text_zeilen; i++) {
            printf("%s\n",help_text[i]); //Help-Text ausgeben
            }
         exit(0); //Programmende
      }
   }
return 0;
}

void syslog_init() {
setlogmask (LOG_UPTO (LOG_NOTICE)); //Log-Level festlegen
openlog (NULL, LOG_CONS | LOG_PID | LOG_NDELAY, LOG_USER); //Log-Einträge konfigurieren
}

int hardware_init() {

    char command[200]=""; //Kommando-Zwischenspeicher

    printf("Hardware-Initialisierung\n");

    // GPIO-Init
    for (i=2;i<=27;i++) gpio_unexport(i); //alle alten Definitionen löschen
    delay(100); //100ms warten

    gpio_export(GPIO_BLITZLICHT); //Pins definieren in /sys/class/gpio/
    gpio_export(GPIO_SIRENE);
    gpio_export(GPIO_FLURLICHT);
    gpio_export(GPIO_RELAIS_4);
    gpio_export(GPIO_LED_OK);
    gpio_export(GPIO_LED_ACTIVE);
    gpio_export(GPIO_LED_ALARM);
    //Eingänge aus config-file initialisieren
    if(sensor_anzahl) for(i=1;i<=sensor_anzahl;i++) gpio_export(sensoren[i]);
    delay(100); //Zeit für Betriebssystem zum Erstellen der Dateien

    gpio_direction(GPIO_BLITZLICHT, OUT); gpio_write(GPIO_BLITZLICHT, HIGH);
    gpio_direction(GPIO_SIRENE, OUT); gpio_write(GPIO_SIRENE, HIGH);
    gpio_direction(GPIO_FLURLICHT, OUT); gpio_write(GPIO_FLURLICHT, HIGH);
    gpio_direction(GPIO_RELAIS_4, OUT); gpio_write(GPIO_RELAIS_4, HIGH);
    gpio_direction(GPIO_LED_OK, OUT); gpio_write(GPIO_LED_OK, LOW);
    gpio_direction(GPIO_LED_ACTIVE, OUT); gpio_write(GPIO_LED_ACTIVE, LOW);
    gpio_direction(GPIO_LED_ALARM, OUT); gpio_write(GPIO_LED_ALARM, LOW);
    //Eingänge aus config-file initialisieren
    if(sensor_anzahl) for(i=1;i<=sensor_anzahl;i++) {
       gpio_direction(sensoren[i], IN);
       //gpio_edge(sensoren[i], 'r');
    }
    delay(100); //100ms warten

    //Kamera-Init
    sprintf(command,"v4l2-ctl -d 0 --set-ctrl brightness=%d >/dev/null &",cam1_brightness);
    if(sim) printf("command:  -->%s\n",command);
    if(!sim) system(command); //Helligkeit Kamera 1 setzen
    sprintf(command,"v4l2-ctl -d 1 --set-ctrl brightness=%d >/dev/null &",cam2_brightness);
    if(sim) printf("command:  -->%s\n",command);
    if(!sim) system(command); //Helligkeit Kamera 2 setzen

    return 0;
}

int start_cam() {

    char command[220]="";
    time_t zeit;
    char zeit_string[15];

    zeit = time(NULL); //zeit abfragen
    strftime(zeit_string,sizeof zeit_string,"%Y%m%d%H%M%S",localtime(&zeit));
    //printf("Zeit: %s\n",zeit_string);
    sprintf(command,"ffmpeg -f v4l2 -input_format mjpeg -framerate 15 -video_size 640x480 -i /dev/video0 -threads 1 %scam1_%s.avi 2>/dev/null &",work_dir,zeit_string);
    if(sim) printf("command: -->%s\n",command);
    if(!sim) system(command); //Cam1 starten
    sprintf(command,"ffmpeg -f v4l2 -input_format mjpeg -framerate 15 -video_size 640x480 -i /dev/video1 -threads 1 %scam2_%s.avi 2>/dev/null &",work_dir,zeit_string);
    if(sim) printf("command: -->%s\n",command);
    if(!sim) system(command); //Cam2 starten
    return 0;
}

int stop_cam() {

    char command[220]="";

    sprintf(command,"killall ffmpeg >/dev/null &");
    if(sim) printf("command: -->%s\n",command);
    if(!sim) system(command); //alle Kameras abschalten
    //kill(pid_cam1,SIGTERM); //nur Kamera1 abschalten
    return 0;
}

int send_sms(char sms_text[160]) {
    char command[220]="";

    sprintf(command,"echo \"%s\" | sudo gammu-smsd-inject TEXT \"%s\" >/dev/null &",sms_text,tel_nr);
    if(sim) printf("command: -->%s\n",command);
    if(!sim) system(command); //SMS mit gammu versenden
    return 0;
}

int send_email(char email_text[160]) {
   char command[1024];
   char email_header[]="To: sven@boenischnet.de\\nFrom: Alarmanlage\\nSubject: Alarm\\n";

   sprintf(command,"echo \"%s%s\" | msmtp %s &",email_header,email_text,email_addr);
   if(sim) printf("command: -->%s\n",command);
   if(!sim) system(command); //Email mit msmtp versenden
   return 0;
}

void status_init() {
printf("Anlagenstatus-Initialisierung\n");
fd_status = fopen(status_file,"rt+"); //Statusdatei schon da?
if(fd_status != NULL) {
   fscanf(fd_status,"%d",&anlage_status[0]); //alten Anlagenstatus auslesen & setzen
   rewind(fd_status); //Zeiger auf Anfang der Datei zurücksetzen
   syslog(LOG_NOTICE, "Statusdatei geöffnet");
}
else if( (fd_status=fopen(status_file,"wt+")) != NULL) { //Statusdatei neu anlegen & initialisieren
   fprintf(fd_status,"0"); //Anlagenstatus "aus" initialisieren
   if (fflush(fd_status) != 0) { perror("Fehler bei Schreiben in Statusdatei"); exit(1); }
   rewind(fd_status); //Zeiger auf Anfang der Datei zurücksetzen
   syslog(LOG_NOTICE, "Statusdatei angelegt");
}
else { //Falls alles nicht klappt
   perror("Fehler beim Anlegen der Statusdatei");
   syslog(LOG_ERR, "Fehler: Statusdatei konnte nicht angelegt werden");
   exit(1);
}
return;
}

void status_write() {
   fprintf(fd_status,"%d",anlage_status[1]); //aktuellen Anlagenstatus auf Disk speichern
   if (fflush(fd_status) != 0) { perror("Fehler bei Schreiben in Statusdatei"); exit(1); }
   rewind(fd_status); //Zeiger auf Anfang der Datei zurücksetzen
   syslog(LOG_NOTICE, "Anlagenstatus gespeichert"); //Syslog-Eintrag
}

void status_close() {
  ret=fclose(fd_status); //Statusdatei schließen
  if (ret != 0) { perror("Fehler beim Schliessen der Statusdatei"); exit(1); }
}

void start_threads() {
   printf("Thread-Erzeugung\n");
   ret = pthread_create(&rfidd_thread, NULL, rfidd, NULL); //Thread für RFID-Reader
      if(ret) {
         perror("ERROR pthread_create()");
         exit(50);
      }
      syslog(LOG_NOTICE, "Thread für RFID-Reader gestartet"); //Syslog-Eintrag
   ret = pthread_create(&led_thread, NULL, led_out, NULL); //Thread für LED-Ausgabe
      if(ret) {
         perror("ERROR pthread_create()");
         exit(50);
      }
      syslog(LOG_NOTICE, "Thread für LED-Ausgabe gestartet"); //Syslog-Eintrag

   syslog(LOG_NOTICE, "%dsec Aufwärmzeit für PIR-Sensoren abwarten",sensor_warmup); //Syslog-Eintrag
   sleep(sensor_warmup); //60sec Aufwärmzeit für PIR-Sensoren unbedingt abwarten um Fehlalarm zu vermeiden
   if(sensor_anzahl) for(i=1;i<=sensor_anzahl;i++) {
      ret = pthread_create(&sensor_thread[i], NULL, pir_read, (void*)i); //Sensor-Threads
      if(ret) {
         perror("ERROR pthread_create()");
         exit(50);
      }
      syslog(LOG_NOTICE, "Thread für Sensor-Nr. %d an GPIO%d gestartet",i,sensoren[i]); //Syslog-Eintrag
   }
   delay(100); //100ms warten
}

// Thread für PIR-Sensor Auslese
void *pir_read(void *threadid) {
  int index, pin, state[2];
  index=(int)threadid;

  printf("Sensor(Thread)-Nr: %d --> GPIO-Pin: %d\n",index,sensoren[index]);

//   #0 Initialisierung der State-Machine
  state[0]=1; //bei Zustand "Aus" starten
  pin=0; //Pin löschen

  while(1) { //Endlosschleife
    pin = gpio_read(sensoren[index]); //Pegel am GPIO-Pin auslesen
    //printf("pir_read(): Thread-Nr %d GPIO-Pin %d Pegel %d\n",index,sensoren[index],pin);

    if(pin < 0) perror("Thread pir_read(): Error gpio_read()");
    else {
       //   #1 wenn Zustand "Aus" und Sensor High
       if(state[0] == 1 && pin) {
          state[1]=2; //nächster Zustand "Wird aktiviert"
        }
       //   #2 wenn Zustand "Aus" und Sensor Low
       if(state[0] == 1 && !pin) {
          state[1]=1; //nächster Zustand "Aus"
       }
       //   #3 wenn Zustand "Wird aktiviert" und Sensor High
       if(state[0] == 2 && pin) {
          state[1]=3; //nächster Zustand "Alarm"
          printf("\033[31mpir_read(): ---> Alarm <--- Thread-Nr %d GPIO-Pin %d Pegel %d\033[m\n",index,sensoren[index],pin);
          alarm_sensor=index; //aktivierenden Alarmsensor übergeben
          pir_flag=1; //PIR-Sensor-Flag setzen
          alarm_zeit=time(NULL); //Alarmzeit speichern
        }
       //   #4 wenn Zustand "Wird aktiviert" und Sensor Low
       if(state[0] == 2 && !pin) {
          state[1]=4; //nächster Zustand "Fehlalarm"
          printf("\033[33mpir_read(): ---> Fehlalarm <--- Thread-Nr %d GPIO-Pin %d Pegel %d\033[m\n",index,sensoren[index],pin);
          syslog(LOG_NOTICE, "Fehlalarm, Sensor: %d",index); //Syslog-Eintrag
       }
       //   #5 wenn Zustand "Fehlalarm" und Sensor High
       if(state[0] == 4 && pin) {
          state[1]=2; //nächster Zustand "Wird aktiviert"
       }
       //   #6 wenn Zustand "Fehlalarm" und Sensor Low
       if(state[0] == 4 && !pin) {
          state[1]=1; //nächster Zustand "Aus"
       }
       //   #7 wenn Zustand "Alarm" und Sensor High
       if(state[0] == 3 && pin) {
          state[1]=3; //nächster Zustand "Alarm"
          //aber hier keine erneute Signalisierung, bevor nicht Zustand "Aus" durchlaufen wurde
       }
       //   #8 wenn Zustand "Alarm" und Sensor Low
       if(state[0] == 3 && !pin) {
          state[1]=5; //nächster Zustand "Wird deaktiviert"
       }
       //   #9 wenn Zustand "Wird deaktiviert" und Sensor Low
       if(state[0] == 5 && !pin) {
          state[1]=1; //nächster Zustand "Aus"
       }
       //   #10 wenn Zustand "Wird deaktiviert" und Sensor High
       if(state[0] == 5 && pin) {
          state[1]=3; //nächster Zustand "Alarm"
       }
    }
  state[0]=state[1]; //aktuellen Anlagenstatus auf vorherigen Status speichern
  delay(100); //100ms warten, max. 10 Events pro sec
  }
pthread_exit(NULL);
}

//Thread für RFID-Auslese
void *rfidd() {

printf("Thread rfidd() gestartet\n");
while(1) { //nicht-blockierendes, hot-plug fähiges Auslesen des USB RFID-Readers
    while( (fd_rfid = fopen(rfid_device, "r")) == NULL) { // /dev/hidraw0 öffnen
       perror("Fehler beim Öffnen von /dev/hidraw0"); //falls Fehler
       sleep(1); //1sec warten, danach erneut versuchen
    }
    //printf("RFID-Tag auflegen!\n");
    res = fread(buf, 1, 176, fd_rfid); //aus datei lesen
    //if (res != 176) { perror("Fehler beim Lesen von /dev/hidraw0"); exit(1); }
    // achtung hier wartet der prozess solange bis ein Tag aufgelegt wird
    if (res != 176) {
       syslog(LOG_NOTICE, "RFID-Tag Lesefehler, Tag-Nr. auf 0 setzen");
       rfid_tag=0; //Falls Fehler beim Lesen, RFID-Tag auf 0 setzen
    }
    else {
        j=9; //Zähler für 10-stellige Tag-Nr. initialisieren
        rfid_tag=0; //Tag-Nr. auf 0 (Unbekannt) initialisieren
        rfid_nr=0; //RFID Listen-Nummer auf 0 "Unbekannt" initialisieren
        for (i = 2; i < 160; i=i+16) {
           rfid_tag=rfid_tag+(pow(10,j)*((buf[i]-29)%10)); //RFID-Tag Nummer dekodieren
           j=j-1;
        }
        syslog(LOG_NOTICE, "RFID-Tag gelesen, Tag-Nr: %010d",rfid_tag);
    }
    res=fclose(fd_rfid); // /dev/hidraw0 schliessen
    if (res != 0) { perror("Fehler beim Schliessen von /dev/hidraw0"); exit(1); }
    rfid_good=0;
    rfid_bad=0;
    for(i=1; i<tag_anzahl+1; i++) { //Tag-Liste überprüfen
       if(rfid_tags[i]==rfid_tag) { //wenn RFID-Tag bekannt ist
          rfid_good=1; //Flag setzen
          rfid_nr=i; //laufende Nummer speichern
       }
    }
    rfid_bad=!rfid_good; //wenn Tag nicht gefunden wurde Bad-Flag setzen
    alarm_sensor=sensor_rfid; //aktivierenden Sensor auf virtuellen RFID-Sensor setzen
    alarm_zeit=time(NULL); //Alarmzeit speichern
}
pthread_exit(NULL);
}

// Thread für LED-Ausgabe an GPIO
void *led_out() {
  int wait=1000/(2*led_freq); //1/2 Periode

  printf("Thread led_out() gestartet\n");

  while(1) {
  switch(led_ok_status) {
    case 0: gpio_write(GPIO_LED_OK, LOW); break; //LED aus
    case 1: gpio_write(GPIO_LED_OK, HIGH); break; //LED ein
    case 2: gpio_write(GPIO_LED_OK, HIGH); break; //LED ein
  }
  switch(led_active_status) {
    case 0: gpio_write(GPIO_LED_ACTIVE, LOW); break; //LED aus
    case 1: gpio_write(GPIO_LED_ACTIVE, HIGH); break; //LED ein
    case 2: gpio_write(GPIO_LED_ACTIVE, HIGH); break; //LED ein
  }
  switch(led_alarm_status) {
    case 0: gpio_write(GPIO_LED_ALARM, LOW); break; //LED aus
    case 1: gpio_write(GPIO_LED_ALARM, HIGH); break; //LED ein
    case 2: gpio_write(GPIO_LED_ALARM, HIGH); break; //LED ein
  }
  delay(wait); // 1/2 Periode warten
  switch(led_ok_status) {
    case 0: gpio_write(GPIO_LED_OK, LOW); break; //LED aus
    case 1: gpio_write(GPIO_LED_OK, HIGH); break; //LED ein
    case 2: gpio_write(GPIO_LED_OK, LOW); break; //LED aus
  }
  switch(led_active_status) {
    case 0: gpio_write(GPIO_LED_ACTIVE, LOW); break; //LED aus
    case 1: gpio_write(GPIO_LED_ACTIVE, HIGH); break; //LED ein
    case 2: gpio_write(GPIO_LED_ACTIVE, LOW); break; //LED aus
  }
  switch(led_alarm_status) {
    case 0: gpio_write(GPIO_LED_ALARM, LOW); break; //LED aus
    case 1: gpio_write(GPIO_LED_ALARM, HIGH); break; //LED ein
    case 2: gpio_write(GPIO_LED_ALARM, LOW); break; //LED aus
  }
  delay(wait); // 1/2 Periode warten
  }
pthread_exit(NULL);
}

void check_alarmd_task() {
    sprintf(command,"ps -A | grep -c \"alarmd\" | awk '$1==\"1\" {exit(0)} {exit(1)}'");
    ret = system(command); //Bash System Kommando absetzen
    if( ret ) { //Anzahl von alarmd-Tasks ungleich 1 ???
       syslog(LOG_ERR, "Fehler: Alarm-Daemon läuft schon... Abbruch");
       perror("Fehler: Alarm-Daemon läuft schon... Abbruch");
       exit(100); //Abbruch
       }
    else return; //ansonsten alles gut weitermachen
}

void blitzlicht_ein() {
   gpio_write(GPIO_BLITZLICHT, LOW); //Blitzlicht aktivieren
return;
}

void blitzlicht_aus() {
   gpio_write(GPIO_BLITZLICHT, HIGH); //Blitzlicht deaktivieren
return;
}

void sirene_ein() {
   gpio_write(GPIO_SIRENE, LOW); //Sirene & Blitzlicht aktivieren
return;
}

void sirene_aus() {
   gpio_write(GPIO_SIRENE, HIGH); //Sirene & Blitzlicht deaktivieren
return;
}

void flurlicht_ein() {
   gpio_write(GPIO_FLURLICHT, LOW); //Flurlicht einschalten (Impuls)
   delay(250); //250ms warten
   gpio_write(GPIO_FLURLICHT, HIGH);
return;
}

void flurlicht_aus() {
   gpio_write(GPIO_FLURLICHT, LOW); //Flurlicht ausschalten (Impuls)
   delay(250); //250ms warten
   gpio_write(GPIO_FLURLICHT, HIGH);
return;
}

void config_init() {
   config=iniparser_load(config_file); //Config-Datei laden und parsen
   if(config==NULL) { //Falls das schiefgeht
      syslog(LOG_NOTICE, "Fehler: Konfig-Datei nicht vorhanden oder fehlerhaft");
      perror("Fehler: Konfig-Datei nicht vorhanden oder fehlerhaft");
      exit(1); //Abbruch
   }
   //iniparser_dump(config,stdout); //geparste Struktur anzeigen
   //Variablen zuweisen
   sim=iniparser_getboolean(config,":sim_switch",1);
   work_dir=iniparser_getstring(config,":work_dir","");
   status_file=iniparser_getstring(config,":status_file","");
   tel_nr=iniparser_getstring(config,":tel_nr","");
   sw_sms=iniparser_getboolean(config,":send_sms",0);
   sw_blitzlicht=iniparser_getboolean(config,":flashlight_enable",0);
   sw_sirene=iniparser_getboolean(config,":horn_enable",0);
   sw_flurlicht=iniparser_getboolean(config,":light_enable",0);
   watchdog_time=iniparser_getint(config,":watchdog_time",0);
   sensor_warmup=iniparser_getint(config,":sensor_warmup",60);
   alarm_verzoegerung=iniparser_getint(config,":alarm_verzoegerung",0);
   alarm_dauer=iniparser_getint(config,":alarm_dauer",0);
   rfid_device=iniparser_getstring(config,":rfid_device","");
   cam1_brightness=iniparser_getint(config,":cam1_brightness",0);
   cam2_brightness=iniparser_getint(config,":cam2_brightness",0);
   einschalt_verzoegerung=iniparser_getint(config,":einschalt_verzoegerung",0);
   email_addr=iniparser_getstring(config,":email_addr","");
   sw_email=iniparser_getboolean(config,":send_email",0);
   //Tag und Namen-Liste initialisieren
   tag_anzahl=iniparser_getint(config,":tag_anzahl",0);
   rfid_tags[0]=0; strcpy(personen[0],"Unbekannt"); //Feld 0: ID 0, Unbekannt
   for (i=1; i<=tag_anzahl; i++) { //für alle Tags
      sprintf(str_buf,":tag_%d",i); //Variablennamen erzeugen
      rfid_tags[i]=iniparser_getint(config,str_buf,0); //Auslesen und in Array schreiben
      sprintf(str_buf,":person_%d",i);  //Variablennamen erzeugen
      strcpy(personen[i],iniparser_getstring(config,str_buf,0)); //Auslesen und in Array schreiben
   }
   //Sensor-Liste initialisieren
   sensor_anzahl=iniparser_getint(config,":sensor_anzahl",0);
   sensoren[0]=0; //Feld 0: Pin 0, Unbenutzt
   for (i=1; i<=sensor_anzahl; i++) { //für alle Sensoren
      sprintf(str_buf,":sensor_%d",i); //Variablennamen erzeugen
      sensoren[i]=iniparser_getint(config,str_buf,0); //Auslesen und in Array schreiben
   }
   syslog(LOG_NOTICE, "Konfiguration aus %s gelesen",config_file);
}

void cleanup() {
   printf("\ncleanup(): GPIO-Pins und Kameras abschalten, Aufräumen\n");
   syslog(LOG_NOTICE, "Abschaltung durch SIGINT"); //Syslog-Eintrag
   //alle GPIO-Ausgänge abschalten
   gpio_write(GPIO_BLITZLICHT, HIGH);
   gpio_write(GPIO_SIRENE, HIGH);
   gpio_write(GPIO_FLURLICHT, HIGH);
   gpio_write(GPIO_RELAIS_4, HIGH);
   gpio_write(GPIO_LED_OK, LOW);
   gpio_write(GPIO_LED_ACTIVE, LOW);
   gpio_write(GPIO_LED_ALARM, LOW);
   syslog(LOG_NOTICE, "Alarme und LEDs abgeschaltet"); //Syslog-Eintrag
   stop_cam(); //Kameras abschalten
   syslog(LOG_NOTICE, "Kameras gestoppt"); //Syslog-Eintrag
   status_close(); //Anlagenstatus(datei) schließen
   syslog(LOG_NOTICE, "Anlagenstatus-Datei geschlossen"); //Syslog-Eintrag
   //Alle laufenden Threads beenden
   pthread_cancel(rfidd_thread);
   pthread_cancel(led_thread);
   if(sensor_anzahl) for(i=1;i<=sensor_anzahl;i++) {
      pthread_cancel(sensor_thread[i]);
   }
   syslog(LOG_NOTICE, "Alle laufenden Threads beendet"); //Syslog-Eintrag
   iniparser_freedict(config); //Ini-Parser Struktur freigeben
   syslog(LOG_NOTICE, "Alarm-Daemon beendet");
   closelog(); //Log-Verbindung schließen
return;
}

void ctrl_c_handler (int signal) {
   //printf("\nCTRL-C gedrückt\n");
   run=0; //Hauptschleife verlassen
return;
}
