//*************************************************
// alarmd_17.c
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
// V0.16 PIR-Sensoren Abfrage mit Flankendetektion, 8sec-timeout, separater Thread und Signalisierung über SIGUSR2
// V0.17 Geändert: rfidd als Thread, keine Signale, kein IPC-Mem, main-Endlosschleife anpassen
//
//*************************************************

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
//Initialisierung hier
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
pthread_t pir_thread[256], rfidd_thread; //Thread-Identifier Array (opaque, IEEE POSIX 1003.1c standard)
int watchdog_zaehler = 0; //Watchdog-Zähler für Syslog-Eintrag
int countdown; //Countdown für Betreten und Verlassen der Wohnung
int pir_warmup = 0; //60sec Aufwärmzeit für PIR-Sensoren
int help_text_zeilen = 6; //Anzahl der Zeilen für Hilfetext
char help_text[][100]={
"Aufruf: alarmd [OPTION]",
"Version 0.17",
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

//**************************************************
//Hauptprogramm
//**************************************************
int main(int argc, char *argv[])
{
//Allgemeine Initialisierungen
syslog_init(); //Syslog initialisieren
ppid=getpid(); //Prozess-ID des Hauptprozesses ermitteln
syslog(LOG_NOTICE, "Alarm-Daemon gestartet"); //Syslog-Eintrag
config_init(); //Variablen-Initialisierung aus Config-Datei
check_options(argc, argv); //Parameterübergabe an main auswerten
check_alarmd_task(); //Test ob schon ein alarmd-Task läuft -->Info-->exit(100)
//Anlagen-Initialisierung
hardware_init(); //Hardware-Initialisierung (GPIO, Kameras)
status_init(); //Anlagenstatus initialisieren (Zustand vor Neustart)
start_threads(); //Threads erzeugen und starten

  while(1) { //Hauptprogramm in Endlosschleife wird jede Sekunde 1x aufgerufen
//   #1 wenn Anlage erstmalig aktiviert wird
if(!anlage_status[0] && !anlage_countdown[0] && !alarm_status[0] && !alarm_countdown[0] && rfid_good) {
        anlage_countdown[1]=1; //neuen Anlagenzustand schreiben
        countdown=einschalt_verzoegerung; //Einschalt-Countdown starten
        rfid_good=0; //Trigger löschen
        syslog(LOG_NOTICE, "Einschalt-Countdown gestartet"); //Syslog-Eintrag
     }
//   #2 Einschalt-Countdown abbrechen
if(!anlage_status[0] && anlage_countdown[0] && !alarm_status[0] && !alarm_countdown[0] && rfid_good) {
        anlage_status[1]=0; //Anlage deaktivieren
        anlage_countdown[1]=0; //Anlagen-Countdown abschalten
        rfid_good=0; //Trigger löschen
        //countdown=0; //Zaehler rücksetzen problem mit #3
        syslog(LOG_NOTICE, "Einschalt-Countdown abgebrochen"); //Syslog-Eintrag
     }
//   #3 Einschalt-Countdown auf 0
if(!anlage_status[0] && anlage_countdown[0] && !alarm_status[0] && !alarm_countdown[0] && countdown==0) {
        anlage_status[1]=1; //Anlage aktivieren
        anlage_countdown[1]=0; //Countdown abschalten
        alarm_status[1]=0; //alle Alarme löschen
        alarm_countdown[1]=0; //Alarm-Countdown abschalten
        syslog(LOG_NOTICE, "Anlage aktiviert, RFID-Tag: %010d, Name: %s",rfid_tag,personen[rfid_nr]); //Syslog-Eintrag
        status_write(); //aktuellen Anlagenstatus auf Disk speichern
     }
//   #4 wenn Anlage deaktiviert wird
if(anlage_status[0] && !anlage_countdown[0] && !alarm_status[0] && !alarm_countdown[0] && rfid_good) {
        anlage_status[1]=0; //Anlage deaktivieren
        rfid_good=0; //Trigger löschen
        syslog(LOG_NOTICE, "Anlage deaktiviert, RFID-Tag: %010d, Name: %s",rfid_tag,personen[rfid_nr]); //Syslog-Eintrag
        status_write(); //aktuellen Anlagenstatus auf Disk speichern
     }
//   #5 wenn Alarm erstmalig aktiviert wird
if(anlage_status[0] && !anlage_countdown[0] && !alarm_status[0] && !alarm_countdown[0] && rfid_bad) {
        alarm_countdown[1]=1; //neuen Anlgenzustand schreiben
        countdown=alarm_verzoegerung; //Alarm-Countdown starten
        rfid_bad=0; //Trigger löschen
        syslog(LOG_NOTICE, "Alarm-Countdown gestartet"); //Syslog-Eintrag
     }
//   #6 Alarm-Countdown abbrechen und Anlage ausschalten
if(anlage_status[0] && !anlage_countdown[0] && !alarm_status[0] && alarm_countdown[0] && rfid_good) {
        anlage_status[1]=0;  //Anlage deaktivieren
        alarm_countdown[1]=0; //Countdown abschalten
        rfid_good=0; //Trigger löschen
        syslog(LOG_NOTICE, "Alarm-Countdown gestoppt"); //Syslog-Eintrag
     }
//   #7 Alarm-Countdown auf 0 ODER erneut falsches RFID-Tag gelesen
if(anlage_status[0] && !anlage_countdown[0] && !alarm_status[0] && alarm_countdown[0] && (countdown==0 || rfid_bad)) {
        alarm_status[1]++; //Alarm-Status hochzählen
        alarm_countdown[1]=0; //Alarm-Countdown abschalten
        rfid_bad=0; //Trigger löschen
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
        sprintf(text,"Alarm & Anlage mit RFID deaktiviert, Name: %s, Zeit: %s",personen[rfid_nr],ctime(&alarm_zeit));
        if (sw_sms) send_sms(text); //Deaktivierungs-SMS verschicken
        if (sw_sms) syslog(LOG_NOTICE, "Deaktivierungs-SMS gesendet"); //Syslog-Eintrag
        if (sw_email) send_email(text); //Deaktivierungs-Email verschicken
        if (sw_email) syslog(LOG_NOTICE, "Deaktivierungs-Email gesendet"); //Syslog-Eintrag
     }
//   #10 laufender Alarm wird reaktiviert
     if(anlage_status[0] && !anlage_countdown[0] && alarm_status[0] && !alarm_countdown[0] && rfid_bad) {
        alarm_status[1]++; //Alarm-Status hochzählen
        countdown=alarm_dauer; //erneut Countdown für Alarm-Dauer starten
        rfid_bad=0; //Trigger löschen
        syslog(LOG_NOTICE, "Alarm reaktiviert, Sensor: %d, Alarm-Anzahl: %d",alarm_sensor,alarm_status[1]); //Syslog-Eintrag
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
  status_close(); //Anlagenstatus(datei) schließen
  pthread_exit(NULL); //Alle laufenden Threads beenden
  closelog(); //Log-Verbindung schließen
  iniparser_freedict(config); //Ini-Parser Struktur freigeben
  return 0;
}  /* Ende main */


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
       gpio_edge(sensoren[i], 'r');
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
   char email_header[]="To: sven.boenisch@alumni.tu-berlin.de\\nFrom: Alarmanlage\\nSubject: Alarm\\n";

   sprintf(command,"echo \"%s%s\" | msmtp %s &",email_header,email_text,email_addr);
   if(sim) printf("command: -->%s\n",command);
   if(!sim) system(command); //Email mit msmtp versenden
   return 0;
}

void status_init() {
printf("Anlagenstatus-Initialisierung\n");
fd_status = fopen(status_file,"rt+"); //Statusdatei schon da?
if(fd_status != NULL) {
   fscanf(fd_status,"%d",&anlage_status[1]); //Anlagenstatus auslesen & setzen
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
      syslog(LOG_NOTICE, "RFID-Reader gestartet"); //Syslog-Eintrag

   sleep(pir_warmup); //Aufwärmzeit für PIR-Sensoren unbedingt abwarten um Fehlalarm zu vermeiden
   if(sensor_anzahl) for(i=1;i<=sensor_anzahl;i++) {
      ret = pthread_create(&pir_thread[i], NULL, pir_read, (void*)i); //Sensor-Threads
      if(ret) {
         perror("ERROR pthread_create()");
         exit(50);
      }
      syslog(LOG_NOTICE, "Sensor-Nr. %d an GPIO%d aktiviert",i,sensoren[i]); //Syslog-Eintrag
   }
   delay(100); //100ms warten
}

// Thread für PIR-Sensor Auslese
void *pir_read(void *threadid) {
  int index;
  index=(int)threadid;

  printf("Tread-Nr: %d --> Sensor-Nr: %d\n",index,sensoren[index]);
  while(1) {
    ret = gpio_wait(sensoren[index], -1); //timeout negativ --> ewig auf ereignis warten
    if(ret < 0) perror("Thread pi_read(): Error poll()");
    else if(ret == 0) perror("Thread pir_read(): Timeout");
    else {
       printf("\033[31mpir_read() Flanke detektiert! Thread-Nr: %d GPIO-Pin: %d Pegel: %d\033[m\n",index,sensoren[index],ret-1);
       alarm_sensor=sensoren[index]; //aktivierenden Alarmsensor übergeben
       alarm_zeit=time(NULL); //Alarmzeit speichern
    }
  delay(100); //100ms warten, max. 10 Events pro sec
  }
pthread_exit(NULL);
}

//Thread für RFID-Auslese
void *rfidd() {

printf("Thread rfidd gestartet\n");
while(1) { //nicht-blockierendes, hot-plug fähiges Auslesen des USB RFID-Readers
    while( (fd_rfid = fopen(rfid_device, "r")) == NULL) { // /dev/hidraw0 öffnen
       perror("Fehler beim Öffnen von /dev/hidraw0"); //falls Fehler
       sleep(1); //1sec warten, danach erneut versuchen
    }
    //printf("RFID-Tag auflegen!\n");
    res = fread(buf, 1, 176, fd_rfid); //aus datei lesen
    //if (res != 176) { perror("Fehler beim Lesen von /dev/hidraw0"); exit(1); }
    // achtung hier wartet der prozess solange bis ein Tag aufgelegt wird (also f$
    if (res != 176) rfid_tag=0; //Falls Fehler beim Lesen, RFID-Tag auf 0 setzen
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
   gpio_write(GPIO_FLURLICHT, LOW); //Flurlicht aktivieren
return;
}

void flurlicht_aus() {
   gpio_write(GPIO_FLURLICHT, HIGH); //Flurlicht deaktivieren
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
   syslog(LOG_NOTICE, "Konfiguration aus %s gelesen\n",config_file);
}
