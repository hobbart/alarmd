//*************************************************
// alarmd_9.c
//
// Meine Alarmanlage :-)
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
//
//*************************************************

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>
#include <time.h>
#include <linux/hidraw.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/shm.h>
#include <fcntl.h>
#include <math.h>
#include <errno.h>
#include <syslog.h>

#define MAXWATCHDOGTIME 3600 //Zeitdauer in sec für Syslogeinträge, dass Programm noch läuft
#define SENSOR_FLUR_TUER_SCHLAFZIMMER 1 //Sensornummern
#define SENSOR_FLUR_TUER_EINGANG 2
#define SENSOR_RFID 256
#define SENSOR_SIGUSR2 257
#define ALARM_VERZOEGERUNG 5 //Alarmverzögerung in sec

//**************************************************
//Funktionsdeklarationen
//**************************************************
void memory_init();
int hardware_init();
int start_cam();
int stop_cam();
int send_sms(char sms_text[]);
void status_init();
void status_write();
void status_close();
void signal_handler(int signal);

//**************************************************
//Globale Variablen
//**************************************************
char work_dir[20]="/home/pi/Videos/"; //Arbeitsverzeichnis
char status_file[30]="/var/lib/alarmd/alarmd.status"; //Datei mit Anlagenstatus
char tel_nr[13]="+49"; //Ziel-Nr. fuer SMS-Benachrichtigung
int ret; //Rückgabewert-Wert von Funktionen
int cam1_brightness=0; //Helligkeit Kamera 1
int cam2_brightness=128; //Helligkeit Kamera 2
char text[160]; //Buffer für SMS-Text
char command[200]=""; //Kommando-Zwischenspeicher
FILE *fd_rfid, *fd_status; //Filehandles für RFID-Reader und Status-Datei
char buf[176]; //Buffer für RFID-Reader
char *rfid_device = "/dev/hidraw0"; //RFID-Reader device
int i, j, res = 0;
int allowed_rfid_tags[5]={0, 100, 200, 300, 400}; //bekannte RFID-Tags
char personen[5][10]={"Unbekannt","Pers1","Pers2","Pers3","Pers4"}; //bekannte Personen
int res_rfid_check, mymemID;
pid_t ppid, pid; //Prozess-IDs von Eltern- und Kindprozess
int alarmdauer=180; //Alarmdauer in sec (max. 3min erlaubt)
int *mymemptr; //Zeiger auf Anfang des gemeinsamen Speicherbereichs (gefüllt mit int-Variablen)
struct { //Struktur für shared memory Variablenübergabe deklarieren und initialisieren
int anlage_status; //Anlagenstatus: 0=aus, 1=ein
int alarm_status;  //Alarmstatus: 0=aus, 1=ein
time_t alarm_zeit; //Alarmzeit: Zeitpunkt einer Alarmaktivierung
int rfid_tag;      //dekodierter RFID-Tag
int alarm_sensor;  //Sensornummer der Alarm aktiviert hat
int rfid_nr;       //Laufende Nummer in Personenliste für RFID-Tag
} mymem = {};
int watchdog_zaehler = 0; //Watchdog-Zähler für Syslog-Eintrag
int sw_sim = 0; //Schalter für Simulation des Programmblaufs: 0=volle Funktionalität (default) 1=nur Simulation
int alarm_countdown = -1; //Alarm-Countdown für Verzögerung für Abschalten mit RFID-Tag
//**************************************************
//Hauptprogramm
//**************************************************
int main(int argc, char *argv[])
{
//Syslog initialisieren
setlogmask (LOG_UPTO (LOG_NOTICE)); //Log-Level festlegen
openlog ("alarmd", LOG_CONS | LOG_PID | LOG_NDELAY, LOG_LOCAL1); //Log-Einträge konfigurieren
syslog(LOG_NOTICE, "Alarm-Daemon gestartet"); //Syslog-Eintrag
//Parameterübergabe an main auswerten
if(argc > 1) { //Wenn Parameter übergeben wurden
   //printf("parameter: %d %s\n", argc, argv[1]);
   if( !strcmp(argv[1],"-s") ) { //Switch Simulation eingeschaltet
      sw_sim=1; //Simulationsmodus setzen
      syslog(LOG_NOTICE, "Alarm-Daemon im Simulationsmodus"); //Syslog-Eintrag
   }
}
//Initialisierungen
ppid=getpid(); //Prozess-ID des Hauptprozesses ermitteln
signal(SIGUSR1, signal_handler); //Signal SIGUSR1 auf signal_handler umleiten
signal(SIGUSR2, signal_handler); //Signal SIGUSR2 auf signal_handler umleiten
signal(SIGALRM, signal_handler); //Signal SIGALRM auf signal_handler umleiten
memory_init();
printf("Speicher-Initialisierung\n");
hardware_init(); //Hardware-Initialisierung (Kameras)
printf("Hardware-Initialisierung\n");
status_init(); //Anlagenstatus initialisieren (Zustand vor Neustart)
printf("Anlagenstatus-Initialisierung\n");

//einen weiteren Prozess in Case-Struktur erzeugen
switch( pid=fork() ) {

  case -1:   /* Fehler bei fork() */

     perror("Fehler beim Erzeugen des Kindprozesses");
     break;

  case  0:   /* Anfang Kindprozess   */

syslog(LOG_NOTICE, "RFID-Daemon gestartet"); //Syslog-Eintrag
while(1) { //nicht-blockierendes, hot-plug fähiges Auslesen des USB RFID-Readers
    j=9;              //10-stellige Tag-Nr.
    mymem.rfid_tag=0; //Tag-Nr. auf 0 (Unbekannt) initialisieren
    mymem.rfid_nr=0; //RFID-Nummer auf "Unbekannt" initialisieren
    while( (fd_rfid = fopen(rfid_device, "r")) == NULL) { // /dev/hidraw0 öffnen
       perror("Fehler beim Öffnen von /dev/hidraw0");
       sleep(1);
    }
    //printf("RFID-Tag auflegen!\n");
    res = fread(buf, 1, 176, fd_rfid); //aus datei lesen
    //if (res != 176) { perror("Fehler beim Lesen von /dev/hidraw0"); exit(1); }
    // achtung hier wartet der prozess solange bis ein Tag aufgelegt wird (also f$
    if (res != 176) mymem.rfid_tag=0; //Falls Fehler beim Lesen, RFID-Tag auf 0 setzen
    else {
        for (i = 2; i < 160; i=i+16) {
           mymem.rfid_tag=mymem.rfid_tag+(pow(10,j)*((buf[i]-29)%10)); //RFID-Tag Nummer dekodieren
           j=j-1;
       }
    syslog(LOG_NOTICE, "RFID-Tag gelesen, Tag-Nr: %010d\n",mymem.rfid_tag);
    mymemptr[3]=mymem.rfid_tag; //an Eltern-Prozess übergeben
    }
    res=fclose(fd_rfid); // /dev/hidraw0 schliessen
    if (res != 0) { perror("Fehler beim Schliessen von /dev/hidraw0"); exit(1); }
    res_rfid_check=0;
    for(i=1; i<5; i++) { //Tag-Liste überprüfen
       if(mymem.rfid_tag==allowed_rfid_tags[i]) { //wenn RFID-Tag bekannt ist
       res_rfid_check=1; //Flag setzen
       mymem.rfid_nr=i; //laufende Nummer speichern
       }
    }
    mymemptr[5]=mymem.rfid_nr; //an Elternprozess übergeben
    //if(res_rfid_check) printf("RFID-Tag vorhanden\n");
    raise(SIGUSR1); //Signal SIGUSR1 senden und damit Signal-Handler (im Kind-Prozess) aufrufen
    //printf("Signal SIGUSR1 gesendet\n");
}
exit(0); /* Ende Kindprozess */

  default:   /* Anfang Elternprozess */

  while(1) { //Hauptprogramm in Endlosschleife wird jede Sekunde 1x aufgerufen
     mymem.rfid_tag=mymemptr[3]; //RFID-Tag auslesen
     mymem.alarm_sensor=mymemptr[4]; //Sensor-Nr. auslesen
     if(mymemptr[1] && !mymem.alarm_status) { //wenn Alarm erstmalig aktiviert wird
        alarm_countdown=ALARM_VERZOEGERUNG; //Countdown starten
        syslog(LOG_NOTICE, "Countdown gestartet"); //Syslog-Eintrag
     }
     if(mymemptr[1] && alarm_countdown==0) { //wenn Alarm immer noch aktiviert und Countdown auf 0
        alarm_countdown=-1; //Countdown abschalten
        syslog(LOG_NOTICE, "Alarm aktiviert, Sensor: %d",mymem.alarm_sensor); //Syslog-Eintrag
        start_cam(); //Kameraaufnahme starten
        syslog(LOG_NOTICE, "Kameras gestartet"); //Syslog-Eintrag
        sprintf(text,"ALARM! Zeit: %s Sensor: %d",ctime(&mymem.alarm_zeit),mymem.alarm_sensor);
        send_sms(text); //Alarm-SMS verschicken
        syslog(LOG_NOTICE, "Alarm-SMS gesendet"); //Syslog-Eintrag
     }
     if( (mymemptr[1] > mymem.alarm_status) && mymem.alarm_status ) { //wenn Alarm reaktiviert wird
        syslog(LOG_NOTICE, "Alarm reaktiviert, Sensor: %d, Alarm-Anzahl: %d",mymem.alarm_sensor,mymemptr[1]); //Syslog-Eintrag
     }
     if(!mymemptr[1] && mymem.alarm_status) { //wenn Alarm deaktiviert wird
        syslog(LOG_NOTICE, "Alarm deaktiviert"); //Syslog-Eintrag
        stop_cam(); //Kameraaufnahme stoppen
        syslog(LOG_NOTICE, "Kameras gestoppt"); //Syslog-Eintrag
     }
     if(!mymemptr[1] && mymem.alarm_status && alarm_countdown>0) { //wenn Alarm bei laufendem Countdown deaktiviert wird
     alarm_countdown=0; //Countdown stoppen
     syslog(LOG_NOTICE, "Countdown gestoppt"); //Syslog-Eintrag
     }
     if(mymemptr[0] && !mymem.anlage_status) { //wenn Anlage aktiviert wird
        syslog(LOG_NOTICE, "Anlage aktiviert, RFID-Tag: %010d, Name: %s",mymem.rfid_tag,personen[mymemptr[5]]); //Syslog-Eintrag
        status_write(); //aktuellen Anlagenstatus auf Disk speichern
     }
     if(!mymemptr[0] && mymem.anlage_status) { //wenn Anlage deaktiviert wird
        syslog(LOG_NOTICE, "Anlage deaktiviert, RFID-Tag: %010d, Name: %s",mymem.rfid_tag,personen[mymemptr[5]]); //Syslog-Eintrag
        status_write(); //aktuellen Anlagenstatus auf Disk speichern
     }
     if( !mymemptr[1] && mymem.alarm_status && !mymemptr[0] && mymem.anlage_status && alarm_countdown==-1 ) { //wenn Anlage bei laufendem Alarm (mit RFID-Tag) deaktiviert wird
     sprintf(text,"Alarm & Anlage mit RFID deaktiviert, Name: %s, Zeit: %s",personen[mymemptr[5]],ctime(&mymem.alarm_zeit));
     send_sms(text); //Deaktivierungs-SMS verschicken
     syslog(LOG_NOTICE, "Deaktivierungs-SMS gesendet"); //Syslog-Eintrag
}
     if(alarm_countdown>0) alarm_countdown--; //wenn Countdown gestartet ist, runterzählen
     watchdog_zaehler++; //Watchdog-Zähler hochzählen
     if(watchdog_zaehler==MAXWATCHDOGTIME) { //wenn 1h erreicht: Syslog-Eintrag und rücksetzen
        syslog(LOG_NOTICE, "Alarm-Daemon aktiv"); //Syslog-Eintrag
        watchdog_zaehler=0; //Zähler zurücksetzen
        }
     memcpy(&mymem,mymemptr,sizeof(mymem)); //Variablen aus shared memory von Kind-Prozess in globale Struktur mymem an Elternprozess übergeben
     printf("Anlagenstatus: %d, Sensor: %d, Alarmstatus: %d, Restzeit: %d sec, Countdown: %d sec\n",mymem.anlage_status,mymem.alarm_sensor,mymem.alarm_status,alarmdauer-time(NULL)+mymem.alarm_zeit,alarm_countdown);
     sleep(1); //1 sec schlafen
  } //Ende Endlosschleife
  shmdt(mymemptr); //geminsamen Speicherbereich löschen, wird bei Progrmmabbruch mit Ctrl+C nie erreicht
  closelog(); //Log-Verbindung schließen
  status_close(); //Anlagenstatus(datei) schließen
  return 0; /* Ende Elternprozess */

} // Ende Case-Struktur

}  /* Ende main */


//**************************************************
//Funktionen
//**************************************************
void memory_init() {
   if ((mymemID = shmget(IPC_PRIVATE, sizeof(mymem), IPC_CREAT | 0600)) < 0) {
        perror("Fehler bei Erzeugen des shared memory Bereiches");
        exit(1);
      } //gemeinsamen Speicherbereich erzeugen
   if ((mymemptr = shmat(mymemID, NULL, 0)) == (int *) -1) {
        perror("Fehler bei Anbinden des shared memory Bereiches");
        exit(1);
      } //gemeinsamen Speicherbereich an Prozess anbinden (auch im Kind-Prozess bekannt)
   memcpy(mymemptr,&mymem,sizeof(mymem)); //gemeinsamen Speicherbereich initialisieren
}

int hardware_init()
{
    char command[200]=""; //Kommando-Zwischenspeicher

    sprintf(command,"v4l2-ctl -d 0 --set-ctrl brightness=%d >/dev/null &",cam1_brightness);
    if(sw_sim) printf("command:  -->%s\n",command);
    if(!sw_sim) ret=system(command); //Helligkeit Kamera 1 setzen
    sprintf(command,"v4l2-ctl -d 1 --set-ctrl brightness=%d >/dev/null &",cam2_brightness);
    if(sw_sim) printf("command:  -->%s\n",command);
    if(!sw_sim) ret=system(command); //Helligkeit Kamera 2 setzen
    return 0;
}

int start_cam()
{
    char command[220]="";
    int ret;
    time_t zeit;
    char zeit_string[15];

    zeit = time(NULL); //zeit abfragen
    strftime(zeit_string,sizeof zeit_string,"%Y%m%d%H%M%S",localtime(&zeit));
    //printf("Zeit: %s\n",zeit_string);
    sprintf(command,"ffmpeg -f v4l2 -input_format mjpeg -framerate 15 -video_size 640x480 -i /dev/video0 -threads 1 %scam1_%s.avi 2>/dev/null &",work_dir,zeit_string);
    if(sw_sim) printf("command: -->%s\n",command);
    if(!sw_sim) ret = system(command); //Cam1 starten
    sprintf(command,"ffmpeg -f v4l2 -input_format mjpeg -framerate 15 -video_size 640x480 -i /dev/video1 -threads 1 %scam2_%s.avi 2>/dev/null &",work_dir,zeit_string);
    if(sw_sim) printf("command: -->%s\n",command);
    if(!sw_sim) ret = system(command); //Cam2 starten
    return 0;
}

int stop_cam()
{
    char command[220]="";
    int ret;

    sprintf(command,"killall ffmpeg >/dev/null &");
    if(sw_sim) printf("command: -->%s\n",command);
    if(!sw_sim) ret = system(command); //alle Kameras abschalten
    //kill(pid_cam1,SIGTERM); //Kamera1 abschalten
    return 0;
}

int send_sms(char sms_text[160])
{
    char command[220]="";
    int ret;

    sprintf(command,"echo \"%s\" | sudo gammu-smsd-inject TEXT \"%s\" >/dev/null &",sms_text,tel_nr);
    if(sw_sim) printf("command: -->%s\n",command);
    if(!sw_sim) ret = system(command); //SMS mit gammu versenden
    return 0;
}

void status_init() {
fd_status = fopen(status_file,"rt+"); //Statusdatei schon da?
if(fd_status != NULL) {
   fscanf(fd_status,"%d",&mymemptr[0]); //Anlagenstatus auslesen & setzen
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
   fprintf(fd_status,"%d",mymemptr[0]); //aktuellen Anlagenstatus auf Disk speichern
   if (fflush(fd_status) != 0) { perror("Fehler bei Schreiben in Statusdatei"); exit(1); }
   rewind(fd_status); //Zeiger auf Anfang der Datei zurücksetzen
   syslog(LOG_NOTICE, "Anlagenstatus gespeichert"); //Syslog-Eintrag
}

void status_close() {
  res=fclose(fd_status); //Statusdatei schließen
  if (res != 0) { perror("Fehler beim Schliessen der Statusdatei"); exit(1); }
}

// Signal-Handler für SIGUSR1, SIGUSR2, SIGALRM (läuft im Kind-Prozess)
void signal_handler(int signal) {
   memcpy(&mymem,mymemptr,sizeof(mymem)); //shared memory, d.h. Variablen aus Elternprozess an Kind-Prozess übergeben
   if(signal == SIGUSR1) {
      if(res_rfid_check && mymem.anlage_status) { //RFID berechtigt und Anlage war ein
         printf("Alarm aus!\n");
         mymem.alarm_status=0; //Alarm ausschalten
         alarm(0); //Alarmtimer stoppen
         mymem.alarm_sensor=0; //Sensor-Nr. löschen
         printf("Anlage aus!\n");
         mymem.anlage_status=0; //Anlage aussschalten
      }
      else if(res_rfid_check && !mymem.anlage_status) { //RFID berechtigt und Anlage war aus
         printf("Anlage ein!\n");
         mymem.anlage_status=1; //Anlage einschalten
      }
      else if(!res_rfid_check && mymem.rfid_tag) { //RFID unberechtigt
         printf("Anlage ein!\n");
         mymem.anlage_status=1; //Anlage einschalten
         printf("Alarm ein!\n");
         mymem.alarm_status++; //Alarmzähler hochzählen
         alarm(alarmdauer); //Alarmtimer starten
         mymem.alarm_sensor=SENSOR_RFID; //Sensor-Nr. setzen (virtuelle Sensor-Nr. für RFID-Aktivierung)
         mymem.alarm_zeit=time(NULL); //Startzeit auslesen
      }
   }
   else if(signal == SIGUSR2) { //Alarmaktivierung und Deaktivierung via User-Signal
      if(mymem.alarm_status) { //Alarm war ein
         printf("Alarm aus!\n");
         mymem.alarm_status=0; //Alarm ausschalten
         alarm(0); //Alarmtimer stoppen
         mymem.alarm_sensor=0; //Sensor-Nr. löschen
         printf("Anlage aus!\n");
         mymem.anlage_status=0; //Anlage aussschalten
         //mymem.rfid_nr=3; //nur für test
      }
      else if(!mymem.alarm_status) { //Alarm war aus
         printf("Anlage ein!\n");
         mymem.anlage_status=1; //Anlage einschalten
         printf("Alarm ein!\n");
         mymem.alarm_status++; //Alarmzähler hochzählen
         alarm(alarmdauer); //Alarmtimer starten
         mymem.alarm_sensor=SENSOR_SIGUSR2; //Sensor-Nr. setzen (virtuelle Sensor-Nr. für Aktivierung via User-Signal SIGUSR2)
         mymem.alarm_zeit=time(NULL); //Startzeit auslesen
      }
   }
   else if(signal == SIGALRM) { //Timer Alarmdauer abgelaufen
      printf("Alarm aus!\n");
      mymem.alarm_status=0; //Alarm ausschalten
      mymem.alarm_sensor=0; //Sensor-Nr. löschen
   }
   memcpy(mymemptr,&mymem,sizeof(mymem)); //lokale Variablen aus Kindprozess an Elternprozess übergeben
return;
}
