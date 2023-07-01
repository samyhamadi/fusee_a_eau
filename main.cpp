#include "mbed.h"
#include "BME280.h"
#include "LSM9DS0.h"

//dif Altitude max - altitude  actuelle pour ouvrir le parachute
#define DECISION_ALT  400

#define LSM9DS0_XM  0x1D // Would be 0x1E if SDO_XM is LOW
#define LSM9DS0_G   0x6B // Would be 0x6A if SDO_G is LOW

LocalFileSystem local("local"); // créer le système de fichier 

DigitalOut myled1(LED1);
DigitalOut myled2(LED2);
DigitalOut myled3(LED3);
DigitalOut myled4(LED4);
DigitalIn xbee_secure(p12,PullDown);
PwmOut  sig(p21);
DigitalIn bouton(p8,PullUp);

Serial bt(p13,p14);

I2C i2c(p9,p10);
I2C i2c_2(p28,p27);
BME280 sensor(i2c, (char)(0x77*2));
LSM9DS0 dof(p9, p10,LSM9DS0_G, LSM9DS0_XM);

float t_reg = 0;
float p_init,alti,alti_max,altimoyenne;

//pitot 
char data[2],bcom=0x07; 
unsigned short reading = 0xFFFF; 
unsigned short pitot_init=0;

unsigned time_parachute;
float tempC ;
int16_t Gx, Gy, Gz ,Ax, Ay, Az;
float pressure; // unité

int16_t AXlog[1000],AYlog[1000],AZlog[1000],GZlog[1000];
__attribute((section("AHBSRAM0"),aligned))  float presslog[1000];
__attribute((section("AHBSRAM0"),aligned))  int16_t GXlog[1000],GYlog[1000];
//short tempclog3[1000];
__attribute((section("AHBSRAM0"),aligned)) short tempclog[1000];
unsigned timelog[1000];
int tabalt[1000];
unsigned short pitot_val[1000];
unsigned int indexm;
char etat =0;
bool flagMessage = false;
void led(char a)
{
    switch (a) {
    case 0 :  myled1=0;myled2=0; myled3=0; myled4=0; break;
    case 1 :  myled1=1;myled2=0; myled3=0; myled4=0; break;
    case 2 :  myled1=0;myled2=1; myled3=0; myled4=0; break;
    case 3 :  myled1=0;myled2=0; myled3=1; myled4=0; break;
    case 4 :  myled1=0;myled2=0; myled3=0; myled4=1; break;
    case 5 :  myled1=1;myled2=1; myled3=1; myled4=1; break;
    }
}
void setup()
{
    // Use the begin() function to initialize the LSM9DS0 library.
    uint16_t status = dof.begin();
   //Make sure communication is working
//     bt.puts("LSM9DS0 WHO_AM_I's returned: 0x\n\r",status);
//     bt.puts("Should be 0x49D4\n\r");
// }
}
void callback(){
       if(bt.readable()){
        switch (bt.getc()&0xFF) {
            case '1': 
                    flagMessage=true;
                    break;
            case 's':
            case 'S':

                    break;
            default : 
                    break;
        }
       
    }
}
void mesure() // fonction de mesure et d'enregistrment 
{
    tempC = sensor.getTemperature();
    pressure = sensor.getPressure();
    alti = (double)(4433076.923*(  1-pow( double(pressure/p_init) , double(0.1902949572) )));
    altimoyenne= (altimoyenne+alti)/2;
    dof.readAccel();
    dof.readGyro();
    Ax =dof.ax;
    Ay =dof.ay;
    Az =dof.az;
    Gx=dof.gx;
    Gy=dof.gy;
    Gz=dof.gz;
}
int mesure_pitot()
{
    i2c_2.write(0xEA,&bcom,1); 
    wait_ms(10);
    i2c_2.read(0xEA,(char *) &reading,2);
    //printf("%u\r\n",reading&0xFFFF);
    //wait_ms(800); 
    return reading&0xFFFF;
}
volatile char flag50ms;
void func(){ //CETTE FONCTION EST EXECUTEE TOUTES LES 50ms et met flag50ms à 1
    flag50ms=1;
}

Timer timex;
Ticker t1;
char buffer[128];
int main(){  
    flagMessage = false;
    sig.period_ms(20);
    sig.pulsewidth_us(1800); // position initial du servo pour fermer la gachette 
    //Setup sensor
    bt.attach(&callback);
    setup();  
//setup bypass mode
        dof.setBYPASSmode();
    bt.puts("bypass mode activated\n\r");
    t1.attach(&func,0.05); //initialise le ticker pour que la fonction func soit exectuee tous les 50ms
    i2c.frequency(400000);
    wait(1.0);
    timex.reset();
    timex.start();
    led(0);
    while(1) {
       if(flag50ms == 1){
           
            flag50ms=0;
            if(xbee_secure==1){
                sig.pulsewidth_us(600); // ouverture du parachute
                time_parachute=timex.read_ms();
                 etat=3;
            }
            switch (etat&0xFF){
                case 0 : 
                        //calcule P0  
                        for ( int i=0; i<100; i++) { // calcule de la pression P0 en prenant la moyenne sur 100 mesures 
                                t_reg = sensor.getTemperature(); 
                                p_init = p_init + sensor.getPressure();
                                //pitot_init=pitot_init+ mesure_pitot();
                                wait(0.05);
                                myled1=!myled1;
                        }
                        p_init=p_init/100;
                        pitot_init=pitot_init/100;
                        altimoyenne=0;
                       // printf(" \r\n la pression initiale est de : %06.3f \r\n",p_init);
                        etat = 1; // on passe a l'etat suivant apres l'initialisation ( ca peut egalement se faire au moyen d'une iteruption exemple fil attaché a la terre )
                        
                        break;
                case 1 : 
                        led(1); // etat de repos en attendant de mesurer et d'enregistrer
                        if ((bouton==0) || (flagMessage)){
                                flagMessage=false;
                                etat = 2;
                                led(2); // la led s'allume au passage a l'etat 2 ==> debut de l'enregistrement  
                                wait(1); // le wait est pour éviter de basculer d'un état à l'autre trop vite car on maintient le bouton appuyé pendant plus de 50 ms
                        }
                        break;
                case 2 : // etat de mesure, on enregistre le temps/temperature/pression. si altitude max atteinte on attend 10 m pour ouvrir                 
                        mesure();
                        timelog[indexm]= timex.read_ms();
                        tempclog[indexm]= (short)(tempC*100);
                        presslog[indexm] = pressure;
                        AXlog[indexm]=Ax;
                        AYlog[indexm]=Ay;
                        AZlog[indexm]=Az;                        
                        GXlog[indexm]=Gx;
                        GYlog[indexm]=Gy;
                        GZlog[indexm]=Gz;                        
                        tabalt[indexm]=alti;
                        pitot_val[indexm]=mesure_pitot();
                        indexm++;
                        
                        if(alti>=alti_max) {
                                alti_max=alti;
                        }else{
                                if((alti<alti_max-DECISION_ALT) && (alti>=1000)) { //distance en cm
                                        sig.pulsewidth_us(600); // ouverture du parachute
                                        time_parachute=timex.read_ms();
                                        etat=3;
                                }
                        }
                        if(indexm>=1000) {
                                    // stoper l'enregistrement si on depasse les 1000 echantillons  
                                    etat = 4;
                        }
                        // if ((bouton==0) || flagMessage){
                        //         flagMessage=false;
                        //         etat = 4;
                        //         wait(1); // le wait est pour éviter de basculer d'un état à l'autre trop vite car on maintient le bouton appuyé pendant plus de 50 ms
                        // }
                        break;
                case 3 :
                        led(2);
                        mesure();
                        timelog[indexm]= timex.read_ms();
                        tempclog[indexm]= (short)(tempC*100);
                        presslog[indexm] = pressure;
                        AXlog[indexm]=Ax;
                        AYlog[indexm]=Ay;
                        AZlog[indexm]=Az;
                        GXlog[indexm]=Gx;
                        GYlog[indexm]=Gy;
                        GZlog[indexm]=Gz;                        
                        tabalt[indexm]=alti;
                        pitot_val[indexm]=mesure_pitot();
                        indexm++;
                        if((bouton==0 )|| (flagMessage)) {
                            flagMessage=false;
                                    // stoper l'enregistrement si on depasse les 1000 echantillons 
                                etat = 4;
                        }
                        if(indexm>=1000) {
                                    // stoper l'enregistrement si on depasse les 1000 echantillons  
                                    etat = 4;
                        };
                        break;
                case 4 :  //fin du travail et enregistrement
                        //ouverture du fichier
                        myled2=0;
                        FILE *p = fopen("/local/out.txt", "w");
                        //fprintf(p,"pitot_val initial : %5u\r\n",pitot_init);
                        fprintf(p,"temps parachute : %5u\r\n",time_parachute);
                        fprintf(p,"pression initiale : %f\r\n",p_init);
                        fprintf(p,"alti_max : %f\r\n",alti_max);
                        fprintf(p,"Time(ms)|temp(1/100 deg C)|Pre(Pa)|Alti(cm)|pitot_val\r\n");
                        for(int i=0; i<indexm;i++){
                                fprintf(p,"%5u|%5d|%5d|%5d|%u\r\n",timelog[i],(int)(tempclog[i]),(int)(presslog[i]*100),tabalt[i],pitot_val[i]);
                        }
                        fclose(p); 

                        FILE *p4 = fopen("/local/out_accel.txt", "w");
                        fprintf(p4,"Time(ms)|Ax/100|Ay/100|Az/100\r\n");
                         for(int i=0; i<indexm;i++){
                                fprintf(p4,"%5u|%d|%d|%d\r\n",timelog[i],(int)AXlog[i],(int)AYlog[i],(int)AZlog[i]);
                        }
                        fclose(p4);

                        FILE *p5 = fopen("/local/out_gyro.txt", "w");
                        fprintf(p5,"Time(ms)|Gx/100|Gy/100|Gz/100\r\n");
                        for(int i=0; i<indexm;i++){
                                fprintf(p5,"%5u|%d|%d|%d\r\n",timelog[i],GXlog[i],GYlog[i],GZlog[i]);
                        }
                        fclose(p5);

                        etat=5;
                        myled1=0;
                        myled2=0;
                        break;
                case 5 :
                        if ((bouton==0) || flagMessage){
                                    etat = 6;
                                    flagMessage=false;
                                    //bt.puts("fichier en transfert...\r\n");
                                    wait(1); // le wait est pour éviter de basculer d'un état à l'autre trop vite car on maintient le bouton appuyé pendant plus de 50 ms
                        }
                        led(3);                        
                        break;
                case 6 : // transfert des données par bluetooth
                        led(4); 
                        FILE *fp = fopen("/local/out.txt", "r");
                        if(!fp) { 
                             error("Could not open file!\n");
                        }              
                        while(fgets(buffer, 128, fp)) {
                    //printf("Line: %s\n", buffer);
                                bt.puts(buffer);
                                wait_ms(5);
                        }
                //wait(2);
                        fclose(fp);
                        bt.puts("\n#\n");

                        wait(1);

                        FILE *fp4 = fopen("/local/out_accel.txt.txt", "r");
        
                        if(!fp4) { 
                                 error("Could not open file!\n");
                        }

                        while(fgets(buffer, 128, fp4)) {
                            //printf("Line: %s\n", buffer);
                            bt.puts(buffer);
                            wait_ms(5);
                        }
                        //wait(2);
                        fclose(fp4);
                        bt.puts("\n#\n");

                        wait(1);

                        FILE *fp5 = fopen("/local/out_gyro.txt.txt", "r");
                
                        if(!fp5) { 
                            error("Could not open file!\n");
                        }
                        while(fgets(buffer, 128, fp5)) {
                            //printf("Line: %s\n", buffer);
                            bt.puts(buffer);
                            wait_ms(5);
                        }
                        //wait(2);
                        fclose(fp5);
                        bt.puts("\n#\n");

                        //printf("Done\n");
                        etat = 7;
                        break;     
                case 7 : 
                //bt.puts("fichier transfere\r\n");
                        while (1) {
                            led(5);
                        }
                         break;  
                default :
                        break;
            }  
        }
    }
}

