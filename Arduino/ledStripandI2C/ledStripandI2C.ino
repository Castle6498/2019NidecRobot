
#include <FastLED.h>
#include <Wire.h>


#if defined(FASTLED_VERSION) && (FASTLED_VERSION < 3001000)
#warning "Requires FastLED 3.1 or later; check github for latest code."
#endif


#define DATA_PIN     13
#define LED_TYPE    WS2812
#define COLOR_ORDER GRB
#define NUM_LEDS    14
CRGB leds[NUM_LEDS];

#define BRIGHTNESS          255 //0 to 255
#define FRAMES_PER_SECOND  80 //was 120


//SETUP THE DEVICES

//plug sda on RoboRIO into A4
//plug scl on RoboRIO into A5
//connect the two grounds

static String _piOutput = "blank";//string to be sent to the robot

               
static String input = "IDLE";  //string received from the robot



void setup() {
  delay(2000);
   
   // tell FastLED about the LED strip configuration
  FastLED.addLeds<LED_TYPE,DATA_PIN,COLOR_ORDER>(leds, NUM_LEDS).setCorrection(TypicalLEDStrip);
  
  //Serial.begin(9600);

  FastLED.setBrightness(BRIGHTNESS);
  
   
   Wire.begin(4);                // join i2c bus with address #4 as a slave device
   Wire.onReceive(receiveEvent); // Registers a function to be called when a slave device receives a transmission from a master
   Wire.onRequest(requestEvent); // Register a function to be called when a master requests data from this slave device

}


CRGB color=CRGB::Blue;
int stripeLocation=0;
int stripeWidth=4;
boolean inputChange=false;

void loop() {
  

  
  if(input=="IDLE"){
    sinelon();
    // send the 'leds' array out to the actual LED strip
    FastLED.show();  
    // insert a delay to keep the framerate modest
    FastLED.delay(1000/FRAMES_PER_SECOND); 
  }else if(input=="SHOOT"){
    if (inputChange) stripeLocation=0; //reset stripe

     for (int dot=0; dot < NUM_LEDS; dot++) { 
      leds[dot] = CRGB::Blue; //SET all to blue
    }
    for(int i=0; i<stripeWidth;i++){
      int stripeDot=stripeLocation+i;
      if(stripeDot<NUM_LEDS&&stripeDot>=0){ //prevent from going out of bounds
        leds[stripeDot]=CRGB::Yellow; //set only stripe to yellow
      }
    }

    stripeLocation++;
    if(stripeLocation>=NUM_LEDS){
      stripeLocation=1-stripeWidth; //make the strip appear to be entering
    }

    FastLED.show();
    delay(30); //speed of stripe
    
  }

}


void sinelon()
{
  // a colored dot sweeping back and forth, with fading trails
  fadeToBlackBy( leds, NUM_LEDS, 20);
  int pos = beatsin16( 30, 0, NUM_LEDS-1 );

  leds[pos] += color;
}

void requestEvent(){//called when RoboRIO request a message from this device
  Wire.write(_piOutput.c_str()); //writes data to the RoboRIO, converts it to string

}

void receiveEvent(int bytes){//called when RoboRIO "gives" this device a message
  String holder="";  
  while(Wire.available() > 0) // loop through all but the last
  {
    char c = Wire.read(); // receive byte as a character
    holder.concat(c);
   delay(10);
    
  }
  if(holder!=input){
    input=holder;
    inputChange=true;
    Serial.println(input);
  }else{
    inputChange=false;
  }
  
  delay(100);//was 500
}
