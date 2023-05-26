/*
   Energy LED Pole

   R0 - initial version, got array reading and some processing of valyes
   R1 - corrected processing of values, eg
        50 0 0  > 0,5   6,6   6,6
        0 50 0  > 0,0   0,5   6,6
        0 0 50  > 0,0   0,0   0,5
        50 50 50> 0,5   6,11  11,16  ** < need to fix this, should be 12,17
        10000 10000 2000    > 0,137   138,143   143,144 ** same as above, but otherwise OK

  R2 - Added UDP receive to see how it gets on with real data.  Works with Neopixels quite well but...
        Need to blank unused pixels, need to address gaps between colours (may be same issue)

  R3 - Fixed aboce R1 and R2 issues - was overcomplicating it, no need to add gaps in array, eg
        0,137   137,143   143,144  does actually work correctely
       Changed binning

  R4 - Changed to array building whats getting output and addedd a final LED dimming function depending on power...
       not sure it was worth it!   Probably needs some gamma/log correction for proper effect and then exlude first counts where it doesnt light etc
       Spotted the zero mark actually used as 10W mark... can probably live with this.
       Need to understand slow update via UDP, possibly need to tidy up the loop - i.e. remove delay
       The gradiations work well, TBC if the mixing is good or bad, and TBC if hiding a mark unless showing data is also TBC a keeper

  R5 - Changed to reading JSON from the Solis RS485 MODBUS interface


*/


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//******************* LIBRARIES  ******************* //
#include <ArduinoJson.h>
#include <ESP8266HTTPClient.h>
#include <Arduino_JSON.h>
#include <ESP8266WiFi.h>
#include <Adafruit_NeoPixel.h>

//============ WiFi ===============

const char *ssid = "private";
const char *password = "private";

IPAddress local_IP(192, 168, 1, 111);
IPAddress gateway(192, 168, 1, 1);
IPAddress subnet(255, 255, 255, 0);
IPAddress primaryDNS(8, 8, 8, 8);   //optional
IPAddress secondaryDNS(8, 8, 4, 4); //optional

/////////////////////////////////////////////////////////
//******************  RECEIVE DATA  ****************** //

const char* serverName = "http://192.168.1.149/R/?addr=33135.1,33079.2,33130.2";
  //33135.1,  BATTCD_R  0 charge, 1 discharge
  //33079.2,  activePwr1_R  Inverter active power (always positive, irrespective of discharge)
  //33130.2   GRID_R  +ve export, -ve import, grid meter active power


int BATTCD_R = 0;
int activePwr1_R = 0;
int GRID_R = 0;

#define PIN_NEO_PIXEL  4   // Arduino pin that connects to NeoPixel
#define NUM_PIXELS     144  // The number of LEDs (pixels) on NeoPixel

Adafruit_NeoPixel NeoPixel(NUM_PIXELS, PIN_NEO_PIXEL, NEO_GRB + NEO_KHZ800);

word liveArray[144][4];   //create array to write to strip  R  |  G  |  B  |  Brightness

word powerArray[144][4];

word identsArray[144][4] = {   //array for the idents/gadiation markings

  0, 25, 25, 25,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  25, 25, 25, 25,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 25, 25, 25,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  25, 25, 25, 25,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  25, 0, 25, 25,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  25, 25, 25, 25,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  25, 0, 25, 25,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  25, 25, 25, 25,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  25, 25, 0, 25,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,
  25, 25, 25, 25,
  0, 0, 0, 0,
  0, 0, 0, 0,
  0, 0, 0, 0,


};


// power values from some place, doesnt need much resolution so int fine
// looking at what the house is using so its not so interested in the grid
int powerHouse = 0;
int powerPV = 0;
int powerBattery = 0;
int powerImport = 0;
int powerExport = 0;

int powerBinPV[2]; // PV bin led position from and to
int powerBinBattery[2]; // Battery bin led position from and to
int powerBinImport[2]; // Import bin led position from and to


// array for the power bin for each LED, i.e. the first LED (0) is 0 to 10 W power
int nBins = 144;
int powerBin[] =
{
  0,
  10,
  20,
  30,
  40,
  50,
  60,
  70,
  80,
  90,
  100,
  110,
  120,
  130,
  140,
  150,
  160,
  170,
  180,
  190,
  200,
  210,
  220,
  230,
  240,
  250,
  260,
  270,
  280,
  290,
  300,
  310,
  320,
  330,
  340,
  350,
  360,
  370,
  380,
  390,
  400,
  410,
  420,
  430,
  440,
  450,
  460,
  470,
  480,
  490,
  500,
  525,
  550,
  575,
  600,
  625,
  650,
  675,
  700,
  725,
  750,
  775,
  800,
  825,
  850,
  875,
  900,
  925,
  950,
  975,
  1000,
  1100,
  1200,
  1300,
  1400,
  1500,
  1600,
  1700,
  1800,
  1900,
  2000,
  2100,
  2200,
  2300,
  2400,
  2500,
  2600,
  2700,
  2800,
  2900,
  3000,
  3100,
  3200,
  3300,
  3400,
  3500,
  3600,
  3700,
  3800,
  3900,
  4000,
  4100,
  4200,
  4300,
  4400,
  4500,
  4600,
  4700,
  4800,
  4900,
  5000,
  5250,
  5500,
  5750,
  6000,
  6250,
  6500,
  6750,
  7000,
  7250,
  7500,
  7750,
  8000,
  8250,
  8500,
  8750,
  9000,
  9250,
  9500,
  9750,
  10000,
  11000,
  12000,
  13000,
  14000,
  15000,
  16000,
  17000,
  18000,
  19000,
  20000,
  21000,
  22000,
  23000
};


void setup()
{
  /////////////////////////////////////////////////////////
  //******************* SERIAL SETUP ******************* //

  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  delay(500); // let serial settle etc

  Serial.print("wdtEnable... > ");
  ESP.wdtEnable(5000);
  Serial.println("... done wdtEnable");

  // Configures static IP address
  if (!WiFi.config(local_IP, gateway, subnet, primaryDNS, secondaryDNS)) {
    Serial.println("STA Failed to configure");
  }

  // Connect to Wi-Fi network with SSID and password
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  // Print local IP address and start web server
  Serial.println("");
  Serial.println("WiFi connected.");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());

  NeoPixel.begin();

  delay(1000);
}

void loop()
{
  delay(100);

  //NeoPixel.clear(); // Set all pixel colors to 'off'


  //=========== JSON HTTP GET receive ============

  if (WiFi.status() == WL_CONNECTED) {
    WiFiClient client;
    HTTPClient http;
    http.begin(client, serverName);
    Serial.println(serverName);

    int httpResponseCode = http.GET();
    Serial.println(httpResponseCode);
    Serial.println(http.getString());

    if (httpResponseCode == 200) {
      String payload = http.getString();
      Serial.println("HTTP GET request successful");
      Serial.println(payload);

      // Parse JSON response
      const size_t capacity = JSON_ARRAY_SIZE(3); // Adjust the capacity based on your JSON structure
      DynamicJsonDocument doc(capacity * 2);

      DeserializationError error = deserializeJson(doc, payload);

      if (error) {
        Serial.print("JSON parsing error: ");
        Serial.println(error.c_str());
      } else {
        // Access the "data" array
        JsonArray data = doc["data"];

        // Iterate through the array and print the values
        for (JsonVariant value : data) {
          Serial.println(value.as<int>());
        }
        BATTCD_R = data[0]; // 1
        activePwr1_R = data[1]; // 503
        GRID_R = data[2]; // -6

        Serial.print(BATTCD_R);
        Serial.print(",");
        Serial.print(activePwr1_R);
        Serial.print(",");
        Serial.print(GRID_R);
        Serial.println(",");
      }
    } else {
      Serial.print("HTTP GET request failed, error code: ");
      Serial.println(httpResponseCode);
    }

    http.end();
  }


  // Condition JSON data for use here

  if (GRID_R <= 0)
  {
    powerImport = ( GRID_R * -1 );
  }
  else
  {
    powerImport = 0;
  }

  if (BATTCD_R == 1)
  {
    powerBattery = activePwr1_R;
  }
  else

  {
    powerBattery = 0;
  }



  /*  //Read in some dummy data from the serial port
    while (Serial.available() > 0)
    {
      // look for the next valid integer in the incoming serial stream:
      int serVal1 = Serial.parseInt();
      // do it again:
      int serVal2 = Serial.parseInt();
      // do it again:
      int serVal3 = Serial.parseInt();

      if (Serial.read() == '\n')
      {
        powerPV = serVal1;
        powerBattery = serVal2;
        //powerImport = serVal3;
      }
  */

Serial.print("PV, Battery, Import W: ");
Serial.print(powerPV);
Serial.print(",");
Serial.print(powerBattery);
Serial.print(",");
Serial.println(powerImport);

//===========================================
//LED mapping to power=======================

// Assess LED positions for PV power
for ( int j = 0; j < nBins; ++j ) //
{
  if (powerBin[j] >= powerPV) // for each powerBin value, find if we have found the bin for the current power
  {
    powerBinPV[0] = 0; // as PV will always be used "first" it will start at zero
    powerBinPV[1] = j; // this is the upper bin number, could be zero eg if theres no generation
    break;
  }
}

// Assess LED positions for battery power

powerBinBattery[0] = powerBinPV[1];

for ( int j = 0; j < nBins; ++j ) //
{
  if (powerBin[j] >= (powerPV + powerBattery) ) // for each powerBin value, find if we have found the bin for the current power
  {
    powerBinBattery[1] = j; // this is the upper bin number, could be zero eg if theres no battery or PV
    break;
  }
}

// Assess LED positions for grid import power
powerBinImport[0] = powerBinBattery[1];

for ( int j = 0; j < nBins; ++j ) //
{
  if (powerBin[j] >= (powerPV + powerBattery + powerImport) ) // for each powerBin value, find if we have found the bin for the current power
  {
    powerBinImport[1] = j; // this is the upper bin number, could be zero eg if theres no battery or PV.. or no grid.
    break;
  }
}

// here we should know what numbers are being turned on for which input.

Serial.print("PV Bins: ");
for ( int j = 0; j < 2; ++j ) //
{
  Serial.print(powerBinPV[j]);
  Serial.print(",");
}
Serial.println("");

Serial.print("Battery Bins: ");
for ( int j = 0; j < 2; ++j ) //
{
  Serial.print(powerBinBattery[j]);
  Serial.print(",");
}
Serial.println("");

Serial.print("Import Bins: ");
for ( int j = 0; j < 2; ++j ) //
{
  Serial.print(powerBinImport[j]);
  Serial.print(",");
}
Serial.println("");


// need to do some sense checking before writing values.   Eg there is nothing to write if from and to are equal or from is bigger than to etc

// Check on writing any PV LEDs
int writePV = 0;
if (powerBinPV[0] == powerBinPV[1])
{
  writePV = 0;
}

else if (powerBinPV[0] >= powerBinPV[1])
{
  writePV = 0;
}

else
{
  writePV = 1;
}

// Check on writing any battery LEDs
int writeBattery = 0;
if (powerBinBattery[0] == powerBinBattery[1])
{
  writeBattery = 0;
}

else if (powerBinBattery[0] >= powerBinBattery[1])
{
  writeBattery = 0;
}

else
{
  writeBattery = 1;
}

// Check on writing any grid import LEDs
int writeImport = 0;
if (powerBinImport[0] == powerBinImport[1])
{
  writeImport = 0;
}

else if (powerBinImport[0] >= powerBinImport[1])
{
  writeImport = 0;
}

else
{
  writeImport = 1;
}


// Generate array for PV into the power Array   (sets colour and brightness)
if (writePV == 1)
{
  for (int pixel = powerBinPV[0]; pixel < powerBinPV[1]; pixel++)
  { // for each pixel
    powerArray[pixel][0] = 0; // red
    powerArray[pixel][1] = 128; // green
    powerArray[pixel][2] = 0; // blue
    powerArray[pixel][3] = 255; // brightness
  }
}

// Generate array for battery into the power Array   (sets colour and brightness)
if (writeBattery == 1)
{
  for (int pixel = powerBinBattery[0]; pixel < powerBinBattery[1]; pixel++)
  { // for each pixel
    powerArray[pixel][0] = 0; // red
    powerArray[pixel][1] = 0; // green
    powerArray[pixel][2] = 128; // blue
    powerArray[pixel][3] = 255; // brightness
  }
}

// Generate array for import into the power Array   (sets colour and brightness)
if (writeImport == 1)
{
  for (int pixel = powerBinImport[0]; pixel < powerBinImport[1]; pixel++)
  { // for each pixel
    powerArray[pixel][0] = 128; // red
    powerArray[pixel][1] = 0; // green
    powerArray[pixel][2] = 0; // blue
    powerArray[pixel][3] = 255; // brightness
  }
}


// Fade last pixel based on difference between last full bin and the total power
int lastBinNo = powerBinImport[1];

word totalPower = powerPV + powerBattery + powerImport;   // sum of all the power
word lastBinValue = powerBin[lastBinNo];  // what is the wattage of the last bin which was lit (this is the intensity we need to reduce
word powerBinStep = powerBin[lastBinNo + 1] - lastBinValue ; // The last lit pixel, whats its wattage step
word powerBinDelta = totalPower - (lastBinValue - powerBinStep) ;

word lastRed = powerArray[lastBinNo - 1][0];
word lastGreen = powerArray[lastBinNo - 1][1];
word lastBlue = powerArray[lastBinNo - 1][2];


if (powerBinStep == 0)
{
  powerBinStep = 1; //avoid devide by zero error
}

word partialRed = (powerBinDelta * lastRed) / powerBinStep;
word partialGreen = (powerBinDelta * lastGreen) / powerBinStep;
word partialBlue = (powerBinDelta * lastBlue) / powerBinStep;

powerArray[lastBinNo - 1][0] = partialRed; // red
powerArray[lastBinNo - 1][1] = partialGreen; // green
powerArray[lastBinNo - 1][2] = partialBlue; // blue


Serial.print("Fade really last pixel: ");
Serial.print(totalPower);
Serial.print(",");
Serial.print(lastBinValue);
Serial.print(",");
Serial.print(powerBinStep);
Serial.print(",");
Serial.print(powerBinDelta);
Serial.print(",,");

Serial.print(lastRed);
Serial.print(",");
Serial.print(lastGreen);
Serial.print(",");
Serial.print(lastBlue);
Serial.print(",,");

Serial.print(partialRed);
Serial.print(",");
Serial.print(partialGreen);
Serial.print(",");
Serial.print(partialBlue);
Serial.print(",");


Serial.print(",,");

// Generate blanking of pixels which shouldnt be lit
for (int pixel = lastBinNo; pixel < nBins; pixel++)
{ // for each pixel
  powerArray[pixel][0] = 0; // red
  powerArray[pixel][1] = 0; // green
  powerArray[pixel][2] = 0; // blue
  powerArray[pixel][3] = 0; // brightness
}


// Combine the ident markers and power array values to be what should be written to the pixels
for (int pixel = 0; pixel < nBins; pixel++)
{
  liveArray[pixel][0] = ( identsArray[pixel][0] + powerArray[pixel][0] ) / 2;
  liveArray[pixel][1] = ( identsArray[pixel][1] + powerArray[pixel][1] ) / 2;
  liveArray[pixel][2] = ( identsArray[pixel][2] + powerArray[pixel][2] ) / 2;
  liveArray[pixel][3] = ( identsArray[pixel][3] + powerArray[pixel][3] ) / 2;
}

writePixels(); //sub to write the liveArray

//NeoPixel.show();
}




void writePixels()
{
  for (int pixel = 0; pixel < NeoPixel.numPixels(); pixel++)
  {
    word brightness = liveArray[pixel][3];

    word red = ( liveArray[pixel][0] * brightness ) / 255;
    word green = ( liveArray[pixel][1] * brightness ) / 255;
    word blue = ( liveArray[pixel][2] * brightness ) / 255;


    NeoPixel.setPixelColor(pixel, NeoPixel.Color(red, green, blue));

    NeoPixel.show();
    //delay(1);
  }
}
