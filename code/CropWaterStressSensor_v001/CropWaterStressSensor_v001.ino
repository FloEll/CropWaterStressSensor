/*
  Calculate the CWSI (Crop Water Stress Index) based on thermal images
  By: Florian Ellsäßer
  Justus-Liebig University Gießen, Germany
  Email: florian.ellsaesser@zeu.uni-giessen.de
  Date: August 29th, 2022
  License: MIT. 

  This sketch is conceptualized for the ESP32 NodeMCU-32s and initializes the MLX90640, 
  a DS1307 real time clock (RTC), a 3.2" ILI9341 screen (not the touch screen version) with SD card slot
  and a strip of three WS2812B NeoPixels as hardware components. Please refer to the github project
  for wiring instructions and updates. 

  This relies on the driver written by Melexis and can be found at:
  https://github.com/melexis/mlx90640-library

  This sketch was written based on information and examples from by Nathan Seidle (Read the temperature 
  pixels from the MLX90640 IR array) from the SparkFun Electronics MLX90640 example libraries, a video 
  by ShotokuTech https://www.youtube.com/watch?v=pYp1Nxmfdeg for the screen and SD card implementation and
  the Adafruit buttoncycler example as found in the Adafruit Neopixel library. The trends are calculated
  using the slope of a linear regression based on this post:
  https://jwbrooks.blogspot.com/2014/02/arduino-linear-regression-function.html
  Many thanks for publishing such great examples and tutorials to the authors. 

  Disclaimer: 
  This sketch has been tested on the described hardware setup and in the field under actual 
  field conditions in the agricultural research station "Weilburger Grenze" in Gießen, Germany. However,
  it has not been tested against other similar equipment in a scientificaly sound experiments or anything 
  similar yet. If you plan to use this project for your research please be aware that this is still
  very experimental, not tested against established methods so far (I'm currently doing that) and therefore
  more like an exemplary showcase than an actual scientific tool. No guarantees are given from my side. 
*/

// import the libraries
#include "Wire.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "TFT_eSPI.h"
#include "SD.h"
#include "DS1307.h"                       
#include "Adafruit_NeoPixel.h"

// define the RTC object
DS1307 rtc; 

// define the MLX90640 object
// define 7-bit default address -> change in case this conflicts with other addresses
const byte MLX90640_address = 0x33; 
// define default shift for MLX90640 in open air
#define TA_SHIFT 8 //Default shift 
static float mlx90640To[768];
paramsMLX90640 mlx90640;

// define NeoPixel reference handle, signal pin and total number and object
#define GFXFF 1
#define PIN 27 
#define N_LEDS 3 
Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_LEDS, PIN, NEO_GRB + NEO_KHZ800);

// define TFT display object for hardware SPI
TFT_eSPI tft = TFT_eSPI();

// define temperatures array
float myArray[768];
// create variables for Tmin, Tmax, Tmean, the cwsi and the daily mean cwsi
float maxVal;
float minVal;
float meanVal;
float cwsi;
float mean_cwsi = 0;

// the thermal emissivity of plant is usually around 0.98. However if you have specifications 
// that differ from this general value, you can define them here
float emissivity = 0.98;

// create arrays for daily average, last reading, the past 14 days averages, and arrow components
float today_average = 0.83;
float last_reading = 0.77;
float pastArray[14];
int pixel_num;
int pixel_pos;
int arrow[35] = {130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130,130};
float trend;
String first_log = "";

// define the logging time variables
unsigned long startMillis;
unsigned long currentMillis;
// this value defines thelogging cycle in milliseconds 
const unsigned long period = 900000; //this is the equivalent of 15 min in milliseconds -> this is the logging cycle
int start_token =1;

// start the setup function
void setup()
{ // start the I²C bus
  Wire.begin();
  // increase I²C clock speed
  Wire.setClock(400000); 
  // start Serial Bus
  Serial.begin(115200);
  // Set all chip selects high to avoid bus contention during initialisation of each peripheral
  // digitalWrite(22, HIGH); // Touch controller chip select (if used)
  digitalWrite(15, HIGH); // TFT screen chip select
  digitalWrite( 5, HIGH); // SD card chips select, must use GPIO 5 (ESP32 SS)
  // Wait for user to open terminal
  while (!Serial); 
  Serial.println("Crop Water Stress Sensor");

  // check if the MLX90640 camera is connected
  if (isConnected() == false)
  {
    Serial.println("MLX90640 not detected at default I2C address.");
    while (1);
  }
  Serial.println("MLX90640 works!");

  // get MLX90640 parameters
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  // print to Serial if the parameters were received
  if (status != 0)
    Serial.println("Failed to load system parameters");
  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0)
    Serial.println("Parameter extraction failed");

  // start the TFT display
  tft.begin();
  // rotate the display orientation by 90 degrees
  tft.setRotation(1);
  // fill the entire screen with black 
  tft.fillScreen(TFT_BLACK);
  // run the refresh_screen function to get the screen background
  refresh_screen();
  
  // start the RTC 
  rtc.begin();
  // outcomment this section to calibrate the RTC and adjust to current date
    /*
    rtc.fillByYMD(2022,8,31);
    rtc.fillByHMS(17,39,30);
    rtc.fillDayOfWeek(WED);
    rtc.setTime();
    */

    // start the SD card writer
    if (!SD.begin()) {
    // print to serial if that worked
    Serial.println("Card Mount Failed");
    return;
  }
  // get the SD card type and store into variable
  uint8_t cardType = SD.cardType();
  // print to Serial if that didn't work
  if (cardType == CARD_NONE) {
    Serial.println("No SD card attached");
    return;
  }

  // Define card type and print results to Serial
  Serial.print("SD Card Type: ");
  if (cardType == CARD_MMC) {
    Serial.println("MMC");
  } else if (cardType == CARD_SD) {
    Serial.println("SDSC");
  } else if (cardType == CARD_SDHC) {
    Serial.println("SDHC");
  } else {
    Serial.println("UNKNOWN");
  }
  // define card size and print results to Serial
  uint64_t cardSize = SD.cardSize() / (1024 * 1024);
  Serial.printf("SD Card Size: %lluMB\n", cardSize);
  Serial.println("initialisation done.");

  // now two files are created. The datalog.csv file with all individual measurements
  // and the dailylog.csv file that logs the calculated daily cwsi mainly for the 
  // displaysed statistics on the screen. The files are freshly created if they don't 
  // exist on the SD card. If these files exist, nothing new is created and the new 
  // measurement are just added to the existing files
  
  // check if the log file already exists
  if (SD.exists("/datalog.csv")){
    // if it exists, just return
    return;
  }
  // however if the log file doesn't exist create a new one and write a header
  else {
  // write a header for the data file
  File file = SD.open("/datalog.csv", FILE_WRITE);
  file.print("datetime,Tmax,Tmin,Tmean,cwsi,\n");
  file.close();
  }

    // check if the dailylog file already exists
  if (SD.exists("/dailylog.csv")){
    // if it exists, just return
    Serial.println("File exists already");
    return;
  }
  // however if the log file doesn't exist create a new one and write a header
  else {
  // write a header for the data file
  File file = SD.open("/dailylog.csv", FILE_WRITE);
  file.println("date,mean_cwsi");
  file.close();
  Serial.println("Created new file!");
  }

  // start the NeoPixels
  strip.begin();

  // initial start time + time period to start straight away
  startMillis = millis()+period;
  
}

// start the loop function
void loop()
{ //get the current "time" (actually the number of milliseconds since the program started)
  currentMillis = millis();  
  if (currentMillis - startMillis >= period || start_token ==1)  
  { start_token = 0;
  // first we get the current time from the RTC 
  rtc.getTime();
  // print to Serial 
  Serial.print(rtc.hour, DEC);
  Serial.print(":");
  Serial.print(rtc.minute, DEC);
  Serial.print(":");
  Serial.print(rtc.second, DEC);
  Serial.print("  ");
  Serial.print(rtc.month, DEC);
  Serial.print("/");
  Serial.print(rtc.dayOfMonth, DEC);
  Serial.print("/");
  Serial.print(rtc.year+2000, DEC);
  Serial.print(" ");
  Serial.print(rtc.dayOfMonth);
  Serial.print("*");
  // also print the day of the week
  switch (rtc.dayOfWeek)
    {
        case MON:
        Serial.print("MON");
        break;
        case TUE:
        Serial.print("TUE");
        break;
        case WED:
        Serial.print("WED");
        break;
        case THU:
        Serial.print("THU");
        break;
        case FRI:
        Serial.print("FRI");
        break;
        case SAT:
        Serial.print("SAT");
        break;
        case SUN:
        Serial.print("SUN");
        break;
    }
    Serial.println(" ");

  // This section reads the MLX90640s both subpages
  for (byte x = 0 ; x < 2 ; x++) 
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0)
    {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
      delay(2000);
    }
    // save into variable
    float vdd = MLX90640_GetVdd(mlx90640Frame, &mlx90640);
    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);
    // reflected temperature based on the sensor ambient temperature
    float tr = Ta - TA_SHIFT; 

    // calculate temperatures 
    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
  }

// make a list for all pixel temperatures and print to Serial
  for (int x = 0 ; x < 768 ; x++)
  {
    // add to the array
    myArray[x] = mlx90640To[x];
  }

  // get the smallest and the highest value from the thermal image and sum up all values
  for (int i = 0; i < (sizeof(myArray) / sizeof(myArray[0])); i++) {
      maxVal = max(myArray[i],maxVal);
      minVal = min(myArray[i],minVal);
      meanVal = meanVal + myArray[i];
  }
  // now take the sum of all temperatures and devide by 
  meanVal = meanVal/768;
  // print the values to Serial
  Serial.print("Max Temp: ");
  Serial.println(maxVal);
  Serial.print("Min Temp: ");
  Serial.println(minVal);
  Serial.print("Mean Temp: ");
  Serial.println(meanVal);

  // calculate the crop water stress index (cwsi)
  cwsi = ( meanVal - minVal ) /  ( maxVal - minVal );
  // print the results to serial
  Serial.print("CWSI: ");
  Serial.println(cwsi);

  // write results to csv file
  File file = SD.open("/datalog.csv", FILE_APPEND);
    
  // the data should always have the same length -> create empty arrays to fill with the data
  char timeStamp[20];
  char cwsi_data[5]; 
  char maxVal_data[6]; 
  char minVal_data[6]; 
  char meanVal_data[6]; 

  // use the sprintf function to write lines that are exactly of the same length
  sprintf(timeStamp, "%04d.%02d.%02d-%02d:%02d:%02d", rtc.year+2000,rtc.month,rtc.dayOfMonth,rtc.hour,rtc.minute,rtc.second);
  sprintf(cwsi_data, "%04s", String(cwsi));
  sprintf(maxVal_data, "%05s", String(maxVal));
  sprintf(minVal_data, "%05s", String(minVal));
  sprintf(meanVal_data, "%05s", String(meanVal));
  

  // an entire line should have 20+4+5+5+5+5 -> 44 characters
  file.print(timeStamp);
  file.print(",");
  file.print(cwsi_data);
  file.print(",");
  file.print(maxVal_data);
  file.print(",");
  file.print(minVal_data);
  file.print(",");
  file.print(meanVal_data);
  file.print("\n");
  // close the file again
  file.close();

  // use the NeoPixel traffic light to indicate the current crop status in a very simple way
  if (cwsi <=0.33){
    strip.setPixelColor(0,strip.Color(0,255,0));
    strip.setPixelColor(1,0);
    strip.setPixelColor(2,0);
  } else if (cwsi > 0.33 && cwsi < 0.66){
    strip.setPixelColor(0,0);
    strip.setPixelColor(1,strip.Color(230,80,0));
    strip.setPixelColor(2,0);
  } else{
    strip.setPixelColor(0,0);
    strip.setPixelColor(1,0);
    strip.setPixelColor(2,strip.Color(255,0,0));
  } 
  // show the NeoPixel strip in the trafic light layout
  strip.show();

  // the following section prints the results to the TFT screen
  // first the background is refreshed
  refresh_screen();

  // todays average
  // calculate todays average from all logged values today

  // draw the average to the screen
  tft.drawString("Todays average:" , 0,30, GFXFF);
  tft.drawString(String(mean_cwsi) , 100,30, GFXFF);
  // draw the last reading to the screen
  tft.drawString("Last reading:" , 0,40, GFXFF);
  tft.drawString(String(cwsi) , 100,40, GFXFF);

  // draw the latest log time to the display
  tft.drawString("Log time:" , 140,30, GFXFF);
  tft.drawString(timeStamp, 200,30, GFXFF);
  
  // draw the first log time from the datalog to the screen
  // get the first time stamp that was logged
  first_log = get_first_log_date();
  // then draw to screen
  tft.drawString("Log begin:" , 140,40, GFXFF);
  tft.drawString(first_log, 200,40, GFXFF);
  
  // draw the past 14 days diagram
  // for this we need to get all the available daily cwsi means and in case 
  // we were not already logging for the last 14 days replace the non existant
  // daily means with nan or zero 
  
  // first we get the total amount of daily logs by reading the entire file and 
  // deviding by row length
  File myFile;
  myFile = SD.open("/dailylog.csv");
  Serial.print("number of daily logs: ");
  // get the entire file as a string
  String myFile_data = myFile.readStringUntil('xyz');
  // define the length of the rows that were loged
  int length_row_2 = 15;
  // now calculate the number of total logs
  int number_of_daily_logs = (myFile_data.length())/length_row_2;
  Serial.println(number_of_daily_logs);
  // close the file:
  myFile.close();

  // check if there are more than 14 days available
  int read_begin; 
  // get the last 14 readings
  if (number_of_daily_logs >= 14){
    read_begin = number_of_daily_logs-14;
  }else{
    read_begin = 0;    
  }
  // then define a counter and add nans to the array for the missing data
  int daily_count=0;
  if (number_of_daily_logs <14){
      int missing_values = 14 - number_of_daily_logs;
      // make a loop that fills the first digits with zeros
      for (int i = 0; i <= missing_values; i++){
        pastArray[daily_count];
        // move the counter one position ahead
        daily_count++;
    }
    }
  // now we for through the actual loged rows and add the daily cwsi means to the array
  for (int i = read_begin; i < number_of_daily_logs; i++){   
    // use the get_daily_cwsi_data function to get the daily cwsi readings
    float daily_cwsi_reading;
    daily_cwsi_reading = get_daily_cwsi_data(i).toFloat();
    //float daily_cwsi_reading = get_cwsi_daily_data(i).toFloat();
    Serial.println(daily_cwsi_reading);
    // add new value to the array 
    pastArray[daily_count] = daily_cwsi_reading;
    // advance the counter
    daily_count++;  
  }
    
  // add pixels to diagramm
  for (int x = 0 ; x < 14 ; x++)
  { // get x and y position of the pixels 
    pixel_num = 215-pastArray[x]*140 ;
    pixel_pos = 46 + x*15 ;
    // then draw to the cwsi results to the display in black dots 
    tft.fillCircle(pixel_pos,pixel_num,3,TFT_BLACK);
  }

  // draw the trend diagram 
  // get the trend of the last 5 days 
  float trend_array_x[5] = {1,2,3,4,5};
  float trend_array_y[5] = {pastArray[10],pastArray[11],pastArray[12],pastArray[13],pastArray[14]};
  float lrCoef[2]={0,0};

  // call simple linear regression algorithm
  simpLinReg(trend_array_x, trend_array_y, lrCoef, 5);

  // the trend is the slop of the resulting equation 
  trend = lrCoef[1];
  
  // draw the trend results to the screen on top of the arrow diagram and adjust the text color
  if (trend >= 0.1){
    tft.setTextColor(TFT_RED);
    tft.drawString("+"+String(trend*100)+"%" , 265 ,75, GFXFF);
  } else if (trend < 0.1 && trend > -0.1){
    tft.setTextColor(TFT_YELLOW);
    if (trend >= 0){
      tft.drawString("+"+String(trend*100)+"%" , 265 ,72, GFXFF);
    } else{
      tft.drawString(String(trend*100)+"%" , 265 ,75, GFXFF);
    }  
} else {
    tft.setTextColor(TFT_GREEN);
  tft.drawString(String(trend*100)+"%" , 265 ,75, GFXFF);
} 
  // set the text color back to white
  tft.setTextColor(TFT_WHITE);

  // draw circle around trend arrow
  tft.drawCircle(285,120,30,TFT_WHITE);
  // add the trend arrow now 
  if (trend >= 0.1){
    for (int x = 0 ; x < 35 ; x++)
  {
    pixel_num = (arrow[x]-10)+(x-16)*trend*(-1);
    pixel_pos = 268 + x*1;
  tft.drawPixel(pixel_pos,pixel_num,TFT_RED);
  }
  // make an arrowhead
  tft.fillTriangle(298-trend*2,120+18*trend*(-1)-3,298+trend*2,120+18*trend*(-1)+3,303,120+20*trend*(-1),TFT_RED);
  } else if (trend < 0.1 && trend > -0.1){
    for (int x = 0 ; x < 35 ; x++)
  {
    pixel_num = (arrow[x]-10)+(x-16)*trend*(-1);
    pixel_pos = 268 + x*1;
  tft.drawPixel(pixel_pos,pixel_num,TFT_YELLOW);
  }
  // make an arrowhead
  tft.fillTriangle(298-trend*2,120+18*trend*(-1)-3,298+trend*2,120+18*trend*(-1)+3,303,120+20*trend*(-1),TFT_YELLOW);  
  } else {
    for (int x = 0 ; x < 35 ; x++)
  {
    pixel_num = (arrow[x]-10)+(x-16)*trend*(-1);
    pixel_pos = 268 + x*1;
  tft.drawPixel(pixel_pos,pixel_num,TFT_GREEN);
  }
  // make an arrowhead
  tft.fillTriangle(298-trend*2,120+18*trend*(-1)-3,298+trend*2,120+18*trend*(-1)+3,303,120+20*trend*(-1),TFT_GREEN);
  } 

  // reset all the variables
  maxVal = 0;
  minVal = 999;
  meanVal = 0;    
  cwsi = 999;

  // get the daily cwsi mean and save to the dailylog.csv file
  get_daily_cwsi_mean();

  //save the start time to the current state
  startMillis = currentMillis; 
  }  
}

// additional functions //

// this returns true if the MLX90640 is detected on the I²C bus
boolean isConnected()
{
  Wire.beginTransmission((uint8_t)MLX90640_address);
  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK
  return (true);
}


// this function resets the display background
void refresh_screen(){
  // clean/reset the text fields
  tft.fillRect(100,30,40,30,TFT_BLACK);
  tft.fillRect(200,30,120,30,TFT_BLACK);
  
  // Set text datum to middle centre (MC_DATUM)
  tft.setTextDatum(TL_DATUM);
  // Set text colour to white with black background
  tft.setTextColor(TFT_WHITE, TFT_BLACK);

  // set a title
  tft.setTextSize(2);
  tft.drawString("Crop Water Stress Sensor", 18, 0, GFXFF);// Print the string name of the font

  // get the average and last reading
  tft.setTextSize(1);

  // 14 days diagram overview 
  // make title
  tft.drawString("last 14 days:" , 100 ,62, GFXFF);
  // draw diagram axis lines x,y,x,y
  tft.drawLine(40,215,248,215,TFT_WHITE);
  tft.drawLine(40,75,40,215,TFT_WHITE);
  // draw legend
  tft.drawString("cwsi 1" , 0,75, GFXFF);
  tft.drawString("cwsi 0" , 0,209, GFXFF);
  // draw the stress indicator labels
  tft.setTextColor(TFT_RED);
  tft.drawString("large" , 0,90, GFXFF);
  tft.drawString("stress" , 0,100, GFXFF);
  tft.setTextColor(TFT_YELLOW);
  tft.drawString("medium" , 0,135, GFXFF);
  tft.drawString("stress" , 0,145, GFXFF);
  tft.setTextColor(TFT_GREEN);
  tft.drawString("no" , 0,180, GFXFF);
  tft.drawString("stress" , 0,190, GFXFF);
  // reset the text color to white again
  tft.setTextColor(TFT_WHITE);
  // draw the diagram background rectangles in the respective colors
  tft.fillRect(41,75,208,47,TFT_RED);    
  tft.fillRect(41,122,208,47,TFT_YELLOW);    
  tft.fillRect(41,169,208,46,TFT_GREEN); 
  // add x ticks to the diagram
  for (int x = 0 ; x < 14 ; x++)
  {
    pixel_pos = 46 + x*15 ;
    // draw the ticks  
    tft.drawLine(pixel_pos,215,pixel_pos,218,TFT_WHITE);
    // draw the tick labels
    tft.drawString(String((13-x)) , pixel_pos-3,220, GFXFF);    
  }
  // draw the x axis label
  tft.drawString("previous days average", 80,230, GFXFF); 

  // trend diagram
  // make title
  tft.drawString("trend:" , 265 ,62, GFXFF);

}

// this function returns a string with the first logging date 
String get_first_log_date(){
  // re-open the file for reading:
  File myFile;
  myFile = SD.open("/datalog.csv");
  if (myFile) {
    int line_count = 0;
    while (myFile.available()) {
      String line = myFile.readStringUntil('\n');  
      line_count++;
      if (line_count >=2){
        // return only the datetime stamp of the first logging cycle 
        return line.substring(0, 19); 
        // then stop and break
        break;
      }
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening /datalog.csv");
  }
}

// this function returns a daily cswi mean 
float get_daily_cwsi_mean(){
  // get number of todays logging cycles
  int log_cycles_today = rtc.hour*4 + rtc.minute/15;
  Serial.print("Logs today: ");
  Serial.println(log_cycles_today);
    
  // re-open the file for checking the total number of logs
  File myFile;
  myFile = SD.open("/datalog.csv");
  Serial.print("number of logs: ");
  // this will just create a long String from the entire file 
  String myFile_data = myFile.readStringUntil('xyz');
  // remove 31 characters of first row and devide by regular row length 
  int length_row = 43;
  // calculate the file length
  int number_of_logs = (myFile_data.length()-31)/length_row;
  Serial.println(number_of_logs);
  // close the file:
  myFile.close();

  // get all CWSI recordings of today after 6am and before 6pm 
  int start_hour_log_time = 6; //6am
  int end_hour_log_time = 18; //6pm
  int total_hour_log_time = (end_hour_log_time-start_hour_log_time);
  // replace the 4 with a function of logs per hour!! 
  int begin_relevant_logs = log_cycles_today-start_hour_log_time*4;
  int end_relevant_logs = log_cycles_today-end_hour_log_time*4;
  int start_of_the_day = number_of_logs-log_cycles_today;
  Serial.print("Relevant logs today: ");
  Serial.println(begin_relevant_logs);

  // take only the relevant hours in account
  if (begin_relevant_logs >= 0){
     // create an empty array 
     int number_of_todays_relevant_logs = (start_of_the_day+log_cycles_today) - (start_of_the_day+start_hour_log_time*4);
     // check if during log time for daily average
     if (number_of_todays_relevant_logs <= total_hour_log_time*4){    
     Serial.print("Todays logs: ");
     Serial.println(number_of_todays_relevant_logs);
     float cwsi_today[log_cycles_today-start_hour_log_time*4];
     // now fill this empty array 
     int loop_count = 0;
     for (int x = (start_of_the_day+start_hour_log_time*4) ; x < (start_of_the_day+start_hour_log_time*4+number_of_todays_relevant_logs); x++)
      { 
        Serial.print("CWSI test ");
        Serial.print(x);
        //Serial.print("  ");
        //Serial.print(loop_count);
        //Serial.print("  ");
        // get the data
        float cwsi_reading = get_cwsi_data(x).toFloat();
        //String cwsi_reading = get_cwsi_data(x);
        Serial.println(cwsi_reading);
        // add the cwsi readings to the array
        cwsi_today[loop_count] = cwsi_reading;
        // forward 
        loop_count++;
      }
  // sum the cwsi results up         
  float sum_cwsi = 0;
  
  // now take the sum of all temperatures and devide by 
  for (int x = 0 ; x < number_of_todays_relevant_logs ; x++){
    sum_cwsi = sum_cwsi + cwsi_today[x];
    }
  // calculate the mean
  mean_cwsi = sum_cwsi/number_of_todays_relevant_logs;
  Serial.print("Mean CWSI: "); 
  Serial.println(mean_cwsi);

  // if number_of_todays_relevant_logs == 48 write to daily average log file
  if (number_of_todays_relevant_logs == total_hour_log_time*4){
    // write results to csv file
    File file = SD.open("/dailylog.csv", FILE_APPEND);
    // use the sprintf function to write lines that are exactly of the same length
    char timeStamp[10];
    char mean_cwsi_data[5]; 
    
    sprintf(timeStamp, "%04d.%02d.%02d", rtc.year+2000,rtc.month,rtc.dayOfMonth);
    sprintf(mean_cwsi_data, "%04s", String(mean_cwsi));
    
    // print the entire line
    file.print(timeStamp);
    file.print(",");
    file.print(mean_cwsi_data);
    file.print("\n");
    // close the file again
    file.close();
  }
     }else{
      Serial.println("Outside of daily log time!");
      }
  }
}

// this function returns a string with the according cwsi value
String get_cwsi_data(int in_row){
  // re-open the file for reading:
  File myFile;
  myFile = SD.open("/datalog.csv");
  if (myFile) {
    int line_count = 0;
    while (myFile.available()) {
      String line = myFile.readStringUntil('\n');  // \n character is discarded from buffer
      line_count++;
      if (line_count-1 == in_row){
        // return only the datetime stamp of the first logging cycle 
        return line.substring(20, 24); 
        // then stop and break
        break;
      }
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening /datalog.csv");
  }
}

// this function returns a string with the according cwsi value
String get_daily_cwsi_data(int in_row){
   // re-open the file for reading:
  File myFile;
  myFile = SD.open("/dailylog.csv");
  if (myFile) {
    int line_count = 0;
    while (myFile.available()) {
      String line = myFile.readStringUntil('\n');  // \n character is discarded from buffer
      line_count++;
      if (line_count-1 == in_row){
        // return only the datetime stamp of the first logging cycle 
        return line.substring(11, 15); 
        // then stop and break
        break;
      }
    }
    // close the file:
    myFile.close();
  } else {
    // if the file didn't open, print an error:
    Serial.println("error opening /dailylog.csv");
  }
}

// This function calculates a linear regression 
 void simpLinReg(float* x, float* y, float* lrCoef, int n){
  // initialize variables
  float xbar=0;
  float ybar=0;
  float xybar=0;
  float xsqbar=0;
  
  // calculations required for linear regression
  for (int i=0; i<n; i++){
    xbar=xbar+x[i];
    ybar=ybar+y[i];
    xybar=xybar+x[i]*y[i];
    xsqbar=xsqbar+x[i]*x[i];
  }
  xbar=xbar/n;
  ybar=ybar/n;
  xybar=xybar/n;
  xsqbar=xsqbar/n;
  
  // simple linear regression algorithm
  lrCoef[0]=(xybar-xbar*ybar)/(xsqbar-xbar*xbar);
  lrCoef[1]=ybar-lrCoef[0]*xbar;
}
