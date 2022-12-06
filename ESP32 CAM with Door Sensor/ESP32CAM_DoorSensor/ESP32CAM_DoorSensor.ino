#include "esp_camera.h"
#include "SPI.h"
#include "driver/rtc_io.h"
#include "ESP32_MailClient.h"
#include <FS.h>
#include <SPIFFS.h>
#include <WiFi.h>
#include <Arduino.h>

// replace with your network (make mobile hotspot on your laptop and enter its login ID and password)
// figure out if you can make a hotspot on mac
const char* ssid = "chopper";
const char* password = "tonytony";


// to send emails using gmail on port 465 (SSL) -needed to create an app password: https://support.google.com/accounts/answer/185833

//-----------------email account details------------------
#define emailSenderAccount    "espcamphoto@gmail.com"
#define emailSenderPassword   "vilvwjckyqcxange"
#define smtpServer            "smtp.gmail.com"
#define smtpServerPort        465
#define emailSubject          "ESP32-CAM Photo Captured"
#define emailRecipient        "jimenez.qyania@gmail.com"
//--------------------------------------------------------

//----pins def of camera model -- ESP32 CAM AI thinker ---

#define CAMERA_MODEL_AI_THINKER

#if defined(CAMERA_MODEL_AI_THINKER)
  #define PWDN_GPIO_NUM     32
  #define RESET_GPIO_NUM    -1
  #define XCLK_GPIO_NUM      0
  #define SIOD_GPIO_NUM     26
  #define SIOC_GPIO_NUM     27
  
  #define Y9_GPIO_NUM       35
  #define Y8_GPIO_NUM       34
  #define Y7_GPIO_NUM       39
  #define Y6_GPIO_NUM       36
  #define Y5_GPIO_NUM       21
  #define Y4_GPIO_NUM       19
  #define Y3_GPIO_NUM       18
  #define Y2_GPIO_NUM        5
  #define VSYNC_GPIO_NUM    25
  #define HREF_GPIO_NUM     23
  #define PCLK_GPIO_NUM     22
#else
  #error "Camera model not selected"
#endif

//---------------------------------------------------------

// email sending data object contains config and data to send
SMTPData smtpData;

// photo file name to save in SPIFFS
#define FILE_PHOTO "/photo.jpg"


// Wifi connection timeout settings - 20 seconds
#define WIFI_TIMEOUT_MS 20000


int state;//door status
int PIR_STATE;//PIR status


void setup() 
{
  
  WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0); //disable brownout detector
 
  esp_err_t gpio_pullup_en(14);//door switch status pin pulled up internally
  esp_err_t gpio_pulldown_en(13);//PIR status pin pulled down internally

  Serial.begin(115200);//start serial communication at rate of 115200
  delay(1000);

  Serial.println();

 
  state=digitalRead(14);//read door state
 
 //------------------if the door is opened------------------
 
 if(state==HIGH)
  {
  
    Serial.println("Door opened");//print door opened
  
  //-----------------make initializations-------------------
  connect_wifi();
  mount_spiffs();
  show_wifi_details();
  camera_configuration();
  //--------------------------------------------------------
  

    PIR_STATE=digitalRead(13);//read PIR status pin
    
    
  //---if motion detected -- take picture and send email----
    
  if(PIR_STATE==HIGH)//if pin 13 is high -> motion observed
    {
    Serial.println("Motion detected");//print motion detected
    takepic_sendemail();//take picture and send email
    
    
    while((digitalRead(13))==HIGH);//wait for the PIR pin to reach original state
    }
    
  //--------------------------------------------------------
      
    
    
  //--if no motion detected -- wait for motion till the door not closed--
    
  else if(PIR_STATE==LOW)//if pin 13 is low -> no motion observed
  {
    
    //----look for the motion untill the door not closed----
    
    while((digitalRead(14))==HIGH)//wait till the door not closed
    {
      
      if((digitalRead(13))==HIGH)//if motion observed 
      {
      Serial.println("Motion detected");//print motion detected
      takepic_sendemail();//take picture + send email
      }
      
     
      while((digitalRead(13))==HIGH);//wait until PIR pin returns to original state
      
    }
    
    //------------------------------------------------------

  }
  
  //--------------------------------------------------------
  }
  //--------------------------------------------------------
  
  
  //---if the door is closed -- print door close message----
  
  else if(state==HIGH)
  {
    Serial.println("Door closed");//print the door close message

  }
  
  //---------------------------------------------------------
  
  
  //-------after printing message -- enter sleep mode--------
  esp_sleep_enable_ext0_wakeup(GPIO_NUM_14,1); //wake up only if pin 14 is high -- wake up if door isopen 
  Serial.println("Going to sleep now");
  delay(1000);
  esp_deep_sleep_start();//go to sleep mode 
  //---------------------------------------------------------

  
 
  
  
  
}


//---------------------loop function-------------------------
void loop() {
  

}
// Check if photo capture was successful
bool checkPhoto( fs::FS &fs ) {
  File f_pic = fs.open( FILE_PHOTO );
  unsigned int pic_sz = f_pic.size();
  return ( pic_sz > 100 );
}

//------------capture photo + save it to SPIFFS--------------

void capturePhotoSaveSpiffs( void ) {
  camera_fb_t * fb = NULL; // pointer
  bool ok = 0; // boolean indicating if picture is taken correctly

  do {
    // take photo with camera
    Serial.println("Taking a photo...");

    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      return;
    }

    // photo file name
    Serial.printf("Picture file name: %s\n", FILE_PHOTO);
    File file = SPIFFS.open(FILE_PHOTO, FILE_WRITE);

    // insert the data in the photo file
    if (!file) {
      Serial.println("Failed to open file in writing mode");
    }
    else {
      file.write(fb->buf, fb->len); // payload (image), payload length
      Serial.print("The picture has been saved in ");
      Serial.print(FILE_PHOTO);
      Serial.print(" - Size: ");
      Serial.print(file.size());
      Serial.println(" bytes");
    }
    // close file
    file.close();
    esp_camera_fb_return(fb);

    // check if file has been correctly saved in SPIFFS
    ok = checkPhoto(SPIFFS);
  } while ( !ok );
}

//-----------------------------------------------------------

//----------------send photo through email-------------------

void sendPhoto( void ) {
  // preparing email
  Serial.println("Sending email...");
  // set SMTP server email host, port, account + password
  smtpData.setLogin(smtpServer, smtpServerPort, emailSenderAccount, emailSenderPassword);
  
  // set sender name + email
  smtpData.setSender("ESP32-CAM", emailSenderAccount);
  
  // set email priority or importance high, normal, low or 1 to 5 (1 is highest)
  smtpData.setPriority("High");

  // set subject
  smtpData.setSubject(emailSubject);
    
  // set email message in HTML format
  smtpData.setMessage("<h2>Photo captured with ESP32-CAM and attached in this email.</h2>", true);
  // set email message in text format
  //smtpData.setMessage("Photo captured with ESP32-CAM and attached in this email.", false);

  // add recipients -- can add more than one recipient
  smtpData.addRecipient(emailRecipient);
  //smtpData.addRecipient(emailRecipient2);

  // add attach files from SPIFFS
  smtpData.addAttachFile(FILE_PHOTO, "image/jpg");
  // set storage type to attach files in email (SPIFFS)
  smtpData.setFileStorageType(MailClientStorageType::SPIFFS);

  smtpData.setSendCallback(sendCallback);
  
  // start sending email -- can be set callback function to track the status
  if (!MailClient.sendMail(smtpData))
    Serial.println("Error sending Email, " + MailClient.smtpErrorReason());

  // clear all data from email object to free memory
  smtpData.empty();
}

//-----------------------------------------------------------


void connect_wifi()
{
  //-------------------connect to WiFi-----------------------
  
  WiFi.begin(ssid, password);
  WiFi.mode(WIFI_STA);//addded for testing
  Serial.print("Connecting to WiFi...");
  /*while (WiFi.status() != WL_CONNECTED) {*/
  unsigned long startAttemptTime=millis();
  while (WiFi.status() != WL_CONNECTED && millis()-startAttemptTime<WIFI_TIMEOUT_MS) {
    delay(500);
    Serial.print(".");
  }

  if(WiFi.status() != WL_CONNECTED) {
    Serial.println("Failed to Connect!");
    while(1);
    
  }
  else{
    Serial.println("Connected!");
    Serial.println(WiFi.localIP());
    
  }
  
  //---------------------------------------------------------
}
void mount_spiffs()
{
  //-----initialize the SPIFFS to save last picure taken-----
  
  if (!SPIFFS.begin(true)) {
    Serial.println("An Error has occurred while mounting SPIFFS");
    ESP.restart();
  }
  else {
    delay(500);
    Serial.println("SPIFFS mounted successfully");
  }
  
  //---------------------------------------------------------
  
}
void show_wifi_details()
{
  //--------------print ESP32 local IP address---------------
  Serial.print("IP Address: http://");
  Serial.println(WiFi.localIP());
  //---------------------------------------------------------
  
}
void camera_configuration()
{
  //--------configure the camera + set camera settings-------
  
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  
  //------camera resolution + image quality settings---------
  if(psramFound()){
    config.frame_size = FRAMESIZE_HQVGA;
    config.jpeg_quality = 5;    //10//then adjusted to set to 100 for better result
    config.fb_count = 2;//previous value 2
  } else {
    Serial.println("Entering else");
    config.frame_size = FRAMESIZE_HQVGA;
    config.jpeg_quality = 5;    //12//then adjusted to set to 100 for better result
    config.fb_count = 1;
  }
  //-----------------------------------------------------------

  //--------------------initialize camera----------------------
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }
  //-----------------------------------------------------------
  
  //-----------------------------------------------------------
  
  
//----------------additional camera settings-------------------

  sensor_t * s = esp_camera_sensor_get();  
  s->set_brightness(s, 0);     // -2 to 2 //previous 0
  s->set_contrast(s, 0);       // -2 to 2 //previous 0
  s->set_saturation(s, 0);     // -2 to 2 //previous 0
  s->set_special_effect(s, 0); // 0 to 6 (0 - No Effect, 1 - Negative, 2 - Grayscale, 3 - Red Tint, 4 - Green Tint, 5 - Blue Tint, 6 - Sepia) 
  s->set_whitebal(s, 1);       // 0 = disable , 1 = enable  
  s->set_awb_gain(s, 1);       // 0 = disable , 1 = enable  
  s->set_wb_mode(s, 0);        // 0 to 4 - if awb_gain enabled (0 - Auto, 1 - Sunny, 2 - Cloudy, 3 - Office, 4 - Home)  
  s->set_exposure_ctrl(s, 1);  // 0 = disable , 1 = enable  
  s->set_aec2(s, 0);           // 0 = disable , 1 = enable  
  s->set_ae_level(s, 0);       // -2 to 2 
  s->set_aec_value(s, 300);    // 0 to 1200 
  s->set_gain_ctrl(s, 1);      // 0 = disable , 1 = enable  
  s->set_agc_gain(s, 0);       // 0 to 30 
  s->set_gainceiling(s, (gainceiling_t)0);  // 0 to 6 
  s->set_bpc(s, 0);            // 0 = disable , 1 = enable  
  s->set_wpc(s, 1);            // 0 = disable , 1 = enable  
  s->set_raw_gma(s, 1);        // 0 = disable , 1 = enable  
  s->set_lenc(s, 1);           // 0 = disable , 1 = enable  
  s->set_hmirror(s, 0);        // 0 = disable , 1 = enable  
  s->set_vflip(s, 0);          // 0 = disable , 1 = enable  
  s->set_dcw(s, 1);            // 0 = disable , 1 = enable  
  s->set_colorbar(s, 0);       // 0 = disable , 1 = enable
  
//-------------------------------------------------------------
  
}

void takepic_sendemail()
{
  //--------------take picture + send it on email--------------
 
  
  capturePhotoSaveSpiffs();//capture photo + save it to spiffs
  
  sendPhoto();//send the photo through email


  
}
//-------------------------------------------------------------

//-------callback function to get email sending status---------
void sendCallback(SendStatus msg) {
  //Print the current status
  Serial.println(msg.info());
}
//-------------------------------------------------------------
