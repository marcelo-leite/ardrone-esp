#include <ardrone-esp.h>

//ArdroneESP Contructor
ArdroneESP::ArdroneESP(void){
  this->ssid = "ardrone2_129312"; //1 = 092417, 0 = 116276
  this->navPort = 5554;
  this->videoPort = 5555;
  this->atPort = 5556;  
  strncpy(this->ardata.signature, "NAVDATA", 7);
}

//Function Member
void ArdroneESP::ardroneConnect(void){
   // Connect to WiFi network
    Serial.println("\n\n\nconnecting to AR WiFi");
    WiFi.mode(WIFI_STA);
    WiFi.begin(this->ssid);
    while (WiFi.status() != WL_CONNECTED) {
      delay(200);
    }
    Serial.print("Connected, address=");
    Serial.println(WiFi.localIP()); 
  
    // Prep UDP
    Serial.println("Starting UDP...");
    this->NavUdp.begin(this->navPort); //Open port for navdata
    this->NavUdp.flush();
    this->AT.begin(this->atPort);
    this->AT.flush();
  
    this->lNav = 0;
    this->sequence = 1;
  
    while(this->NavUdp.parsePacket() == 0) {
      delay(10);
      this->NavUdp.beginPacket(drone, this->navPort);
      this->NavUdp.write(0x01);
      this->NavUdp.endPacket();
      delay(20);
      this->configCommand("general:navdata_demo", "FALSE");
      this->configCommand("control:altitude_max", "3000");
    }
  
    unsigned long emergencyInitCounter = millis();
    while (millis() - emergencyInitCounter < 1000) {
      this->configCommand("general:navdata_demo", "FALSE");
      delay(30);
    }
    littlecounter = millis();
}

//Function Member
void ArdroneESP::serializeAT(ATCommand command, int Nargs){
  String serialized;
  String argument;
  serialized = "AT*" + (String)command.type + "=" + (String)this->sequence;
  for(int i =0; i < Nargs; i++){
    argument = "," + (String)command.cmdargs[i];
    serialized.concat(argument);
  }
  ++this->sequence;
  
  this->sendPacket(serialized);
}

//Function Member
void ArdroneESP::sendPacket(String command){
  int len;
  len = command.length() + 1;
  char sendChar[len];
  command.toCharArray(sendChar, len);
  sendChar[command.length()] = '\r';
  
  this->AT.beginPacket(drone, this->atPort);
  this->AT.write(sendChar);
  this->AT.endPacket();
  this->lSend = millis();

}

//Function Member 
void ArdroneESP::captureNavdata(void){
  uint32_t nbsat;
  this->navdata.head = *((int32_t*)&this->incoming[0]);
  this->navdata.ardrone_state = *((int32_t*)&this->incoming[4]);
  this->navdata.sequence = *((int32_t*)&this->incoming[8]);
  this->navdata.vision = *((int32_t*)&this->incoming[12]); 
  int len = 0;
  uint16_t k = 16;
  
  if(this->NavUdp.parsePacket()) {
    this->lReceive = millis();
    len = this->NavUdp.read(this->incoming, 4096); 
    }
  this->navdata.id = (uint16_t*)&this->incoming[16];
  this->navdata.siz = (uint16_t*)&this->incoming[18];
  
  for(uint16_t i = 0; i < 28; i++){
    
    for(uint16_t j = 0; j <= *this->navdata.siz; j++){
      this->block[j] = this->incoming[k + j];
      
    }
    this->navdata.id = (uint16_t*)&this->block[0];
    this->navdata.siz = (uint16_t*)&this->block[2];
    k += *this->navdata.siz;

    switch (*this->navdata.id){
      
      case 0:
        this->navdata.block.navdata_demo = *((navdata_demo_t*)&this->block[4]);
        break;
      case 1:
        this->navdata.block.navdata_time = *((navdata_time_t*)&this->block[4]);
        break;
      case 2:
        this->navdata.block.navdata_raw_measures = *((navdata_raw_measures_t*)&this->block[4]);
        break;
      case 3:
        this->navdata.block.navdata_phys_measures = *((navdata_phys_measures_t*)&this->block[4]);
        break;
      case 4:
        this->navdata.block.navdata_gyros_offsets = *((navdata_gyros_offsets_t*)&this->block[4]);
        break;
      case 5:
        this->navdata.block.navdata_euler_angles = *((navdata_euler_angles_t*)&this->block[4]);
        break;
      case 6:
        this->navdata.block.navdata_references = *((navdata_references_t*)&this->block[4]);
        break;
      case 7:
        this->navdata.block.navdata_trims = *((navdata_trims_t*)&this->block[4]);
        break;
      case 8:
        this->navdata.block.navdata_rc_references = *((navdata_rc_references_t*)&this->block[4]);
        break;
      case 9:
        this->navdata.block.navdata_pwm = *((navdata_pwm_t*)&this->block[4]);
        break;
      case 10:
        this->navdata.block.navdata_altitude = *((navdata_altitude_t*)&this->block[4]);
        break;
      case 11:
        this->navdata.block.navdata_vision_raw = *((navdata_vision_raw_t*)&this->block[4]);
        break;
      case 12:
        this->navdata.block.navdata_vision_of = *((navdata_vision_of_t*)&this->block[4]);
        break;
      case 13:
        this->navdata.block.navdata_vision = *((navdata_vision_t*)&this->block[4]);
        break;
      case 14:
        this->navdata.block.navdata_vision_perf = *((navdata_vision_perf_t*)&this->block[4]);
        break;
      case 15:
        this->navdata.block.navdata_trackers_send = *((navdata_trackers_send_t*)&this->block[4]);
        break;
      case 16:
        this->navdata.block.navdata_vision_detect = *((navdata_vision_detect_t*)&this->block[4]);
        break;
      case 17:
        this->navdata.block.navdata_watchdog = *((navdata_watchdog_t*)&this->block[4]);
        break;
      case 18:
        this->navdata.block.navdata_adc_data_frame = *((navdata_adc_data_frame_t*)&this->block[4]);
        break;
      case 19:
        this->navdata.block.navdata_video_stream = *((navdata_video_stream_t*)&this->block[4]);
        break;
      case 20:
        this->navdata.block.navdata_games = *((navdata_games_t*)&this->block[4]);
        break;
      case 21:
        this->navdata.block.navdata_pressure_raw = *((navdata_pressure_raw_t*)&this->block[4]);
        break;
      case 22:
        this->navdata.block.navdata_magneto = *((navdata_magneto_t*)&this->block[4]);
        break;
      case 23:
        this->navdata.block.navdata_wind_speed = *((navdata_wind_speed_t*)&this->block[4]);
        break;
      case 24:
        this->navdata.block.navdata_kalman_pressure = *((navdata_kalman_pressure_t*)&this->block[4]);
        break;
      case 25:
        this->navdata.block.navdata_hdvideo_stream = *((navdata_hdvideo_stream_t*)&this->block[4]);
        break;
      case 26:
        this->navdata.block.navdata_wifi = *((navdata_wifi_t*)&this->block[4]);
        break;
      case 27:
        this->navdata.block.navdata_gps = *((navdata_gps_t*)&this->block[4]);
        break;
      default:
        break;
    }
  }
  this->ardata.fligth_data.sequence = this->navdata.sequence;
  this->ardata.fligth_data.adrone_state = this->navdata.ardrone_state;
  this->ardata.fligth_data.baterry = this->navdata.block.navdata_demo.baterry;
  this->ardata.fligth_data.theta = this->navdata.block.navdata_demo.theta;
  this->ardata.fligth_data.phi = this->navdata.block.navdata_demo.phi;
  this->ardata.fligth_data.psi = this->navdata.block.navdata_demo.psi;
  // this->ardata.fligth_data.psi = this->navdata.block.navdata_gps.degree;


  this->ardata.fligth_data.altitude = this->navdata.block.navdata_demo.altitude;
  this->ardata.fligth_data.pression = this->navdata.block.navdata_pressure_raw.pression_meas;
  
  this->ardata.fligth_data.v.x = this->navdata.block.navdata_demo.vx;
  this->ardata.fligth_data.v.y = this->navdata.block.navdata_demo.vy;
  this->ardata.fligth_data.v.z = this->navdata.block.navdata_demo.vz;
  
  this->ardata.fligth_data.phys_accs = this->navdata.block.navdata_phys_measures.phys_accs;
  this->ardata.fligth_data.phys_gyros = this->navdata.block.navdata_phys_measures.phys_gyros;
  
  this->ardata.fligth_data.wind_speed = this->navdata.block.navdata_wind_speed.wind_speed;
  this->ardata.fligth_data.wind_angle = this->navdata.block.navdata_wind_speed.wind_angle;
  
  this->ardata.fligth_data.motor[0] = this->navdata.block.navdata_pwm.motor1;
  this->ardata.fligth_data.motor[1] = this->navdata.block.navdata_pwm.motor2;
  this->ardata.fligth_data.motor[2] = this->navdata.block.navdata_pwm.motor3;
  this->ardata.fligth_data.motor[3] = this->navdata.block.navdata_pwm.motor4;
  
  this->ardata.fligth_data.link_quality = this->navdata.block.navdata_wifi.link_quality;
  
  this->ardata.fligth_data.latitude = this->navdata.block.navdata_gps.latitude;
  this->ardata.fligth_data.longitude = this->navdata.block.navdata_gps.longitude;
  this->ardata.fligth_data.elevation = this->navdata.block.navdata_gps.elevation;
  this->ardata.fligth_data.gps_state = this->navdata.block.navdata_gps.gps_state;
  this->ardata.fligth_data.nbsat = this->navdata.block.navdata_gps.nbsat;
// ardata->fligth_data.altitude = this->navdata.block.navdata_demo.altitude;

  if (millis() - this->lSend >= 40) {
    this->comwdgCommand();
    this->lSend = millis();
  }
    
}

//Function Member
void ArdroneESP::refCommand(bool takeoff, bool emergency){
  ATCommand command;
      command.type = "REF";
      if (emergency) {
        command.cmdargs[0] = "290717952";
      } else {
        if (takeoff) {
          command.cmdargs[0] = "290718208";
        } else {
          command.cmdargs[0] = "290717696";
        }
      }
      this->serializeAT(command, 1);
}

//Function Member
void ArdroneESP::pcmdCommand(bool mode, float vel[4]){
  ATCommand command;
  command.type = "PCMD";
  command.cmdargs[0] = mode ? "1" : "0"; // Enables(1) or diables(0) (hover only) movement;            //Progressive command or hover command
  
  command.cmdargs[1] = *((int*)&vel[1]);  //Vy
  command.cmdargs[2] = *((int*)&vel[0]);  //Vx
  command.cmdargs[3] = *((int*)&vel[2]);  //Vz
  
  command.cmdargs[4] = *((int*)&vel[3]);  //Wz
  
  this->serializeAT(command, 5);
}

//Function Member
void ArdroneESP::ftrimCommand(void){
  ATCommand command;
  command.type = "FTRIM";
  this->serializeAT(command, 0);
}

//Function Member
void ArdroneESP::comwdgCommand(void){
  ATCommand command;
  command.type = "COMWDG";
  this->serializeAT(command, 0);
}

//Function Member
void ArdroneESP::ledCommand(void){
  ATCommand command;
  command.type = "LED";
  this->serializeAT(command, 4);
}

//Function Member
void ArdroneESP::calibCommand(void){
  ATCommand command;
  command.type = "CALIB";
  this->serializeAT(command, 2);
}

//Function Member
void ArdroneESP::pwmCommand(int pwm[4]){
  ATCommand command;
  //  [0-500] PWM Interval
  command.type = "PWM";
  command.cmdargs[0] = String(pwm[0]);  //M1
  command.cmdargs[1] = String(pwm[1]);  //M2
  command.cmdargs[2] = String(pwm[2]);  //M3
  command.cmdargs[3] = String(pwm[3]);  //M4
  this->serializeAT(command, 4);

}

//Function Member
void ArdroneESP::configCommand(String key, String value){
  ATCommand command;
  command.type = "CONFIG";
  command.cmdargs[0] = "\"" + String(key) + "\"";
  command.cmdargs[1] = "\"" + String(value) + "\"";
  this->serializeAT(command, 2);
}

//Function Member
void ArdroneESP::land(void){
  this->refCommand(false, false);
  delay(5000);
}

//Function Member
void ArdroneESP::takeoff(void){
  this->refCommand(true, false);
}

//Function Member
void ArdroneESP::emergency(void){
  this->refCommand(false, true);
}

void ArdroneESP::showNavdata(void){
      
  Serial.print("Sinal Wifi:\t");
  Serial.print(this->navdata.block.navdata_wifi.link_quality);
  Serial.print("\n");
  
  Serial.print("Estado:\t");
  Serial.print(this->navdata.block.navdata_demo.ctrl_state);
  Serial.print("\n");

  Serial.print("Altitude Relativa:\t");
  Serial.print(this->navdata.block.navdata_demo.altitude);
  Serial.print("\n");

  Serial.print("Sequencia:\t");
  Serial.print(navdata.sequence);
  Serial.print("\n");
  
  Serial.print("Nivel Bateria:\t");
  Serial.print(this->navdata.block.navdata_demo.baterry);
  Serial.print("\n");
  
  Serial.print("Pressão:\t");  
  Serial.print(this->navdata.block.navdata_pressure_raw.pression_meas);
  Serial.print("\n"); 

  Serial.print("Latitude:\t");
  Serial.print(this->navdata.block.navdata_gps.latitude, 8);
  Serial.print("\n");
  
  Serial.print("Longitude:\t");
  Serial.print(this->navdata.block.navdata_gps.longitude, 8);
  Serial.print("\n");
  
  Serial.print("Elevação:\t");
  Serial.print(this->navdata.block.navdata_gps.elevation, 8);
  Serial.print("\n");
  
  Serial.print("Numero de Satelites:\t");
  Serial.print(this->navdata.block.navdata_gps.nbsat);
  Serial.print("\n\n\n");


  // Serial.print("Magnometro:\n");
  // Serial.println(this->navdata.block.navdata_magneto.mx);
  // Serial.println(this->navdata.block.navdata_magneto.my);
  // Serial.println(this->navdata.block.navdata_magneto.mz);
  // Serial.print("\n");
  
}