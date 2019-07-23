#include <SerialCMD.h>

//Check these values in the SerialCMD.h file of the SerialCMD library:
/*
#define CMD_MAX_COMMANDS        20    // Máximo numero de comandos permitidos
#define CMD_MAX_COMMANDS_LENGHT 10    // Longitud máxima de caracteres por cada comando
#define CMD_MAX_COMMANDS_ARGS   3     // Número máximo de argumentos por comando
#define CMD_MAX_INPUT_LENGHT    50    // Tamaño del buffer de entrada de comandos y argumentos
*/


// Command line class instance...
SerialCMD cmd;  

void commandLine(void){
  cmd.begin(Serial, F("LoRaX1>"), welcome);

  // Añade comandos...
  cmd.addCommand(command_pow,   F("pow"),       F("[5...23]  RF power (5:min...23:max)"));
  cmd.addCommand(command_stime, F("stime"),     F("[5...999] Send time delay (seconds)"));
  cmd.addCommand(command_retri, F("retri"),     F("[0...20]  Data send retries"));  
  cmd.addCommand(command_node,  F("node"),      F("[0...250] Node id"));
  cmd.addCommand(command_pan,   F("pan"),       F("[0...250] PAN id"));
  cmd.addCommand(command_relay, F("relay"),     F("[on|off]  Relay on/off"));   
  cmd.addCommand(command_read,  F("read"),      F("Read sensors"));
  cmd.addCommand(command_send,  F("send"),      F("Read and send sensors data to server"));
  cmd.addCommand(command_param, F("all"),       F("Read all parameters"));
  cmd.addCommand(command_help,  F("?"),         F("Command list help"));  

  // Stay in a loop while you have the TTL programmer connected...
  while(1){
    if(!digitalRead(RX)){
      delay(100);
      if(!digitalRead(RX)) break;
    }
    cmd.task();
  }
}

// Comando pow...
int command_pow(int argc, char** argv){
  bool ok = false;
  long temp;

  temp = config.rfPower;
  if(argc == 1) ok = true;
  if(argc >= 2){
    if(cmd.testLong(argv[1], 5, 23, &temp)){
      config.rfPower = (byte)temp;
      rf95.setTxPower(config.rfPower, false);
      EEPROM.put(DIR_EEPROM_CFG, config);
      ok = true;
    }
  }  
  if(ok) cmd.printf(F("pow = %ld\r\n"),(long)config.rfPower);
  return(ok);
}

// Comando stime...
int command_stime(int argc, char** argv){
  bool ok = false;
  long temp;

  temp = config.sendTime;  
  if(argc == 1) ok = true;
  if(argc >= 2){
    if(cmd.testLong(argv[1], 5, 999, &temp)){
      config.sendTime = (byte)temp;
      EEPROM.put(DIR_EEPROM_CFG, config);
      ok = true;
    }
  }  
  if(ok) cmd.printf(F("stime = %ld\r\n"),(long)config.sendTime);
  return(ok);
}

// Comando retri...
int command_retri(int argc, char** argv){
  bool ok = false;
  long temp;

  temp = config.rfRetries;  
  if(argc == 1) ok = true;
  if(argc >= 2){
    if(cmd.testLong(argv[1], 0, 20, &temp)){
      config.rfRetries = (byte)temp;
      manager.setRetries(config.rfRetries);
      EEPROM.put(DIR_EEPROM_CFG, config);    
      ok = true;
    }
  }  
  if(ok) cmd.printf(F("retri = %ld\r\n"),(long)config.rfRetries);
  return(ok);
}

// Comando node...
int command_node(int argc, char** argv){
  bool ok = false;
  long temp;

  temp = config.rfNode;
  if(argc == 1) ok = true;
  if(argc >= 2){
    if(cmd.testLong(argv[1], 0, 250, &temp)){
      config.rfNode = (byte)temp;
      EEPROM.put(DIR_EEPROM_CFG, config);        
      ok = true;
    }
  }  
  if(ok) cmd.printf(F("node = %ld\r\n"),(long)config.rfNode);
  return(ok);  
}

// Comando pan...
int command_pan(int argc, char** argv){
  bool ok = false;
  long temp;

  temp = config.rfPan;
  if(argc == 1) ok = true;
  if(argc >= 2){
    if(cmd.testLong(argv[1], 0, 250, &temp)){
      config.rfPan = (byte)temp;
      theData.pan_id = config.rfPan;  
      EEPROM.put(DIR_EEPROM_CFG, config);  
      ok = true;
    }
  }  
  if(ok) cmd.printf(F("pan = %ld\r\n"),(long)config.rfPan);
  return(ok);   
}

// Comando relay...
int command_relay(int argc, char** argv){
  bool ok = false;
  boolean temp;

  if(argc == 1) ok = true;
  if(argc >= 2){
    if(cmd.testBoolean(argv[1], &temp)){
      if(temp)
        digitalWrite(RELAY, HIGH);
      else
        digitalWrite(RELAY, LOW);
      ok = true;
    }
  }  
  if(ok){
    if(temp)
      cmd.print(F("relay = on\r\n"));
    else
      cmd.print(F("relay = off\r\n"));
  }
  return(ok); 
}

// Comando read...
int command_read(int argc, char** argv){
  char s[10];
  unsigned long a = millis();  
  
  lee_ade();

  i2DecimalPoint(theData.frequency, 1, s);
  cmd.printf(F("Frequency       = %s Hz\r\n"),s);
  cmd.printf(F("Voltage RMS     = %d V\r\n"),theData.v_rms);
  cmd.printf(F("Voltage PEAK    = %d V\r\n"),theData.v_peak);  
  i2DecimalPoint(theData.i_rms, 2, s);
  cmd.printf(F("Current RMS     = %s A\r\n"),s);  
  cmd.printf(F("Active power    = %d W\r\n"),theData.p_act);
  cmd.printf(F("Active energy   = %d KWh\r\n"),theData.e_act);
  cmd.printf(F("Spent time      = %d mS\r\n"), millis() - a);

  ade.vPeakReset();
  return(true);  
}

// Comando send...
int command_send(int argc, char** argv){
  unsigned long a = millis();

  cmd.println(F("Read sensors and send data..."));
  led_st_on();

  // Sensors read...
  lee_ade();
  cmd.printf (F("Pan     = %d\r\n"), config.rfPan);
  cmd.printf (F("Node    = %d\r\n"), config.rfNode);
  cmd.printf (F("Power   = %d\r\n"), config.rfPower);  
  cmd.printf (F("Retries = %d\r\n"), config.rfRetries);
  cmd.println(F("Sending..."));
      
  // Send payload to server...
  if(send_to_server()){
    cmd.println(F("Send OK"));     
  }else{
    cmd.println(F("Send ERROR!"));  
  }
  cmd.printf (F("Spent time = %d mS\r\n"), millis() - a);

  led_st_off();
   
  return(true);    
}

// Comando param...
int command_param(int argc, char** argv){
  cmd.printf(F("pow   = %d\r\n"), (long)config.rfPower);  
  cmd.printf(F("stime = %d\r\n"), (long)config.sendTime);
  cmd.printf(F("retri = %d\r\n"), (long)config.rfRetries);
  cmd.printf(F("Node  = %d\r\n"), (long)config.rfNode);
  cmd.printf(F("pan   = %d\r\n"), (long)config.rfPan);
  return(true);
}

// Comando help...
bool command_help(int argc, char** argv){
  cmd.println(F("Command list:"));
  cmd.printCommands();
  return(false);
}

// Mensaje de bienvenida
void welcome(void){
  cmd.println(F("Ray Ingenieria Electronica, S.L."));
  cmd.print  (F("LoRaX1 - Firm.: v"));
  cmd.print  (F(FIRMWARE_VERSION));
  cmd.print  (F(" Hard.: "));
  cmd.println(F(HARDWARE_VERSION));  
  cmd.println(F("Press '?' for help")); 
}


