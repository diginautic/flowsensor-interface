

//#define DEBUG_SERIAL_ENABLE

#include <Arduino.h>
#include <CaseumSerialConfig.h>
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>
#include <DiginauticN2k.h>
#include <DiginauticFluid.h>
#include <EEPROM.h>

//Definitions
#define N2K_SOURCE 108

//DeviceBankInstance
#define FLUIDBANKINSTANCE 2
#define NUM_SWITCHES 3


#define FRESH_WATER_LEVEL_SAMPLING_PERIOD 1000
#define BLACK_WATER_LEVEL_SAMPLING_PERIOD 300000
#define BLACK_WATER_LEVEL_WAIT_PERIOD 20000
#define BLACK_WATER_LEVEL_PRESS_PERIOD 500

#define PIN_FLOWSENSOR 8
#define PIN_BLUE 14
#define PIN_GREEN 15
#define PIN_YELLOW 16
#define PIN_RED 17
#define PIN_BUTTON 18

NMEA2k_FluidLevel _FluidLevel;

//Parameter för flöde
#define FLOW_FACTOR 7.5

//EEPROM adress
#define FLOW_EEPROM_ADR 0

//Volym att lägga till vid varje tryck
#define ADDVOLUME 10.0

//Tankens kapacitet
#define FRESH_TANKCAPACITY 80.0

//Tankens kapacitet
#define WASTE_TANKCAPACITY 56.0

//Fresh water tank instance
unsigned char FreshWaterInstance = 0;

//Waste water tank instance
unsigned char BlackWaterInstance = 1;

//Timingparametrar
unsigned long _lLastFlowTime = 0;
unsigned long _lLastBlackWaterTime = 0;
unsigned long _lLastStartTime = 0;
unsigned long _lLastButtonPressTime = 0;
bool _bPressed = false;

//Färskvatten
Fluid _Flow(FLOW_FACTOR, N2kft_Water, FreshWaterInstance);
volatile unsigned long _lCounter = 0;

//Svartvatten
Fluid _BlackWater(0, N2kft_BlackWater, BlackWaterInstance);
#define BTN_CHECK_TOILET 3

//Switches
tN2kBinaryStatus _Switches = 0;


//*****************************************************************************
void BankControl(const tN2kMsg &N2kMsg)
{
  unsigned char DeviceBankInstance;
  tN2kBinaryStatus TempSwitchBoard;
  tN2kOnOff ItemStatus;

  if (ParseN2kSwitchbankControl(N2kMsg, DeviceBankInstance, TempSwitchBoard) )
  {
    if (DeviceBankInstance == FLUIDBANKINSTANCE)
    {
      for (int PIN = 1; PIN <= NUM_SWITCHES; PIN++)
      {
        ItemStatus = N2kGetStatusOnBinaryStatus(TempSwitchBoard,PIN);
        if (ItemStatus != N2kOnOff_Unavailable)
        { 
          switch (PIN)
          {
            case 1:
              _Flow.setLevelL(FRESH_TANKCAPACITY);
              EEPROM.put(FLOW_EEPROM_ADR,FRESH_TANKCAPACITY);
              handleTankLevel(PIN, N2kOnOff_Off);
            break;

            case 2:
              _Flow.setLevelL(_Flow.getLevelL() + ADDVOLUME);
              EEPROM.put(FLOW_EEPROM_ADR,_Flow.getLevelL());
              handleTankLevel(PIN, N2kOnOff_Off);
            break;

            case 3:
              if (!_bPressed)
              {
                dbSerialPrintln("Wohoo ingen har tryckt!");

                _bPressed = true;
                //handleTankLevel(PIN, ItemStatus);
                handleTankLevel(PIN, N2kOnOff_On);
                handleBlackWaterLevelData();
              }
              else
              {
                dbSerialPrintln("Redan tryckt");
              }
            break;
          }
          break;
        }
      }
    }
  }
  else
  {
    dbSerialPrint("Failed to parse PGN: ");
    dbSerialPrintln(N2kMsg.PGN);
  }
}

void handleTankLevel(int PIN, tN2kOnOff PinStatus)
{
  N2kSetStatusBinaryOnStatus(_Switches, PinStatus, PIN);
  SendN2kBinaryStatus(true);
}
//************************************************************

typedef struct
{
  unsigned long PGN;
  void (*Handler)(const tN2kMsg &N2kMsg);
} tNMEA2000Handler;

tNMEA2000Handler NMEA2000Handlers[] =
{
  {127502L, &BankControl},
  {0, 0}
};

//NMEA 2000 message handler
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg)
{
  int iHandler;
  // Find handler
  for (iHandler = 0; NMEA2000Handlers[iHandler].PGN != 0 && !(N2kMsg.PGN == NMEA2000Handlers[iHandler].PGN); iHandler++);
  if (NMEA2000Handlers[iHandler].PGN != 0)
  {
    NMEA2000Handlers[iHandler].Handler(N2kMsg);
  }
}

//Intrerrupt handler for flow sensor
void flow_isr ()
{
   _lCounter++;
}

//************************************************************

void setup(void)
{
  dbSerialBegin(115200);

  //Flödesmätaren
  //Interruptrutin
  attachInterrupt(digitalPinToInterrupt(PIN_FLOWSENSOR), flow_isr, RISING);
  //Pull up pin
  pinMode(PIN_FLOWSENSOR, INPUT_PULLUP);
  pinMode(PIN_BUTTON, OUTPUT);
  pinMode(PIN_BLUE, INPUT_PULLUP);
  pinMode(PIN_GREEN, INPUT_PULLUP);
  pinMode(PIN_YELLOW, INPUT_PULLUP);
  pinMode(PIN_RED, INPUT_PULLUP);

  //Initiera vattentanken
  _Flow.setCapacity(FRESH_TANKCAPACITY);

  //Initiera toatanken
  _BlackWater.setCapacity(WASTE_TANKCAPACITY);

  //Starta första mätning av toatanken
  //Mätningen hantreras av första anropet i loop()
  _bPressed = true;

  //Läs in aktuell tanknivå
  double _dTemp;
  EEPROM.get(FLOW_EEPROM_ADR, _dTemp);
  if (_dTemp <= 0 || isnan(_dTemp))
  {
    _Flow.setLevelL(0);
    EEPROM.put(FLOW_EEPROM_ADR,0);
    //Varför satte jag den till full tank om jag inte kan tolka
    //nivån eller om det inte är ett numerikst värde???
    //Sätter den till noll istället
    //_Flow.setLevelL(FRESH_TANKCAPACITY);
    //EEPROM.put(FLOW_EEPROM_ADR,FRESH_TANKCAPACITY);
  }
  else
  {
    _Flow.setLevelL(_dTemp);
  }

  // Reserve enough buffer for sending all messages. This does not work on small memory devices like Uno or Mega
  NMEA2000.SetN2kCANSendFrameBufSize(250);
  // Set Product information
  NMEA2000.SetProductInformation("00000008",                         // Manufacturer's Model serial code
                                 108,                                // Manufacturer's product code
                                 "Diginautic Fluid Level Interface", // Manufacturer's Model ID
                                 "1.1",                            // Manufacturer's Software version code
                                 "1.1",                            // Manufacturer's Model version
                                 1                                   //LEN
  );
  // Set device information
  NMEA2000.SetDeviceInformation(10008, // Unique number. Use e.g. Serial number.
                                150,   // Device function=Analog to NMEA 2000 Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                75,    // Device class=Inter/Intranetwork Device. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046   // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );
  // If you also want to see all traffic on the bus use N2km_ListenAndNode instead of N2km_NodeOnly below
  NMEA2000.SetMode(tNMEA2000::N2km_NodeOnly, N2K_SOURCE);
  NMEA2000.EnableForward(false); // Disable all msg forwarding to USB (=Serial)
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);
  NMEA2000.Open();

  dbSerialPrintln("setup done");
}

//************************************************************

#define FluidLevelUpdatePeriod 1000
#define StatusUpdatePeriod 500

bool IsTimeToUpdate(unsigned long NextUpdate)
{
  return (NextUpdate < millis());
}
unsigned long InitNextUpdate(unsigned long Period, unsigned long Offset = 0)
{
  return millis() + Period + Offset;
}

void SetNextUpdate(unsigned long &NextUpdate, unsigned long Period)
{
  while (NextUpdate < millis())
    NextUpdate += Period;
}

//************************************************************

//Vattentankar
void SendN2kFluidLevelData(void)
{
  static unsigned long FluidLevelDataUpdated = InitNextUpdate(FluidLevelUpdatePeriod);
  tN2kMsg N2kMsg;

  if (IsTimeToUpdate(FluidLevelDataUpdated))
  {
    SetNextUpdate(FluidLevelDataUpdated, FluidLevelUpdatePeriod);

    //Fresh water
    SetN2kFluidLevel(N2kMsg, _Flow.getInstance(), _Flow.getFluidType(), _Flow.getLevel(), _Flow.getCapacity());
    NMEA2000.SendMsg(N2kMsg);

    //Black water
    SetN2kFluidLevel(N2kMsg, _BlackWater.getInstance(), _BlackWater.getFluidType(), _BlackWater.getLevel(), _BlackWater.getCapacity());
    NMEA2000.SendMsg(N2kMsg);
  }
}


//************************************************************

void SendN2kBinaryStatus(bool bDoOverride)
{
  static unsigned long StatusDataUpdated = InitNextUpdate(StatusUpdatePeriod);
  tN2kMsg N2kMsg;
  if (IsTimeToUpdate(StatusDataUpdated) || bDoOverride)
  {
    SetNextUpdate(StatusDataUpdated, StatusUpdatePeriod);
    SetN2kBinaryStatus(N2kMsg, FLUIDBANKINSTANCE, _Switches);
    NMEA2000.SendMsg(N2kMsg);
  }
}


//******************************************************************
void handleFreshWaterLevelData(void)
{
  if (millis() - _lLastFlowTime >= FRESH_WATER_LEVEL_SAMPLING_PERIOD)
  {
    if (_lCounter != 0)
    {
      noInterrupts();
      _Flow.addCount(_lCounter);
      _lCounter = 0;
      interrupts();
      EEPROM.put(FLOW_EEPROM_ADR,_Flow.getLevelL());
    }
    _lLastFlowTime = millis();
  }
}

//******************************************************************
void handleBlackWaterLevelData(void)
{
  if (millis() - _lLastBlackWaterTime >= BLACK_WATER_LEVEL_SAMPLING_PERIOD || _bPressed)
  {
    static int _iTask = 1;

    if (_iTask == 1) //Initiera mätningen
    {
      dbSerialPrintln("Task = 1");

      //Tänd knappen på NMEA2000
      handleTankLevel(BTN_CHECK_TOILET, N2kOnOff_On);


      //Tryck in knappen och starta timern
      digitalWrite(PIN_BUTTON, HIGH);
      _lLastButtonPressTime = millis();

      _lLastStartTime = millis();

      _iTask = 2;
    }
    //Om knappen är tryckt och tiden för ett tryck har gått ut så släpp knappen
    else if (_iTask == 2 && millis() - _lLastButtonPressTime >= BLACK_WATER_LEVEL_PRESS_PERIOD)
    {
      dbSerialPrintln("Task = 2");
      digitalWrite(PIN_BUTTON, LOW);
      _lLastButtonPressTime = 0;

      _iTask = 3;
    }
    //Om väntetiden för lamporna på displayen gått ut är det dags att läsa av lamporna och stoppa kontrollen
    else if (_iTask == 3 && millis() - _lLastStartTime >= BLACK_WATER_LEVEL_WAIT_PERIOD)
    {
      dbSerialPrintln("Task = 3");

      double dLevel = 0;
      int iBlue = !digitalRead(PIN_BLUE);
      int iGreen = !digitalRead(PIN_GREEN);
      int iYellow = !digitalRead(PIN_YELLOW);
      int iRed = !digitalRead(PIN_RED);

      //Om flera LED lyser signalerar det ett fel på en eller flera givare
      if ((iBlue + iGreen + iYellow + iRed) == 1)
      {
        //dbSerialPrintln("Bara en lampa lyser");

        dLevel += (double)iBlue * 0.0;
        dLevel += (double)iGreen * 25.0;
        dLevel += (double)iYellow * 50.0;
        dLevel += (double)iRed * 75.0;
      }
      else
      {
        //dbSerialPrintln("Alla lampor eller ingen lampa lyser");
        dLevel = -99.0;
      }

      //dbSerialPrintln("Nivån satt till: " + (String)dLevel);

      _BlackWater.setLevel(dLevel);

      //_lLastStartTime = millis();
      //_lLastBlackWaterTime = millis();
      //Släck knappen
      handleTankLevel(BTN_CHECK_TOILET, N2kOnOff_Off);

      //Sätt utgången till optokopplaren hög så att +12V
      //på gobius dras låg och stänger av sig
      //dbSerialPrintln("Drar optokopplaren låg + _dPressed=true");
      digitalWrite(PIN_BUTTON, HIGH);
      _lLastButtonPressTime = millis();

      _iTask = 4;
    }
    else if (_iTask == 4 && millis() - _lLastButtonPressTime >= BLACK_WATER_LEVEL_PRESS_PERIOD)
    {
      dbSerialPrintln("Task = 4");
      dbSerialPrintln("------------------------------------");

      _bPressed = false;
      digitalWrite(PIN_BUTTON, LOW);
      _lLastButtonPressTime = 0;
      
      _lLastBlackWaterTime = millis();
      _iTask = 1;
    }
  }
}

//******************************************************************

void loop(void)
{
  // * **********************************************
  // * Samla in data
  // * **********************************************
  //Hantera Färskvatten
  handleFreshWaterLevelData();

  //Hantera toavatten
  handleBlackWaterLevelData();

  // * **********************************************
  // * Lyssna på NMEA2000
  // * **********************************************
  //Lyssna efter meddelanden på NMEA2000-bussen
  NMEA2000.ParseMessages();

  // * **********************************************
  // * Skicka data på NMEA2000
  // * **********************************************
  //Skicka NMEA2000-meddelanden på bussen
  //Vätskenivå
  SendN2kFluidLevelData();
  //Skicka bank status
  SendN2kBinaryStatus(false);
}


