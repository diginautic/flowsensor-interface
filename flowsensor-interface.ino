

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
#define NUM_SWITCHES 2


#define FLUID_LEVEL_SAMPLING_PERIOD 1000

#define PIN_FLOWSENSOR 8

NMEA2k_FluidLevel _FluidLevel;

//Parameter för flöde
#define FLOW_FACTOR 7.5

//EEPROM adress
#define FLOW_EEPROM_ADR 0

//Volym att lägga till vid varje tryck
#define ADDVOLUME 10.0

//Tankens kapacitet
#define TANKCAPACITY 80.0

//Flöde
Fluid _Flow(FLOW_FACTOR);
volatile unsigned long _lCounter = 0;

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
              _Flow.setLevelL(TANKCAPACITY);
              EEPROM.put(FLOW_EEPROM_ADR,TANKCAPACITY);
              handleTankLevel(PIN);
            break;

            case 2:
              _Flow.setLevelL(_Flow.getLevelL() + ADDVOLUME);
              EEPROM.put(FLOW_EEPROM_ADR,_Flow.getLevelL());
              handleTankLevel(PIN);
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

void handleTankLevel(int PIN)
{
  N2kSetStatusBinaryOnStatus(_Switches, N2kOnOff_Off, PIN);
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

  //Initiera vattentanken
  _Flow.setCapacity(TANKCAPACITY);

  //Läs in aktuell tanknivå
  double _dTemp;
  EEPROM.get(FLOW_EEPROM_ADR, _dTemp);
  if (_dTemp <= 0 || isnan(_dTemp))
  {
    _Flow.setLevelL(TANKCAPACITY);
    EEPROM.put(FLOW_EEPROM_ADR,TANKCAPACITY);
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
                                 "1.0.1",                            // Manufacturer's Software version code
                                 "1.0.1",                            // Manufacturer's Model version
                                 1                                   //LEN
  );
  // Set device information
  NMEA2000.SetDeviceInformation(10008, // Unique number. Use e.g. Serial number.
                                150,   // Device function=Analog to NMEA 2000 Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                75,    // Device class=Inter/Intranetwork Device. See codes on  http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
                                2046   // Just choosen free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );
  // Uncomment 3 rows below to see, what device will send to bus
  // NMEA2000.SetForwardStream(&dbSerial);  // PC output on due programming port
  // NMEA2000.SetForwardType(tNMEA2000::fwdt_Text); // Show in clear text. Leave uncommented for default Actisense format.
  // NMEA2000.SetForwardOwnMessages();

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

//Temperaturer
void SendN2kFluidLevelData(void)
{
  static unsigned long FluidLevelDataUpdated = InitNextUpdate(FluidLevelUpdatePeriod);
  tN2kMsg N2kMsg;

  if (IsTimeToUpdate(FluidLevelDataUpdated))
  {
    SetNextUpdate(FluidLevelDataUpdated, FluidLevelUpdatePeriod);
    SetN2kFluidLevel(N2kMsg, 0, N2kft_Water, _Flow.getLevel(), _Flow.getCapacity());
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

unsigned long _lLastTempTime = 0;

void handleFluidLevelData(void)
{
  if (millis() - _lLastTempTime >= FLUID_LEVEL_SAMPLING_PERIOD)
  {
    if (_lCounter != 0)
    {
      noInterrupts();
      _Flow.addCount(_lCounter);
      _lCounter = 0;
      interrupts();
      EEPROM.put(FLOW_EEPROM_ADR,_Flow.getLevelL());
    }
    _lLastTempTime = millis();
  }
}

void loop(void)
{
  // * **********************************************
  // * Samla in data
  // * **********************************************
  //Hantera Temperaturer
  handleFluidLevelData();

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


