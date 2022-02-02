/*
  nRF24L01 Instrument Station
  Main sketch
  Arduino Mega

  D C Potts 2021
  School of Engineering
  University of Liverpool
*/

#define DG_FIRMWARE_VER_0 0
#define DG_FIRMWARE_VER_1 1

// -----------------------------------------------------------------------------
// Settings

#define IS_INTERVAL_DISCOVER 100

#define IS_INTERVAL_PRINTOUT 2500

#define IS_INTERVAL_SAMPLE_REQUEST 20

#define IS_AR_COUNT 5           // default is 15
#define IS_AR_DELAY 5           // multiples of 250us, default is 5

#define IS_RADIO_CHANNEL 84       // anything above 84 is outside of 802.11

// -----------------------------------------------------------------------------
// Built-in includes
#include "stdint.h"
#include "stdio.h"

#include "avr/io.h"
#include "avr/wdt.h"

#include "Arduino.h"

// Library includes
#include "RF24.h"               // https://github.com/nRF24/RF24
#include "Bounce2.h"            // https://github.com/thomasfredericks/Bounce2

// Application includes
#include "pins.h"

#include "libs/InstrumentStation/utils.hpp"
#include "libs/InstrumentStation/PolledInterval.hpp"

// -----------------------------------------------------------------------------
// Objects

// State Machine
typedef enum {
  APP_INIT,
  APP_IDLE,
  APP_SAMPLING,
} appState_t;

typedef struct {
  appState_t state;
  bool isNewState;
  uint8_t discoverIndex;
  uint8_t requestIndex;
} appData_t;

appData_t appData;
bool appIsNewState();
void appChangeState(appState_t state);

// Buttons
Bounce goButton;

// Radio
RF24 radio(PIN_RADIO_CE, PIN_RADIO_CS);

#define IS_RADIO_MAX_NODES 16      // absolute maximum is 253

#define IS_PAYLOAD_MAX_SIZE 24
#define IS_PAYLOAD_SAMPLES 8
#define IS_PAYLOAD_NAME_MAX 17

// Radio pipes (addresses)
// The master node always listens on the "prefix + 0" address
// Configured nodes listen on "prefix + 1" to "prefix + 253"
// New nodes (need configuration) listen on "prefix + 254"
// Nodes in the error state listen on "prefix + 255"
const uint32_t IS_PIPE_PREFIX = 0xD0D1D2D3;

#define IS_PAYLOAD_TYPE_COMMAND 0
#define IS_PAYLOAD_TYPE_NODE_INFO 1
#define IS_PAYLOAD_TYPE_NODE_DATA 2

#define IS_COMMAND_DISCOVER 0
#define IS_COMMAND_START_SAMPLING 1
#define IS_COMMAND_STOP_SAMPLING 2
#define IS_COMMAND_GET_SAMPLES 3
#define IS_COMMAND_SLEEP 10

typedef struct {
  uint8_t packetType;
  uint8_t nodeID;
  uint8_t noOfChannels;
  uint8_t nodeState;
  uint16_t battV;                         // battery voltage, mV
  char nodeName[IS_PAYLOAD_NAME_MAX + 1]; // C null terminated string 17 chars max
} is_node_info_p;                         // Node to station

typedef struct {
  uint8_t packetType;
  uint8_t nodeID;
  uint8_t command;
  uint16_t parameter0;
  uint16_t parameter1;
} is_node_command_p;                      // Station to node

typedef struct {
    uint8_t packetType;
    uint8_t nodeID;
    uint8_t nodeState;
    uint32_t sampleId;
    uint8_t noOfSamples;
    uint16_t samples[IS_PAYLOAD_SAMPLES];
} is_node_data_p;                         // Node to station

void appHandleMessage(uint8_t * buffer, uint8_t length, uint8_t rxPipe);

void IRQ_Radio( void );

typedef struct {
  uint8_t arc;
  bool enabled;
  uint8_t errorCount;
  is_node_info_p payload;
} is_node_entry_t;

typedef struct {
  uint8_t noOfNodes;
  is_node_entry_t nodes[IS_RADIO_MAX_NODES];
} is_node_list_t;

is_node_list_t nodeList;

void discoverNodes(uintptr_t context);  // ran on interval, accesses global var appData (write to discoverIndex) and nodeList (write to arc for each nodeID)
void handleNodeInfo(is_node_list_t * nodeList, is_node_info_p * newNode);
void printoutNodes(uintptr_t context);  // ran on interval, accesses global var node list (read only)

void sendSamplingCommand(is_node_list_t * nodeList, bool start);
void requestSample(uintptr_t context);  // ran on interval, accesses global var appData (write to requestIndex) and nodeList (write to arc for each nodeID)

void handleNodeData(is_node_list_t * nodeList, is_node_data_p * nodeData);

PolledInterval discoverInterval(discoverNodes, IS_INTERVAL_DISCOVER);
PolledInterval printoutInterval(printoutNodes, IS_INTERVAL_PRINTOUT);
PolledInterval sampleRequestInterval(requestSample, IS_INTERVAL_SAMPLE_REQUEST);

// -----------------------------------------------------------------------------
// Setup
void setup() {
  delay(1);

  Serial.begin(115200);
  Serial.println(F("Booting..."));

  pinMode(13, OUTPUT);
  pinMode(PIN_RADIO_IRQ, INPUT_PULLUP);

  // Radio IRQ
  //attachInterrupt(digitalPinToInterrupt(PIN_RADIO_IRQ), IRQ_Radio, FALLING);

  // Buttons
  pinMode(PIN_BUTTON, INPUT_PULLUP);
  goButton.attach(PIN_BUTTON);
  goButton.interval(10);

  // Radio
  if (!radio.begin()) {
    Serial.println(F("No radio!"));
    while (1) {;;};
  }

  if (!radio.isPVariant()) {
    Serial.println(F("Radio is not compatbile!"));
    while (1) {;;};
  }

  radio.stopListening();
  radio.setPALevel(RF24_PA_LOW);
  radio.setPayloadSize(24);
  radio.setAddressWidth(5);
  radio.setDataRate(RF24_2MBPS);
  radio.setRetries(IS_AR_DELAY, IS_AR_COUNT);
  radio.setChannel(IS_RADIO_CHANNEL);
  radio.disableDynamicPayloads();
  radio.disableAckPayload();
  radio.setCRCLength(RF24_CRC_8);

  uint64_t writePipe = (uint64_t)IS_PIPE_PREFIX << 8 | 1;
  uint64_t readPipe = (uint64_t)IS_PIPE_PREFIX << 8 | 0;

  Serial.print("Reading Pipe: ");
  print64ln(readPipe, HEX);

  radio.openWritingPipe(writePipe);
  radio.openReadingPipe(1, readPipe);
  radio.flush_tx();
  radio.flush_rx();
  radio.startListening();
};

// -----------------------------------------------------------------------------
// Main Loop

void loop() {

  goButton.update();

  // Receive messages
  uint8_t rxPipe;
  uint8_t rxBuffer[IS_PAYLOAD_MAX_SIZE];
  if (radio.available(&rxPipe)) {
    uint8_t rxLength = radio.getPayloadSize();
    radio.read(&rxBuffer, rxLength);
    appHandleMessage(rxBuffer, rxLength, rxPipe);
  }

  // Handle intervals
  discoverInterval.run();
  printoutInterval.run();
  sampleRequestInterval.run();

  // Main state machine
  switch (appData.state) {

    // -------------------------------------------------------------------------
    // INIT STATE
    case APP_INIT:
    {
      // todo
      // configure device
      appChangeState(APP_IDLE);
    }
    break;

    // -------------------------------------------------------------------------
    // IDLE STATE
    case APP_IDLE:
    {
      if (appIsNewState()) {
        Serial.println("APP: Entered idle state");
        discoverInterval.enable();
        printoutInterval.enable();
        sampleRequestInterval.disable();
      }

      if (goButton.fell()) {

        if (nodeList.noOfNodes != 0) {
          appChangeState(APP_SAMPLING);
        } else {
          Serial.println("APP: Not switching to sampling, no registered nodes...");
        }
      }

    }
    break;

    // -------------------------------------------------------------------------
    // SAMPLING STATE
    case APP_SAMPLING:
    {
      if (appIsNewState()) {
        discoverInterval.disable();
        printoutInterval.disable();
        sampleRequestInterval.enable();
        Serial.println("APP: Entered sampling state");
        sendSamplingCommand(&nodeList, true);
      }

      if (goButton.fell()) {
        // Stop sampling
        sendSamplingCommand(&nodeList, false);
        appChangeState(APP_IDLE);
      }
    }
    break;

  }
};

// -----------------------------------------------------------------------------
// Main message handler
void appHandleMessage(uint8_t * buffer, uint8_t length, uint8_t rxPipe)
{
  if (!buffer) {
    return;
  }

  if (length <= 0) {
    return;
  }

  if (rxPipe != 1) {
    // Should only be receiving messages on pipe 1
    Serial.print("GOT A MESSAGE ON PIPE "); Serial.println(rxPipe);
    //return;
  }

  uint8_t packetType = buffer[0];
  //Serial.print("Got a message, packet type = ");Serial.println(packetType);

  switch (packetType) {

    // todo for ALL cases:
    // check if length of packet matches expected

    case IS_PAYLOAD_TYPE_COMMAND:
    {
      // Master shouldn't receive these
    }
    break;

    case IS_PAYLOAD_TYPE_NODE_INFO:
    {
      handleNodeInfo(&nodeList, reinterpret_cast<is_node_info_p *>(buffer));
    }
    break;

    case IS_PAYLOAD_TYPE_NODE_DATA:
    {
      handleNodeData(&nodeList, reinterpret_cast<is_node_data_p *>(buffer));
    }
    break;

    default:
    {
      Serial.println("UNKNOWN MESSAGE TYPE RECIVED!");
    }

  }

  return;
};

// -----------------------------------------------------------------------------
// Node list and manager
void discoverNodes(uintptr_t context)
{
  uint8_t nodeID = appData.discoverIndex;

  //Serial.print(F("\nSending discover packet for node "));
  //Serial.println(nodeID);

  is_node_command_p discoverPayload = {
    IS_PAYLOAD_TYPE_COMMAND,
    0,
    IS_COMMAND_DISCOVER,
    0,
    0
  };

  uint64_t writePipe = (uint64_t)IS_PIPE_PREFIX << 8 | nodeID;

  radio.stopListening();
  radio.openWritingPipe(writePipe);
  bool report = radio.write(&discoverPayload, sizeof(is_node_command_p));
  (void) report;
  //radio.writeFast(&discoverPayload, sizeof(is_node_command_p), false);
  radio.startListening();

  nodeList.nodes[appData.requestIndex].arc = radio.getARC();
  //Serial.print("ARC for node "); Serial.print(nodeID); Serial.print(" is ");
  //Serial.println(nodeList.nodes[appData.requestIndex].arc);

  appData.discoverIndex++;
  if (appData.discoverIndex > IS_RADIO_MAX_NODES) {
    appData.discoverIndex = 0;
  }
};

void handleNodeInfo(is_node_list_t * nodeList, is_node_info_p * newNodePacket)
{
  if (!nodeList) {
    return;
  }

  if (!newNodePacket) {
    return;
  }

  bool nodeExists = false;
  for (uint8_t i = 0; i < nodeList->noOfNodes; i++) {
    if (nodeList->nodes[i].payload.nodeID == newNodePacket->nodeID) {
      nodeExists = true;
      break;
    }
  }

  if (nodeExists) {
    return;
  }

  // This is a new node, add it to the list
  memcpy(&nodeList->nodes[nodeList->noOfNodes].payload, newNodePacket, sizeof(is_node_info_p));
  nodeList->nodes[nodeList->noOfNodes].arc = 0;
  nodeList->nodes[nodeList->noOfNodes].enabled = true;

  nodeList->noOfNodes++;
  Serial.print("Added node with ID = ");
  Serial.print(newNodePacket->nodeID);
  Serial.println(" to list");
};

void sendSamplingCommand(is_node_list_t * nodeList, bool start)
{
  if (!nodeList) {
    return;
  }

  if (nodeList->noOfNodes == 0) {
    return;
  }

  is_node_command_p commandPayload = {
    IS_PAYLOAD_TYPE_COMMAND,
    0,  // the same packet is sent to all enabled nodes
    start ? (uint8_t)IS_COMMAND_START_SAMPLING : (uint8_t)IS_COMMAND_STOP_SAMPLING,
    0,
    0,
  };

  for (uint8_t i = 0; i < nodeList->noOfNodes; i++) {
    uint8_t nodeID = i + 1;

    if (nodeList->nodes[i].enabled == false) {
      continue;
    }

    //Serial.print("Sending sampling command to node ID "); Serial.println(nodeID);
    uint64_t writePipe = (uint64_t)IS_PIPE_PREFIX << 8 | nodeID;

    radio.stopListening();
    radio.openWritingPipe(writePipe);
    radio.write(&commandPayload, sizeof(is_node_command_p));
    radio.startListening();
  }
};

void requestSample(uintptr_t context)
{
  uint8_t nodeID = appData.requestIndex + 1;

  //Serial.print(nodeList.noOfNodes); Serial.print(" nodes connected, requesting sample for node "); Serial.println(nodeID);

  // request data
  is_node_command_p requestPayload = {
    IS_PAYLOAD_TYPE_COMMAND,
    nodeID,
    IS_COMMAND_GET_SAMPLES,
    0,
    0,
  };

  uint64_t writePipe = (uint64_t)IS_PIPE_PREFIX << 8 | nodeID;

  radio.stopListening();
  radio.openWritingPipe(writePipe);
  bool report = radio.write(&requestPayload, sizeof(is_node_command_p));
  radio.startListening();

  nodeList.nodes[appData.requestIndex].arc = radio.getARC();
  //Serial.print("ARC for node "); Serial.print(nodeID); Serial.print(" is ");
  //Serial.println(nodeList.nodes[appData.requestIndex].arc);

  if (!report) {
    Serial.print("Failed to send sample request for node ID ");
    Serial.println(nodeID);
  }

  appData.requestIndex++;
  if (appData.requestIndex >= nodeList.noOfNodes) {
    appData.requestIndex = 0;
  }
};

void handleNodeData(is_node_list_t * nodeList, is_node_data_p * nodeData)
{
  if (!nodeList) {
    return;
  }

  if (!nodeData) {
    return;
  }

  uint8_t nodeID = nodeData->nodeID;
  /*
  Serial.print("Got a data packet from node "); Serial.println(nodeID);
  Serial.print("Samples index = "); Serial.println(nodeData->sampleId);
  for (int i = 0; i < IS_PAYLOAD_SAMPLES; i++) {
    Serial.println(nodeData->samples[i]);
  }
  Serial.println("");
  */
  // todo:
  // handle data
};

void printoutNodes(uintptr_t context)
{
  if (nodeList.noOfNodes == 0) {
    return;
  }

  Serial.println(F("\nConnected nodes:"));
  for (uint8_t i = 0; i < nodeList.noOfNodes; i++) {
    char printBuffer[64];
    snprintf(printBuffer, 64, "[%u] ID=%u S=%u B=%umV - %s", i, nodeList.nodes[i].payload.nodeID, nodeList.nodes[i].payload.nodeState, nodeList.nodes[i].payload.battV, nodeList.nodes[i].payload.nodeName);
    Serial.println(printBuffer);
  }
};

// -----------------------------------------------------------------------------
// Radio IRQ - not used (yet)
void IRQ_Radio( void )
{
  Serial.print("RADIO IRQ: txOk  = ");
  delayMicroseconds(250);
  bool txOk, txFail, rxReady;
  radio.whatHappened(txOk, txFail, rxReady);
  Serial.print(txOk); Serial.print(", txFail: ");
  Serial.print(txFail); Serial.print(", rxReady: ");
  Serial.println(rxReady);

  if (txFail) {
    // Transmit failed, clear tx FIFO
    radio.flush_tx();
  }
};

// -----------------------------------------------------------------------------
// Flow Control
bool appIsNewState()
{
  if (appData.isNewState) {
    appData.isNewState = false;
    return true;
  }
  return false;
};

void appChangeState(appState_t state)
{
  appData.isNewState = true;
  appData.state = state;
};
