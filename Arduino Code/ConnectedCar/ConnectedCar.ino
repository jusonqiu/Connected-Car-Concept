

/* Copyright (c) 2014, Nordic Semiconductor ASA
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */


/*
IMPORTANT NOTE:

IN ORDER TO COMPILE THIS CODE, YOU WILL NEED THE LIBRARIES ALSO INCLUDED WITH THIS CODE.
THESE LIBRARIES ARE:

BLE
TIME

ALONG SIDE THESE, YOU WILL NEED THE ARDUINO IDE, SPECIFICALLY VERSION 1.0.6 . This version includes SPI and EEPROM
LIBRARIES WHICH ARE NEEDED TO COMPILE THIS CODE. THE IDE IS ALSO INCLUDED AS A ZIP FILE IN THIS DOCUMENTATION.

*/
 
 
/*
Import libraries
*/

#include <SPI.h>
#include <lib_aci.h>
#include <aci_setup.h>
#include "services.h"
#include <EEPROM.h>
#include <Time.h>


/* these variables are used to manage time after login into device*/
long t;
int second_now;
long int seconds_passed = 0;


/*three different leds used. Green for connection, red for Disconnected and yellow for Bluetooth locked out*/

int green_LED = A5;
int yellow_LED = 2;
int red_LED = 3;

/* Four relays manage this system. */
int relay_garage_stop = 7;
int relay_garage_open = 4;
int relay_garage_close = 5;
int relay_lights_switch = 10;

/*Push Buttons */

int pushbutton_garage_stop = A0;
int pushbutton_garage_open = A1;
int pushbutton_garage_close = A2;
int pushbutton_light_switch = A3;
int pushbutton_wireless_unlock = A4;

// char value eeprom_password[0] to store byte sized password value.
char eeprom_password[0];

//value to store result of login into the gatebox system. 0 is unsuccessful login and 1 is successful login.
uint8_t result = 0;

//value to store result of password change of the gatebox system. 
uint8_t change_result = 0;

//1: garage closed. 0: garage open.
uint8_t garage_state = 1;

//1: g
uint8_t light_state = 1;
uint8_t light_state_now = 1;
uint8_t garage_state_now = 1;

//pipe_num value determines which pipe number is being used to communicate with the gatebox. Different pipes are used to talk to different characteristics.
uint8_t pipe_num;

//new password to rewrite the old password.
uint8_t overwrite_password_value[16];


/* Data is transmitted over the pipe as two hexadecimal digits.
The four digit password can start with 0s. If that is the case, then these 0s will be ignored o
ver the transmission. If the transmission of a byte was less than 2 hexadicamal digits
we know that the password began from a zero. This 0 must be inserted at the front of the file. */
String pipe_data_part_1;
String pipe_data_part_2;

/*The two portions pipe_data_part_1 and pipe_data_part_2 are then concatenated to form the password */
String pipe_data_full_hex;

//the entered password must be changed into string so that its individual digits can be read in an order.
String pipe_data_full_dec;



//char incoming_data[34];

uint8_t data_size;

//incorrect_password_counter counts the number of times data has been entered incorrectly into the system.
uint8_t incorrect_password_counter = 0;

//test boolean for garage and light
boolean connection_light = false;
boolean wireless_locked = false;

/*
boolean light_push_button = false;
boolean garage_push_button = false;
*/

boolean password_check_state = false;
boolean password_change_state = false;
//boolean overwrite_check_password_now = false;
boolean overwrite_change_password_now = false;

boolean overwrite_password_now = false;

/*
boolean garage_open_state = false;
boolean garage_close_state = false;
boolean garage_stop_state = false;
boolean lights_change_state = false;
*/

boolean garage_state_send = false;
boolean light_state_send = false;

//these integers will be used to store the individual digits from the string in the for loop. The for loop with compare the two password strings at each position.
int eeprom_password_digit;
int entered_password_digit;

int address = 0;

//may have to delete this later.
//char entered_password_as_string[34];

//char entered_password_part_1[34];

//char entered_password_part_2[34];

//the password currently stored in eeprom will be copied into this variable.
char password_in_eeprom[0];


  

#ifdef SERVICES_PIPE_TYPE_MAPPING_CONTENT
  static services_pipe_type_mapping_t
      services_pipe_type_mapping[NUMBER_OF_PIPES] = SERVICES_PIPE_TYPE_MAPPING_CONTENT;
#else
  #define NUMBER_OF_PIPES 0
  static services_pipe_type_mapping_t * services_pipe_type_mapping = NULL;
#endif


/*
Store the nRF8001 setup information generated on the flash of the AVR.
This reduces the RAM requirements for the nRF8001.
*/
static hal_aci_data_t setup_msgs[NB_SETUP_MESSAGES] PROGMEM = SETUP_MESSAGES_CONTENT;
// an aci_struct that will contain
// total initial credits
// current credit
// current state of the aci (setup/standby/active/sleep)
// open remote pipe pending
// close remote pipe pending
// Current pipe available bitmap
// Current pipe closed bitmap
// Current connection interval, slave latency and link supervision timeout
// Current State of the the GATT client (Service Discovery)

static aci_state_t aci_state;

static hal_aci_evt_t aci_data;
static hal_aci_data_t aci_cmd;

/*
We will store the bonding info for the nRF8001 in the EEPROM/Flash of the MCU to recover from a power loss situation
*/
static bool bonded_first_time = true;

static bool radio_ack_pending  = false;
static bool timing_change_done = false;

/* Define how assert should function in the BLE library */
void __ble_assert(const char *file, uint16_t line)
{
  Serial.print("ERROR ");
  Serial.print(file);
  Serial.print(": ");
  Serial.print(line);
  Serial.print("\n");
  while(1);
}

/*************NOTE**********
Scroll to the end of the file and read the loop() and setup() functions.
The loop/setup functions is the equivalent of the main() function
*/

/*
Read the Dymamic data from the EEPROM and send then as ACI Write Dynamic Data to the nRF8001
This will restore the nRF8001 to the situation when the Dynamic Data was Read out
*/


aci_status_code_t bond_data_restore(aci_state_t *aci_stat, uint8_t eeprom_status, bool *bonded_first_time_state)
{
  aci_evt_t *aci_evt;
  uint8_t eeprom_offset_read = 1;
  uint8_t write_dyn_num_msgs = 0;
  uint8_t len = 0;
  

  // Get the number of messages to write for the eeprom_status
  write_dyn_num_msgs = eeprom_status & 0x7F;

  //Read from the EEPROM
  while(1)
  {
    len = EEPROM.read(eeprom_offset_read);
    eeprom_offset_read++;
    aci_cmd.buffer[0] = len;

    for (uint8_t i=1; i<=len; i++)
    {
        aci_cmd.buffer[i] = EEPROM.read(eeprom_offset_read);
        eeprom_offset_read++;
    }
    //Send the ACI Write Dynamic Data
    if (!hal_aci_tl_send(&aci_cmd))
    {
      Serial.println(F("bond_data_restore: Cmd Q Full"));
      return ACI_STATUS_ERROR_INTERNAL;
    }

    //Spin in the while loop waiting for an event
    while (1)
    {
      if (lib_aci_event_get(aci_stat, &aci_data))
      {
        aci_evt = &aci_data.evt;

        if (ACI_EVT_CMD_RSP != aci_evt->evt_opcode)
        {
          //Got something other than a command response evt -> Error
          Serial.print(F("bond_data_restore: Expected cmd rsp evt. Got: 0x"));
          Serial.println(aci_evt->evt_opcode, HEX);
          return ACI_STATUS_ERROR_INTERNAL;
        }
        else
        {
          write_dyn_num_msgs--;

          //ACI Evt Command Response
          if (ACI_STATUS_TRANSACTION_COMPLETE == aci_evt->params.cmd_rsp.cmd_status)
          {
            //Set the state variables correctly
            *bonded_first_time_state = false;
            aci_stat->bonded = ACI_BOND_STATUS_SUCCESS;

            delay(10);

            return ACI_STATUS_TRANSACTION_COMPLETE;
          }
          if (0 >= write_dyn_num_msgs)
          {
            //should have returned earlier
            return ACI_STATUS_ERROR_INTERNAL;
          }
          if (ACI_STATUS_TRANSACTION_CONTINUE == aci_evt->params.cmd_rsp.cmd_status)
          {
            //break and write the next ACI Write Dynamic Data
            break;
          }
        }
      }
    }
  }
}



/*
This function is specific to the atmega328
@params ACI Command Response Evt received from the Read Dynmaic Data
*/


void bond_data_store(aci_evt_t *evt)
{
  static int eeprom_write_offset = 1;

  //Write it to non-volatile storage
  EEPROM.write( eeprom_write_offset, evt->len -2 );
  eeprom_write_offset++;

  EEPROM.write( eeprom_write_offset, ACI_CMD_WRITE_DYNAMIC_DATA);
  eeprom_write_offset++;

  for (uint8_t i=0; i< (evt->len-3); i++)
  {
    EEPROM.write( eeprom_write_offset, evt->params.cmd_rsp.params.padding[i]);
    eeprom_write_offset++;
  }
}



bool bond_data_read_store(aci_state_t *aci_stat)
{
  /*
  The size of the dynamic data for a specific Bluetooth Low Energy configuration
  is present in the ublue_setup.gen.out.txt generated by the nRFgo studio as "dynamic data size".
  */
  
  bool status = false;
  aci_evt_t * aci_evt = NULL;
  uint8_t read_dyn_num_msgs = 0;

  //Start reading the dynamic data
  lib_aci_read_dynamic_data();
  read_dyn_num_msgs++;

  while (1)
  {
    if (true == lib_aci_event_get(aci_stat, &aci_data))
    {
      aci_evt = &aci_data.evt;

      if (ACI_EVT_CMD_RSP != aci_evt->evt_opcode)
      {
        //Got something other than a command response evt -> Error
        status = false;
        break;
      }

      if (ACI_STATUS_TRANSACTION_COMPLETE == aci_evt->params.cmd_rsp.cmd_status)
      {
        //Store the contents of the command response event in the EEPROM
        //(len, cmd, seq-no, data) : cmd ->Write Dynamic Data so it can be used directly
        bond_data_store(aci_evt);

        //Set the flag in the EEPROM that the contents of the EEPROM is valid
        EEPROM.write(0, 0x80|read_dyn_num_msgs );
        //Finished with reading the dynamic data
        status = true;

        break;
      }

      if (!(ACI_STATUS_TRANSACTION_CONTINUE == aci_evt->params.cmd_rsp.cmd_status))
      {
        //We failed the read dymanic data
        //Set the flag in the EEPROM that the contents of the EEPROM is invalid
        EEPROM.write(0, 0xFF);

        status = false;
        break;
      }
      else
      {
        //Store the contents of the command response event in the EEPROM
        // (len, cmd, seq-no, data) : cmd ->Write Dynamic Data so it can be used directly when re-storing the dynamic data
        bond_data_store(aci_evt);

        //Read the next dynamic data message
        lib_aci_read_dynamic_data();
        read_dyn_num_msgs++;
      }
    }
  }
  return status;
}



void aci_loop()
{
  static bool setup_required = false;

  // We enter the if statement only when there is a ACI event available to be processed
  if (lib_aci_event_get(&aci_state, &aci_data))
  {
    aci_evt_t * aci_evt;

    aci_evt = &aci_data.evt;
    switch(aci_evt->evt_opcode)
    {
      /**
      As soon as you reset the nRF8001 you will get an ACI Device Started Event
      */
      case ACI_EVT_DEVICE_STARTED:
      {
        aci_state.data_credit_total = aci_evt->params.device_started.credit_available;
        switch(aci_evt->params.device_started.device_mode)
        {
          case ACI_DEVICE_SETUP:
            /**
            When the device is in the setup mode
            */
            aci_state.device_state = ACI_DEVICE_SETUP;
            Serial.println(F("Evt Device Started: Setup"));
            setup_required = true;
            break;

          case ACI_DEVICE_STANDBY:
            aci_state.device_state = ACI_DEVICE_STANDBY;
            Serial.println(F("Evt Device Started: Standby"));
            if (aci_evt->params.device_started.hw_error)
            {
              delay(20); //Magic number used to make sure the HW error event is handled correctly.
            }
            else
            {              
              //Manage the bond in EEPROM of the AVR
              {
                uint8_t eeprom_status = 0;
                eeprom_status = EEPROM.read(0);
                if (eeprom_status != 0xFF)
                {
                  Serial.println(F("Previous Bond present. Restoring"));
                  Serial.println(F("Using existing bond stored in EEPROM."));
                  Serial.println(F("   To delete the bond stored in EEPROM, connect Pin 6 to 3.3v and Reset."));
                  Serial.println(F("   Make sure that the bond on the phone/PC is deleted as well."));
                  //We must have lost power and restarted and must restore the bonding infromation using the ACI Write Dynamic Data
                  if (ACI_STATUS_TRANSACTION_COMPLETE == bond_data_restore(&aci_state, eeprom_status, &bonded_first_time))
                  {
                    Serial.println(F("Bond restored successfully"));
                  }
                  else
                  {
                    Serial.println(F("Bond restore failed. Delete the bond and try again."));
                  }
                }
              }
              
              // Start bonding as all proximity devices need to be bonded to be usable
              
              if (ACI_BOND_STATUS_SUCCESS != aci_state.bonded)
              {
                lib_aci_bond(180,0x0050);
                Serial.println(F("No Bond present in EEPROM."));
                Serial.println(F("Advertising started : Waiting to be connected and bonded"));
              }
              
              else 
              {
                
                //connect to an already bonded device
                //Use lib_aci_direct_connect for faster re-connections with PC, not recommended to use with iOS/OS X
                lib_aci_connect(100/* in seconds */, 0x0020 /* advertising interval 20ms*/);
                Serial.println(F("Already bonded : Advertising started : Waiting to be connected"));
              }
            }
            break;
            
        } 
      }
        break; //ACI Device Started Event

      case ACI_EVT_CMD_RSP:
        //If an ACI command response event comes with an error -> stop
        if (ACI_STATUS_SUCCESS != aci_evt->params.cmd_rsp.cmd_status)
        {
          //ACI ReadDynamicData and ACI WriteDynamicData will have status codes of
          //TRANSACTION_CONTINUE and TRANSACTION_COMPLETE
          //all other ACI commands will have status code of ACI_STATUS_SCUCCESS for a successful command
          Serial.print(F("ACI Command "));
          Serial.println(aci_evt->params.cmd_rsp.cmd_opcode, HEX);
          Serial.print(F("Evt Cmd respone: Status "));
          Serial.println(aci_evt->params.cmd_rsp.cmd_status, HEX);
        }
        
        if (ACI_CMD_GET_DEVICE_VERSION == aci_evt->params.cmd_rsp.cmd_opcode)
        {
          //Store the version and configuration information of the nRF8001 in the Hardware Revision String Characteristic
          lib_aci_set_local_data(&aci_state, PIPE_DEVICE_INFORMATION_HARDWARE_REVISION_STRING_SET,
            (uint8_t *)&(aci_evt->params.cmd_rsp.params.get_device_version), sizeof(aci_evt_cmd_rsp_params_get_device_version_t));
        }

        break;
 /*       
     case ACI_EVT_CONNECTED:
            radio_ack_pending  = false;
            timing_change_done = false;
            aci_state.data_credit_available = aci_state.data_credit_total;
            Serial.println(F("Evt Connected"));
            break;
*/
     case ACI_EVT_CONNECTED:
        Serial.println(F("Evt Connected"));
        aci_state.data_credit_available = aci_state.data_credit_total;        
        timing_change_done = false;
        
        /*
        Get the device version of the nRF8001 and store it in the Hardware Revision String
        */
        
        lib_aci_device_version();
        
        
        
        break;
/*        
      case ACI_EVT_CONNECTED:
        radio_ack_pending  = false;
        //aci_state.data_credit_available = aci_state.data_credit_total;
        Serial.println(F("Evt Connected"));
        aci_state.data_credit_available = aci_state.data_credit_total;
        timing_change_done = false;
      
        //Get the device version of the nRF8001 and store it in the Hardware Revision String
       
        lib_aci_device_version();
        Serial.println("srs ACI_EVT_CONNECTED ");
        break;
*/
      case ACI_EVT_BOND_STATUS:
        aci_state.bonded = aci_evt->params.bond_status.status_code;
        break;
/*
      case ACI_EVT_PIPE_STATUS:
        Serial.println(F("Evt Pipe Status"));
        //Link is encrypted when the PIPE_LINK_LOSS_ALERT_ALERT_LEVEL_RX_ACK_AUTO is available
        if ((false == timing_change_done) &&
          //lib_aci_is_pipe_available(&aci_state, PIPE_LINK_LOSS_ALERT_ALERT_LEVEL_RX_ACK_AUTO))
          lib_aci_is_pipe_available(&aci_state, PIPE_CONNECTEDCAR_PASSWORDCHANGE_RX_ACK_AUTO))
        {
          lib_aci_change_timing_GAP_PPCP(); // change the timing on the link as specified in the nRFgo studio -> nRF8001 conf. -> GAP.
                                            // Used to increase or decrease bandwidth
          timing_change_done = true;
        }
        
        
        // The pipe will be available only in an encrpyted link to the phone 
        if ((ACI_BOND_STATUS_SUCCESS == aci_state.bonded) &&
              (lib_aci_is_pipe_available(&aci_state, PIPE_CONNECTEDCAR_LIGHTSON_TX_ACK_1)))
        {
          //Note: This may be called multiple times after the Arduino has connected to the right phone
          Serial.println(F("phone Detected."));
          Serial.println(F("Do more stuff here. when your phone is detected"));
        }
        
        break;
*/      
      case ACI_EVT_PIPE_STATUS:
        Serial.println(F("Evt Pipe Status"));
        /** check if the peer has subscribed to any of the ConnectedCar characteristics for Notifications.
        */
        //if data on any of these pipes is available, it will be sent in the form of notifications to the client (smartphone app).
        if ((lib_aci_is_pipe_available(&aci_state, PIPE_CONNECTEDCAR_RESULTS_TX) ||
        lib_aci_is_pipe_available(&aci_state, PIPE_CONNECTEDCAR_CHANGERESULTS_TX) ||
        lib_aci_is_pipe_available(&aci_state, PIPE_CONNECTEDCAR_LIGHTSON_TX) ||
        lib_aci_is_pipe_available(&aci_state, PIPE_CONNECTEDCAR_GARAGEDOOR_TX)) &&
        (false == timing_change_done))
        {
          
          /*
          Request a change to the link timing as set in the GAP -> Preferred Peripheral Connection Parameters
          Change the setting in nRFgo studio -> nRF8001 configuration -> GAP Settings and recompile the xml file.
          */
          
          Serial.println("mstests");
          lib_aci_change_timing_GAP_PPCP();
          timing_change_done = true;
        }

        break;
      
      case ACI_EVT_TIMING:
        Serial.println(F("Evt link connection interval changed"));
        //Disconnect as soon as we are bonded and required pipes are available
        //This is used to store the bonding info on disconnect and then re-connect to verify the bond
        if((ACI_BOND_STATUS_SUCCESS == aci_state.bonded) &&
           (true == bonded_first_time) &&
           (GAP_PPCP_MAX_CONN_INT >= aci_state.connection_interval) &&
           (GAP_PPCP_MIN_CONN_INT <= aci_state.connection_interval) && //Timing change already done: Provide time for the the peer to finish
           (lib_aci_is_pipe_available(&aci_state, PIPE_CONNECTEDCAR_PASSWORD_RX) &&
           lib_aci_is_pipe_available(&aci_state, PIPE_CONNECTEDCAR_PASSWORDCHANGE_RX) &&
           lib_aci_is_pipe_available(&aci_state, PIPE_CONNECTEDCAR_LIGHTSON_RX) &&
           lib_aci_is_pipe_available(&aci_state, PIPE_CONNECTEDCAR_GARAGEDOOR_RX)))
         {
           Serial.println("yup");
           lib_aci_disconnect(&aci_state, ACI_REASON_TERMINATE);
           
         }
         
         break;
 
     case ACI_EVT_DATA_CREDIT:
        aci_state.data_credit_available = aci_state.data_credit_available + aci_evt->params.data_credit.credit;
        
        /**
        Bluetooth Radio ack received from the peer radio for the data packet sent.
        This also signals that the buffer used by the nRF8001 for the data packet is available again.
        */
        
        radio_ack_pending = false;
        break;
      
      case ACI_EVT_PIPE_ERROR:
        /**
        Send data failed. ACI_EVT_DATA_CREDIT will not come.
        This can happen if the pipe becomes unavailable by the peer unsubscribing to the Heart Rate
        Measurement characteristic.
        This can also happen when the link is disconnected after the data packet has been sent.
        */
        //radio_ack_pending = false;

        //See the appendix in the nRF8001 Product Specication for details on the error codes
        Serial.print(F("ACI Evt Pipe Error: Pipe #:"));
        Serial.print(aci_evt->params.pipe_error.pipe_number, DEC);
        Serial.print(F("  Pipe Error Code: 0x"));
        Serial.println(aci_evt->params.pipe_error.error_code, HEX);

        //Increment the credit available as the data packet was not sent.
        //The pipe error also represents the Attribute protocol Error Response sent from the peer and that should not be counted
        //for the credit.
        if (ACI_STATUS_ERROR_PEER_ATT_ERROR != aci_evt->params.pipe_error.error_code)
        {
          aci_state.data_credit_available++;
        }
        break;


      case ACI_EVT_DISCONNECTED:
        Serial.println(F("Evt Disconnected. Link Lost or Advertising timed out"));
        
        //Upon disconnection, reset the value of result, change_result and seconds_passed to 0.
        result = 0;
        change_result = 0;
        seconds_passed = 0;
        connection_light = false;
        
        if (ACI_BOND_STATUS_SUCCESS == aci_state.bonded)
        {
          if (ACI_STATUS_EXTENDED == aci_evt->params.disconnected.aci_status) //Link was disconnected
          {
            if (bonded_first_time)
            {
              bonded_first_time = false;
              //Store away the dynamic data of the nRF8001 in the Flash or EEPROM of the MCU
              // so we can restore the bond information of the nRF8001 in the event of power loss
              if (bond_data_read_store(&aci_state))
              {
                Serial.println(F("Dynamic Data read and stored successfully"));
              }
            }
            if (0x24 == aci_evt->params.disconnected.btle_status)
            {
              //The error code appears when phone or Arduino has deleted the pairing/bonding information.
              //The Arduino stores the bonding information in EEPROM, which is deleted only by
              // the user action of connecting pin 6 to 3.3v and then followed by a reset.
              //While deleting bonding information delete on the Arduino and on the phone.
              Serial.println(F("phone/Arduino has deleted the bonding/pairing information"));
            }
            /* If proximitiy disconnect, then put the device in discovery mode and automatic pairing.
            The proximity service method can perhaps be used for this purpose once it is modified.
            
            proximity_disconect_evt_rcvd(aci_evt->params.disconnected.btle_status);
            */
          }
          lib_aci_connect(180, 0x0100);
          
          
          Serial.println(F("Using existing bond stored in EEPROM."));
          Serial.println(F("   To delete the bond stored in EEPROM, connect Pin 6 to 3.3v and Reset."));
          Serial.println(F("   Make sure that the bond on the phone/PC is deleted as well."));
          Serial.println(F("Advertising started. Connecting."));
        }
        else
        {
          //There is no existing bond. Try to bond.
          lib_aci_bond(180/* in seconds */, 0x0050 /* advertising interval 50ms*/);
          Serial.println(F("Advertising started. Bonding."));
        }
               
        
        break;

      case ACI_EVT_DATA_RECEIVED:
        
        Serial.print(F("Pipe #"));
        Serial.print(aci_evt->params.data_received.rx_data.pipe_number, DEC);
        Serial.print(F("-> "));
        Serial.println(aci_evt->params.data_received.rx_data.aci_data[0], HEX);
        Serial.println(aci_evt->params.data_received.rx_data.aci_data[1], HEX);
        
        pipe_num = aci_evt->params.data_received.rx_data.pipe_number;
        pipe_data_part_1 = String(aci_evt->params.data_received.rx_data.aci_data[0], HEX);
        pipe_data_part_2 = String(aci_evt->params.data_received.rx_data.aci_data[1], HEX);
        
        //if the 1st digit begins with a zero, it will be ignored and hence the length of the string will be short by one character. This character must be manually inserted into the string.
        if (pipe_data_part_1.length() < 2) {
          pipe_data_part_1 = '0' + pipe_data_part_1;
        }
        
        //if the 3rd digit begins with a zero, it will be ignored and hence the length of the string will be short by one character. This character must be manually inserted into the string.
        if (pipe_data_part_2.length() < 2) {
          pipe_data_part_2 = '0' + pipe_data_part_2;
        }       
        
        //concatenate the two portions of the password.
        pipe_data_full_hex = pipe_data_part_1 + pipe_data_part_2;        
        
        Serial.println(pipe_data_full_hex);
        
        //Convert the hexadecimal password to a decimal password.
        pipe_data_full_dec = String(hexToDec(pipe_data_full_hex));
        
        Serial.println(pipe_data_full_dec);        
        
        //while the system is not locked out, accept data on the pipes.
        if (wireless_locked == false) {
        
          switch (pipe_num)
          
          {
            //if the data is on the pipe called PIPE_CONNECTEDCAR_PASSWORD_RX,
            //then convert the data into pipe_data_full_dec and use the checkPassword method to determine if the password is correct.
            case PIPE_CONNECTEDCAR_PASSWORD_RX:
            
              result = checkPassword(pipe_data_full_dec);
              
              //display the password on the screen
              Serial.println(result);
              
              if (result == 1) {
                
                //once logged in, reset the timed out counter.
                seconds_passed = 0;
                
                //refresh the incorrect password counter;
                incorrect_password_counter = 0;
                Serial.println("correct password");
                
                //read the garage state stored in the eeprom. States are stored in eeprom so that if the device loses power but still maintains connection,
                //garage_state = read_state_in_eeprom(garage_state_in_eeprom);
                //light_state = read_state_in_eeprom(light_state_in_eeprom);
                
                //update the car on the current garage state and the light state.
                //Once these are set to true, the garage and light states' pipes are available. These means that if
                //the client (windows phone app) asks for data from available pipes, the garage state and light state will be sent.
                //sent as notifications.
                garage_state_send = true;
                light_state_send = true;
                
              } else {
                
                //if password is incorrect, increment the passwrod incorrect counter by 1.
                incorrect_password_counter++;
                Serial.println("Incorrect password.");
                Serial.print(3-incorrect_password_counter);
                Serial.println(" tries left");
                
              }
              
              //the result of the password check will be sent through a pipe that is set to available once password_check_state is true.
              password_check_state = true;
              

              //send result;
            /*  
              if (lib_aci_is_pipe_available(&aci_state, PIPE_CONNECTEDCAR_RESULTS_TX)) Serial.println("mew1");
              if (false == radio_ack_pending) Serial.println("mew2");
              if (true == timing_change_done) Serial.println("mew3");
              
              if (lib_aci_is_pipe_available(&aci_state, PIPE_CONNECTEDCAR_RESULTS_TX) 
              && (false == radio_ack_pending) 
              && (true == timing_change_done))
              
              {
                if (send_byte(PIPE_CONNECTEDCAR_RESULTS_TX, result))
                {
                  aci_state.data_credit_available--;
                  Serial.print(F("result: "));
                  Serial.println(result);
                  radio_ack_pending = true;
                }
            
              }    */     
              break;
            
            //pipe to change the password
            case PIPE_CONNECTEDCAR_PASSWORDCHANGE_RX:
            
              //first check if login credentials were correct.
              if (result == 1) {
                
                //reset the timed out timer to 0.
                seconds_passed = 0;
                
                //change the password to the data that is stored in pipe_data_full_dec and store the result in change_result (1 if change was successful).
                change_result = change_password_in_eeprom(pipe_data_full_dec);
                
                //set password change state to true. This will indicate that the pipe for transmitting the result of password change is available waiting to be sent. The smartphone can
                //request the data from the pipe at which point it will be sent in the form of a notification.
                password_change_state = true;
                
              }
              
              Serial.println("password changed");
              
              break;
            
            //pipe to change the state of the lights
            case PIPE_CONNECTEDCAR_LIGHTSON_RX:
              
              //if login credentials were correct
              if (result == 1) {
                
                //reset the timed out timer to 0.
                seconds_passed = 0;
                
                //store the light state to be set into light_state_now
                light_state_now = aci_evt->params.data_received.rx_data.aci_data[0];
                
                //execute the lights method.
                lights();
                
                
              }
              
              break;
              
            //pipe to change the state of the garage
            case PIPE_CONNECTEDCAR_GARAGEDOOR_RX:
              
              //if login credentials were correct.
              if (result == 1) {
                
                //reset the timed out timer to 0.
                seconds_passed = 0;
                
                //store the garage state to be into garage_state_now
                garage_state_now = aci_evt->params.data_received.rx_data.aci_data[0];
                
                //execute the garage method
                garage();
                
              }
              
              break;
          
          }
          
        }
        
        break;




      case ACI_EVT_HW_ERROR:
        Serial.print(F("HW error: "));
        Serial.println(aci_evt->params.hw_error.line_num, DEC);

        for(uint8_t counter = 0; counter <= (aci_evt->len - 3); counter++)
        {
        Serial.write(aci_evt->params.hw_error.file_name[counter]); //uint8_t file_name[20];
        }
        Serial.println();

        //Manage the bond in EEPROM of the AVR
        {
          uint8_t eeprom_status = 0;
          eeprom_status = EEPROM.read(0);
          if (eeprom_status != 0xFF)
          {
            Serial.println(F("Previous Bond present. Restoring"));
            Serial.println(F("Using existing bond stored in EEPROM."));
            Serial.println(F("   To delete the bond stored in EEPROM, connect Pin 6 to 3.3v and Reset."));
            Serial.println(F("   Make sure that the bond on the phone/PC is deleted as well."));
            //We must have lost power and restarted and must restore the bonding infromation using the ACI Write Dynamic Data
            if (ACI_STATUS_TRANSACTION_COMPLETE == bond_data_restore(&aci_state, eeprom_status, &bonded_first_time))
            {
              Serial.println(F("Bond restored successfully"));
            }
            else
            {
              Serial.println(F("Bond restore failed. Delete the bond and try again."));
            }
          }
        }

        // Start bonding as all proximity devices need to be bonded to be usable
        if (ACI_BOND_STATUS_SUCCESS != aci_state.bonded)
        {
          lib_aci_bond(180, 0x0050 );
          Serial.println(F("No Bond present in EEPROM."));
          Serial.println(F("Advertising started : Waiting to be connected and bonded"));
        }
    else
        {
          //connect to an already bonded device
          //Use lib_aci_direct_connect for faster re-connections with PC, not recommended to use with iOS/OS X
          lib_aci_connect(100/* in seconds */, 0x0020 /* advertising interval 20ms*/);
          Serial.println(F("Already bonded : Advertising started : Waiting to be connected"));
        }
        break;

    }
  }
  else
  {
    //Serial.println(F("No ACI Events available"));
    // No event in the ACI Event queue and if there is no event in the ACI command queue the arduino can go to sleep
    // Arduino can go to sleep now
    // Wakeup from sleep from the RDYN line
  }
  
  /* setup_required is set to true when the device starts up and enters setup mode.
   * It indicates that do_aci_setup() should be called. The flag should be cleared if
   * do_aci_setup() returns ACI_STATUS_TRANSACTION_COMPLETE.
   */
  if(setup_required)
  {
    if (SETUP_SUCCESS == do_aci_setup(&aci_state))
    {
      setup_required = false;
    }
  }
}

//This method changes the value of the garage to whatever it recieved over the pipe from the smartphone app.
void garage() {
 
    garage_state = garage_state_now;
    //write_state_to_eeprom(garage_state_in_eeprom, garage_state);
    
    switch (garage_state) 
    
    {
      
      //stop the garage from moving
      case 1:
      
        digitalWrite(relay_garage_stop, HIGH);
        digitalWrite(relay_garage_open, LOW);
        digitalWrite(relay_garage_close, LOW);
        
        Serial.println("garage_State_stop");
        break;
      
      //open the garage
      case 2:
      
        digitalWrite(relay_garage_stop, LOW);
        digitalWrite(relay_garage_open, HIGH);
        digitalWrite(relay_garage_close, LOW);
        
        Serial.println("garage_State_open");     
        break;
     
     //close the garage
      case 3:
      
        digitalWrite(relay_garage_stop, LOW);
        digitalWrite(relay_garage_open, LOW);
        digitalWrite(relay_garage_close, HIGH);
        
        Serial.println("garage_State_close");   
        break;
      
   }
   
   //if logged in, get ready to send the current state of the garage.
   if (result == 1) {
      
      garage_state_send = true;
      
   }
   
   Serial.println("garage state");
   Serial.println(garage_state);
  
}

//the following method simply changes the state of the lights to the value it recieved over the pipe from the smartphone app.
void lights() {

  light_state = light_state_now;
  //write_state_to_eeprom(light_state_in_eeprom, light_state);
  
  switch (light_state)
  {
    
    //turn off the lights
    case 1:
      
      digitalWrite(relay_lights_switch, LOW);
      Serial.println("light_State_off");   
      break;
    
    //turn on the lights
    case 2:
      
      digitalWrite(relay_lights_switch, HIGH);
      Serial.println("light state on");
      break;
    
  }
  
  //if login was successful, send the light_state_send.
  if (result == 1) {
    
    light_state_send = true;
  }
  
  Serial.println("lights state");
  Serial.println(light_state);  

}

//the following method is used to read password stored into EEROM.
void read_password_in_eeprom() {

  //read the four digits from the eeprom from 1000th byte to 1004th byte and store them in a the string password_in_eeprom.
  for (int i = 0; i < 5; i++) {
    //Serial.println (entered_password_as_string);      
    Serial.print("address");
    address=1000+i; 
    Serial.print(address);
    Serial.print(" => ");
    eeprom_password[i] = (char)EEPROM.read(address);
    Serial.println(eeprom_password[i]);

  }
  
  Serial.println(eeprom_password);
  
}

//Hexadecimal sring to decimal string converter by Ben Rugg (www.benrugg.com) made available under the MIT License.

unsigned int hexToDec(String hexString) {
  
  unsigned int decValue = 0;
  int nextInt;
  
  for (int i = 0; i < hexString.length(); i++) {
    
    nextInt = int(hexString.charAt(i));
    if (nextInt >= 48 && nextInt <= 57) nextInt = map(nextInt, 48, 57, 0, 9);
    if (nextInt >= 65 && nextInt <= 70) nextInt = map(nextInt, 65, 70, 10, 15);
    if (nextInt >= 97 && nextInt <= 102) nextInt = map(nextInt, 97, 102, 10, 15);
    nextInt = constrain(nextInt, 0, 15);
    
    decValue = (decValue * 16) + nextInt;
  }
  
  return decValue;
}

//this method is used to check the password in EEPROM. If the password is correct a byte value of 1 is returned. If incorrect, a byte value of 0 is returned.
uint8_t checkPassword(String entered_password) {
  
  //Read the password from the eeprom and store it into the password variable eeprom_password[]
  read_password_in_eeprom();
  
  Serial.println("checking password in eeprom");
  Serial.println(eeprom_password);
  
  // compare the first four digits of password_in_eeprom string with the last four digits of enterd_password_as_string.
  for (int i = 0; i < 4; i++) {
    
    if (eeprom_password[i] != entered_password[i]) {
      
      return 0;

    }
  }
  
  Serial.println("check password outputs");  
  return 1;
  
}

//this method is used to change passord in eeprom to the password entered into the String entered_password.
uint8_t change_password_in_eeprom (String entered_password) {
    
  int address = 0;
  
   //the password is stored on the 1000th to the 1003rd address bits in EEPROM.
   for (int i = 0; i < 4; i++){
     
     Serial.print("address ");
     address=1000+i;
     Serial.print(address);
     Serial.print(" => ");
     EEPROM.write(address, entered_password[i]);
     Serial.println(entered_password[i]);   
   }
   
   return 1; 

}



//this method is used to send byte data. It requires the pipe on which data is being sent and the byte data that is to be sent as inputs.
bool send_byte (int pipe_num, uint8_t send_data) {
      
    //if the pipe is available and radio_ack_pending is false and time_change_done is true
    if (lib_aci_is_pipe_available(&aci_state, pipe_num) 
    && (false == radio_ack_pending) 
    && (true == timing_change_done))
    
    {
      //If sending the data over the pipe was successful return true
      if (lib_aci_send_data(pipe_num, (uint8_t *)&send_data, 1))
      {
        
        aci_state.data_credit_available--;
        Serial.print(F("data sent: "));
        Serial.println(send_data);
        radio_ack_pending = true;
        return true;
      }
  
    }
      
    return false;
}

//a different method is used for password verifiction because it is two bytes long rather than one byte.
bool overwrite_password(int pipe_num, uint16_t value)
{
    //if the pipe is available and radio_ack_pending is false and time_change_done is true
    if (lib_aci_is_pipe_available(&aci_state, pipe_num) 
    && (false == radio_ack_pending) 
    && (true == timing_change_done))
    
    {
      overwrite_password_value[0] = (uint8_t)value;
      overwrite_password_value[1] = (uint8_t)(value>>8);
      //return lib_aci_send_data(PIPE_CONNECTEDCAR_PASSWORD_TX, (uint8_t *)&overwrite_password_value, data_index);
      
      //If sending the data over the pipe was successful return true
      if (lib_aci_send_data(pipe_num, (uint8_t *)&overwrite_password_value, 2))
      {
        aci_state.data_credit_available--;
        Serial.print(F("password overwritten: "));
        Serial.println(value);
        radio_ack_pending = true;
        return true;
      }
    }
    return false;
}

/*
Description:

<Add description of the proximity application.

The ACI Evt Data Credit provides the radio level ack of a transmitted packet.
*/
void setup(void)
{
  
  Serial.begin(115200);
  //Wait until the serial port is available (useful only for the Leonardo)
  //As the Leonardo board is not reseted every time you open the Serial Monitor
  #if defined (__AVR_ATmega32U4__)
    while(!Serial)
    {}
    delay(5000);  //5 seconds delay for enabling to see the start up comments on the serial board
  #elif defined(__PIC32MX__)
    delay(1000);
  #endif
  
  Serial.println(F("Arduino setup"));
  

  //set the pins for the LEDs as outputs.
  pinMode(red_LED, OUTPUT);
  pinMode(green_LED, OUTPUT);
  pinMode(yellow_LED, OUTPUT);
  
  //set the pins for the relays as outputs.
  pinMode(relay_garage_stop, OUTPUT);
  pinMode(relay_garage_open, OUTPUT);
  pinMode(relay_garage_close, OUTPUT);
  pinMode(relay_lights_switch, OUTPUT);
  
  //set the pins for the pushbuttons as inputs.  
  pinMode(pushbutton_garage_stop, INPUT);
  pinMode(pushbutton_garage_close, INPUT);
  pinMode(pushbutton_garage_open, INPUT);
  pinMode(pushbutton_light_switch, INPUT);
  pinMode(pushbutton_wireless_unlock, INPUT);
    
  /**
  Point ACI data structures to the the setup data that the nRFgo studio generated for the nRF8001
  */
  if (NULL != services_pipe_type_mapping)
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = &services_pipe_type_mapping[0];
  }
  else
  {
    aci_state.aci_setup_info.services_pipe_type_mapping = NULL;
  }
  aci_state.aci_setup_info.number_of_pipes    = NUMBER_OF_PIPES;
  aci_state.aci_setup_info.setup_msgs         = setup_msgs;
  aci_state.aci_setup_info.num_setup_msgs     = NB_SETUP_MESSAGES;

  //Tell the ACI library, the MCU to nRF8001 pin connections
  aci_state.aci_pins.board_name = BOARD_DEFAULT; //See board.h for details
  aci_state.aci_pins.reqn_pin   = 9;
  aci_state.aci_pins.rdyn_pin   = 8;
  aci_state.aci_pins.mosi_pin   = MOSI;
  aci_state.aci_pins.miso_pin   = MISO;
  aci_state.aci_pins.sck_pin    = SCK;

  aci_state.aci_pins.spi_clock_divider      = SPI_CLOCK_DIV8;//SPI_CLOCK_DIV8  = 2MHz SPI speed
                                                             //SPI_CLOCK_DIV16 = 1MHz SPI speed

  //aci_state.aci_pins.reset_pin              = 4; //4 for Nordic board, UNUSED for REDBEARLABS
  aci_state.aci_pins.active_pin             = UNUSED;
  aci_state.aci_pins.optional_chip_sel_pin  = UNUSED;

  aci_state.aci_pins.interface_is_interrupt = false;
  aci_state.aci_pins.interrupt_number       = UNUSED;

  //We reset the nRF8001 here by toggling the RESET line connected to the nRF8001
  //and initialize the data structures required to setup the nRF8001
  //The second parameter is for turning debug printing on for the ACI Commands and Events so they be printed on the Serial
  lib_aci_init(&aci_state, false);
  aci_state.bonded = ACI_BOND_STATUS_FAILED;

  pinMode(6, INPUT); //Pin #6 on Arduino -> PAIRING CLEAR pin: Connect to 3.3v to clear the pairing
  
  if (0x01 == digitalRead(6))
  {
    //Clear the pairing
    Serial.println(F("Pairing/Bonding info cleared from EEPROM."));
    Serial.println(F("Remove the wire on Pin 6 and reset the board for normal operation."));
    //Address. Value
    EEPROM.write(0, 0xFF);
    while(1) {};
  }
  
 seconds_passed = 0;

}

void loop()
{
  //static uint8_t value = 2;

  aci_loop();
  
/*
Before sending data, the following if statement carries out a check to confirm that the pipe
it is sending data on is infact available.
*/

  //if connection_light is true
  if (connection_light == true) {
    
    //turn the green LED on and the red LED off.
    digitalWrite(green_LED, HIGH);
    digitalWrite(red_LED, LOW);
    
  } else {
    
    //if connection_light is not true, turn the green LED off and the red LED on.
    digitalWrite(green_LED, LOW);
    digitalWrite(red_LED, HIGH);    
    
  }
    
//the timer increments by 1 every second.
  t = now();
  
  //if 65 seconds or greater have passed
  if (seconds_passed >= 65) {
    
    //disconnect the device
    lib_aci_disconnect(&aci_state, ACI_REASON_TERMINATE);
    
    //reset the timer.
    seconds_passed = 0;
    
    //reset result and change_result to 0.
    result = 0;
    change_result = 0;
    
    //set connection light to false. This will turn off the green LED and turn on the red LED.
    connection_light = false;

  }
  
  // while logged into the device and a second has passed
  if ((result == 1) && (second_now != second(t))) {
    
    //update the value of second
    second_now = second(t);
    
    //increment seconds_passed by 1
    seconds_passed++;
    Serial.println(seconds_passed);
    
    //set connection light to true.
    connection_light = true;
    
  }
  

  //if password was entered incorrectly 3 times, put the device under wireless lock.
  if (incorrect_password_counter >= 3) {
    
    wireless_locked = true;
    
  }
  
  //if the device is under wireless lock, disconnect the device immediately.
  if (wireless_locked == true) {
    
    //if authentication failed after connection, disconnect.
    lib_aci_disconnect(&aci_state, ACI_REASON_TERMINATE);
    result = 0;
    change_result = 0;
    
    //turn the yellow light ON indicating the device is locked.
    digitalWrite(yellow_LED, HIGH);
    
    
  }
  
  //if data is to be sent over the garagedoor characteristic's pipe
  if (garage_state_send == true) 
  
  {
    
    //if send_byte was successful
    if (send_byte(PIPE_CONNECTEDCAR_GARAGEDOOR_TX, garage_state)) {
      
      //set garage_state_send to false
      garage_state_send = false;
      Serial.println("garage state sent");
    }
    
  }
  
  //if data is to be sent over the lightson characteristic's pipe
  if (light_state_send == true) 
  
  {
    
    //if send_byte was successful
    if (send_byte(PIPE_CONNECTEDCAR_LIGHTSON_TX, light_state)) {
      
      //set light_state_send to false
      light_state_send = false;
      Serial.println("light state sent");
    }
    
  }
  
  //if data is to be sent over the results characteristic's pipe
  if (password_check_state == true) {
   
   //if send_byte was successful
   if (send_byte(PIPE_CONNECTEDCAR_RESULTS_TX, result)) 
   
   {
     //set password_check_to false
     password_check_state = false;
     Serial.println("password checked");
     
     //set overwrite_password_now to true.
     overwrite_password_now = true;
     
   }
   
  }
    
  //overwrite_password_now is used because once password is written by the smartphone, it is still present on the pipe. In order to erase this data,
  //the password characteristic's pipe sends a notification (which the smartphone app is programmed to recieve) once it has sent a password. The notification
  //consists of 0000. This overwrites the previous value on the characteristic.
  
  //set overwrite_password_now to true.
  if (overwrite_password_now == true) {
    
    //transmit 0000 over the pipe.
    if (overwrite_password(PIPE_CONNECTEDCAR_PASSWORD_TX, 0)) 
    
    {
      //set overwrite_password_now to false.
      overwrite_password_now = false;
      Serial.println("check password overwritten");

    } 
  }
 
  //if password_change_state is true
  if (password_change_state == true) {
   
   //if send_byte was successful
   if (send_byte(PIPE_CONNECTEDCAR_CHANGERESULTS_TX, change_result)) 
     {
       
       //set password_change_state to false.
       password_change_state = false;
       Serial.println("password changed");
       
       //set overwrite_change_password_now to true
       overwrite_change_password_now = true;
       
     }
  }
  
  //overwrite_change_password_now is used because once password is written by the smartphone, it is still present on the pipe. In order to erase this data,
  //the passwordchange characteristic's pipe sends a notification (which the smartphone app is programmed to recieve) once it has sent a password. The notification
  //consists of 0000. This overwrites the previous value on the characteristic.
  
  //if overwrite_change_password_now is set to true.
  if (overwrite_change_password_now == true) {
    
    //transmit 0000 over the pipe.
    if (overwrite_password(PIPE_CONNECTEDCAR_PASSWORDCHANGE_TX,0))
    
    {
      //overwrite_change_password_now
      overwrite_change_password_now = false;
      Serial.println("check password overwritten");

    }
    
  }

  //this is to manually release the wireless lock on the device by pressing the bluetooth unlock command.
  if (digitalRead(pushbutton_wireless_unlock) == LOW) {
    
    //debounce
    delay(50);
    
    //reset the incorrect_password_counter to 0
    incorrect_password_counter = 0;
    
    //wireless_locked is set to false.
    wireless_locked = false;
    
    //turn OFF the yellow LED
    digitalWrite(yellow_LED, LOW);

  }
  
  //if the garage stop pushbutton is pressed
  if (digitalRead(pushbutton_garage_stop) == LOW) {
    
    //debounce
    delay(50);
    
    //set the garage_state_now to 1
    garage_state_now = 1;
    Serial.println("garage_state_stop");
  }
  
  //if the garage open pushbutton is pressed
  if (digitalRead(pushbutton_garage_open) == LOW) {
    
    //debounce
    delay(50);
    
    //set the garage_state_now to 2
    garage_state_now = 2;
    Serial.println("garage_state_open");
  }
  
  //if the garage close pushbutton is pressed  
  if (digitalRead(pushbutton_garage_close) == LOW) {
    
    //debounce
    delay(50);
    
    //set the garage_state_now to 3
    garage_state_now = 3;
    Serial.println("garage_state_close");
  }
  
  //if the light switch pushbutton is pressed    
  if (digitalRead(pushbutton_light_switch) == LOW) {
        
    //debounce
    delay(50);
    
    //if the light was ON, turn it OFF. If the light was OFF turn it ON.
    if (light_state_now == 1) {
      
      light_state_now = 2;
      Serial.println("light_state_on");
      
    } else {
      
      light_state_now = 1;
      Serial.println("light_state_off");
      
    }

  }
  
  //if the garage state has been changed on the smartphone.
  if (garage_state_now != garage_state) {
    
    garage();
    
  }
  
  // if light_state has been changed on the smartphone.
  if (light_state_now != light_state) {  

    lights();
    
    delay(1000);
    
  }
  
}

/*
Further work needed on this code:

1. Testing

2. Tidy up code 
 - remove unncessary ACI commands.
 - uniform syntax
 - well commented and checked for grammar
 
*/
