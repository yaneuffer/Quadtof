#include <string.h>
#include <stdint.h>
#include <stdbool.h>

#include "app.h"

#include "FreeRTOS.h"
#include "task.h"

#include "debug.h"
#include "crtp.h"
#include "vl53l5cx_api.h"
#include "I2C_expander.h"
#include "log.h"
#include "kalman_core.h"
#include "arm_math.h"
#include "estimator.h"
#include "AAParamsYannick.h"
#include "stabilizer_types.h"

#define DEBUG_MODULE "TOFMATRIX"



static VL53L5CX_Configuration tof_dev[NR_OF_SENSORS];
static VL53L5CX_ResultsData tof_data0;
static VL53L5CX_ResultsData tof_data1;
static VL53L5CX_ResultsData tof_data2;
static VL53L5CX_ResultsData tof_data3;
// static int32_t logTof0;
// static int32_t logTof65;
// static int32_t logTof0;
// static int32_t logTof1;
// static int32_t logTof2;
// static int32_t logTof3;



void send_command(uint8_t command, uint8_t arg);
void send_data_packet(uint8_t *data, uint16_t data_len);
void send_data_packet_28b(uint8_t *data, uint8_t size, uint8_t index);

uint8_t config_sensors(VL53L5CX_Configuration *p_dev, uint16_t new_i2c_address);

#define TOF_I2C_ADDR 0x56
uint8_t tof_i2c_addresses[NR_OF_SENSORS];
uint16_t distancesprev[NR_OF_PIXELS * NR_OF_SENSORS];
uint64_t timestampprev = 0;
void appMain() {
   DEBUG_PRINT("Size of configuration %d \n", sizeof(VL53L5CX_Configuration));

   DEBUG_PRINT("Configured for %d ToF sensor(s) \n", NR_OF_SENSORS);
   // Configure GPIO expander pins modes
   I2C_expander_initialize();

   // Define the address of each ToF matrix sensor
   for(uint8_t i=0; i<NR_OF_SENSORS; i++)
      tof_i2c_addresses[i] = TOF_I2C_ADDR + 2 + 2*i;

   // Configure the ToF sensor(s)
   for(uint8_t i=0; i<NR_OF_SENSORS; i++) {
      I2C_expander_set_pin(i, 1); 

      // Configure the current sensor
      uint8_t status = config_sensors(&tof_dev[i], tof_i2c_addresses[i]);
      DEBUG_PRINT("Sensor %d conf. status: %d  (0 means ok) \n", i, status);
      
      // Start ranging
      status = vl53l5cx_start_ranging(&tof_dev[i]);
      DEBUG_PRINT("Sensor %d ranging status: %d  (0 means ok) \n", i, status);

      if (status == 0)
         I2C_expander_set_pin(LED0, 1);
   }

   uint8_t reg_value;
   i2cdevReadByte(I2C1_DEV, I2C_EXPANDER_I2C_ADDRESS, OUTPUT_REG_ADDRESS, &reg_value);
   DEBUG_PRINT("Sensor %d \n", reg_value);

   uint8_t ranging_ready = 255;
   uint8_t get_data_success = 255;
   //uint8_t to_send_buffer[4*NR_OF_PIXELS];
   
   while(1) {
      
      vTaskDelay(M2T(70));
      vl53l5cx_check_data_ready(&tof_dev[0], &ranging_ready);  // poll for data-ready
      MultitofMeasurement_t MultitofData;

      if (ranging_ready == 1) {
         get_data_success = vl53l5cx_get_ranging_data(&tof_dev[0], &tof_data0);
         memcpy(&MultitofData.distances, tof_data0.distance_mm, sizeof(uint16_t) * NR_OF_PIXELS);
         // if (get_data_success == VL53L5CX_STATUS_OK) { 
         //     for(uint8_t i=0; i<NR_OF_PIXELS; i++) {
         //        logTof0 = (int32_t)tof_data0.distance_mm[i]; 
         //    //     //DEBUG_PRINT("%d: %ld \n", i,  (int32_t)tof_data.distance_mm[i]);
               
         //     }
         //    //memcpy(&to_send_buffer[0], (uint8_t *)(&tof_data.distance_mm[0]), 2*NR_OF_PIXELS);
         //    //memcpy(&to_send_buffer[2*NR_OF_PIXELS], (uint8_t *)(&tof_data.nb_target_detected[0]), NR_OF_PIXELS);
         //    //memcpy(&to_send_buffer[3*NR_OF_PIXELS], (uint8_t *)(&tof_data.target_status[0]), NR_OF_PIXELS);

         //    // send_command(1, (4*NR_OF_PIXELS)/28 + 1);
         //    // send_data_packet(&to_send_buffer[0], 4*NR_OF_PIXELS);
            
            
         // }
      }
      // ranging_ready = 2;
      // if(NR_OF_SENSORS >= 2) {
      //    vTaskDelay(M2T(70));
      //    vl53l5cx_check_data_ready(&tof_dev[1], &ranging_ready);  // poll for data-ready

      //    if (ranging_ready == 1) {
      //       get_data_success = vl53l5cx_get_ranging_data(&tof_dev[1], &tof_data1);
      // memcpy(&MultitofData.distances + NR_OF_PIXELS, tof_data0.distance_mm, sizeof(uint16_t) * NR_OF_PIXELS);
            
      //    }
      //    ranging_ready = 2;
      // }

      // if(NR_OF_SENSORS >= 3) {
      // vTaskDelay(M2T(70));
      // vl53l5cx_check_data_ready(&tof_dev[2], &ranging_ready);  // poll for data-ready

      // if (ranging_ready == 1) {
      //    get_data_success = vl53l5cx_get_ranging_data(&tof_dev[2], &tof_data2);
      // memcpy(&MultitofData.distances + 2 * NR_OF_PIXELS, tof_data0.distance_mm, sizeof(uint16_t) * NR_OF_PIXELS);
         
      // }
      // ranging_ready = 2;
      // }

      // if(NR_OF_SENSORS >= 4) {
      // vTaskDelay(M2T(70));
      // vl53l5cx_check_data_ready(&tof_dev[3], &ranging_ready);  // poll for data-ready

      // if (ranging_ready == 1) {
      //    get_data_success = vl53l5cx_get_ranging_data(&tof_dev[3], &tof_data3);
      // memcpy(&MultitofData.distances + 3 * NR_OF_PIXELS, tof_data0.distance_mm, sizeof(uint16_t) * NR_OF_PIXELS);
        
      // }
      // ranging_ready = 2;

      // }
   uint16_t sum = 0;
   for(uint8_t j = 0; j<NR_OF_PIXELS; j++){
      sum += tof_data0.range_sigma_mm[j];
   }
   uint16_t averageDelta = sum / NR_OF_PIXELS; 
         
      memcpy(&MultitofData.distancesprev, &distancesprev, sizeof(uint16_t) * NR_OF_PIXELS * NR_OF_SENSORS);
      memcpy(&distancesprev, &MultitofData.distances, sizeof(uint16_t) * NR_OF_PIXELS * NR_OF_SENSORS);
      
      MultitofData.stdDev = averageDelta;
      uint64_t time = usecTimestamp();
      MultitofData.timestamp = time;
      MultitofData.timestampprev = timestampprev;
      timestampprev = time; 
      estimatorEnqueueMultiTOF(&MultitofData);
      
   }

// void  kalmanCoreUpdateWithTof(kalmanCoreData_t* this, tofMeasurement_t *tof){

//     kalmanCoreScalarUpdate(this, &H, measuredDistance-predictedDistance, tof->stdDev);
  
// }
   
}



void send_data_packet(uint8_t *data, uint16_t data_len) {
   uint8_t packets_nr = 0;
   if (data_len%28 > 0)
      packets_nr = data_len/28 + 1;
   else
      packets_nr = data_len/28;

   for (uint8_t idx=0; idx<packets_nr; idx++)
      if(data_len - 28*idx >= 28)
         send_data_packet_28b(&data[28*idx], 28, idx);
      else
         send_data_packet_28b(&data[28*idx], data_len - 28*idx, idx);
}


void send_data_packet_28b(uint8_t *data, uint8_t size, uint8_t index) {
   CRTPPacket pk;
   pk.header = CRTP_HEADER(1, 0); // first arg is the port number
   pk.size = size + 2;
   pk.data[0] = 'D';
   pk.data[1] = index;
   memcpy(&(pk.data[2]), data, size);
   crtpSendPacketBlock(&pk);
}

void send_command(uint8_t command, uint8_t arg) {
   CRTPPacket pk;
   pk.header = CRTP_HEADER(1, 0); // first arg is the port number
   pk.size = 5;
   pk.data[0] = 'C';
   pk.data[1] = command;
   pk.data[2] = arg;
   crtpSendPacketBlock(&pk);
}


uint8_t config_sensors(VL53L5CX_Configuration *p_dev, uint16_t new_i2c_address) {
   p_dev->platform = VL53L5CX_DEFAULT_I2C_ADDRESS; // use default adress for first use

   uint8_t status = 0;
   // Initialize the sensor
   status += vl53l5cx_init(p_dev); 

   // Change I2C address
   status += vl53l5cx_set_i2c_address(p_dev, new_i2c_address);
   status += vl53l5cx_set_resolution(p_dev, VL53L5CX_RESOLUTION_8X8);

   // 15Hz frame rate
   status += vl53l5cx_set_ranging_frequency_hz(p_dev, 15);
   status += vl53l5cx_set_target_order(p_dev, VL53L5CX_TARGET_ORDER_CLOSEST);
   status += vl53l5cx_set_ranging_mode(p_dev, VL53L5CX_RANGING_MODE_CONTINUOUS);

   return status;
}

// LOG_GROUP_START(Quadtof1)

// LOG_ADD(LOG_INT32, SensorTofDeck0, &logTof0)
// LOG_ADD(LOG_INT32, SensorTofDeck1, &logTof1)
// LOG_ADD(LOG_INT32, SensorTofDeck2, &logTof2)
// LOG_ADD(LOG_INT32, SensorTofDeck3, &logTof3)
// LOG_GROUP_STOP(Quadtof1)
// LOG_ADD(LOG_INT32, SensorTofDeck0, &logTof0)
// LOG_ADD(LOG_INT32, SensorTofDeck65, &logTof65)


//CLOAD_CMDS='-w radio://0/60/2M/E7E7E7E7E7' make cload
