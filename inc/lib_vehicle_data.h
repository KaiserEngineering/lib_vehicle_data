#ifndef LIB_VEHICLE_DATA_H
#define LIB_VEHICLE_DATA_H

#include "lib_pid.h"

#define LIB_VEHICLE_MAX_PARAMS 5
#define LIB_VEHICLE_MAX_DATA LIB_VEHICLE_MAX_PARAMS * 2
#define LIB_VEHICLE_DATA_BUFFER_SIZE 10

typedef enum _vehicle_data_status {
    VEHICLE_PARAM_UNSUPPORTED,
    VEHICLE_DATA_OK,
    VEHICLE_MAX_PARAMS,
} VEHICLE_DATA_STATUS, *PTR_VEHICLE_DATA_STATUS;

typedef struct _vehicle_data_equation {
    uint8_t equation;
        #define VEHICLE_DATA_EQ_NOT_DEFINED     0
        #define VEHICLE_DATA_EQ_VAL1_MINUS_VAL2 1
        #define VEHICLE_DATA_EQ_TOGGLE_ON_TRUE  2
} VEHICLE_DATA_EQUATION, *PTR_VEHICLE_DATA_EQUATION;

typedef struct _vehicle_data_manager {

    PTR_PID_DATA stream[LIB_VEHICLE_MAX_PARAMS];

    PTR_PID_DATA data1[LIB_VEHICLE_MAX_PARAMS];

    PTR_PID_DATA data2[LIB_VEHICLE_MAX_PARAMS];

    uint8_t flag;

    VEHICLE_DATA_EQUATION formula[LIB_VEHICLE_MAX_PARAMS];

    uint8_t data_status[LIB_VEHICLE_MAX_DATA];
        #define VEHICLE_DATA_STATUS_READY  0
        #define VEHICLE_DATA_STATUS_REMOVE 1
        #define VEHICLE_DATA_STATUS_ADD    2

    clear_pid_request clear_pid;

    request_pid_data req_pid;

    uint8_t num_data;

    uint8_t num_pids;

} VEHICLE_DATA_MANAGER, *PTR_VEHICLE_DATA_MANAGER;

void Vehicle_Init( PTR_VEHICLE_DATA_MANAGER dev );

VEHICLE_DATA_STATUS Vehicle_add_parameter( PTR_VEHICLE_DATA_MANAGER dev, PTR_PID_DATA pid );

VEHICLE_DATA_STATUS Vehicle_remove_PID_request( PTR_VEHICLE_DATA_MANAGER dev, PTR_PID_DATA pid );

void Vehicle_service( PTR_VEHICLE_DATA_MANAGER dev );

void Vehicle_tick( void );

#endif // LIB_VEHICLE_DATA_H
