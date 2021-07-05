#include "lib_vehicle_data.h"

uint8_t new_req = 0;

uint32_t vehicle_tick = 0;

void Vehicle_Init( PTR_VEHICLE_DATA_MANAGER dev )
{
    dev->num_pids = 0;
    dev->num_data = 0;

    for( uint8_t i = 0; i < LIB_VEHICLE_MAX_PARAMS; i++) {
        lib_pid_clear_PID( dev->data1[i] );
        lib_pid_clear_PID( dev->data2[i] );
    }

    for( uint8_t i = 0; i < LIB_VEHICLE_MAX_PARAMS; i++)
        dev->formula[i].equation = VEHICLE_DATA_EQ_NOT_DEFINED;
}

VEHICLE_DATA_STATUS Vehicle_add_parameter( PTR_VEHICLE_DATA_MANAGER dev, PTR_PID_DATA pid )
{
    switch( pid->mode )
    {
        case CALC1:
            switch( pid->pid )
            {
                case CALC1_TURBOCHARGER_COMPRESSOR_INLET_PRESSURE:

                    /* Set the flag to later request the needed data */
                    new_req = 1;

                    /* Verify another PID can be added */
                    if( dev->num_pids + 1 > LIB_VEHICLE_MAX_PARAMS )
                        return VEHICLE_MAX_PARAMS;

                    /* Save the PID pointer */
                    dev->stream[dev->num_pids++] = pid;

                    return VEHICLE_DATA_OK;

                default:
                    return VEHICLE_PARAM_UNSUPPORTED;
            }
            break;

        default:
            return VEHICLE_PARAM_UNSUPPORTED;
    }
}

VEHICLE_DATA_STATUS Vehicle_remove_PID_request( PTR_VEHICLE_DATA_MANAGER dev, PTR_PID_DATA pid )
{
    /* Set the request to sync PIDs */
    new_req = 1;

    /* Cycle through all the PIDs to find which one must be removed */
    for( uint8_t index = 0; index < dev->num_pids; index++ )
    {
        /* If found, pop that pointer reference */
        if( dev->stream[index] == pid )
        {
            if( dev->num_pids > 1 )
            {
                for( uint8_t i = index; i < dev->num_pids; i++ ) {
                    dev->stream[i] = dev->stream[i + 1];
                    dev->stream[i + 1] = NULL;
                }
            }

            /* Remove the first data point needed for the PID */
            dev->clear_pid( dev->data1[index] );

            /* Remove the second data point needed for the PID */
            dev->clear_pid( dev->data2[index] );

            /* Remove the PID */
            if( dev->num_pids > 0 )
                dev->num_pids--;

            /* Return a success */
            return VEHICLE_DATA_OK;
        }
    }

    /* Return a success */
    return VEHICLE_DATA_OK;
}

void Vehicle_service( PTR_VEHICLE_DATA_MANAGER dev )
{
    if( new_req )
    {
        PID_DATA req;
        for( uint8_t i = 0; i < dev->num_pids; i++)
        {
            switch( dev->stream[i]->mode )
            {
                case CALC1:
                    switch( dev->stream[i]->pid )
                    {
                        case CALC1_TURBOCHARGER_COMPRESSOR_INLET_PRESSURE:
                            req.mode     = MODE1;
                            req.pid      = MODE1_INTAKE_MANIFOLD_ABSOLUTE_PRESSURE;
                            req.pid_unit = MODE1_INTAKE_MANIFOLD_ABSOLUTE_PRESSURE_UNITS;

                            /* Add the PID request */
                            dev->data1[i] = dev->req_pid( &req );

                            req.mode     = MODE1;
                            req.pid      = MODE1_ABSOLUTE_BAROMETRIC_PRESSURE;
                            req.pid_unit = MODE1_ABSOLUTE_BAROMETRIC_PRESSURE_UNITS;

                            /* Add the PID request */
                            dev->data2[i] = dev->req_pid( &req );

                            /* Boost = MAP - Baro */
                            dev->formula[i].equation = VEHICLE_DATA_EQ_VAL1_MINUS_VAL2;
                            break;
                    }
                    break;

                default:
                    break;
            }
        }

        new_req = 0;
    }

    for( uint8_t i = 0; i < dev->num_pids; i++)
    {
        switch( dev->formula[i].equation )
        {
            case VEHICLE_DATA_EQ_VAL1_MINUS_VAL2:
                /* Calaculate the value: PID_Value = Val1 - Val2 */
                dev->stream[i]->pid_value = dev->data1[i]->pid_value - dev->data2[i]->pid_value;

                /* Only update the PID timestamp if both PIDs have been acquired. */
                if( (dev->data1[i]->timestamp > 0) & (dev->data2[i]->timestamp > 0) )
                    dev->stream[i]->timestamp = vehicle_tick;

                break;

            case VEHICLE_DATA_EQ_NOT_DEFINED:
            default:
                break;
        }
    }
}

void Vehicle_tick( void )
{
    vehicle_tick++;
}
