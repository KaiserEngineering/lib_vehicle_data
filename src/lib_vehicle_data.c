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

    dev->flag = 0;

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
                case CALC1_CRUISE_CONTROL_OFF_BUTTON_TOGGLE:

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
            } else {
                dev->stream[index] = NULL;
            }

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

                            /* The Boost/Vacuum base unit is kPa */
                            dev->stream[i]->base_unit = PID_UNITS_KPA;

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

                        case CALC1_CRUISE_CONTROL_OFF_BUTTON_TOGGLE:

                            /* The Cruise Control OFF button base unit is None */
                            dev->stream[i]->base_unit = PID_UNITS_NONE;

                            req.mode     = SNIFF;
                            req.pid      = SNIFF_CRUISE_CONTROL_OFF_BUTTON;
                            req.pid_unit = SNIFF_CRUISE_CONTROL_OFF_BUTTON_UNITS;

                            /* Add the PID request */
                            dev->data1[i] = dev->req_pid( &req );

                            /* Only 1 data point is needed */
                            dev->data2[i] = NULL;

                            /* Cruise Control Toggle = Toggle when True */
                            dev->formula[i].equation = VEHICLE_DATA_EQ_TOGGLE_ON_TRUE;
                            break;
                    }
                    break;

                default:
                    /*
                     * Check to see if the stream has been NULL'd. If so the data points have to be
                     * cleared too.
                     */
                    if( (dev->stream[i] == NULL) & ((dev->data1[i] != NULL) | (dev->data1[i] != NULL)) )
                    {
                        /* Remove the first data point needed for the PID */
                        if( dev->data1[i] != NULL )
                            dev->clear_pid( dev->data1[i] );

                        /* Remove the second data point needed for the PID */
                        if( dev->data2[i] != NULL )
                            dev->clear_pid( dev->data2[i] );

                        /* Remove the PID */
                        if( dev->num_pids > 0 )
                            dev->num_pids--;
                    }
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
                /* Calculate the value: PID_Value = Val1 - Val2 */
                dev->stream[i]->pid_value = dev->data1[i]->pid_value - dev->data2[i]->pid_value;

                /* Only update the PID timestamp if both PIDs have been acquired. */
                if( (dev->data1[i]->timestamp > 0) & (dev->data2[i]->timestamp > 0) )
                    dev->stream[i]->timestamp = vehicle_tick;

                break;

            case VEHICLE_DATA_EQ_TOGGLE_ON_TRUE:
                /*
                 * Check if the PID value is true, if so, see if this is a state change or
                 * if the button is still being held.
                 */
                if( dev->data1[i]->pid_value ) {
                    /*
                     * flag is used to indicate when the state change has been seen.
                     * A 0 means the value has yet to be toggled, a 1 means the button is still
                     * being held and the value has already been toggled. Wait until the value
                     * returns to FALSE.
                     */
                    if( dev->flag == 0 )
                        dev->stream[i]->pid_value = ( dev->stream[i]->pid_value == 0 ) ? 1 : 0;

                    /* Log that the state is still TRUE */
                    dev->flag = 1;
                } else{
                    /* Log that the state is FALSE */
                    dev->flag = 0;
                }

                /* Only 1 timestamp needs to be checked.. */
                if( (dev->data1[i]->timestamp > 0) )
                    dev->stream[i]->timestamp = vehicle_tick;

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
