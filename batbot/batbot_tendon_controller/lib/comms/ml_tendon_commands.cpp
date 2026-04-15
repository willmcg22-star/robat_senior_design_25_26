#include "ml_tendon_commands.hpp"

CommandReturn_t ML_EchoCommand_execute(struct ML_EchoCommand * self)
{
    return CommandReturn_t{
        self->numParams,
        self->params
    };
}

CommandReturn_t ML_ReadStatusCommand_execute(struct ML_ReadStatusCommand * self)
{
    return CommandReturn_t{
        1,
        {0}
    };
}

CommandReturn_t ML_ReadAngleCommand_execute(struct ML_ReadAngleCommand * self)
{
    int16_t angle = (self->base.motor_ref->Get_Angle());
    
    CommandReturn_t ret;
    ret.numParams = 2;
    ret.params = new uint8_t[2];
    ret.params[0] = TENDON_CONTROL_GET_UPPER_8B(angle);
    ret.params[1] = TENDON_CONTROL_GET_LOWER_8B(angle);

    return ret;
}

CommandReturn_t ML_WriteAngleCommand_execute(struct ML_WriteAngleCommand * self)
{
    self->base.motor_ref->Set_Goal_Angle(self->angle);

    return CommandReturn_t{
        0,
        {}
    };
}

CommandReturn_t ML_WritePIDCommand_execute(struct ML_WritePIDCommand * self)
{
    self->base.motor_ref->Set_PID_Param(self->P, self->I, self->D, 6000);

    return CommandReturn_t {
        0,
        {}
    };
}

CommandReturn_t ML_SetZeroAngleCommand_execute(struct ML_SetZeroAngleCommand * self)
{
    self->base.motor_ref->Reset_Encoder_Zero();

    return CommandReturn_t {
        0,
        {}
    };
}

CommandReturn_t ML_SetMaxAngleCommand_execute(struct ML_SetMaxAngleCommand * self)
{
    self->base.motor_ref->Set_Max_Angle(self->angle);

    return CommandReturn_t {
        0,
        {}
    };
}

CommandReturn_t ML_DisableMotorCommand_execute(struct ML_SetMaxAngleCommand * self)
{
    // self->base.motor_ref->Set_Max_Angle(self->angle);f

    return CommandReturn_t {
        0,
        {}
    };
}

CommandReturn_t ML_EnableMotorCommand_execute(struct ML_SetMaxAngleCommand * self)
{
    // self->base.motor_ref->Set_Max_Angle(self->angle);f

    return CommandReturn_t {
        0,
        {}
    };
}

tendon_comm_result_t ML_EchoCommand_create(
    ML_TendonCommandBase** command,
    TendonControl_data_packet_s* dataPacket,
    TendonController* tendons
)
{
    *command = (ML_TendonCommandBase *)(new ML_EchoCommand);
    (*command)->fn = (CommandExecuteFn)ML_EchoCommand_execute;
    return COMM_SUCCESS;
}

tendon_comm_result_t ML_ReadStatusCommand_create(
    ML_TendonCommandBase** command,
    TendonControl_data_packet_s* dataPacket,
    TendonController* tendons
)
{
    uint8_t id = dataPacket->data_packet_u.data_packet_s.motorId;

    if (id >= 8) // TODO: Move the definition of the motor number somewhere thats visible to this file
    {
        return COMM_ID_ERROR;
    } else {
        *command = (ML_TendonCommandBase *)(new ML_ReadStatusCommand);
        (*command)->motor_ref = &tendons[id];
        (*command)->fn = (CommandExecuteFn)ML_ReadAngleCommand_execute;

        return COMM_SUCCESS;
    }
}

tendon_comm_result_t ML_ReadAngleCommand_create(
    ML_TendonCommandBase** command,
    TendonControl_data_packet_s* dataPacket,
    TendonController* tendons
)
{
    uint8_t id = dataPacket->data_packet_u.data_packet_s.motorId;

    if (id >= 8) // TODO: Move the definition of the motor number somewhere thats visible to this file
    {
        return COMM_ID_ERROR;
    } else {
        ML_ReadAngleCommand* read_command = new ML_ReadAngleCommand;
        read_command->base = {
            (CommandExecuteFn)ML_ReadAngleCommand_execute,
            &tendons[id]
        };

        *command = (ML_TendonCommandBase *)read_command;

        return COMM_SUCCESS;
    }
}

tendon_comm_result_t ML_WriteAngleCommand_create(
    ML_TendonCommandBase** command,
    TendonControl_data_packet_s* dataPacket,
    TendonController* tendons
)
{
    uint8_t id = dataPacket->data_packet_u.data_packet_s.motorId;
    size_t numParams = dataPacket->data_packet_u.data_packet_s.len - 4;

    if (id >= 8) // TODO: Move the definition of the motor number somewhere thats visible to this file
    {
        return COMM_ID_ERROR;
    } else if (numParams != 2) {
        return COMM_PARAM_ERROR;
    }
    else {
        ML_WriteAngleCommand* write_command = new ML_WriteAngleCommand;
        write_command->base = {
            (CommandExecuteFn)ML_WriteAngleCommand_execute,
            &tendons[id]
        };
        write_command->angle = TENDON_CONTROL_MAKE_16B_WORD(
            dataPacket->data_packet_u.data_packet_s.pkt_params[0],
            dataPacket->data_packet_u.data_packet_s.pkt_params[1]
        );

        *command = (ML_TendonCommandBase *)write_command;

        return COMM_SUCCESS;
    }
}

tendon_comm_result_t ML_WritePIDCommand_create(
    ML_TendonCommandBase** command,
    TendonControl_data_packet_s* dataPacket,
    TendonController* tendons
)
{
    uint8_t id = dataPacket->data_packet_u.data_packet_s.motorId;
    size_t numParams = dataPacket->data_packet_u.data_packet_s.len - 4;

    if (id >= 8) // TODO: Move the definition of the motor number somewhere thats visible to this file
    {
        return COMM_ID_ERROR;
    } else if (numParams != 12) {
        return COMM_PARAM_ERROR;
    }
    else {
        ML_WritePIDCommand* pid_command = new ML_WritePIDCommand;
        pid_command->base = {
            (CommandExecuteFn)ML_WritePIDCommand_execute,
            &tendons[id]
        };
    
        float kp, ki, kd;
        uint8_t kp_bytes[] = {
            dataPacket->data_packet_u.data_packet_s.pkt_params[3],
            dataPacket->data_packet_u.data_packet_s.pkt_params[2],
            dataPacket->data_packet_u.data_packet_s.pkt_params[1],
            dataPacket->data_packet_u.data_packet_s.pkt_params[0],
        };
        memcpy(&kp, kp_bytes, 4);

        uint8_t ki_bytes[] = {
            dataPacket->data_packet_u.data_packet_s.pkt_params[7],
            dataPacket->data_packet_u.data_packet_s.pkt_params[6],
            dataPacket->data_packet_u.data_packet_s.pkt_params[5],
            dataPacket->data_packet_u.data_packet_s.pkt_params[4],
        };
        memcpy(&ki, ki_bytes, 4);

        uint8_t kd_bytes[] = {
            dataPacket->data_packet_u.data_packet_s.pkt_params[11],
            dataPacket->data_packet_u.data_packet_s.pkt_params[10],
            dataPacket->data_packet_u.data_packet_s.pkt_params[9],
            dataPacket->data_packet_u.data_packet_s.pkt_params[8],
        };
        memcpy(&kd, kd_bytes, 4);

        pid_command->P = kp;
        pid_command->I = ki;
        pid_command->D = kd;
        
        *command = (ML_TendonCommandBase *)pid_command;

        return COMM_SUCCESS;
    }
}

tendon_comm_result_t ML_SetZeroAngleCommand_create(
    ML_TendonCommandBase** command,
    TendonControl_data_packet_s* dataPacket,
    TendonController* tendons
)
{
    uint8_t id = dataPacket->data_packet_u.data_packet_s.motorId;

    if (id >= 8) // TODO: Move the definition of the motor number somewhere thats visible to this file
    {
        return COMM_ID_ERROR;
    } else {

        ML_SetZeroAngleCommand* zero_command = new ML_SetZeroAngleCommand;
        zero_command->base = {
            (CommandExecuteFn)ML_SetZeroAngleCommand_execute,
            &tendons[id]
        };

        *command = (ML_TendonCommandBase *)zero_command;

        return COMM_SUCCESS;
    }
}

tendon_comm_result_t ML_SetMaxAngleCommand_create(
    ML_TendonCommandBase** command,
    TendonControl_data_packet_s* dataPacket,
    TendonController* tendons
)
{
    uint8_t id = dataPacket->data_packet_u.data_packet_s.motorId;
    size_t numParams = dataPacket->data_packet_u.data_packet_s.len - 4;

    if (id >= 8) // TODO: Move the definition of the motor number somewhere thats visible to this file
    {
        return COMM_ID_ERROR;
    } else if (numParams != 2) {
        return COMM_PARAM_ERROR;
    } else {
        ML_SetMaxAngleCommand* max_command = new ML_SetMaxAngleCommand;
        max_command->base = {
            (CommandExecuteFn)ML_SetMaxAngleCommand_execute,
            &tendons[id]
        };
        uint8_t angle_h = dataPacket->data_packet_u.data_packet_s.pkt_params[0];
        uint8_t angle_l = dataPacket->data_packet_u.data_packet_s.pkt_params[1];
        
        max_command->angle = (int)TENDON_CONTROL_MAKE_16B_WORD(angle_h, angle_l);

        *command = (ML_TendonCommandBase *)max_command;

        return COMM_SUCCESS;
    }   
}

tendon_comm_result_t ML_DisableMotorCommand_create(
    ML_TendonCommandBase** command,
    TendonControl_data_packet_s* dataPacket,
    TendonController* tendons
)
{
    uint8_t id = dataPacket->data_packet_u.data_packet_s.motorId;
    // size_t numParams = dataPacket->data_packet_u.data_packet_s.len - 4;

    if (id >= 8) // TODO: Move the definition of the motor number somewhere thats visible to this file
    {
        return COMM_ID_ERROR;
    } else {
        ML_DisableMotorCommand* max_command = new ML_DisableMotorCommand;
        max_command->base = {
            (CommandExecuteFn)ML_DisableMotorCommand_execute,
            &tendons[id]
        };

        *command = (ML_TendonCommandBase *)max_command;

        return COMM_SUCCESS;
    }   
}

tendon_comm_result_t ML_EnableMotorCommand_create(
    ML_TendonCommandBase** command,
    TendonControl_data_packet_s* dataPacket,
    TendonController* tendons
)
{
    uint8_t id = dataPacket->data_packet_u.data_packet_s.motorId;
    // size_t numParams = dataPacket->data_packet_u.data_packet_s.len - 4;

    if (id >= 8) // TODO: Move the definition of the motor number somewhere thats visible to this file
    {
        return COMM_ID_ERROR;
    } else {
        ML_EnableMotorCommand* max_command = new ML_EnableMotorCommand;
        max_command->base = {
            (CommandExecuteFn)ML_EnableMotorCommand_execute,
            &tendons[id]
        };

        *command = (ML_TendonCommandBase *)max_command;

        return COMM_SUCCESS;
    }   
}

tendon_comm_result_t CommandFactory_CreateCommand(
    ML_TendonCommandBase** command,
    TendonControl_data_packet_s* dataPacket,
    TendonController* tendons
)
{
    tendon_comm_result_t result = COMM_SUCCESS;

    switch (dataPacket->data_packet_u.data_packet_s.opcode)
    {
        case ECHO:
            result = ML_EchoCommand_create(command, dataPacket, tendons);
            break;
        case READ_STATUS:
            result = ML_ReadStatusCommand_create(command, dataPacket, tendons);
            break;
        case READ_ANGLE:
            result =  ML_ReadAngleCommand_create(command, dataPacket, tendons);
            break;
        case WRITE_ANGLE:
            result = ML_WriteAngleCommand_create(command, dataPacket, tendons);
            break;
        case WRITE_PID:
            result =  ML_WritePIDCommand_create(command, dataPacket, tendons);
            break;
        case SET_ZERO_ANGLE:
            result =  ML_SetZeroAngleCommand_create(command, dataPacket, tendons);
            break;
        case SET_MAX_ANGLE:
            result = ML_SetMaxAngleCommand_create(command, dataPacket, tendons);
            break;
        case DISABLE_MOTOR:
            result = ML_DisableMotorCommand_create(command, dataPacket, tendons);
            break;
        case ENABLE_MOTOR:
            result = ML_DisableMotorCommand_create(command, dataPacket, tendons);
            break;
        default:
            result = COMM_INSTRUCTION_ERROR;
    }

    return result;
}