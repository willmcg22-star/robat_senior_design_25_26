#ifndef ML_TENDON_COMMANDS_H
#define ML_TENDON_COMMANDS_H

#include "ml_tendon_comm_protocol.hpp"
#include <TendonMotor.h>
    
/**
 * @brief Forward declaration of ML_TendonCommandBase
 */
struct ML_TendonCommandBase;

/**
 * @brief A struct containing the output result of executing a command
 */
typedef struct CommandReturn_t {
    size_t numParams;
    uint8_t* params;
} CommandReturn_t;

/**
 * @brief A typedef defining the standard format of tendon command functions
 */
typedef CommandReturn_t (*CommandExecuteFn)(struct ML_TendonCommandBase * self);

/**
 * @brief A base struct containing a motor command.
 * 
 * For info on the specific functionality of each command, see the documentation for 
 * ml_tendon_comm_protocol.hpp
 */
typedef struct ML_TendonCommandBase {
    CommandExecuteFn fn;            // The execution function
    TendonController* motor_ref;    // A reference to the motor(s) to act on
} ML_TendonCommandBase;

typedef struct ML_EchoCommand {
    ML_TendonCommandBase base;
    size_t numParams;
    uint8_t* params;
} ML_EchoCommand;

typedef struct ML_ReadStatusCommand {
    ML_TendonCommandBase base;
} ML_ReadStatusCommand;

typedef struct ML_ReadAngleCommand {
    ML_TendonCommandBase base;
} ML_ReadAngleCommand;

typedef struct ML_WriteAngleCommand {
    ML_TendonCommandBase base;
    int angle;
} ML_WriteAngleCommand;

typedef struct ML_WritePIDCommand {
    ML_TendonCommandBase base;
    float P;
    float I;
    float D;
} ML_WritePIDCommand;

typedef struct ML_SetZeroAngleCommand {
    ML_TendonCommandBase base;
} ML_SetZeroAngleCommand;

typedef struct ML_SetMaxAngleCommand {
    ML_TendonCommandBase base;
    int angle;
} ML_SetMaxAngleCommand;

typedef struct ML_DisableMotorCommand {
    ML_TendonCommandBase base;
} ML_DisableMotorCommand;

typedef struct ML_EnableMotorCommand {
    ML_TendonCommandBase base;
} ML_EnableMotorCommand;

/**
 * @brief A factory function for creating commands given a data packet
 * 
 * Returns the command via the `command` output parameter and returns a status
 * code indicating the success of command creation. The following are the possible
 * codes that can be 
 * 
 * COMM_SUCCESS: Command was successfully created
 * COMM_INSTRUCTION_ERROR: The given opcode doesn't correspond to any defined instruction
 * COMM_PARAM_ERROR: An number of supplied parameters doesn't match the required parameters for the instruction
 * 
 * @param command An output parameter for the command created
 * @param dataPacket The packet to be processed
 * @param tendons A reference to the tendons used by the application
 * @return A numerical status code indicating the success/failure of command creation
 */
tendon_comm_result_t CommandFactory_CreateCommand(
    ML_TendonCommandBase** command,
    TendonControl_data_packet_s* dataPacket,
    TendonController* tendons
);

CommandReturn_t ML_EchoCommand_execute(struct ML_EchoCommand * self);
CommandReturn_t ML_ReadStatusCommand_execute(struct ML_ReadStatusCommand * self);
CommandReturn_t ML_ReadAngleCommand_execute(struct ML_ReadAngleCommand * self);
CommandReturn_t ML_WriteAngleCommand_execute(struct ML_WriteAngleCommand * self);
CommandReturn_t ML_WritePIDCommand_execute(struct ML_WritePIDCommand * self);
CommandReturn_t ML_SetZeroAngleCommand_execute(struct ML_SetZeroAngleCommand * self);
CommandReturn_t ML_SetMaxAngleCommand_execute(struct ML_SetMaxAngleCommand * self);
CommandReturn_t ML_DisableMotorCommand_execute(struct ML_SetMaxAngleCommand * self);


#endif