#include <Arduino.h>
#include "ml_tendon_commands.hpp"
#include "ml_tendon_comm_protocol.hpp"
#include "unity.h"

TendonController tendons[8] = {
    TendonController("motor 1"),
    TendonController("motor 2"),
    TendonController("motor 3"),
    TendonController("motor 4"),
    TendonController("motor 5"),
    TendonController("motor 6"),
    TendonController("motor 7"),
    TendonController("motor 8")};

void test_packet_validation(void)
{
    // Scenario 1: Test with wrong CRC
    TendonControl_data_packet_s pkt;
    pkt.data_packet_u.data_packet_s.header[0] = 0xFF;
    pkt.data_packet_u.data_packet_s.header[1] = 0x00;
    pkt.data_packet_u.data_packet_s.motorId = 0;
    pkt.data_packet_u.data_packet_s.opcode = 0;
    pkt.data_packet_u.data_packet_s.len = 4;
    pkt.data_packet_u.data_packet_s.pkt_params[0] = 0x00;
    pkt.data_packet_u.data_packet_s.pkt_params[1] = 0x00;

    tendon_comm_result_t result = validatePacket(&pkt);

    TEST_ASSERT_EQUAL(COMM_CRC_ERROR, result);

    // Scenario 2: Test with right CRC
    uint16_t crc = updateCRC(0, pkt.data_packet_u.data_packet, 4 + 3 - TENDON_CONTROL_PKT_NUM_CRC_BYTES);
    pkt.data_packet_u.data_packet_s.pkt_params[0] = TENDON_CONTROL_GET_UPPER_8B(crc);
    pkt.data_packet_u.data_packet_s.pkt_params[1] = TENDON_CONTROL_GET_LOWER_8B(crc);

    result = validatePacket(&pkt);
    TEST_ASSERT_EQUAL(COMM_SUCCESS, result);
}

void test_command_factory1(void)
{
    TendonControl_data_packet_s pkt;
    
    // Scenario 1: Test an invalid opcode
    pkt.data_packet_u.data_packet_s.header[0] = 0xFF;
    pkt.data_packet_u.data_packet_s.header[1] = 0x00;
    pkt.data_packet_u.data_packet_s.motorId = 0;
    pkt.data_packet_u.data_packet_s.opcode = 7;
    pkt.data_packet_u.data_packet_s.len = 4;
    pkt.data_packet_u.data_packet_s.pkt_params[0] = 0x00;
    pkt.data_packet_u.data_packet_s.pkt_params[1] = 0x00;

    ML_TendonCommandBase *cmd = NULL;
    tendon_comm_result_t result = CommandFactory_CreateCommand(
        &cmd,
        &pkt,
        tendons
    );

    TEST_ASSERT_EQUAL(COMM_INSTRUCTION_ERROR, result);
    TEST_ASSERT_EQUAL(NULL, cmd);

    free(cmd);
}

void test_command_factory2(void)
{
    TendonControl_data_packet_s pkt;
    tendons[0].Set_Max_Angle(360);
    tendons[0].Reset_Encoder_Zero();
    tendons[0].Set_PID_Param(1, 0, 0, 5000);
    
    // Scenario 1: Test a valid write motor command packet
    pkt.data_packet_u.data_packet_s.header[0] = 0xFF;
    pkt.data_packet_u.data_packet_s.header[1] = 0x00;
    pkt.data_packet_u.data_packet_s.motorId = 0;
    pkt.data_packet_u.data_packet_s.opcode = WRITE_ANGLE;
    pkt.data_packet_u.data_packet_s.len = 6;
    pkt.data_packet_u.data_packet_s.pkt_params[0] = TENDON_CONTROL_GET_UPPER_8B(180);
    pkt.data_packet_u.data_packet_s.pkt_params[1] = TENDON_CONTROL_GET_LOWER_8B(180);

    ML_TendonCommandBase *cmd = NULL;
    tendon_comm_result_t result = CommandFactory_CreateCommand(
        &cmd,
        &pkt,
        tendons
    );

    TEST_ASSERT_EQUAL(COMM_SUCCESS, result);
    TEST_ASSERT_NOT_NULL(cmd);
    TEST_ASSERT_EQUAL(ML_WriteAngleCommand_execute, ((ML_WriteAngleCommand*)cmd)->base.fn);
    TEST_ASSERT_EQUAL(&tendons[0], ((ML_WriteAngleCommand*)cmd)->base.motor_ref);
    TEST_ASSERT_EQUAL(180, ((ML_WriteAngleCommand*)cmd)->angle);

    // Scenario 2: Assert that the motors goal angle has been updated to the set goal angle
    CommandReturn_t cmd_ret = cmd->fn(cmd);
    TEST_ASSERT_EQUAL(0, cmd_ret.numParams);
    TEST_ASSERT_FLOAT_WITHIN(1, 180, tendons[0].Get_Goal_Angle());
    
    // Scenario 3: Try to exceed max angle
    pkt.data_packet_u.data_packet_s.pkt_params[0] = TENDON_CONTROL_GET_UPPER_8B(370);
    pkt.data_packet_u.data_packet_s.pkt_params[1] = TENDON_CONTROL_GET_LOWER_8B(370);
    result = CommandFactory_CreateCommand(
        &cmd,
        &pkt,
        tendons
    );
    TEST_ASSERT_EQUAL(COMM_SUCCESS, result);
    TEST_ASSERT_NOT_NULL(cmd);
    TEST_ASSERT_EQUAL(ML_WriteAngleCommand_execute, ((ML_WriteAngleCommand*)cmd)->base.fn);
    TEST_ASSERT_EQUAL(&tendons[0], ((ML_WriteAngleCommand*)cmd)->base.motor_ref);
    TEST_ASSERT_EQUAL(370, ((ML_WriteAngleCommand*)cmd)->angle);
    cmd_ret = cmd->fn(cmd);
    TEST_ASSERT_EQUAL(0, cmd_ret.numParams);
    TEST_ASSERT_FLOAT_WITHIN(1, 360, tendons[0].Get_Goal_Angle());

    // Scenario 4: Negative angle
    pkt.data_packet_u.data_packet_s.pkt_params[0] = TENDON_CONTROL_GET_UPPER_8B((int16_t)-10);
    pkt.data_packet_u.data_packet_s.pkt_params[1] = TENDON_CONTROL_GET_LOWER_8B((int16_t)-10);
    TEST_ASSERT_EQUAL(pkt.data_packet_u.data_packet_s.pkt_params[0], 0xFF);
    TEST_ASSERT_EQUAL(pkt.data_packet_u.data_packet_s.pkt_params[1], 0xF6);
    result = CommandFactory_CreateCommand(
        &cmd,
        &pkt,
        tendons
    );
    TEST_ASSERT_EQUAL(COMM_SUCCESS, result);
    TEST_ASSERT_NOT_NULL(cmd);
    TEST_ASSERT_EQUAL(ML_WriteAngleCommand_execute, ((ML_WriteAngleCommand*)cmd)->base.fn);
    TEST_ASSERT_EQUAL(&tendons[0], ((ML_WriteAngleCommand*)cmd)->base.motor_ref);
    TEST_ASSERT_EQUAL(-10, ((ML_WriteAngleCommand*)cmd)->angle);
    cmd_ret = cmd->fn(cmd);
    TEST_ASSERT_EQUAL(0, cmd_ret.numParams);
    TEST_ASSERT_FLOAT_WITHIN(1, -10, tendons[0].Get_Goal_Angle());

    // Scenario 5: Test angle larger than 8 bits
    pkt.data_packet_u.data_packet_s.pkt_params[0] = TENDON_CONTROL_GET_UPPER_8B(256);
    pkt.data_packet_u.data_packet_s.pkt_params[1] = TENDON_CONTROL_GET_LOWER_8B(256);
    TEST_ASSERT_EQUAL(pkt.data_packet_u.data_packet_s.pkt_params[0], 0x01);
    TEST_ASSERT_EQUAL(pkt.data_packet_u.data_packet_s.pkt_params[1], 0x00);
    result = CommandFactory_CreateCommand(
        &cmd,
        &pkt,
        tendons
    );
    TEST_ASSERT_EQUAL(COMM_SUCCESS, result);
    TEST_ASSERT_NOT_NULL(cmd);
    TEST_ASSERT_EQUAL(ML_WriteAngleCommand_execute, ((ML_WriteAngleCommand*)cmd)->base.fn);
    TEST_ASSERT_EQUAL(&tendons[0], ((ML_WriteAngleCommand*)cmd)->base.motor_ref);
    TEST_ASSERT_EQUAL(256, ((ML_WriteAngleCommand*)cmd)->angle);
    cmd_ret = cmd->fn(cmd);
    TEST_ASSERT_EQUAL(0, cmd_ret.numParams);
    TEST_ASSERT_FLOAT_WITHIN(1, 256, tendons[0].Get_Goal_Angle());

    free(cmd);
}

void test_command_factory3(void)
{
    // Scenario 1: Test a valid set max angle command packet
    TendonControl_data_packet_s pkt;
    
    pkt.data_packet_u.data_packet_s.header[0] = 0xFF;
    pkt.data_packet_u.data_packet_s.header[1] = 0x00;
    pkt.data_packet_u.data_packet_s.motorId = 0;
    pkt.data_packet_u.data_packet_s.opcode = SET_ZERO_ANGLE;
    pkt.data_packet_u.data_packet_s.len = 6;
    pkt.data_packet_u.data_packet_s.pkt_params[0] = TENDON_CONTROL_GET_UPPER_8B(100);
    pkt.data_packet_u.data_packet_s.pkt_params[1] = TENDON_CONTROL_GET_LOWER_8B(100);

    ML_TendonCommandBase *cmd = NULL;
    tendon_comm_result_t result = CommandFactory_CreateCommand(
        &cmd,
        &pkt,
        tendons
    );
    TEST_ASSERT_EQUAL(COMM_SUCCESS, result);
    TEST_ASSERT_NOT_NULL(cmd);
    TEST_ASSERT_EQUAL(ML_SetZeroAngleCommand_execute, ((ML_SetZeroAngleCommand*)cmd)->base.fn);
    TEST_ASSERT_EQUAL(&tendons[0], ((ML_SetZeroAngleCommand*)cmd)->base.motor_ref);

    // Scenario 2: Assert that the motors zero angle has been reset
    CommandReturn_t cmd_ret = cmd->fn(cmd);
    TEST_ASSERT_EQUAL(0, cmd_ret.numParams);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0, tendons[0].Get_Angle());
    TEST_ASSERT_FLOAT_WITHIN(0.01, 0, tendons[0].Get_Goal_Angle());

    free(cmd);
}

void test_command_factory4(void)
{
    // Scenario 1: Test a valid set zero angle command packet
    TendonControl_data_packet_s pkt;
    
    pkt.data_packet_u.data_packet_s.header[0] = 0xFF;
    pkt.data_packet_u.data_packet_s.header[1] = 0x00;
    pkt.data_packet_u.data_packet_s.motorId = 0;
    pkt.data_packet_u.data_packet_s.opcode = SET_MAX_ANGLE;
    pkt.data_packet_u.data_packet_s.len = 6;
    pkt.data_packet_u.data_packet_s.pkt_params[0] = TENDON_CONTROL_GET_UPPER_8B(100);
    pkt.data_packet_u.data_packet_s.pkt_params[1] = TENDON_CONTROL_GET_LOWER_8B(100);
    

    ML_TendonCommandBase *cmd = NULL;
    tendon_comm_result_t result = CommandFactory_CreateCommand(
        &cmd,
        &pkt,
        tendons
    );

    TEST_ASSERT_EQUAL(COMM_SUCCESS, result);
    TEST_ASSERT_NOT_NULL(cmd);
    TEST_ASSERT_EQUAL(ML_SetMaxAngleCommand_execute, ((ML_SetMaxAngleCommand*)cmd)->base.fn);
    TEST_ASSERT_EQUAL(&tendons[0], ((ML_SetMaxAngleCommand*)cmd)->base.motor_ref);

    // Scenario 2: Assert that the max angle has been set
    CommandReturn_t cmd_ret = cmd->fn(cmd);
    TEST_ASSERT_EQUAL(0, cmd_ret.numParams);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 100, tendons[0].Get_Max_Angle());

    free(cmd);
}

void test_command_factory5(void)
{
    tendons[0].Reset_Encoder_Zero();
    
    // Scenario 1: Test a valid read angle command packet
    TendonControl_data_packet_s pkt;
    
    pkt.data_packet_u.data_packet_s.header[0] = 0xFF;
    pkt.data_packet_u.data_packet_s.header[1] = 0x00;
    pkt.data_packet_u.data_packet_s.motorId = 0;
    pkt.data_packet_u.data_packet_s.opcode = READ_ANGLE;
    pkt.data_packet_u.data_packet_s.len = 4;

    ML_TendonCommandBase *cmd = NULL;
    tendon_comm_result_t result = CommandFactory_CreateCommand(
        &cmd,
        &pkt,
        tendons
    );

    TEST_ASSERT_EQUAL(COMM_SUCCESS, result);
    TEST_ASSERT_NOT_NULL(cmd);
    TEST_ASSERT_EQUAL(ML_ReadAngleCommand_execute, ((ML_ReadAngleCommand*)cmd)->base.fn);
    TEST_ASSERT_EQUAL(&tendons[0], ((ML_ReadAngleCommand*)cmd)->base.motor_ref);

    // Scenario 2: Assert that the correct angle has been read
    CommandReturn_t cmd_ret = cmd->fn(cmd);

    TEST_ASSERT_EQUAL(2, cmd_ret.numParams);
    TEST_ASSERT_EQUAL(0, TENDON_CONTROL_MAKE_16B_WORD(cmd_ret.params[0], cmd_ret.params[1]));

    // Scenario 3: Test with negative angle
    tendons[0].Set_Angle(-90);
    cmd_ret = cmd->fn(cmd);

    TEST_ASSERT_EQUAL(2, cmd_ret.numParams);
    TEST_ASSERT_INT_WITHIN(2, -90, TENDON_CONTROL_MAKE_16B_WORD(cmd_ret.params[0], cmd_ret.params[1]));

    free(cmd);
}

void test_command_factory6(void)
{
    tendons[0].Reset_Encoder_Zero();
    
    // Scenario 1: Test a valid set PID angle command packet
    TendonControl_data_packet_s pkt;
    
    pkt.data_packet_u.data_packet_s.header[0] = 0xFF;
    pkt.data_packet_u.data_packet_s.header[1] = 0x00;
    pkt.data_packet_u.data_packet_s.motorId = 2;
    pkt.data_packet_u.data_packet_s.opcode = WRITE_PID;
    pkt.data_packet_u.data_packet_s.len = 16;
    pkt.data_packet_u.data_packet_s.pkt_params[0] = 0;
    pkt.data_packet_u.data_packet_s.pkt_params[1] = 0;
    pkt.data_packet_u.data_packet_s.pkt_params[2] = 0;
    pkt.data_packet_u.data_packet_s.pkt_params[3] = 0;
    pkt.data_packet_u.data_packet_s.pkt_params[4] = 0;
    pkt.data_packet_u.data_packet_s.pkt_params[5] = 0;
    pkt.data_packet_u.data_packet_s.pkt_params[6] = 0;
    pkt.data_packet_u.data_packet_s.pkt_params[7] = 0;
    pkt.data_packet_u.data_packet_s.pkt_params[8] = 0;
    pkt.data_packet_u.data_packet_s.pkt_params[9] = 0;
    pkt.data_packet_u.data_packet_s.pkt_params[10] = 0;
    pkt.data_packet_u.data_packet_s.pkt_params[11] = 0;

    ML_TendonCommandBase *cmd = NULL;
    tendon_comm_result_t result = CommandFactory_CreateCommand(
        &cmd,
        &pkt,
        tendons
    );

    TEST_ASSERT_EQUAL(COMM_SUCCESS, result);
    TEST_ASSERT_NOT_NULL(cmd);
    TEST_ASSERT_EQUAL(ML_WritePIDCommand_execute, ((ML_WritePIDCommand*)cmd)->base.fn);
    TEST_ASSERT_EQUAL(&tendons[2], ((ML_WritePIDCommand*)cmd)->base.motor_ref);

    // Scenario 2: Assert that the PID gains were correctly set
    CommandReturn_t cmd_ret = cmd->fn(cmd);

    TEST_ASSERT_EQUAL(0, cmd_ret.numParams);
    float kp, ki, kd;
    tendons[2].Get_PID(kp, ki, kd);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0, kp);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0, ki);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0, kd);

    // Scenario 3: Test setting P with non-zero gains
    pkt.data_packet_u.data_packet_s.pkt_params[0] = 0x41;
    pkt.data_packet_u.data_packet_s.pkt_params[1] = 0x20;
    pkt.data_packet_u.data_packet_s.pkt_params[2] = 0x00;
    pkt.data_packet_u.data_packet_s.pkt_params[3] = 0x00;
    result = CommandFactory_CreateCommand(
        &cmd,
        &pkt,
        tendons
    );

    TEST_ASSERT_EQUAL(COMM_SUCCESS, result);
    TEST_ASSERT_NOT_NULL(cmd);
    TEST_ASSERT_EQUAL(ML_WritePIDCommand_execute, ((ML_WritePIDCommand*)cmd)->base.fn);
    TEST_ASSERT_EQUAL(&tendons[2], ((ML_WritePIDCommand*)cmd)->base.motor_ref);

    cmd_ret = cmd->fn(cmd);

    TEST_ASSERT_EQUAL(0, cmd_ret.numParams);
    tendons[2].Get_PID(kp, ki, kd);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 10, kp);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0, ki);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0, kd);

    // Scenario 4: Test setting P with negative gains
    pkt.data_packet_u.data_packet_s.pkt_params[0] = 0xc3;
    pkt.data_packet_u.data_packet_s.pkt_params[1] = 0x48;
    pkt.data_packet_u.data_packet_s.pkt_params[2] = 0x00;
    pkt.data_packet_u.data_packet_s.pkt_params[3] = 0x00;
    result = CommandFactory_CreateCommand(
        &cmd,
        &pkt,
        tendons
    );

    TEST_ASSERT_EQUAL(COMM_SUCCESS, result);
    TEST_ASSERT_NOT_NULL(cmd);
    TEST_ASSERT_EQUAL(ML_WritePIDCommand_execute, ((ML_WritePIDCommand*)cmd)->base.fn);
    TEST_ASSERT_EQUAL(&tendons[2], ((ML_WritePIDCommand*)cmd)->base.motor_ref);

    cmd_ret = cmd->fn(cmd);

    TEST_ASSERT_EQUAL(0, cmd_ret.numParams);
    tendons[2].Get_PID(kp, ki, kd);
    TEST_ASSERT_FLOAT_WITHIN(0.1, -200, kp);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0, ki);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0, kd);

    // Scenario 5: Test setting I with non-zero gains
    pkt.data_packet_u.data_packet_s.pkt_params[0] = 0;
    pkt.data_packet_u.data_packet_s.pkt_params[1] = 0;
    pkt.data_packet_u.data_packet_s.pkt_params[2] = 0;
    pkt.data_packet_u.data_packet_s.pkt_params[3] = 0;
    pkt.data_packet_u.data_packet_s.pkt_params[4] = 0x43;
    pkt.data_packet_u.data_packet_s.pkt_params[5] = 0x0b;
    pkt.data_packet_u.data_packet_s.pkt_params[6] = 0x40;
    pkt.data_packet_u.data_packet_s.pkt_params[7] = 0x00;
    result = CommandFactory_CreateCommand(
        &cmd,
        &pkt,
        tendons
    );

    TEST_ASSERT_EQUAL(COMM_SUCCESS, result);
    TEST_ASSERT_NOT_NULL(cmd);
    TEST_ASSERT_EQUAL(ML_WritePIDCommand_execute, ((ML_WritePIDCommand*)cmd)->base.fn);
    TEST_ASSERT_EQUAL(&tendons[2], ((ML_WritePIDCommand*)cmd)->base.motor_ref);

    cmd_ret = cmd->fn(cmd);

    TEST_ASSERT_EQUAL(0, cmd_ret.numParams);
    tendons[2].Get_PID(kp, ki, kd);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0, kp);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 139.25, ki);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0, kd);

    // Scenario 6: Test setting I with negative gains
    pkt.data_packet_u.data_packet_s.pkt_params[4] = 0xC3;
    pkt.data_packet_u.data_packet_s.pkt_params[5] = 0xE5;
    pkt.data_packet_u.data_packet_s.pkt_params[6] = 0xDA;
    pkt.data_packet_u.data_packet_s.pkt_params[7] = 0xE1;
    result = CommandFactory_CreateCommand(
        &cmd,
        &pkt,
        tendons
    );

    TEST_ASSERT_EQUAL(COMM_SUCCESS, result);
    TEST_ASSERT_NOT_NULL(cmd);
    TEST_ASSERT_EQUAL(ML_WritePIDCommand_execute, ((ML_WritePIDCommand*)cmd)->base.fn);
    TEST_ASSERT_EQUAL(&tendons[2], ((ML_WritePIDCommand*)cmd)->base.motor_ref);

    cmd_ret = cmd->fn(cmd);

    TEST_ASSERT_EQUAL(0, cmd_ret.numParams);
    tendons[2].Get_PID(kp, ki, kd);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0, kp);
    TEST_ASSERT_FLOAT_WITHIN(0.1, -459.71, ki);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0, kd);

    // Scenario 7: Test setting D with non-zero gains
    pkt.data_packet_u.data_packet_s.pkt_params[4] = 0;
    pkt.data_packet_u.data_packet_s.pkt_params[5] = 0;
    pkt.data_packet_u.data_packet_s.pkt_params[6] = 0;
    pkt.data_packet_u.data_packet_s.pkt_params[7] = 0;
    pkt.data_packet_u.data_packet_s.pkt_params[8] = 0x42;
    pkt.data_packet_u.data_packet_s.pkt_params[9] = 0x6C;
    pkt.data_packet_u.data_packet_s.pkt_params[10] = 0xB1;
    pkt.data_packet_u.data_packet_s.pkt_params[11] = 0x27;
    result = CommandFactory_CreateCommand(
        &cmd,
        &pkt,
        tendons
    );

    TEST_ASSERT_EQUAL(COMM_SUCCESS, result);
    TEST_ASSERT_NOT_NULL(cmd);
    TEST_ASSERT_EQUAL(ML_WritePIDCommand_execute, ((ML_WritePIDCommand*)cmd)->base.fn);
    TEST_ASSERT_EQUAL(&tendons[2], ((ML_WritePIDCommand*)cmd)->base.motor_ref);

    cmd_ret = cmd->fn(cmd);

    TEST_ASSERT_EQUAL(0, cmd_ret.numParams);
    tendons[2].Get_PID(kp, ki, kd);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0, kp);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0, ki);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 59.173, kd);

    // Scenario 8: Test setting D with negative gains
    pkt.data_packet_u.data_packet_s.pkt_params[8] = 0xC4;
    pkt.data_packet_u.data_packet_s.pkt_params[9] = 0x7A;
    pkt.data_packet_u.data_packet_s.pkt_params[10] = 0x00;
    pkt.data_packet_u.data_packet_s.pkt_params[11] = 0xA4;
    result = CommandFactory_CreateCommand(
        &cmd,
        &pkt,
        tendons
    );

    TEST_ASSERT_EQUAL(COMM_SUCCESS, result);
    TEST_ASSERT_NOT_NULL(cmd);
    TEST_ASSERT_EQUAL(ML_WritePIDCommand_execute, ((ML_WritePIDCommand*)cmd)->base.fn);
    TEST_ASSERT_EQUAL(&tendons[2], ((ML_WritePIDCommand*)cmd)->base.motor_ref);

    cmd_ret = cmd->fn(cmd);

    TEST_ASSERT_EQUAL(0, cmd_ret.numParams);
    tendons[2].Get_PID(kp, ki, kd);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0, kp);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 0, ki);
    TEST_ASSERT_FLOAT_WITHIN(0.1, -1000.01, kd);

    free(cmd);
}

void test_build_response_packet1(void)
{
    
}

void test_packet_handling(void)
{
    // Scenario 1: Test packet handling without CRC
    TendonControl_data_packet_s pkt;
    
    pkt.data_packet_u.data_packet_s.header[0] = 0xFF;
    pkt.data_packet_u.data_packet_s.header[1] = 0x00;
    pkt.data_packet_u.data_packet_s.motorId = 0;
    pkt.data_packet_u.data_packet_s.opcode = SET_MAX_ANGLE;
    pkt.data_packet_u.data_packet_s.len = 6;
    pkt.data_packet_u.data_packet_s.pkt_params[0] = TENDON_CONTROL_GET_UPPER_8B(100);
    pkt.data_packet_u.data_packet_s.pkt_params[1] = TENDON_CONTROL_GET_LOWER_8B(100);


    TendonControl_data_packet_s return_pkt = handlePacket((const char*)pkt.data_packet_u.data_packet, tendons);
    TEST_ASSERT_EQUAL(READ_STATUS, return_pkt.data_packet_u.data_packet_s.opcode);
    TEST_ASSERT_EQUAL(COMM_CRC_ERROR, return_pkt.data_packet_u.data_packet_s.pkt_params[0]);

    // Scenario 2: Test packet handling with CRC
    pkt.data_packet_u.data_packet_s.header[0] = 0xFF;
    pkt.data_packet_u.data_packet_s.header[1] = 0x00;
    pkt.data_packet_u.data_packet_s.motorId = 0;
    pkt.data_packet_u.data_packet_s.opcode = SET_MAX_ANGLE;
    pkt.data_packet_u.data_packet_s.len = 6;
    pkt.data_packet_u.data_packet_s.pkt_params[0] = TENDON_CONTROL_GET_UPPER_8B(100);
    pkt.data_packet_u.data_packet_s.pkt_params[1] = TENDON_CONTROL_GET_LOWER_8B(100);
    uint16_t crc = updateCRC(0, pkt.data_packet_u.data_packet, 3 + 6 - TENDON_CONTROL_PKT_NUM_CRC_BYTES);
    pkt.data_packet_u.data_packet_s.pkt_params[2] = TENDON_CONTROL_GET_UPPER_8B(crc);
    pkt.data_packet_u.data_packet_s.pkt_params[3] = TENDON_CONTROL_GET_LOWER_8B(crc);

    return_pkt = handlePacket((const char*)pkt.data_packet_u.data_packet, tendons);
    TEST_ASSERT_EQUAL(READ_STATUS, return_pkt.data_packet_u.data_packet_s.opcode);

    TEST_ASSERT_EQUAL(COMM_SUCCESS, return_pkt.data_packet_u.data_packet_s.pkt_params[0]);
    TEST_ASSERT_FLOAT_WITHIN(0.01, 100, tendons[0].Get_Max_Angle());

    // Scenario 3: Test read angle
    tendons[0].Reset_Encoder_Zero();
    pkt.data_packet_u.data_packet_s.header[0] = 0xFF;
    pkt.data_packet_u.data_packet_s.header[1] = 0x00;
    pkt.data_packet_u.data_packet_s.motorId = 0;
    pkt.data_packet_u.data_packet_s.opcode = READ_ANGLE;
    pkt.data_packet_u.data_packet_s.len = 4;
    crc = updateCRC(0, pkt.data_packet_u.data_packet, 3 + 4 - TENDON_CONTROL_PKT_NUM_CRC_BYTES);
    pkt.data_packet_u.data_packet_s.pkt_params[0] = TENDON_CONTROL_GET_UPPER_8B(crc);
    pkt.data_packet_u.data_packet_s.pkt_params[1] = TENDON_CONTROL_GET_LOWER_8B(crc);

    return_pkt = handlePacket((const char*)pkt.data_packet_u.data_packet, tendons);
    TEST_ASSERT_EQUAL(READ_STATUS, return_pkt.data_packet_u.data_packet_s.opcode);
    TEST_ASSERT_EQUAL(COMM_SUCCESS, return_pkt.data_packet_u.data_packet_s.pkt_params[0]);

    uint16_t angle = TENDON_CONTROL_MAKE_16B_WORD(return_pkt.data_packet_u.data_packet_s.pkt_params[1], return_pkt.data_packet_u.data_packet_s.pkt_params[2]);
    TEST_ASSERT_EQUAL(0.0, angle);
}

void setup() {
    delay(5000);

    UNITY_BEGIN();
    RUN_TEST(test_packet_validation);
    RUN_TEST(test_command_factory1);
    RUN_TEST(test_command_factory2);
    RUN_TEST(test_command_factory3);
    RUN_TEST(test_command_factory4);
    RUN_TEST(test_command_factory5);
    RUN_TEST(test_command_factory6);
    RUN_TEST(test_packet_handling);
    UNITY_END();
}

void loop() {
    
}