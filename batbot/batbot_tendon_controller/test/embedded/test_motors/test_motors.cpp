#include <Arduino.h>
#include "TendonMotor.h"
#include "unity.h"

TendonController tendon("test_tendon");

void test_motor(void)
{
    tendon.Set_Angle(-90);
    TEST_ASSERT_FLOAT_WITHIN(1, -90, tendon.Get_Angle());
}

void setup() {
    tendon.Attach_Drive_Pin(PORT_GRP_C, 21, PF_F, 5);
    tendon.Attach_Direction_Pin(PORT_GRP_B, 17, PF_B);
    tendon.Attach_EncA_Pin(PORT_GRP_C, 15, PF_A);
    tendon.Attach_EncB_Pin(PORT_GRP_C, 14, PF_A);
    tendon.m_gear_ratio = ML_HPCB_LV_100P1;

    delay(5000);

    UNITY_BEGIN();
    RUN_TEST(test_motor);
    UNITY_END();
}

void loop() {
    
}