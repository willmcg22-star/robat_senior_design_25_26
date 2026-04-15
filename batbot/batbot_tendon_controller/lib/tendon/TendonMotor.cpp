#include <TendonMotor.h>
#include <clocks/ml_clocks.h>

float mapf(float x, float in_min, float in_max, float out_min, float out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


float TendonController::ConvertTicksToAngle(int16_t ticks){
    return ((360.0 * ticks) / (ML_ENC_CPR * m_gear_ratio));
}

float TendonController::ConvertAngleToTicks(int16_t deg){
    return ((deg * ML_ENC_CPR * m_gear_ratio) / 360.0);
}


// create tendon controller
// TendonController::TendonController(uint8_t ccChan, ml_pin phasePin, ml_pin pwmPin, ml_pin encA, ml_pin encB, String name)
TendonController::TendonController(String name, uint8_t tcc_num)
{
    // // set the TCC channel
    switch (tcc_num)
    {
        case 0:
            m_pwm_channel = TCC0;
            break;
        case 2:
            m_pwm_channel = TCC2;
    }
    

    // set PID to default
    pid.Set_Params(1, 0, 0, 6000);

    // name of this tendon
    m_name = name;

    enabled = true;
}

void TendonController::Attach_Drive_Pin(ml_port_group portGroup, ml_pin pin, ml_port_function portFunc, uint8_t cc_channel)
{
    ml_port_parity parity = pin % 2 == 0 ? PP_EVEN : PP_ODD;
    m_drive = {portGroup, pin, portFunc, parity, OUTPUT_PULL_DOWN, DRIVE_ON};
    m_pwm_CC = cc_channel;
}

void TendonController::Attach_Direction_Pin(ml_port_group portGroup, ml_pin pin, ml_port_function portFunc)
{
    ml_port_parity parity = pin % 2 == 0 ? PP_EVEN : PP_ODD;
    m_phase = {portGroup, pin, portFunc, parity, OUTPUT_PULL_DOWN, DRIVE_ON};
}

void TendonController::Attach_EncA_Pin(ml_port_group portGroup, ml_pin pin, ml_port_function portFunc)
{
    ml_port_parity parity = pin % 2 == 0 ? PP_EVEN : PP_ODD;
    m_encoder_a = {portGroup, pin, portFunc, parity, INPUT_PULL_UP, DRIVE_OFF};
}

void TendonController::Attach_EncB_Pin(ml_port_group portGroup, ml_pin pin, ml_port_function portFunc)
{
    ml_port_parity parity = pin % 2 == 0 ? PP_EVEN : PP_ODD;
    m_encoder_b = {portGroup, pin, portFunc, parity, INPUT_PULL_UP, DRIVE_OFF};
}

/**
 * Starts the tcc controller
 */
void TendonController::_init_tcc()
{
    ML_SET_GCLK1_PCHCTRL(TCC0_GCLK_ID);

    TCC_DISABLE(TCC0);
    TCC_SWRST(TCC0);
    TCC_sync(TCC0);

    TCC0->CTRLA.reg =
        (TCC_CTRLA_PRESCALER_DIV2 |
         TCC_CTRLA_PRESCSYNC_PRESC);

    TCC0->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;

    TCC_set_period(TCC0, m_tcc_freq);

    // default output matrix configuration (pg. 1829)
    TCC0->WEXCTRL.reg |= TCC_WEXCTRL_OTMX(0x00);
    TCC0->CC[m_pwm_CC].reg |= TCC_CC_CC(m_tcc_freq / 2);

    TCC_ENABLE(TCC0);
    TCC_sync(TCC0);
}

void TendonController::encoder_ISR()
{
    static int8_t lookup_table[] = {0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0};

    uint8_t a_phase = (uint8_t)(logical_read(&m_encoder_a));
    uint8_t b_phase = (uint8_t)(logical_read(&m_encoder_b));

    uint16_t current_encoded = (a_phase << 1) | b_phase;
    uint8_t idx = (m_lastTicks << 2) | current_encoded;
    m_currentTicks += lookup_table[idx];
    m_lastTicks = current_encoded;

    // Serial.println(idx, BIN);
}

void TendonController::Reset_Encoder_Zero()
{
    m_currentTicks = 0;
    m_lastTicks = 0;
    Set_Goal_Angle(0);
}

void TendonController::Set_EncA_Flag()
{
    m_encA_ticks++;
}

void TendonController::Set_EncB_Flag()
{
    m_encB_ticks++;
}

void TendonController::init_peripheral()
{
    peripheral_port_init(&m_encoder_a);
    peripheral_port_init(&m_encoder_b);
    peripheral_port_init(&m_phase);
    peripheral_port_init(&m_drive);
    logical_set(&m_phase);
}
// set motor duty cycle
void TendonController::Set_Duty_Cycle(uint16_t dutyCycle)
{
    // map pwm to freq range
    dutyCycle > m_tcc_freq ? dutyCycle = 100 : 0;
    uint16_t value = m_tcc_freq * (dutyCycle / 100.0);
    m_cur_pwm = value;
    // m_pwm_channel->CCBUF[m_pwm_CC].reg = TCC_CCBUF_CCBUF(value);
    m_pwm_channel->CCBUF[m_pwm_CC].reg = TCC_CCBUF_CCBUF(value);
    TCC_sync(m_pwm_channel);
}

// set motor duty cycle
void TendonController::set_PWM_Freq(uint16_t pwmValue)
{
    // map pwm to freq range
    pwmValue > m_tcc_freq ? pwmValue = m_tcc_freq : pwmValue;
    m_cur_pwm = pwmValue;
    // m_pwm_channel->CCBUF[m_pwm_CC].reg = TCC_CCBUF_CCBUF(value);
    m_pwm_channel->CCBUF[m_pwm_CC].reg = TCC_CCBUF_CCBUF(pwmValue);
    TCC_sync(m_pwm_channel);
}

// set PID parameters
void TendonController::Set_PID_Param(float kp, float ki, float kd, float max)
{
    pid.Set_Params(kp, ki, kd, max);
}

// set the direction motor turns
void TendonController::Set_Direction(Tendon_Direction dir)
{
    if (dir == OFF)
    {
        m_pwm_channel->CCBUF[m_pwm_CC].reg = TCC_CCBUF_CCBUF(0x00);
        TCC_sync(m_pwm_channel);
    }
    else if (dir == CW)
    {
        logical_set(&m_phase);
    }
    else
    {
        logical_unset(&m_phase);
    }
    m_direction = dir;
}

void TendonController::Toggle_Direction()
{
    switch (m_direction)
    {
    case OFF:
        return;
    case CW:
        Set_Direction(CCW);
        break;
    default:
        Set_Direction(CW);
        break;
    }
}

/**
 * Calibrates minimum PWM
 */
void TendonController::Calibrate_Min_PWM()
{

    int16_t lastTicks = m_currentTicks;

    // find min value
    uint16_t minPwm = 0;
    uint16_t avgCWPwm = 0;
    uint16_t timesSuccess = 0;
    ulong lastRun = 0;
    Set_Direction(CW);
    for (int i = 0; i < 5; i++)
    {
        timesSuccess++;
        while (lastTicks == m_currentTicks)
        {
            if (millis() - lastRun > 50)
            {
                minPwm += 50;
                if (minPwm >= m_tcc_freq)
                {
                    timesSuccess--;
                    break;
                }
                set_PWM_Freq(minPwm);
                lastRun = millis();
            }
        }
        lastTicks = m_currentTicks;
        avgCWPwm += minPwm;
        minPwm = 0;
        set_PWM_Freq(0);
    }
    avgCWPwm /= timesSuccess;

    set_PWM_Freq(0);
    Set_Direction(CCW);
    lastTicks = m_currentTicks;
    minPwm = 0;
    timesSuccess = 0;
    uint16_t avgCCWPwm = 0;
    for (int i = 0; i < 5; i++)
    {
        timesSuccess++;
        while (lastTicks == m_currentTicks)
        {
            if (millis() - lastRun > 50)
            {
                minPwm += 100;
                if (minPwm >= m_tcc_freq)
                {
                    timesSuccess--;
                    break;
                }
                set_PWM_Freq(minPwm);
                lastRun = millis();
            }
        }
        lastTicks = m_currentTicks;
        avgCCWPwm += minPwm;
        minPwm = 0;
        set_PWM_Freq(0);
    }

    avgCCWPwm /= timesSuccess;
    Set_Direction(OFF);
    Serial.print("Min pwmcw : ");
    Serial.println(avgCWPwm);
    Serial.print("Min pwmccw : ");
    Serial.println(avgCCWPwm);
    m_min_CW_PWM = avgCWPwm;
    m_min_CCW_PWM = avgCCWPwm;
    m_calibrated = true;
}

/**
 * Returns current angle of motor
 */
float TendonController::Get_Angle()
{
    return ConvertTicksToAngle(m_currentTicks);
}

int32_t TendonController::Get_Ticks()
{
    return m_currentTicks;
}

void TendonController::Move_To_End(bool cw){
    float pwm_freq = 1500;
    set_PWM_Freq(pwm_freq);
    if (cw){
        Set_Direction(CW);
    }
    else{
        Set_Direction(CCW);
    }
    int32_t curAngle = m_currentTicks + 100;

    while (curAngle != m_currentTicks){
        curAngle = m_currentTicks;
        delay(500);
    }
    Set_Direction(OFF);
    Reset_Encoder_Zero();
}

void TendonController::CalibrateLimits(){
    int16_t min_limit = 0;
    int16_t max_limit = 0;

    Serial.println("Calibrating");
    float freq_pwm  = 2600;


    Set_Direction(CCW);
    set_PWM_Freq(freq_pwm);
    int16_t curAngle = 10;
    while (curAngle != m_currentTicks){
        curAngle = m_currentTicks;
        Serial.printf("Finding min: %.3f\n",ConvertTicksToAngle(curAngle));
        delay(200);
    }
    Serial.printf("Found min: %.3f\n",ConvertTicksToAngle(curAngle));
    min_limit = curAngle;
    Reset_Encoder_Zero();


    Set_Direction(CW);
    set_PWM_Freq(freq_pwm);
    curAngle = 10;
    while (curAngle != m_currentTicks){
        curAngle = m_currentTicks;
        Serial.printf("Finding max: %.3f\n",ConvertTicksToAngle(curAngle));
        delay(200);
    }
    Serial.printf("Found max: %.3f\n",ConvertTicksToAngle(curAngle));

    max_limit = curAngle;

    int16_t center = abs(max_limit)-abs(min_limit);

    float min_angle = ConvertTicksToAngle(min_limit);
    float max_angle = ConvertTicksToAngle(max_limit);
    float center_angle = max_angle/2;


    Serial.printf("Min: %f Max: %f Center: %f \n",min_angle,max_angle,center_angle);
    Set_Direction(OFF);

    unsigned long starttime = millis();
    Set_Goal_Angle(center_angle);
    while(millis() - starttime < 500){
        // UpdateMotorControl(freq_pwm);
        UpdateMotorControl();
    }
    Reset_Encoder_Zero();
    Serial.println("Calibration complete");
}

/**
 * Set target angle
 */
void TendonController::Set_Goal_Angle(float destAngle)
{
    goal_angle = constrain(destAngle, -1 * max_angle, max_angle);

    m_target_ticks = ConvertAngleToTicks(goal_angle);
}

void TendonController::UpdateMotorControl() {
    // grab current time
    unsigned long curTime = micros();
    unsigned long deltaTimeUs = curTime - m_prevPIDTime;

    float sig = pid.Compute_Signal(m_currentTicks, m_target_ticks, deltaTimeUs);

    m_cur_pwm = (uint16_t)fabs(sig);

    // set direction
    m_direction = CW;
    if (sig < 0)
    {
        m_direction = CCW;
    }

    if (!m_calibrated)  // not calibrated
    {
        m_cur_pwm = mapf(m_cur_pwm, 0, 6000, 1000, 6000);
    }
    else    // each motor is calibrated
    {
        uint16_t min = sig < 0 ? m_min_CCW_PWM : m_min_CW_PWM;
        m_cur_pwm = mapf(m_cur_pwm, 0, 6000, min, 6000);
    }

    if (m_cur_pwm > m_tcc_freq)
    {
        m_cur_pwm = m_tcc_freq;
    }

    // Set_Duty_Cyle(m_cur_pwm);
    set_PWM_Freq(m_cur_pwm);
    Set_Direction(m_direction);

    // store previous data
    m_prevPIDTime = curTime;
}

void TendonController::Set_Max_Angle(float angle) {
    max_angle = angle;
}

float TendonController::Get_Max_Angle() {
    return max_angle;
}

float TendonController::Get_Goal_Angle() {
    return ConvertTicksToAngle(m_target_ticks);
}

void TendonController::Set_Angle(float angle) {
    m_currentTicks = ConvertAngleToTicks(-90);
}

void TendonController::EnableMotor() {
    enabled = true;
}
    
void TendonController::DisableMotor() {
    enabled = false;
}
  
void TendonController::Get_PID(float &_kp, float &_ki, float &_kd) {
    pid.Get_Params(_kp, _ki, _kd);
}
