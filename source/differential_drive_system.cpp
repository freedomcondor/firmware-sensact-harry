
#include "differential_drive_system.h"

#include <firmware.h>

/* Port B Pins - Power and faults */
#define DRV8833_EN     0x01
#define DRV8833_FAULT  0x02

/* Port C Pins - Encoder input */
#define ENC_RIGHT_CHA  0x01
#define ENC_RIGHT_CHB  0x02
#define ENC_LEFT_CHA   0x04
#define ENC_LEFT_CHB   0x08

/* Port D Pins - Motor output */
#define LEFT_CTRL_PIN  0x04
#define RIGHT_CTRL_PIN 0x08
#define RIGHT_MODE_PIN 0x10
#define RIGHT_PWM_PIN  0x20
#define LEFT_PWM_PIN   0x40
#define LEFT_MODE_PIN  0x80

/****************************************/
/****************************************/


CDifferentialDriveSystem::CDifferentialDriveSystem() :
   m_cShaftEncodersInterrupt(this, PCINT1_vect_num),
   m_cPIDControlStepInterrupt(this, TIMER1_COMPA_vect_num),
   m_nLeftSteps(0),
   m_nRightSteps(0) {

   /* Initialise pins in a disabled, coasting state */
   PORTB &= ~(DRV8833_EN);
   PORTD &= ~(LEFT_MODE_PIN  |
              LEFT_CTRL_PIN  |
              LEFT_PWM_PIN   |
              RIGHT_MODE_PIN |
              RIGHT_CTRL_PIN |
              RIGHT_PWM_PIN);
   /* set the direction of the output pins to output */
   DDRB |= (DRV8833_EN);
   DDRD |= (LEFT_MODE_PIN  |
            LEFT_CTRL_PIN  |
            LEFT_PWM_PIN   |
            RIGHT_MODE_PIN |
            RIGHT_CTRL_PIN |
            RIGHT_PWM_PIN);

   /* Setup up timer 0 for PWM */
   /* Select phase correct, non-inverting PWM mode on channel A & B */
   TCCR0A |= (1 << WGM00);

   /* Set precaler to 256 (approximately 61.275Hz output frequency) */
   //TCCR0B |= (1 << CS02);

   /* Set precaler to 8 (approximately 61.275Hz output frequency) */
   //TCCR0B |= (1 << CS01);

   /* Set precaler to 1 (approximately 61.275Hz output frequency) */
   TCCR0B |= (1 << CS00);

   /* Initialize left and right motor duty cycle to zero */
   OCR0A = 0;
   OCR0B = 0;

   /* CTC Mode , with precaler set to 64, OCR1A = 2039 (61.275Hz update frequency - same as PWM) */
   TCCR1B |= (1 << WGM12) | (1 << CS11) | (1 << CS10);
   OCR1A = 2039;
   
   /* Enable port change interrupts for right encoder A/B
      and left encoder A/B respectively */
   PCMSK1 |= (1 << PCINT8)  | (1 << PCINT9) |
             (1 << PCINT10) | (1 << PCINT11);
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::SetTargetVelocity(int16_t n_left_speed, int16_t n_right_speed) {
   m_cPIDControlStepInterrupt.SetTargetVelocity(n_left_speed, n_right_speed);
}

/****************************************/
/****************************************/

int16_t CDifferentialDriveSystem::GetLeftDDSParams(	int16_t* error, 
													float* errorIntergral, 
													int16_t* errorDerivative) 
{
   int16_t nVelocity;
   uint8_t unSREG = SREG;
   cli();

   nVelocity = m_nLeftStepsOut;
   *error = m_nLeftErrorOut;
   *errorIntergral = m_fLeftErrorIntegralOut;
   *errorDerivative = m_nLeftErrorDerivativeOut;

   SREG = unSREG;
   return nVelocity;
}

/****************************************/
/****************************************/

int16_t CDifferentialDriveSystem::GetRightDDSParams(	int16_t* error, 
													float* errorIntergral, 
													int16_t* errorDerivative) 
{
   int16_t nVelocity;
   uint8_t unSREG = SREG;
   cli();

   nVelocity = m_nRightStepsOut;
   *error = m_nRightErrorOut;
   *errorIntergral = m_fRightErrorIntegralOut;
   *errorDerivative = m_nRightErrorDerivativeOut;

   SREG = unSREG;
   return nVelocity;
}

/****************************************/
/****************************************/

int16_t CDifferentialDriveSystem::GetLeftVelocity() {
   int16_t nVelocity;
   uint8_t unSREG = SREG;
   cli();
   nVelocity = m_nLeftStepsOut;
   SREG = unSREG;
   return nVelocity;
}

/****************************************/
/****************************************/

int16_t CDifferentialDriveSystem::GetRightVelocity() {
   int16_t nVelocity;
   uint8_t unSREG = SREG;
   cli();
   nVelocity = m_nRightStepsOut;
   SREG = unSREG;
   return nVelocity;
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::Enable() {
   /* Enable the shaft encoder interrupt */
   m_cShaftEncodersInterrupt.Enable();
   /* Enable the PID controller interrupt */
   m_cPIDControlStepInterrupt.Enable();
   /* Enable the motor driver */
   PORTB |= (DRV8833_EN);
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::Disable() {
   /* Disable the motor driver */
   PORTB &= ~(DRV8833_EN);
   /* Disable the PID controller interrupt */
   m_cPIDControlStepInterrupt.Disable();
   /* Disable the shaft encoder interrupt */
   m_cShaftEncodersInterrupt.Disable();
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::ConfigureLeftMotor(CDifferentialDriveSystem::EBridgeMode e_mode,
                                                  uint8_t un_duty_cycle) {
   switch(e_mode) {
   case EBridgeMode::FORWARD_PWM_FD:
      PORTD |= LEFT_CTRL_PIN;
      PORTD &= ~LEFT_MODE_PIN;
      break;
   case EBridgeMode::FORWARD_PWM_SD:
      un_duty_cycle = uint8_t(UINT8_MAX) - un_duty_cycle;
      PORTD &= ~LEFT_CTRL_PIN;
      PORTD |= LEFT_MODE_PIN;
      break;
   case EBridgeMode::REVERSE_PWM_FD:
      PORTD &= ~LEFT_CTRL_PIN;
      PORTD &= ~LEFT_MODE_PIN;
      break;
   case EBridgeMode::REVERSE_PWM_SD:
      un_duty_cycle = uint8_t(UINT8_MAX) - un_duty_cycle;
      PORTD |= LEFT_CTRL_PIN;
      PORTD |= LEFT_MODE_PIN;
      break;
   case EBridgeMode::COAST:
      PORTD &= ~(LEFT_PWM_PIN | LEFT_CTRL_PIN | LEFT_MODE_PIN);
      break;
   case EBridgeMode::FORWARD:
      PORTD |= LEFT_PWM_PIN;
      PORTD &= ~LEFT_MODE_PIN;
      PORTD |= LEFT_CTRL_PIN;
      break;
   case EBridgeMode::REVERSE:
      PORTD |= LEFT_PWM_PIN;
      PORTD &= ~LEFT_MODE_PIN;
      PORTD &= ~LEFT_CTRL_PIN;
      break;
   case EBridgeMode::BRAKE:
      PORTD |= (LEFT_PWM_PIN | LEFT_CTRL_PIN | LEFT_MODE_PIN);
      break;
   }

   if(e_mode == EBridgeMode::COAST   ||
      e_mode == EBridgeMode::REVERSE ||
      e_mode == EBridgeMode::FORWARD ||
      e_mode == EBridgeMode::BRAKE) {
      /* disconnect PWM from the output pin */
      OCR0A = 0;
      TCCR0A &= ~((1 << COM0A1) | (1 << COM0A0));
   }
   else {
      /* connect PWM to the output pin */
      OCR0A = un_duty_cycle;
      TCCR0A |= (1 << COM0A1);
   }
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::ConfigureRightMotor(CDifferentialDriveSystem::EBridgeMode e_mode,
                                                   uint8_t un_duty_cycle) {
   switch(e_mode) {
   case EBridgeMode::FORWARD_PWM_FD:
      PORTD &= ~RIGHT_CTRL_PIN;
      PORTD &= ~RIGHT_MODE_PIN;
      break;
   case EBridgeMode::FORWARD_PWM_SD:
      un_duty_cycle = uint8_t(UINT8_MAX) - un_duty_cycle;
      PORTD |= RIGHT_CTRL_PIN;
      PORTD |= RIGHT_MODE_PIN;
      break;
   case EBridgeMode::REVERSE_PWM_FD:
      PORTD |= RIGHT_CTRL_PIN;
      PORTD &= ~RIGHT_MODE_PIN;
      break;
   case EBridgeMode::REVERSE_PWM_SD:
      un_duty_cycle = uint8_t(UINT8_MAX) - un_duty_cycle;
      PORTD &= ~RIGHT_CTRL_PIN;
      PORTD |= RIGHT_MODE_PIN;
      break;
   case EBridgeMode::COAST:
      PORTD &= ~(RIGHT_PWM_PIN | RIGHT_CTRL_PIN | RIGHT_MODE_PIN);
      break;
   case EBridgeMode::FORWARD:
      PORTD |= RIGHT_PWM_PIN;
      PORTD &= ~RIGHT_MODE_PIN;
      PORTD &= ~RIGHT_CTRL_PIN;
      break;
   case EBridgeMode::REVERSE:
      PORTD |= RIGHT_PWM_PIN;
      PORTD &= ~RIGHT_MODE_PIN;
      PORTD |= RIGHT_CTRL_PIN;
      break;
   case EBridgeMode::BRAKE:
      PORTD |= (RIGHT_PWM_PIN | RIGHT_CTRL_PIN | RIGHT_MODE_PIN);
      break;
   }

   if(e_mode == EBridgeMode::COAST   ||
      e_mode == EBridgeMode::REVERSE ||
      e_mode == EBridgeMode::FORWARD ||
      e_mode == EBridgeMode::BRAKE) {
      /* disconnect PWM from the output pin */
      OCR0B = 0;
      TCCR0A &= ~((1 << COM0B1) | (1 << COM0B0));
   }
   else {
      /* connect PWM to the output pin */
      OCR0B = un_duty_cycle;
      TCCR0A |= (1 << COM0B1);
   }
}

/****************************************/
/****************************************/

CDifferentialDriveSystem::CShaftEncodersInterrupt::CShaftEncodersInterrupt(
   CDifferentialDriveSystem* pc_differential_drive_system,
   uint8_t un_intr_vect_num) :
   m_pcDifferentialDriveSystem(pc_differential_drive_system),
   m_unPortLast(0) {
   Register(this, un_intr_vect_num);
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::CShaftEncodersInterrupt::Enable() {
   /* clear variables */
   m_unPortLast = 0;
   m_pcDifferentialDriveSystem->m_nLeftSteps = 0;
   m_pcDifferentialDriveSystem->m_nRightSteps = 0;
   /* enable interrupt */
   PCICR |= (1 << PCIE1);
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::CShaftEncodersInterrupt::Disable() {
   /* disable interrupt */
   PCICR &= ~(1 << PCIE1);
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::CShaftEncodersInterrupt::ServiceRoutine() {
   uint8_t unPortSnapshot = PINC;
   uint8_t unPortDelta = m_unPortLast ^ unPortSnapshot;
   /* This intermediate value determines whether the motors are moving
      backwards or forwards. The expression uses Reed-Muller logic and
      leverages the symmetry of the encoder inputs */
   uint8_t unIntermediate = (~unPortSnapshot) ^ (m_unPortLast >> 1);
   /* check the left encoder */
   if(unPortDelta & (ENC_LEFT_CHA | ENC_LEFT_CHB)) {
      if(unIntermediate & ENC_LEFT_CHA) {
         m_pcDifferentialDriveSystem->m_nLeftSteps--;
      }
      else {
         m_pcDifferentialDriveSystem->m_nLeftSteps++;
      }
   }
   /* check the right encoder */
   if(unPortDelta & (ENC_RIGHT_CHA | ENC_RIGHT_CHB)) {
      if(unIntermediate & ENC_RIGHT_CHA) {
         m_pcDifferentialDriveSystem->m_nRightSteps++;
      }
      else {
         m_pcDifferentialDriveSystem->m_nRightSteps--;
      }
   }
   m_unPortLast = unPortSnapshot;
}

/****************************************/
/****************************************/

CDifferentialDriveSystem::CPIDControlStepInterrupt::CPIDControlStepInterrupt(
   CDifferentialDriveSystem* pc_differential_drive_system,
   uint8_t un_intr_vect_num) :
   m_pcDifferentialDriveSystem(pc_differential_drive_system),
   m_nLeftTarget(0),
   m_nLeftLastError(0),
   m_fLeftErrorIntegral(0.0f),
   m_nRightTarget(0),
   m_nRightLastError(0),
   m_fRightErrorIntegral(0.0f),
   //m_fKp(0.707f),
   //m_fKi(0.625f),
   //m_fKd(0.056f) {
   m_fKp(0.0f),
   m_fKi(0.0f),
   m_fKd(0.0f) {
   Register(this, un_intr_vect_num);
   windowLength = 254;
   for (uint8_t i = 0; i < windowLength; i++)
   {
	   m_nLeftStepWindow[i] = 0;
	   m_nRightStepWindow[i] = 0;
   }
   windowOutput = 0;
   windowInput = windowLength - 1;
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::CPIDControlStepInterrupt::Enable() {
   /* clear intermediate variables */
   m_nLeftLastError = 0;
   m_fLeftErrorIntegral = 0.0f;
   m_nRightLastError = 0;
   m_fRightErrorIntegral = 0.0f;
   /* enable interrupt */
   TIMSK1 |= (1 << OCIE1A);
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::CPIDControlStepInterrupt::Disable() {
   /* disable interrupt */
   TIMSK1 &= ~(1 << OCIE1A);
}

/****************************************/
/****************************************/

void CDifferentialDriveSystem::CPIDControlStepInterrupt::SetTargetVelocity(int16_t n_left_speed, 
                                                                           int16_t n_right_speed) {
   m_nLeftTarget = n_left_speed;
   m_nRightTarget = n_right_speed;
}

void CDifferentialDriveSystem::SetPIDPara(float fKp, float fKi, float fKd)
{
	m_cPIDControlStepInterrupt.SetPIDPara(fKp,fKi,fKd);
}

void CDifferentialDriveSystem::floattry(uint8_t temp1, uint8_t temp2, uint8_t temp3, uint8_t temp4)
{
	float1 = temp1;
	float2 = temp2;
	float3 = temp3;
	float4 = temp4;
}

void CDifferentialDriveSystem::CPIDControlStepInterrupt::SetPIDPara(float fKp, float fKi, float fKd)
{
	m_fKp = fKp;
	m_fKi = fKi;
	m_fKd = fKd;
}


/****************************************/
/****************************************/

void CDifferentialDriveSystem::CPIDControlStepInterrupt::ServiceRoutine() {
   /* Calculate left PID intermediates */
   int16_t nLeftError = m_nLeftTarget - m_pcDifferentialDriveSystem->m_nLeftSteps;
   /* Accumulate the integral component */
   m_fLeftErrorIntegral += nLeftError;

   /* window */
   /*
   m_fLeftErrorIntegral -= m_nLeftStepWindow[windowOutput];
   m_nLeftStepWindow[windowInput] = nLeftError;
   windowOutput++;
   windowInput++;
   if (windowInput == windowLength) windowInput = 0;
   if (windowOutput == windowLength) windowOutput = 0;
   */

   /* intergral limit*/
   float MaxIntlimit = 300;
   float MinIntlimit = -MaxIntlimit;
   if (m_fLeftErrorIntegral > MaxIntlimit) m_fLeftErrorIntegral = MaxIntlimit;
   if (m_fLeftErrorIntegral < MinIntlimit) m_fLeftErrorIntegral = MinIntlimit;

   /* Calculate the derivate component */
   int16_t nLeftErrorDerivative = (nLeftError - m_nLeftLastError);
   m_nLeftLastError = nLeftError;
   /* Calculate output value */
   float fLeftOutput =
      (m_fKp * nLeftError) +
      (m_fKi * m_fLeftErrorIntegral) +
      //(m_fKd * nLeftErrorDerivative);
      (m_fKd * nLeftErrorDerivative) + m_nLeftTarget;

   /* store the sign of the output */
   bool bLeftNegative = (fLeftOutput < 0.0f);
   /* take the absolute value */
   fLeftOutput = (bLeftNegative ? -fLeftOutput : fLeftOutput);
   /* saturate into the uint8_t range */
   //uint8_t SPEED_MAX = 40;
   uint8_t unLeftDutyCycle = (fLeftOutput < float(UINT8_MAX)) ? uint8_t(fLeftOutput) : UINT8_MAX;
   //uint8_t unLeftDutyCycle = (fLeftOutput < float(SPEED_MAX)) ? uint8_t(fLeftOutput) : SPEED_MAX;
   /* Calculate right PID intermediates */
   int16_t nRightError = m_nRightTarget - m_pcDifferentialDriveSystem->m_nRightSteps;
   /* Accumulate the integral component */
   m_fRightErrorIntegral += nRightError;

   /* intergral limit*/
   if (m_fRightErrorIntegral > MaxIntlimit) m_fRightErrorIntegral = MaxIntlimit;
   if (m_fRightErrorIntegral < MinIntlimit) m_fRightErrorIntegral = MinIntlimit;

   /* Calculate the derivate component */
   int16_t nRightErrorDerivative = (nRightError - m_nRightLastError);
   m_nRightLastError = nRightError;
   /* Calculate output value */
   float fRightOutput =
      (m_fKp * nRightError) +
      (m_fKi * m_fRightErrorIntegral) +
      //(m_fKd * nRightErrorDerivative);
      (m_fKd * nRightErrorDerivative) + m_nRightTarget;
   /* store the sign of the output */
   bool bRightNegative = (fRightOutput < 0.0f);
   /* take the absolute value */
   fRightOutput = (bRightNegative ? -fRightOutput : fRightOutput);
   /* saturate into the uint8_t range */
   uint8_t unRightDutyCycle = (fRightOutput <float(UINT8_MAX)) ? uint8_t(fRightOutput) : UINT8_MAX;
   //uint8_t unRightDutyCycle = (fRightOutput < float(SPEED_MAX)) ? uint8_t(fRightOutput) : SPEED_MAX;

   /* Update right motor */
   //if(((m_nRightTarget < 0) && bRightNegative) || ((m_nRightTarget >= 0) && !bRightNegative)) {
   if(1) {
      m_pcDifferentialDriveSystem->ConfigureRightMotor(
         bRightNegative ? CDifferentialDriveSystem::EBridgeMode::REVERSE_PWM_FD :
                          CDifferentialDriveSystem::EBridgeMode::FORWARD_PWM_FD,
         unRightDutyCycle);
   }
   else {
       m_pcDifferentialDriveSystem->ConfigureRightMotor(CDifferentialDriveSystem::EBridgeMode::COAST);
   }

   /* Update left motor */
   //if(((m_nLeftTarget < 0) && bLeftNegative) || ((m_nLeftTarget >= 0) && !bLeftNegative)) {
   if(1) {
      m_pcDifferentialDriveSystem->ConfigureLeftMotor(
         bLeftNegative  ? CDifferentialDriveSystem::EBridgeMode::REVERSE_PWM_FD :
                          CDifferentialDriveSystem::EBridgeMode::FORWARD_PWM_FD,
         unLeftDutyCycle);
   }
   else {
      m_pcDifferentialDriveSystem->ConfigureLeftMotor(CDifferentialDriveSystem::EBridgeMode::COAST);
   }

   /* copy the step counters for velocity measurements */
   m_pcDifferentialDriveSystem->m_nRightStepsOut = m_pcDifferentialDriveSystem->m_nRightSteps;
   m_pcDifferentialDriveSystem->m_nLeftStepsOut = m_pcDifferentialDriveSystem->m_nLeftSteps; 

   /* clear the step counters */
   m_pcDifferentialDriveSystem->m_nRightSteps = 0;
   m_pcDifferentialDriveSystem->m_nLeftSteps = 0;

   /* copy dds params */
   m_pcDifferentialDriveSystem->m_nLeftErrorOut = nLeftError;
   //m_pcDifferentialDriveSystem->m_nLeftErrorOut = unLeftDutyCycle;
   m_pcDifferentialDriveSystem->m_fLeftErrorIntegralOut = m_fLeftErrorIntegral;
   m_pcDifferentialDriveSystem->m_nLeftErrorDerivativeOut = nLeftErrorDerivative;

   m_pcDifferentialDriveSystem->m_nRightErrorOut = nRightError;
   //m_pcDifferentialDriveSystem->m_nRightErrorOut = unRightDutyCycle;
   m_pcDifferentialDriveSystem->m_fRightErrorIntegralOut = m_fRightErrorIntegral;
   //m_pcDifferentialDriveSystem->m_fRightErrorIntegralOut = m_fKp;
   m_pcDifferentialDriveSystem->m_nRightErrorDerivativeOut = nRightErrorDerivative;
}

/****************************************/
/****************************************/









