#ifndef DIFFERENTIAL_DRIVE_SYSTEM_H
#define DIFFERENTIAL_DRIVE_SYSTEM_H

#include <stdint.h>
#include <interrupt.h>

#define MAX_INTGRALWINDOW 255

class CDifferentialDriveSystem {
public:
   CDifferentialDriveSystem();

   void SetTargetVelocity(int16_t n_left_speed, int16_t n_right_speed);
   void SetPIDPara(float fKp, float fKi, float fKd);

   void floattry(uint8_t temp1, uint8_t temp2, uint8_t temp3, uint8_t temp4);
   uint8_t float1, float2, float3, float4;
   
   int16_t GetLeftVelocity();
   int16_t GetRightVelocity();

   int16_t GetLeftDDSParams(int16_t* error, float* errorIntergral, int16_t* errorDerivative);
   int16_t GetRightDDSParams(int16_t* error, float* errorIntergral, int16_t* errorDerivative);

   void Enable();
   void Disable();

public:
   enum class EBridgeMode {
      COAST,
      REVERSE,
      REVERSE_PWM_FD,
      REVERSE_PWM_SD,
      FORWARD,
      FORWARD_PWM_FD,
      FORWARD_PWM_SD,
      BRAKE
   };

   void ConfigureLeftMotor(EBridgeMode e_mode, uint8_t un_duty_cycle = 0);
   void ConfigureRightMotor(EBridgeMode e_mode, uint8_t un_duty_cycle = 0);

   class CShaftEncodersInterrupt : public CInterrupt {
   public:
      CShaftEncodersInterrupt(CDifferentialDriveSystem* pc_differential_drive_system,
                              uint8_t un_intr_vect_num);
                              
      void Enable();
      void Disable();
   private:
      void ServiceRoutine();
   private:
      CDifferentialDriveSystem* m_pcDifferentialDriveSystem;
      volatile uint8_t m_unPortLast;
   } m_cShaftEncodersInterrupt;

   class CPIDControlStepInterrupt : public CInterrupt {
   public:
      CPIDControlStepInterrupt(CDifferentialDriveSystem* pc_differential_drive_system, 
                               uint8_t un_intr_vect_num);
      void Enable();
      void Disable();
      void SetTargetVelocity(int16_t n_left_speed, int16_t n_right_speed);
      void SetPIDPara(float fKp, float fKi, float fKd);
   private:
      void ServiceRoutine();
   private:   
      CDifferentialDriveSystem* m_pcDifferentialDriveSystem;      

      int16_t m_nLeftTarget;
      int16_t m_nLeftLastError;
      float m_fLeftErrorIntegral;
      int16_t m_nRightTarget;
      int16_t m_nRightLastError;
      float m_fRightErrorIntegral;
      //const float m_fKp;
      //const float m_fKi;
      //const float m_fKd;     
      float m_fKp;
      float m_fKi;
      float m_fKd;     

      int16_t m_nLeftStepWindow[MAX_INTGRALWINDOW];
      int16_t m_nRightStepWindow[MAX_INTGRALWINDOW];
      uint16_t windowLength;
      uint16_t windowOutput;
      uint16_t windowInput;
   } m_cPIDControlStepInterrupt;

   friend CShaftEncodersInterrupt;
   friend CPIDControlStepInterrupt;

   /* Actual step count variable */
   volatile int16_t m_nLeftSteps;
   volatile int16_t m_nRightSteps;
   /* Cached step count variable */
   volatile int16_t m_nLeftStepsOut;
   volatile int16_t m_nRightStepsOut;

   /* dds params output */
   volatile int16_t m_nLeftErrorOut;
   volatile float m_fLeftErrorIntegralOut;
   volatile int16_t m_nLeftErrorDerivativeOut;
   volatile int16_t m_nRightErrorOut;
   volatile float m_fRightErrorIntegralOut;
   volatile int16_t m_nRightErrorDerivativeOut;
};

#endif
