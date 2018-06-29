#include "firmware.h"
#include "interrupt.h"

/* initialisation of the static singleton */
CFirmware CFirmware::_firmware;

/* main function that runs the firmware */
int main(void)
{
   /* FILE structs for fprintf */
   FILE huart;

   /* Set up FILE structs for fprintf */                           
   fdev_setup_stream(&huart, 
                     [](char c_to_write, FILE* pf_stream) {
                        CFirmware::GetInstance().GetHUARTController().Write(c_to_write);
                        return 1;
                     },
                     [](FILE* pf_stream) {
                        return int(CFirmware::GetInstance().GetHUARTController().Read());
                     },
                     _FDEV_SETUP_RW);

   CFirmware::GetInstance().SetFilePointer(&huart);

   /* Execute the firmware */
   CFirmware::GetInstance().Exec();

   /* Shutdown */
   return 0;
}

/***********************************************************/
/***********************************************************/

void CFirmware::Exec() {
   m_cAccelerometerSystem.Init();

   for(;;) {
      m_cPacketControlInterface.ProcessInput();

      if(m_cPacketControlInterface.GetState() == CPacketControlInterface::EState::RECV_COMMAND) {
         CPacketControlInterface::CPacket cPacket = m_cPacketControlInterface.GetPacket();
         switch(cPacket.GetType()) {
         case CPacketControlInterface::CPacket::EType::SET_DDS_ENABLE:
            /* Set the enable signal for the differential drive system */
            if(cPacket.GetDataLength() == 1) {
               const uint8_t* punRxData = cPacket.GetDataPointer();
               if(punRxData[0] == 0) {
                  m_cDifferentialDriveSystem.Disable();
               }
               else {
                  m_cDifferentialDriveSystem.Enable();
               }
            }
            break;
         case CPacketControlInterface::CPacket::EType::SET_DDS_SPEED:
            /* Set the speed of the differential drive system */
            if(cPacket.GetDataLength() == 4) {
               const uint8_t* punRxData = cPacket.GetDataPointer();
               int16_t nLeftVelocity = 0, nRightVelocity = 0;
               
               reinterpret_cast<uint16_t&>(nLeftVelocity) = (punRxData[0] << 8) | punRxData[1];
               reinterpret_cast<uint16_t&>(nRightVelocity) = (punRxData[2] << 8) | punRxData[3];
               m_cDifferentialDriveSystem.SetTargetVelocity(nLeftVelocity, nRightVelocity);
            }
            break;
         case CPacketControlInterface::CPacket::EType::GET_DDS_SPEED:
            if(cPacket.GetDataLength() == 0) {
               /* Get the speed of the differential drive system */               
               int16_t nLeftSpeed = m_cDifferentialDriveSystem.GetLeftVelocity();
               int16_t nRightSpeed = m_cDifferentialDriveSystem.GetRightVelocity();
               uint8_t punTxData[] {
                  reinterpret_cast<uint8_t*>(&nLeftSpeed)[1],
                  reinterpret_cast<uint8_t*>(&nLeftSpeed)[0],
                  reinterpret_cast<uint8_t*>(&nRightSpeed)[1],
                  reinterpret_cast<uint8_t*>(&nRightSpeed)[0],
               };
               m_cPacketControlInterface.SendPacket(CPacketControlInterface::CPacket::EType::GET_DDS_SPEED,
                                                    punTxData,
                                                    sizeof(punTxData));
            }
            break;
         case CPacketControlInterface::CPacket::EType::GET_UPTIME:
            if(cPacket.GetDataLength() == 0) {
               /* timer not implemented to improve interrupt latency for the shaft encoders */
               uint8_t punTxData[] = {0, 0, 0, 0};
               m_cPacketControlInterface.SendPacket(CPacketControlInterface::CPacket::EType::GET_UPTIME,
                                                    punTxData,
                                                    4);
            }
            break;
         case CPacketControlInterface::CPacket::EType::GET_ACCEL_READING:
            if(cPacket.GetDataLength() == 0) {
               CAccelerometerSystem::SReading sReading = m_cAccelerometerSystem.GetReading();
               uint8_t punTxData[] = {
                  uint8_t((sReading.X >> 8) & 0xFF),
                  uint8_t((sReading.X >> 0) & 0xFF),
                  uint8_t((sReading.Y >> 8) & 0xFF),
                  uint8_t((sReading.Y >> 0) & 0xFF),
                  uint8_t((sReading.Z >> 8) & 0xFF),
                  uint8_t((sReading.Z >> 0) & 0xFF),
                  uint8_t((sReading.Temp >> 8) & 0xFF),
                  uint8_t((sReading.Temp >> 0) & 0xFF),                  
               };
               m_cPacketControlInterface.SendPacket(CPacketControlInterface::CPacket::EType::GET_ACCEL_READING,
                                                    punTxData,
                                                    sizeof(punTxData));
            }
            break;
         default:
            /* unknown command */
            break;
         }
      }
   }
}

/***********************************************************/
/***********************************************************/
