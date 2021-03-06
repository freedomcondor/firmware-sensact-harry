
#include <firmware.h>

#include "packet_control_interface.h"

CPacketControlInterface::CPacket::EType CPacketControlInterface::CPacket::GetType() const {
   switch(m_unTypeId) {
   case 0x00:
      return EType::GET_UPTIME;
      break;
   case 0x10:
      return EType::SET_DDS_ENABLE;
      break;
   case 0x11:
      return EType::SET_DDS_SPEED_LEFT;
      break;
   case 0x12:
      return EType::SET_DDS_SPEED_RIGHT;
      break;
   case 0x13:
      return EType::GET_DDS_SPEED;
      break;
   case 0x14:
      return EType::SET_DDS_PARAMS;
      break;
   case 0x15:
      return EType::GET_DDS_PARAMS;
      break;
   default:
      return EType::INVALID;
      break;
   }
}
      
bool CPacketControlInterface::CPacket::HasData() const {
   return (m_unDataLength != 0);
}

uint8_t CPacketControlInterface::CPacket::GetDataLength() const {
   return m_unDataLength;
}

const uint8_t* CPacketControlInterface::CPacket::GetDataPointer() const {
   return m_punData;
}

CPacketControlInterface::EState CPacketControlInterface::GetState() const {
   return m_eState;
}


void CPacketControlInterface::SendPacket(CPacket::EType e_type,
                                         uint8_t* pun_tx_data,
                                         uint8_t un_tx_data_length) {

   uint8_t punTxBuffer[TX_COMMAND_BUFFER_LENGTH];
   uint8_t unTxBufferPointer = 0;
   /* Check if the data will fit into the buffer */
   if(un_tx_data_length + NON_DATA_SIZE > TX_COMMAND_BUFFER_LENGTH)
      return;

   punTxBuffer[unTxBufferPointer++] = PREAMBLE1;
   punTxBuffer[unTxBufferPointer++] = PREAMBLE2;
   punTxBuffer[unTxBufferPointer++] = static_cast<uint8_t>(e_type);
   punTxBuffer[unTxBufferPointer++] = un_tx_data_length;
   while(unTxBufferPointer < DATA_START_OFFSET + un_tx_data_length) {
      punTxBuffer[unTxBufferPointer] = pun_tx_data[unTxBufferPointer - DATA_START_OFFSET];
      unTxBufferPointer++;
   }
   punTxBuffer[unTxBufferPointer++] = ComputeChecksum(punTxBuffer, TX_COMMAND_BUFFER_LENGTH);
   punTxBuffer[unTxBufferPointer++] = POSTAMBLE1;
   punTxBuffer[unTxBufferPointer++] = POSTAMBLE2;

   for(uint8_t unIdx = 0; unIdx < unTxBufferPointer; unIdx++)
      m_cController.Write(punTxBuffer[unIdx]);
}

void CPacketControlInterface::Reset() {
   m_unRxBufferPointer = 0;
   m_unUsedBufferLength = 0;
   m_unReparseOffset = RX_COMMAND_BUFFER_LENGTH;
   m_eState = EState::SRCH_PREAMBLE1;
}
   
void CPacketControlInterface::ProcessInput() {
   bool bBufAdjustReq = false;
   uint8_t unRxByte = 0;
   uint8_t m_unReparseOffset = RX_COMMAND_BUFFER_LENGTH;
 

   if(m_eState == EState::RECV_COMMAND) {
      bBufAdjustReq = true;
      m_eState = EState::SRCH_PREAMBLE1;
   }
 
   while(m_eState != EState::RECV_COMMAND) {
      /* Check if the buffer has overflown */
      if(bBufAdjustReq) {
         /* Search for the beginning of a preamble in buffer */
         for(m_unReparseOffset = 1; m_unReparseOffset < m_unUsedBufferLength; m_unReparseOffset++) {
            if(m_punRxBuffer[m_unReparseOffset] == PREAMBLE1)
               break;
         }
         /* Shift data down in buffer */
         for(uint8_t unBufferIdx = m_unReparseOffset;
             unBufferIdx < m_unUsedBufferLength;
             unBufferIdx++) {
            m_punRxBuffer[unBufferIdx - m_unReparseOffset] = m_punRxBuffer[unBufferIdx];
         }

         m_unUsedBufferLength -= m_unReparseOffset;

         /* The buffer has been adjusted handled */
         bBufAdjustReq = false;
         /* Reparse the buffer */
         m_unRxBufferPointer = 0;
         m_eState = EState::SRCH_PREAMBLE1;
      }
    
      if(m_unRxBufferPointer < m_unUsedBufferLength) {
         unRxByte = m_punRxBuffer[m_unRxBufferPointer];
         m_punRxBuffer[m_unRxBufferPointer++] = unRxByte;
      }     
      else if(m_cController.Available()) {
         unRxByte = m_cController.Read();
         m_punRxBuffer[m_unRxBufferPointer++] = unRxByte;
         m_unUsedBufferLength++;
      }
      else {
         break;
      }

      switch(m_eState) {
      case EState::SRCH_PREAMBLE1:
         if(unRxByte != PREAMBLE1) {
            bBufAdjustReq = true;
         }
         else {
            m_eState = EState::SRCH_PREAMBLE2;
         }
         break;
      case EState::SRCH_PREAMBLE2:
         if(unRxByte != PREAMBLE2) {
            bBufAdjustReq = true;
         }
         else {
            m_eState = EState::SRCH_POSTAMBLE1;
         }
         break;
      case EState::SRCH_POSTAMBLE1:
         if(unRxByte != POSTAMBLE1) {
            m_eState = EState::SRCH_POSTAMBLE1;
         }
         else {
            /* unRxByte == POSTAMBLE1 */
            m_eState = EState::SRCH_POSTAMBLE2;
         }
         break;
      case EState::SRCH_POSTAMBLE2:
         if(unRxByte != POSTAMBLE2) {
            /* previous byte wasn't actually part of postamble */
            if(unRxByte != POSTAMBLE1) {
               m_eState = EState::SRCH_POSTAMBLE1;
            }
            else {
               /* unRxByte == POSTAMBLE1 */
               m_eState = EState::SRCH_POSTAMBLE2;
            }
         }
         else {
            /* check if the length field and checksum are valid */
            if(m_unRxBufferPointer >= NON_DATA_SIZE &&
               m_punRxBuffer[DATA_LENGTH_OFFSET] == (m_unRxBufferPointer - NON_DATA_SIZE) &&
               ComputeChecksum(m_punRxBuffer, RX_COMMAND_BUFFER_LENGTH) == 
               m_punRxBuffer[m_unRxBufferPointer + CHECKSUM_OFFSET]) {
               /* At this point we assume we have a valid command */
               m_eState = EState::RECV_COMMAND;
               /* Populate the packet fields */
               m_cPacket = CPacket(m_punRxBuffer[TYPE_OFFSET],
                                   m_punRxBuffer[DATA_LENGTH_OFFSET],
                                   &m_punRxBuffer[DATA_START_OFFSET]);
            }
            else {
               m_eState = EState::SRCH_POSTAMBLE1;
            }  
         }
         break;
      default:
         break;
      }

      /* buffer overflow condition */
      bBufAdjustReq = bBufAdjustReq || (m_unRxBufferPointer == RX_COMMAND_BUFFER_LENGTH);
   }
}

const char* CPacketControlInterface::StateToString(CPacketControlInterface::EState e_state) const {
   switch(e_state) {
   case EState::SRCH_PREAMBLE1:
      return "SRCH_PREAMBLE1";
      break;
   case EState::SRCH_PREAMBLE2:
      return "SRCH_PREAMBLE2";
      break;
   case EState::SRCH_POSTAMBLE1:
      return "SRCH_POSTAMBLE1";
      break;
   case EState::SRCH_POSTAMBLE2:
      return "SRCH_POSTAMBLE2";
      break;
   case EState::RECV_COMMAND:
      return "RECV_COMMAND";
      break;
   default:
      return "UNKNOWN STATE";
      break;
   }
}

const CPacketControlInterface::CPacket& CPacketControlInterface::GetPacket() const {
   return m_cPacket;
}


uint8_t CPacketControlInterface::ComputeChecksum(uint8_t* pun_buf_data, uint8_t un_buf_length) {
   uint8_t unChecksum = 0;
   for(uint8_t unIdx = TYPE_OFFSET;
       unIdx < ((DATA_START_OFFSET + pun_buf_data[DATA_LENGTH_OFFSET] > un_buf_length) ?
          un_buf_length : (DATA_START_OFFSET + pun_buf_data[DATA_LENGTH_OFFSET]));
       unIdx++) {
      unChecksum += pun_buf_data[unIdx];
   }
   return unChecksum;
}
