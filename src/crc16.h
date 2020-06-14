#ifndef CRC16_H
#define CRC16_H

namespace xqserial_server
{
  void CRC16CheckSum(uint8_t *pDataIn, int len, uint8_t *sum);
  void CRC16CheckSum(uint8_t *pDataIn, int len, uint16_t *sum);
} //namespace xqserial_server
#endif // CRC16_H
