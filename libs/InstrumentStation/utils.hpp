/*
  nRF24L01 Instrument Station
  utils.hpp
  Arduino Mega

  D C Potts 2021
  School of Engineering
  University of Liverpool
*/

#ifndef IS_UTILS_HPP
#define IS_UTILS_HPP

size_t print64(uint64_t number, int base)
{
  size_t n = 0;
  unsigned char buf[64];
  uint8_t i = 0;

  if (base == 16) {
    Serial.print("0x");
  }

  if (number == 0)
  {
      n += Serial.print((char)'0');
      return n;
  }

  if (base < 2) base = 2;
  else if (base > 16) base = 16;

  while (number > 0)
  {
      uint64_t q = number/base;
      buf[i++] = number - q*base;
      number = q;
  }

  for (; i > 0; i--)
  n += Serial.write((char) (buf[i - 1] < 10 ?
  '0' + buf[i - 1] :
  'A' + buf[i - 1] - 10));

  return n;
};

size_t print64ln(uint64_t number, int base)
{
  size_t n = 0;
  n += print64((uint64_t)number, base);
  n += Serial.println();
  return n;
};

#endif
