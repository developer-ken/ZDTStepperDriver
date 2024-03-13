using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ZDTStepperDriver
{
    internal class Verification
    {
        public static byte XOR(byte[] data)
        {
            byte check = (byte)(data[0] ^ data[1]);
            for (int i = 2; i < data.Length; i++)
            {
                check = (byte)(check ^ data[i]);
            }
            return check;
        }

        public static byte CRC8(byte[] data)
        {
            byte crc = 0;
            for (int j = 0; j < data.Length; j++)
            {
                crc ^= data[j];
                for (int i = 0; i < 8; i++)
                {
                    if ((crc & 0x01) != 0)
                    {
                        crc >>= 1;
                        crc ^= 0x8c;
                    }
                    else
                    {
                        crc >>= 1;
                    }
                }
            }
            return crc;
        }
    }
}
