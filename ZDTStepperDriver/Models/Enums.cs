using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ZDTStepperDriver.Models
{
    public enum RotateDirection : byte
    {
        CW = 0x00, CCW = 0x01
    }

    public enum HommingType : byte
    {
        // 单圈内就近
        SingleRotateClosest = 0x00,
        // 单圈内有方向
        SingleRotateDirected = 0x01,
        // 机械碰撞检测（无限位开关）
        CollisionDetect = 0x02,
        // 限位开关
        Endstop = 0x04
    }

    public enum VerificationType
    {
        //0x6B
        Const6B,
        //XOR校验
        Xor,
        //CRC8校验
        CRC8,
        //校验失败
        Failed
    }
}
