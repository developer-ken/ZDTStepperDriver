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

    public enum Command : byte
    {
        PowerCtrl = 0xF3,
        SetSpeed = 0xF6,
        SetPosition = 0xFD,
        Brake = 0xFE,
        Sync = 0xFF,

        SetZeroDegree = 0x93,
        GoHome = 0x9A,
        AbortGoHome = 0x9C,
        ReadHomingConfig = 0x22,
        SetHomingConfig = 0x4C,
        ReadHomingStatus = 0x3B,

        CalibrateEncoder = 0x06,
        ZeroPulseCounter = 0x0A,
        ClogProtectionReset = 0x0E,

        FactoryReset = 0x0F,
        GetDriverVersion = 0x1F,
        ReadMotorCharacteristic = 0x20,
        ReadPosPIDParameters = 0x21,
        GetRailVotage = 0x24,
        GetPhaseCurrent = 0x27,
        GetEncoderReading = 0x31,
        GetPulses = 0x32,
        GetTargetPos = 0x33,
        GetOLTargetPos = 0x34,
        GetRPM = 0x35,
        GetCurrentPos = 0x36,
        GetPosError = 0x37,
        GetMotorStatus = 0x3A,
        GetDriverSettings = 0x42,
        GetDriverStatus = 0x43,

        SetMicroStep = 0x84,
        SetDriverAddr = 0xAE,
        SetDriverMode = 0x46,
        SetOpenLoopCurrent = 0x44,
        SetDriverParameter = 0x48,
        SetPosPIDParameters = 0x4A,
        PutPowerOnCommand = 0xF7,
        SetVelScale = 0x4F
    }
}
