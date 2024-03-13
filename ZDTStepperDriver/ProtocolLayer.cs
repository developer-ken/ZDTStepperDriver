using System;
using System.Collections.Generic;
using System.IO.Ports;
using System.Linq;
using System.Runtime.Intrinsics.Arm;
using System.Text;
using System.Threading.Tasks;
using ZDTStepperDriver.Exceptions;
using ZDTStepperDriver.Models;

namespace ZDTStepperDriver
{
    public class ProtocolLayer
    {
        SerialPort Port;
        public bool WaitForAck;
        public VerificationType VerificationType;
        private bool MoveSync = false;

        public ProtocolLayer(SerialPort port, VerificationType verificationType = VerificationType.Const6B, bool alwaysWaitForAck = true)
        {
            Port = port;
            VerificationType = verificationType;
            WaitForAck = alwaysWaitForAck;
        }

        #region 链路层
        public void SendPack(byte addr, byte[] payload, VerificationType veri)
        {
            byte[] buffer = new byte[payload.Length + 2];
            buffer[0] = addr;
            payload.CopyTo(buffer, 1);
            switch (veri)
            {
                case VerificationType.Const6B:
                    buffer[buffer.Length - 1] = 0x6B;
                    break;
                case VerificationType.Xor:
                    buffer[buffer.Length - 1] = Verification.XOR(buffer.Take(buffer.Length - 1).ToArray());
                    break;
                case VerificationType.CRC8:
                    buffer[buffer.Length - 1] = Verification.CRC8(buffer.Take(buffer.Length - 1).ToArray());
                    break;
                default:
                    throw new ArgumentOutOfRangeException(nameof(veri), "Invalid verification type.");
            }
            Port.Write(buffer, 0, buffer.Length);
        }

        public void SendPack(byte addr, Pack pack)
        {
            byte[] payload = new byte[pack.Payload.Length + 1];
            payload[0] = (byte)pack.Command;
            pack.Payload.CopyTo(payload, 1);
            SendPack(addr, payload, pack.Verification);
        }

        public Pack ReceivePack(out byte addr)
        {
            while (Port.BytesToRead == 0) Thread.Sleep(0);
            byte[] buf = new byte[Port.BytesToRead];
            Port.Read(buf, 0, buf.Length);
            addr = buf[0];
            var packdata = buf.Take(buf.Length - 1).ToArray();
            var veri = buf.Last();
            VerificationType veri_type = VerificationType.Failed;
            if (veri == 0x6B)
            {
                veri_type = VerificationType.Const6B;
            }
            else if (veri == Verification.XOR(packdata))
            {
                veri_type = VerificationType.Xor;
            }
            else if (veri == Verification.CRC8(packdata))
            {
                veri_type = VerificationType.CRC8;
            }

            return new Pack()
            {
                Command = (Command)buf[1],
                Payload = (byte[])buf.Skip(2).Take(buf.Length - 3),
                Verification = veri_type
            };
        }

        public void Assert(Pack pack)
        {
            if (pack.Verification == VerificationType.Failed)
                throw new ValidationFailedException();
        }

        #endregion

        #region 应用层协议

        /// <summary>
        /// 开始同步模式。
        /// <para>同步模式下，发送的指令不会被执行，同步模式结束后立即执行。</para>
        /// <para>此模式可用于多电机需要同时完成不同动作的场景。</para>
        /// </summary>
        public void BeginSync()
        {
            if (MoveSync)
            {
                throw new OperationFailedException("One synced session allowed at the sametime. 同时允许一个同步会话。");
            }
            MoveSync = true;
        }

        /// <summary>
        /// 退出同步模式，丢弃已缓存的指令。
        /// <para>同步模式中发送的指令被丢弃，不执行。</para>
        /// </summary>
        public void DisableSync()
        {
            MoveSync = false;
        }

        /// <summary>
        /// 结束同步模式，开始执行期间发送的指令。
        /// </summary>
        public void EndSync()
        {
            if (!MoveSync)
            {
                throw new OperationFailedException("Not in a synced session. 当前不在同步会话中，无法操作。");
            }
            MoveSync = false;
            Sync();
        }

        internal Pack? HandleReply(byte addr, Command cmd, bool overwrite_wait_for_result = false)
        {
            if ((!overwrite_wait_for_result) && (!WaitForAck))
                return null;
            var pk = ReceivePack(out byte raddr);
            if (pk.Payload[2] == 0xEE)
                throw new ValidationFailedException();
            if (raddr != addr)
                throw new UnexpectedBehaviorException();
            if (pk.Verification == VerificationType.Failed)
                throw new ValidationFailedException();
            if (pk.Command != cmd)
                throw new UnexpectedBehaviorException();
            if (pk.Payload[2] == 0xE2)
                throw new OperationFailedException();
            return pk;
        }

        /// <summary>
        /// 使能/关闭驱动器
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        /// <param name="enable">使能</param>
        public void DriverEnable(byte addr = 0, bool enable = true)
        {
            SendPack(addr, new Pack
            {
                Command = Command.PowerCtrl,
                Payload = new byte[] {
                    0xAB,
                    (byte)(enable ? 0x01 : 0x00),
                    (byte)(MoveSync ? 0x01 : 0x00)
                },
                Verification = VerificationType
            });
            HandleReply(addr, Command.PowerCtrl);
        }

        /// <summary>
        /// 使用固定转速模式转动电机
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        /// <param name="rpm">-5000~+5000 转速和方向</param>
        /// <param name="acc">0~255 加速度</param>
        public void SetMotorSpeed(byte addr, int rpm, byte acc)
        {
            var absspeed = Math.Abs(rpm);
            SendPack(addr, new Pack
            {
                Command = Command.SetSpeed,
                Payload = new byte[] {
                    (byte)(rpm > 0 ? 0x00 : 0x01),

                    (byte)((Math.Abs(absspeed) >> 8) & 0xFF),
                    (byte)(Math.Abs(absspeed) & 0xFF),

                    (byte)(acc & 0xFF),
                    (byte)(MoveSync ? 0x01 : 0x00)
                },
                Verification = VerificationType
            });
            HandleReply(addr, Command.SetSpeed);
        }

        /// <summary>
        /// 使用目标位置模式转动电机
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        /// <param name="rpm">0~5000 转读</param>
        /// <param name="acc">0~255 加速度</param>
        /// <param name="target">转动目标位置和方向</param>
        /// <param name="absolute">绝对位置/相对位置模式</param>
        public void SetMotorPosition(byte addr, uint rpm, byte acc, int target, bool absolute = true)
        {
            var targetabs = Math.Abs(target);
            SendPack(addr, new Pack
            {
                Command = Command.SetPosition,
                Payload = new byte[] {
                    (byte)(target > 0 ? 0x00 : 0x01),

                    (byte)((rpm >> 8) & 0xFF),
                    (byte)(rpm & 0xFF),

                    (byte)(acc & 0xFF),

                    (byte)((targetabs >> 8*3) & 0xFF),
                    (byte)((targetabs >> 8*2) & 0xFF),
                    (byte)((targetabs >> 8*1) & 0xFF),
                    (byte)(targetabs & 0xFF),

                    (byte)(absolute ? 0x01 : 0x00),
                    (byte)(MoveSync ? 0x01 : 0x00)
                },
                Verification = VerificationType
            });
            HandleReply(addr, Command.SetSpeed);
        }

        /// <summary>
        /// 指令电机刹车
        /// <para>闭环状态下，电机如果因惯性转动超过了刹车位置，会反转修正回来。</para>
        /// <para>Motor will rotate back at closed-loop mode when it overshots.</para>
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        public void Brake(byte addr = 0)
        {
            SendPack(addr, new Pack
            {
                Command = Command.Brake,
                Payload = new byte[] {
                    0x98,
                    (byte)(MoveSync ? 0x01 : 0x00)
                },
                Verification = VerificationType
            });
            HandleReply(addr, Command.SetSpeed);
        }

        /// <summary>
        /// 多电机同步运动开始
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        public void Sync(byte addr = 0)
        {
            SendPack(addr, new Pack
            {
                Command = Command.Sync,
                Payload = new byte[] {
                    0x66,
                    (byte)(MoveSync ? 0x01 : 0x00)
                },
                Verification = VerificationType
            });
            HandleReply(addr, Command.SetSpeed);
        }

        /// <summary>
        /// 将当前位置设置为单圈0°
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        /// <param name="mem">保存更改，掉电不丢失</param>
        public void SetZeroDegree(byte addr = 0, bool mem = true)
        {
            SendPack(addr, new Pack
            {
                Command = Command.SetZeroDegree,
                Payload = new byte[] {
                    0x88,
                    (byte)(mem ? 01 : 00),
                    (byte)(MoveSync ? 0x01 : 0x00)
                },
                Verification = VerificationType
            });
            HandleReply(addr, Command.SetSpeed);
        }

        /// <summary>
        /// 开始回零
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        /// <param name="type">回零指令类型</param>
        /// <param name="direction">旋转方向(仅对于SingleRotateDirected有效)</param>
        public void StartHomming(byte addr = 0, HommingType type = HommingType.SingleRotateClosest,
            RotateDirection direction = RotateDirection.CW)
        {
            SendPack(addr, new Pack
            {
                Command = Command.GoHome,
                Payload = new byte[] {
                    (byte)type,
                    (byte)direction
                },
                Verification = VerificationType
            });
            HandleReply(addr, Command.SetSpeed);
        }

        /// <summary>
        /// 立即退出正在执行的回零操作
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        public void AbortHoming(byte addr = 0)
        {
            SendPack(addr, new Pack
            {
                Command = Command.GoHome,
                Payload = new byte[] {
                    0x48
                },
                Verification = VerificationType
            });
            HandleReply(addr, Command.SetSpeed);
        }

        /// <summary>
        /// 获取电机回零设置
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        /// <returns></returns>
        public HomingConfig GetHomingConfig(byte addr = 1)
        {
            SendPack(addr, new Pack
            {
                Command = Command.GoHome,
                Payload = new byte[] {
                    0x22
                },
                Verification = VerificationType
            });
            var pk = HandleReply(addr, Command.GoHome, true);
            var payload = pk!.Value.Payload;
            return new HomingConfig()
            {
                Type = (HommingType)payload[0],
                RotateDirection = (RotateDirection)payload[1],
                RPM = (payload[2] << 8 + payload[3]),
                TimeOutMs = (payload[4] << 8 * 3)
                             + (payload[5] << 8 * 2)
                             + (payload[6] << 8 * 1)
                             + (payload[7]),
                CollisionThresholdRPM = (payload[8] << 8 + payload[9]),
                CollisionThresholdCurrent = (payload[10] << 8 + payload[11]),
                CollisionThresholdTime = (payload[12] << 8 + payload[13]),
                PowerOnHome = payload[14] == 0x01
            };
        }

        public void SetHomingConfig(byte addr = 0, bool mem = true)
        {
            SendPack(addr, new Pack
            {
                Command = Command.SetHomingConfig,
                Payload = new byte[] {
                    0xAE,
                    (byte)(mem?0x01:0x00),

                },
                Verification = VerificationType
            });
        }

        #endregion
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
        ReadHomingSettings = 0x22,
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
        GetCLTargetPos = 0x33,
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

    public struct Pack
    {
        public Command Command;
        public byte[] Payload;
        public VerificationType Verification;
    }
}
