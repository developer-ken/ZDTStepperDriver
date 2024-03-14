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
                Command = Command.ReadHomingConfig,
                Payload = new byte[] {
                    0x22
                },
                Verification = VerificationType
            });
            var pk = HandleReply(addr, Command.ReadHomingConfig, true);
            var payload = pk!.Value.Payload;
            return new HomingConfig
            {
                Type = (HommingType)payload[0],
                RotateDirection = (RotateDirection)payload[1],
                RPM = (payload[2] << 8 + payload[3]),
                TimeOutMs = (payload[4] << 24)
                             + (payload[5] << 16)
                             + (payload[6] << 8)
                             + (payload[7]),
                CollisionThresholdRPM = (payload[8] << 8 + payload[9]),
                CollisionThresholdCurrent = (payload[10] << 8 + payload[11]),
                CollisionThresholdTime = (payload[12] << 8 + payload[13]),
                PowerOnHome = payload[14] == 0x01
            };
        }

        /// <summary>
        /// 更改回零设置
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        /// <param name="config">新的回零设置</param>
        /// <param name="mem">保存更改，掉电不丢失</param>
        public void SetHomingConfig(byte addr, HomingConfig config, bool mem = true)
        {
            SendPack(addr, new Pack
            {
                Command = Command.SetHomingConfig,
                Payload = new byte[] {
                    0xAE,
                    (byte)(mem?0x01:0x00),
                    (byte)config.Type,
                    (byte)config.RotateDirection,

                    (byte)(config.RPM>>8),
                    (byte)(config.RPM&0xFF),

                    (byte)(config.TimeOutMs>>24),
                    (byte)(config.TimeOutMs>>16),
                    (byte)(config.TimeOutMs>>8),
                    (byte)(config.TimeOutMs&0xFF),

                    (byte)(config.CollisionThresholdRPM>>8),
                    (byte)(config.CollisionThresholdRPM&0xFF),

                    (byte)(config.CollisionThresholdCurrent>>8),
                    (byte)(config.CollisionThresholdCurrent&0xFF),

                    (byte)(config.CollisionThresholdTime>>8),
                    (byte)(config.CollisionThresholdTime&0xFF),

                    (byte)(config.PowerOnHome?0x01:0x00)
                },
                Verification = VerificationType
            });

            HandleReply(addr, Command.SetHomingConfig);
        }

        /// <summary>
        /// 获取回零状态
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        /// <returns></returns>
        public HomingStatus GetHomingStatus(byte addr)
        {
            SendPack(addr, new Pack
            {
                Command = Command.ReadHomingStatus,
                Payload = new byte[] { },
                Verification = VerificationType
            });
            var pk = HandleReply(addr, Command.ReadHomingStatus, true);
            var payload = pk!.Value.Payload;
            return new HomingStatus
            {
                EncoderOK = (payload[0] & 0x01) == 0x01,
                CalibrationTableOK = (payload[0] & 0x02) == 0x01,
                Homing = (payload[0] & 0x04) == 0x01,
                HomingFailed = (payload[0] & 0x08) == 0x01,
            };
        }

        /// <summary>
        /// 校准编码器
        /// <para>此操作会转动电机</para>
        /// <para>Motor will rotate!</para>
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        public void CalibrateEncoder(byte addr = 0)
        {
            SendPack(addr, new Pack
            {
                Command = Command.CalibrateEncoder,
                Payload = new byte[] {
                    0x45
                },
                Verification = VerificationType
            });
            HandleReply(addr, Command.CalibrateEncoder);
        }

        /// <summary>
        /// 归零脉冲计数器
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        public void ZeroPulseCounter(byte addr = 0)
        {
            SendPack(addr, new Pack
            {
                Command = Command.ZeroPulseCounter,
                Payload = new byte[] {
                    0x6D
                },
                Verification = VerificationType
            });
            HandleReply(addr, Command.ZeroPulseCounter);
        }

        /// <summary>
        /// 堵转保护复位
        /// <para>使驱动恢复正常状态</para>
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        public void ClogProtectionReset(byte addr = 0)
        {
            SendPack(addr, new Pack
            {
                Command = Command.ClogProtectionReset,
                Payload = new byte[] {
                    0x6D
                },
                Verification = VerificationType
            });
            HandleReply(addr, Command.ClogProtectionReset);
        }

        /// <summary>
        /// 将驱动器恢复出厂设置
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        public void FactoryReset(byte addr = 0)
        {
            SendPack(addr, new Pack
            {
                Command = Command.FactoryReset,
                Payload = new byte[] {
                    0x5F
                },
                Verification = VerificationType
            });
            HandleReply(addr, Command.FactoryReset);
        }

        /// <summary>
        /// 获取驱动器版本信息
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        /// <returns></returns>
        public Models.Version GetDriverVersion(byte addr = 0)
        {
            SendPack(addr, new Pack
            {
                Command = Command.GetDriverVersion,
                Payload = new byte[] { },
                Verification = VerificationType
            });
            var pk = HandleReply(addr, Command.GetDriverVersion, true);
            var payload = pk!.Value.Payload;
            return new Models.Version
            {
                Hardware = payload[0],
                Firmware = payload[1]
            };
        }

        /// <summary>
        /// 获取电机特性参数
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        /// <returns></returns>
        public MotorCharacteristics GetMotorCharacteristics(byte addr)
        {
            SendPack(addr, new Pack
            {
                Command = Command.ReadMotorCharacteristic,
                Payload = new byte[] { },
                Verification = VerificationType
            });
            var pk = HandleReply(addr, Command.ReadMotorCharacteristic, true);
            var payload = pk!.Value.Payload;
            return new MotorCharacteristics
            {
                PhaseResisitance = (ushort)(payload[0] << 8 + payload[1]),
                PhaseInductance = (ushort)(payload[2] << 8 + payload[3]),
            };
        }

        /// <summary>
        /// 获取位置环PID参数
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        /// <returns></returns>
        public PIDParameters GetPosPIDParameters(byte addr)
        {
            SendPack(addr, new Pack
            {
                Command = Command.ReadPosPIDParameters,
                Payload = new byte[] { },
                Verification = VerificationType
            });
            var pk = HandleReply(addr, Command.ReadPosPIDParameters, true);
            var payload = pk!.Value.Payload;
            return new PIDParameters
            {
                Kp = (payload[0] << 24) +
                     (payload[1] << 16) +
                     (payload[2] << 8) +
                     (payload[3]),
                Ki = (payload[4] << 24) +
                     (payload[5] << 16) +
                     (payload[6] << 8) +
                     (payload[7]),
                Kd = (payload[8] << 24) +
                     (payload[9] << 16) +
                     (payload[10] << 8) +
                     (payload[11])
            };
        }

        /// <summary>
        /// 获取电源轨电压
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        /// <returns>电压/V</returns>
        public float GetRailVotage(byte addr)
        {
            SendPack(addr, new Pack
            {
                Command = Command.GetRailVotage,
                Payload = new byte[] { },
                Verification = VerificationType
            });
            var pk = HandleReply(addr, Command.GetRailVotage, true);
            var payload = pk!.Value.Payload;
            return ((payload[0] << 8) + payload[1]) / 1000f;
        }

        /// <summary>
        /// 获取相电流
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        /// <returns>电流/A</returns>
        public float GetPhaseCurrent(byte addr)
        {
            SendPack(addr, new Pack
            {
                Command = Command.GetPhaseCurrent,
                Payload = new byte[] { },
                Verification = VerificationType
            });
            var pk = HandleReply(addr, Command.GetPhaseCurrent, true);
            var payload = pk!.Value.Payload;
            return ((payload[0] << 8) + payload[1]) / 1000f;
        }

        /// <summary>
        /// 获取编码器读数
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        /// <returns></returns>
        public ushort GetEncoderReading(byte addr)
        {
            SendPack(addr, new Pack
            {
                Command = Command.GetEncoderReading,
                Payload = new byte[] { },
                Verification = VerificationType
            });
            var pk = HandleReply(addr, Command.GetEncoderReading, true);
            var payload = pk!.Value.Payload;
            return (ushort)((payload[0] << 8) + payload[1]);
        }

        /// <summary>
        /// 获取脉冲数
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        /// <returns></returns>
        public int GetPulses(byte addr)
        {
            SendPack(addr, new Pack
            {
                Command = Command.GetEncoderReading,
                Payload = new byte[] { },
                Verification = VerificationType
            });
            var pk = HandleReply(addr, Command.GetEncoderReading, true);
            var payload = pk!.Value.Payload;
            return (payload[0] == 0x00 ? 1 : -1) * (
                (payload[1] << 24) +
                (payload[2] << 16) +
                (payload[3] << 8) +
                (payload[4])
                );
        }

        /// <summary>
        /// 获取目标位置
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        /// <returns></returns>
        public int GetTargetPos(byte addr)
        {
            SendPack(addr, new Pack
            {
                Command = Command.GetTargetPos,
                Payload = new byte[] { },
                Verification = VerificationType
            });
            var pk = HandleReply(addr, Command.GetTargetPos, true);
            var payload = pk!.Value.Payload;
            return (payload[0] == 0x00 ? 1 : -1) * (
                (payload[1] << 24) +
                (payload[2] << 16) +
                (payload[3] << 8) +
                (payload[4])
                );
        }

        /// <summary>
        /// 获取开环模式下一个旋转目标位置
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        /// <returns></returns>
        public int GetOLTargetPos(byte addr)
        {
            SendPack(addr, new Pack
            {
                Command = Command.GetOLTargetPos,
                Payload = new byte[] { },
                Verification = VerificationType
            });
            var pk = HandleReply(addr, Command.GetOLTargetPos, true);
            var payload = pk!.Value.Payload;
            return (payload[0] == 0x00 ? 1 : -1) * (
                (payload[1] << 24) +
                (payload[2] << 16) +
                (payload[3] << 8) +
                (payload[4])
                );
        }

        /// <summary>
        /// 获取当前转速
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        /// <returns></returns>
        public int GetRPM(byte addr)
        {
            SendPack(addr, new Pack
            {
                Command = Command.GetRPM,
                Payload = new byte[] { },
                Verification = VerificationType
            });
            var pk = HandleReply(addr, Command.GetRPM, true);
            var payload = pk!.Value.Payload;
            return (payload[0] == 0x00 ? 1 : -1) * (
                (payload[1] << 8) +
                (payload[2])
                );
        }

        /// <summary>
        /// 获取当前位置
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        /// <returns></returns>
        public int GetCurrentPos(byte addr)
        {
            SendPack(addr, new Pack
            {
                Command = Command.GetCurrentPos,
                Payload = new byte[] { },
                Verification = VerificationType
            });
            var pk = HandleReply(addr, Command.GetCurrentPos, true);
            var payload = pk!.Value.Payload;
            return (payload[0] == 0x00 ? 1 : -1) * (
                (payload[1] << 24) +
                (payload[2] << 16) +
                (payload[3] << 8) +
                (payload[4])
                );
        }

        /// <summary>
        /// 获取当前位置到目标位置的差值
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        /// <returns></returns>
        public int GetPosError(byte addr)
        {
            SendPack(addr, new Pack
            {
                Command = Command.GetPosError,
                Payload = new byte[] { },
                Verification = VerificationType
            });
            var pk = HandleReply(addr, Command.GetPosError, true);
            var payload = pk!.Value.Payload;
            return (payload[0] == 0x00 ? 1 : -1) * (
                (payload[1] << 24) +
                (payload[2] << 16) +
                (payload[3] << 8) +
                (payload[4])
                );
        }

        /// <summary>
        /// 获取电机状态
        /// </summary>
        /// <param name="addr">驱动器地址</param>
        /// <returns></returns>
        public MotorStatus GetMotorStatus(byte addr)
        {
            SendPack(addr, new Pack
            {
                Command = Command.GetMotorStatus,
                Payload = new byte[] { },
                Verification = VerificationType
            });
            var pk = HandleReply(addr, Command.GetMotorStatus, true);
            var payload = pk!.Value.Payload;
            return new MotorStatus
            {
                Enabled = (payload[0] & 0x01) == 0x01,
                AtTarget = (payload[1] & 0x02) == 0x01,
                Clogging = (payload[2] & 0x04) == 0x01,
                ClogProtecting = (payload[3] & 0x08) == 0x01
            };
        }



        #endregion
    }

    public struct Pack
    {
        public Command Command;
        public byte[] Payload;
        public VerificationType Verification;
    }
}
