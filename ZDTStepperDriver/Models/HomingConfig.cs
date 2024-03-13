using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ZDTStepperDriver.Models
{
    public struct HomingConfig
    {
        // 回零类型
        public HommingType Type;
        // 回零参考旋转方向
        public RotateDirection RotateDirection;

        public int
            //转速
            RPM,
            //超时
            TimeOutMs,
            //碰撞检测转速阈值
            CollisionThresholdRPM,
            //碰撞检测电流阈值
            CollisionThresholdCurrent,
            //碰撞检测时间阈值
            CollisionThresholdTime;
        // 上电自动开始回零
        public bool PowerOnHome;
    }
}
