using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ZDTStepperDriver.Exceptions
{
    /// <summary>
    /// A package with abnormal data detected.
    /// <para>检测到数据错误的包。</para>
    /// <para>Makesure using the correct protocol version, check hardware connection, or lower the badurate.</para>
    /// <para>确保协议版本正确，检查硬件连接，降低波特率。</para>
    /// </summary>
    public class ValidationFailedException : Exception
    { }
}
