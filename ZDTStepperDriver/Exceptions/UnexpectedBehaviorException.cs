using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ZDTStepperDriver.Exceptions
{
    /// <summary>
    /// Motor driver sent an reply complies with the protocol, but does not fit in the context.
    /// <para>驱动器传回了协议正确的包，但与上下文矛盾。</para>
    /// <para>This lib is not thread-safe! Do not call it multi-threadly.</para>
    /// <para>此库不是线程安全的，不能被多线程调用。</para>
    /// </summary>
    public class UnexpectedBehaviorException : Exception
    { }
}
