using System;
using System.Collections.Generic;
using System.Linq;
using System.Reflection.Metadata;
using System.Text;
using System.Threading.Tasks;

namespace ZDTStepperDriver.Exceptions
{
    /// <summary>
    /// The current state of the driver won't allow this operation to be performed.
    /// <para>当前驱动器状态无法执行此操作</para>
    /// </summary>
    public class OperationFailedException : Exception
    {
        public OperationFailedException() : base() { }
        public OperationFailedException(string message) : base(message) { }
    }
}
