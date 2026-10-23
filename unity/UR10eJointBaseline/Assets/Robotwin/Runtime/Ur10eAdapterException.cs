using System;

namespace Robotwin.Ur10e
{
    public sealed class Ur10eAdapterException : Exception
    {
        public Ur10eAdapterException(string code, string message) : base(message)
        {
            Code = code;
        }

        public string Code { get; }
    }
}
