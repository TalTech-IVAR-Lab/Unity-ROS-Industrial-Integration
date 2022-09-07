namespace EE.TalTech.IVAR.Robotics.ROSIndustrial.Common
{
    using System;

    /// <summary>
    /// Helper base class which can be used to get the names of the error codes based on their number.
    /// </summary>
    /// <typeparam name="TCodeEnum">Enum matching the error code names with their numbers.</typeparam>
    public class StatusCodeBase<TCodeEnum> where TCodeEnum : Enum
    {
        /// <summary>
        /// Integer status code.
        /// </summary>
        public readonly int code;
        /// <summary>
        /// Name of the status code.
        /// </summary>
        public readonly string codeName;

        public StatusCodeBase(int code)
        {
            this.code = code;
            codeName = Enum.GetName(typeof(TCodeEnum), code);
        }

        public override string ToString() { return codeName; }
    }
}