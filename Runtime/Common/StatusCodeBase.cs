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
        public readonly TCodeEnum value;
        /// <summary>
        /// Name of the status code.
        /// </summary>
        public readonly string name;

        public StatusCodeBase(int value)
        {
            this.value = (TCodeEnum)(object)value;
            name = Enum.GetName(typeof(TCodeEnum), value);
        }

        public override string ToString() { return name; }
    }
}