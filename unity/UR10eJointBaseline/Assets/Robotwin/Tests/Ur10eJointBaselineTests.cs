using NUnit.Framework;
using Robotwin.Ur10e;
using Robotwin.Ur10e.Editor;

namespace Robotwin.Ur10e.Tests
{
    public sealed class Ur10eJointBaselineTests
    {
        [Test]
        [Category("Ur10eJointBaseline")]
        public void JointCommandsSettleWithinEngineeringTarget()
        {
            var result = Ur10eJointBaselineRunner.Run();
            Assert.AreEqual("2022.3.62f2", result.UnityVersion);
            Assert.GreaterOrEqual(result.ConfigurationCount, 5);
            var underTarget = result.GlobalMaxJointErrorRad < Ur10eProjectTestParameters.EngineeringTargetRadians;
            if (underTarget && result.TimeoutCount == 0)
            {
                Assert.AreEqual("PASS", result.Result, result.FailureSummary);
            }
            else
            {
                Assert.AreEqual("FAIL", result.Result, result.FailureSummary);
            }
        }
    }
}
