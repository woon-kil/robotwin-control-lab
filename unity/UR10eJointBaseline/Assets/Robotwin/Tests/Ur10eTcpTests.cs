using System.IO;
using NUnit.Framework;
using Robotwin.Ur10e.Editor;
using UnityEngine;

namespace Robotwin.Ur10e.Tests
{
    public sealed class Ur10eTcpTests
    {
        [Test]
        [Category("Ur10eTcp")]
        public void RecordsBaseAndTool0InUnityWorld()
        {
            Ur10eTcpRunner.Run();
            var path = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "..", "evidence", "P-004", "raw", "unity_results.json"));
            Assert.IsTrue(File.Exists(path), path);
            var text = File.ReadAllText(path);
            Assert.IsTrue(text.Contains("\"source\": \"unity\""));
            Assert.IsTrue(text.Contains("\"test_id\": \"A-zero\""));
            Assert.IsTrue(text.Contains("\"test_id\": \"E-fixed-3\""));
            Assert.IsTrue(text.Contains("\"base_pose\""));
            Assert.IsTrue(text.Contains("\"tool0_pose\""));
            Assert.IsFalse(text.Contains("evidence/P-002"));
        }
    }
}
