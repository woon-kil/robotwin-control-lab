using System.IO;
using NUnit.Framework;
using Robotwin.Ur10e.Editor;
using UnityEngine;

namespace Robotwin.Ur10e.Tests
{
    public sealed class Ur10eTcpFixAttempt2Tests
    {
        [Test]
        [Category("Ur10eTcpFixAttempt2")]
        public void RecordsFixAttempt2WithoutTouchingOriginalEvidence()
        {
            var path = Path.GetFullPath(Path.Combine(Application.dataPath, "..", "..", "..", "evidence", "P-004", "fix-attempt2", "raw", "unity_results.json"));
            Ur10eTcpRunner.Run(Path.GetDirectoryName(path));
            Assert.IsTrue(File.Exists(path), path);
            var text = File.ReadAllText(path);
            Assert.IsTrue(text.Contains("\"source\": \"unity\""));
            Assert.IsTrue(text.Contains("\"test_id\": \"A-zero\""));
            Assert.IsTrue(text.Contains("\"test_id\": \"E-fixed-3\""));
            Assert.IsFalse(text.Contains("evidence/P-002"));
        }
    }
}
