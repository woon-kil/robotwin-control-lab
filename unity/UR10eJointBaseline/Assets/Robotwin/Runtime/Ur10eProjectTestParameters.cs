namespace Robotwin.Ur10e
{
    // URDF source에 없는 값이다. 산업 사양으로 읽지 않는다.
    public static class Ur10eProjectTestParameters
    {
        public const float DriveStiffness = 1000000f;
        public const float DriveDamping = 2000f;
        public const int SolverIterations = 24;
        public const int SolverVelocityIterations = 1;
        public const float TimeoutSeconds = 8f;
        public const int SettledFramesRequired = 20;
        public const double VelocityEpsilonRadPerSec = 0.001;
        public const double PositionDeltaEpsilonRad = 1e-5;
        public const double EngineeringTargetDegrees = 0.1;
        public const bool DisableCollidersDuringMeasurement = true;

        public static double EngineeringTargetRadians =>
            EngineeringTargetDegrees * System.Math.PI / 180.0;
    }
}
