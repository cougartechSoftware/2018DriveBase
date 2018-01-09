package org.usfirst.frc.team2228.robot;

public class SRXDriveBaseCfg {
	// configuration flags
		public static boolean isDriveRightMasterMtrReversed = true;
		public static boolean isDriveRightFollowerMtrReversed = true;
		public static boolean isDriveLeftMasterMtrReversed = false;
		public static boolean isDriveLeftFollowerMtrReversed = false;
		
		public static boolean isBrakeEnabled = false;
		
		public static double kDriveStraightCorrection = 1.0;
		
		public static double kRotatePowerLevel = 0.20;
		
		public static boolean isMasterEncodersPresent = false;
		public static boolean isRightEncoderSensorReversed = false;
		public static boolean isLeftEncoderSensorReversed = false;
		
		public static boolean isIMUEnabled = false;
		public static boolean isDriveStraightAssistEnabled = false;
		public static boolean isHeadingModuleEnabled = false;
		
		public static boolean isPIDEnabled = false;
		public static boolean isSRXClosedLoopEnabled = false;
		
		public static double kdriveRightMstrFeedForwardGain = 0.025;
		public static double kdriveRightMstrProportionalGain = 0.3;
		public static double kdriveRightMstrIntegralGain = 0;
		public static double kdriveRightMstrDerivativeGain = 0;
		
		public static double kdriveLeftMstrFeedForwardGain = 0.025;
		public static double kdriveLeftMstrProportionalGain = 0.3;
		public static double kdriveLeftMstrIntegralGain = 0;
		public static double kdriveLeftMstrDerivativeGain = 0;
		
		public static double kdriveRightMstrIzone = 0;
		public static int kdriveRightMstrRampRate = 0;
		public static double kdriveRightMstrProfile = 0;
		
		public static double kdriveleftMstrIzone = 0;
		public static int kdriveLeftMstrRampRate = 0;
		public static double kdriveLeftMstrProfile = 0;
		
		public static double kClosedLoopErr = 100;
		public static double kSRXVelocitySample = 16;
		public static double kTopRPM = 1000;
		public static boolean isLowTimeActive = true;
		public static boolean isSqWaveFnctStartActive = false;
		
		// magic motion
		public static double kWheelDiameter = 6.0;
		public static double kRgtDistanceCalibration = 1.0;
		public static double kLftDistanceCalibration = 1.0;
		public static double kCalibratedRgtWheelCircum = kWheelDiameter*Math.PI;
		public static double kCalibratedLftWheelCircum = kWheelDiameter*Math.PI;
		// AndyMark tough gox mini 14:50 to 16:48
		public static double kGearBoxRatio = (50.0/14.0)*(48.0/16.0);
		public static double kCountsPerRevolution = 80.0 * kGearBoxRatio;

		// CTRE CIMcode magnetic quadrature 20 pulse per revolution
		public static int DRIVE_RIGHT_ENCODER_CNTS_PER_REV = 20 * (int)kGearBoxRatio;
		public static int DRIVE_LEFT_ENCODER_CNTS_PER_REV = 20 * (int)kGearBoxRatio;
		
		// inches per revolution / counts per revolution
		public static double kRgtInchesPerCount = kCalibratedRgtWheelCircum/kCountsPerRevolution;
		public static double kLftInchesPerCount = kCalibratedRgtWheelCircum/kCountsPerRevolution;
        public static double kSpeedDeadBand = 0.1;
}