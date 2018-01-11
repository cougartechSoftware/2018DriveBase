package org.usfirst.frc.team2228.robot;

public class SRXDriveBaseCfg {
	// configuration flags
		public static boolean isDriveRightMasterMtrReversed = true;
		public static boolean isDriveRightFollowerMtrReversed = true;
		public static boolean isDriveLeftMasterMtrReversed = false;
		public static boolean isDriveLeftFollowerMtrReversed = false;
		
		public static boolean isBrakeEnabled = false;
		
		public static double kDriveStraightCorrection = 35.0/30.0;
		
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
		public static int kdriveRightMstrProfile = 0;
		
		public static double kdriveleftMstrIzone = 0;
		public static int kdriveLeftMstrRampRate = 0;
		public static int kdriveLeftMstrProfile = 0;
		
		public static int kClosedLoopErr = 100;
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
		// AndyMark tough box mini 14:50 to 16:48
		public static double kGearBoxRatio = (50.0/14.0)*(48.0/16.0);
		public static double kCountsPerRevolution = 80.0 * kGearBoxRatio;

		// CTRE CIMcode magnetic quadrature 20 pulse per revolution
		public static int DRIVE_RIGHT_ENCODER_CNTS_PER_REV = 20 * (int)kGearBoxRatio;
		public static int DRIVE_LEFT_ENCODER_CNTS_PER_REV = 20 * (int)kGearBoxRatio;
		
		// inches per revolution / counts per revolution
		public static double kRgtInchesPerCount = kCalibratedRgtWheelCircum/kCountsPerRevolution;
		public static double kLftInchesPerCount = kCalibratedRgtWheelCircum/kCountsPerRevolution;
        public static double kSpeedDeadBand = 0.1;
        
        public static double kTurnValueDeadBand = 0;
        public static double kCorrection_Kp = 0;
        public static double kCorrection_Ki = 0;
        public static double kCorrection_Kd = 0;
        
        public static double kStallCurrent = 16.0;
        public static double kStallTimeSec = 3.0;
        public static double kRobotCoastToStopCounts = 0;
        public static double kMoveToPositionVelCmdLevel = 0.3;
        
        public static double kDrivePerpendicularCmdLevel = 0;
        public static double kTrackWidthIn = 0;
        public static double kturnCmdLevel = 0;
        public static double kSquareWaveLowerSpeed = 0;
        public static double kSquareWaveHigherSpeed = 0;
}

/*
DriveTrain object
six wheel tank drive with differental control(4 motor) using SRX ESC modules

- base motion control command -left/right level

- when object is created, it can configure in levels
	(all levels have stall protection software)
    1. sets motors up as master/follower for left/right side
	2. Cmdlevel as a percentage of the bus voltage
	3. All levels have stall protection software
	Sensor levels
	1. 1 with encoders
	2. feedback control(speed level command) which requires encoders
	2. 1 with encoders and heading angle modules (drive staight)
	3. 1 with encoders, heading angle module, and ultrasonic module (drive straight and perpendicular
	
issue: I am missing ultrasonic class
	   
- status/logging
====================================================================
-live window: 4 drives

-returns: position, speed, bus voltage, current, close loop error, rate, pitch, roll, yaw, distance left/right

-smartscreen: update by command
	position, speed, bus voltage, current, close loop error, rate, pitch, roll, yaw, distance left/right

-logging data: update by command
	date, position, speed, bus voltage, current, close loop error, yaw, distance
	   
-teleop commands: (base input- left/right command level)
================================================================

1. tank drive: power/speed level command (SetDriveTrainCmdLevel)
	******NOTE: THIS IS THE ONLY COMMAND NEEDED - all auto/tele commands can be done by user externally except
	            for magic motion and motion profiling
				
2. wpi chessy-throttle/turn: power command (WPISetThrottleTurn)

3. team 2228 chessy-throttle/turn: power/speed command(setThrottleTurn)
		+ drive straight assist (AHRS angle + drive straight correction factor)
		+ drive to wall perpendicular (ultrasonic + drive straight correction factor)
		
4. defense escape(reverse and turn robot 180 degrees left or right) TO DO

-autonomous commands:
==========================================================================
1 velMoveToPosition(double moveInches)
       move straight to position: power/speed/accel/ramp rate/coast distance - configured; distance by user
		NOTE: stopping is by coasting with regen braking on motors. If speed and ramp is used we
		      could get a shorter stopping distance

2. move perpendicular to stop: power/speed/accel ramp rate/coast distance+distance from wall - configured

3. rotate to angle: power/speed/accel ramp rate - configured; angle by user

4. turn to angle: power/speed/accel ramp rate - configured; angle by user

5. Motion Magic: user does calculations

6. Motion Magic move: drive base object does calculations

7. SRX profiling: TO DO

- testing commands
===================================================================
1. squarewave for determining PID parameters

2. AutoTuning using twiddle algorithm: TODO

3. driving straight for base drive straight correction factor
*/
