package org.usfirst.frc.team2228.robot;
/**
* Class SRXBaseDrive
* RELEASE: 1, RevA 
* Team 2228 / RJV
*
*
*/ 
/* ===================================
 * REVISIONS:
 * Release 1
 * RevA: original
 */

//Carrying over the classes from other libraries
import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;
//import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SRXDriveBase {
	
	// cheesy or tank motors
	private CANTalon driveRightMasterMtr;
	private CANTalon driveRightFollowerMtr;
	private CANTalon driveLeftMasterMtr;
	private CANTalon driveLeftFollowerMtr;
	
	private RobotDrive driveStyle;
	
	private boolean isDriveMoving;
	private double rightDrvTrainTargetPosSetPt;
	private double leftDrvTrainTargetPosSetPt;
	
		
	// SRXDriveBase Class Constructor
	public SRXDriveBase() {
		
		// Create CAN SRX motor controller objects
		driveRightMasterMtr = new CANTalon(RobotMap.CAN_ID_1);
		driveRightFollowerMtr = new CANTalon(RobotMap.CAN_ID_2);
		driveLeftMasterMtr = new CANTalon(RobotMap.CAN_ID_3);
		driveLeftFollowerMtr = new CANTalon(RobotMap.CAN_ID_4);
		
		LiveWindow.addActuator("rtM", "RightMaster", driveRightMasterMtr);
		LiveWindow.addActuator("rtF", "RightFollower", driveRightFollowerMtr);
		LiveWindow.addActuator("lftM", "LeftMaster", driveLeftMasterMtr);
		LiveWindow.addActuator("lftF", "LeftFollower", driveLeftFollowerMtr);
		
		isDriveMoving = false;
		
		/*  Set right/left masters and right/left followers 
		*/
		// Set Right master to speed mode
		driveRightMasterMtr.changeControlMode(TalonControlMode.PercentVbus);
		driveRightMasterMtr.enableControl(); 
		driveRightMasterMtr.set(0);
		
		// Set up right follower
		driveRightFollowerMtr.changeControlMode(TalonControlMode.Follower);
		
		// Set follower motor to follow master
		driveRightFollowerMtr.set(driveRightMasterMtr.getDeviceID());
		driveRightFollowerMtr.enableControl(); 

		// Set left master to speed mode
		driveLeftMasterMtr.changeControlMode(TalonControlMode.PercentVbus);
		driveLeftMasterMtr.enableControl();
		driveLeftMasterMtr.set(0);
		
		// Set up left follower
		driveLeftFollowerMtr.changeControlMode(TalonControlMode.Follower);
		
		// Set follower motor to follow master
		driveLeftFollowerMtr.set(driveLeftMasterMtr.getDeviceID());
		driveLeftFollowerMtr.enableControl(); 
		
		driveRightMasterMtr.setInverted(SRXDriveBaseCfg.isDriveRightMasterMtrReversed);
		driveRightFollowerMtr.setInverted(SRXDriveBaseCfg.isDriveRightFollowerMtrReversed);
		driveLeftMasterMtr.setInverted(SRXDriveBaseCfg.isDriveLeftMasterMtrReversed);
		driveLeftFollowerMtr.setInverted(SRXDriveBaseCfg.isDriveLeftFollowerMtrReversed);

		// Set peak and nominal output voltage levels of motor controllers
		driveRightMasterMtr.configNominalOutputVoltage(+0.0f, -0.0f);
		driveRightMasterMtr.configPeakOutputVoltage(+12.0f, -12.0f);
		driveRightFollowerMtr.configNominalOutputVoltage(+0.0f, -0.0f);
		driveRightFollowerMtr.configPeakOutputVoltage(+12.0f, -12.0f);
		driveLeftMasterMtr.configNominalOutputVoltage(+0.0f, -0.0f);
		driveLeftMasterMtr.configPeakOutputVoltage(+12.0f, -12.0f);
		driveLeftFollowerMtr.configNominalOutputVoltage(+0.0f, -0.0f);
		driveLeftFollowerMtr.configPeakOutputVoltage(+12.0f, -12.0f);
		
	
		
		/*
		* Set Brake-Coast mode to coast
		*/
		setBrakeMode(false);
		
		/*
		* Setup encoder feedback if there are encoders on master motors
		*/
		if (SRXDriveBaseCfg.isMasterEncodersPresent) {
			driveRightMasterMtr.setFeedbackDevice(FeedbackDevice.QuadEncoder);
			driveRightMasterMtr.reverseSensor(SRXDriveBaseCfg.isRightEncoderSensorReversed);
			driveRightMasterMtr.configEncoderCodesPerRev(SRXDriveBaseCfg.DRIVE_RIGHT_ENCODER_CNTS_PER_REV); 
			
			driveLeftMasterMtr.setFeedbackDevice(FeedbackDevice.QuadEncoder);
			driveLeftMasterMtr.reverseSensor(SRXDriveBaseCfg.isLeftEncoderSensorReversed);
			driveLeftMasterMtr.configEncoderCodesPerRev(SRXDriveBaseCfg.DRIVE_LEFT_ENCODER_CNTS_PER_REV);
		}
		
		/*
		* Setup PID values if enabled
		*/
		if (SRXDriveBaseCfg.isPIDEnabled) {
			driveRightMasterMtr.setProfile(0);
			driveRightMasterMtr.setF(SRXDriveBaseCfg.kdriveRightMstrFeedForwardGain);
			driveRightMasterMtr.setP(SRXDriveBaseCfg.kdriveRightMstrProportionalGain);
			driveRightMasterMtr.setI(SRXDriveBaseCfg.kdriveRightMstrIntegralGain);
			driveRightMasterMtr.setD(SRXDriveBaseCfg.kdriveRightMstrDerivativeGain);
			
			driveLeftMasterMtr.setProfile(0);
			driveLeftMasterMtr.setF(SRXDriveBaseCfg.kdriveLeftMstrFeedForwardGain);
			driveLeftMasterMtr.setP(SRXDriveBaseCfg.kdriveLeftMstrProportionalGain);
			driveLeftMasterMtr.setI(SRXDriveBaseCfg.kdriveLeftMstrIntegralGain);
			driveLeftMasterMtr.setD(SRXDriveBaseCfg.kdriveLeftMstrDerivativeGain);				
		}
		
		driveStyle = new RobotDrive(driveRightMasterMtr, driveLeftMasterMtr);
		
		/*
		* Clear all sticky faults in drive controllers
		*/
		driveRightMasterMtr.clearStickyFaults();	
		driveRightFollowerMtr.clearStickyFaults();
		driveLeftMasterMtr.clearStickyFaults();
		driveLeftFollowerMtr.clearStickyFaults();
		
	
	}	
	
	public void enableMasterFollowerControl(){
	
	}
			
	
	/*================================================================
	* SRXDriveBase commands
	*/
	
	public void setRightPositionToZero() {
		driveRightMasterMtr.setPosition(0);
	}
	
	public void setLeftPositionToZero() {
		driveLeftMasterMtr.setPosition(0);
		
	}
	
	public void setBrakeMode(boolean isBrakeEnabled){
		if (SRXDriveBaseCfg.isBrakeEnabled) {
		driveRightMasterMtr.enableBrakeMode(false);
		driveRightFollowerMtr.enableBrakeMode(false);
		driveLeftMasterMtr.enableBrakeMode(false);
		driveLeftFollowerMtr.enableBrakeMode(false);	
			
		} else {
		driveRightMasterMtr.enableBrakeMode(true);
		driveRightFollowerMtr.enableBrakeMode(true);
		driveLeftMasterMtr.enableBrakeMode(true);
		driveLeftFollowerMtr.enableBrakeMode(true);	
			
		}
	}
	
	/*=====================================================
	* SRXDriveBase status commands
	*/
	public double getRightPosition() {
		return driveRightMasterMtr.getPosition();
	}
	
	public double getRightMstrMtrCurrent() {
		return driveRightMasterMtr.getOutputCurrent();
	}
	
	public double getRightFollowerMtrCurrent() {
		return driveRightFollowerMtr.getOutputCurrent();
	}
	public double getRightVelocity() {
		return driveRightMasterMtr.getSpeed();		
	}
	
	public double getRightCloseLoopError() {
		return driveRightMasterMtr.getClosedLoopError();
	}
	
	public double getLeftPosition() {
		return driveLeftMasterMtr.getPosition();		
	}
	
	public double getLeftMstrMtrCurrent() {
		return driveLeftMasterMtr.getOutputCurrent();
	}
	public double getLeftFollowerMtrCurrent() {
		return driveLeftFollowerMtr.getOutputCurrent();
	}
	public double getLeftVelocity() {
		return driveLeftMasterMtr.getSpeed();
	}
	
	public double getLeftCloseLoopError() {
		return driveLeftMasterMtr.getClosedLoopError();
	}
	
	/*=========================================================
	* Motion Commands
	*/
	
	 /*
	* Drive power commands. The SRX ESC modules do the rest
	*/
	public void setRightPower(double RightPowerLevel) {
		driveRightMasterMtr.set(RightPowerLevel * SRXDriveBaseCfg.kDriveStraightCorrection) ;
	}
	
	public void setLeftPower(double LeftPowerLevel) {
		driveLeftMasterMtr.set(LeftPowerLevel);
	}
	
	public void setTurnAndThrottle(double rotateValue, double moveValue){
		double rightMotorSpeed;
		double leftMotorSpeed;
		
		if (moveValue > 0.0) {  // forward
		   if (rotateValue > 0.0) {
		        leftMotorSpeed = moveValue - rotateValue;
		        rightMotorSpeed = Math.max(moveValue, rotateValue);
		    } else {
		        leftMotorSpeed = Math.max(moveValue, -rotateValue);
		        rightMotorSpeed = moveValue + rotateValue;
		    }
		 } else {  // reverse
		    if (rotateValue > 0.0) {
		        leftMotorSpeed = -Math.max(-moveValue, rotateValue);
		        rightMotorSpeed = moveValue + rotateValue;
		    } else {
		        leftMotorSpeed = moveValue - rotateValue;
		        rightMotorSpeed = -Math.max(-moveValue, -rotateValue);
		    }
		 }
		 if (SRXDriveBaseCfg.isDriveStraightAssistEnabled) {
				// uses navx to assist in driving straight
		 } else {
			driveRightMasterMtr.set(rightMotorSpeed);
			driveLeftMasterMtr.set(leftMotorSpeed);
			SmartDashboard.putNumber("rCounts", driveRightMasterMtr.getPosition());
			SmartDashboard.putNumber("lCounts", driveLeftMasterMtr.getPosition());
			DebugLogger.log("rightEncoder:" + driveRightMasterMtr.getPosition() 
			                + "leftEncoder:" + driveLeftMasterMtr.getPosition());
			DebugLogger.data(rightMotorSpeed + "," + leftMotorSpeed + "," +
			                 driveRightMasterMtr.getSpeed() + "," + driveLeftMasterMtr.getSpeed());
		}
	}
	
	
	/**
	 * driveIndexRobot method:
	 * @parm indexDistance is in inches
	 * @parm indexTime is in seconds
	 *
	 * indexRobot uses "Magic Motion" in the Talon SRX modules to index the robot.
	 * "Magic Motion" SRX method needs cruise velocity(RPM), acceleration rate (RPM/Sec),
	 * and distance(encoder counts)
	 *
	 * The following equations for a trapezoid with 1/3 time segments are used to determine 
	 * params for "Magic Motion" move:
	 *
	 * Velocity = 1.5*(Distance / Time)
     * Accel = Decel = 4.5*(Distance / Time2)
	 *
	 * Velocity(RPM) =  (1.5*(Distance(in) / Time(sec))*60(sec/min)) / Wheel Circum(in/rev)
	 * Acceleration(RPM/sec) = ((1.5*(Distance(in) / Time(sec))*60(sec/min))
	 *							/ Wheel Circum(in/rev)) / Ta(sec)
	 * Ta should be Time/3
	 * Distance(Encoder Counts) = Distance(in) / (in/count)
	 */
	// RevB, made driveIndexRobot to receive constant calls and return a boolean state 
	public boolean SRXBaseDriveIndexRobot(double indexDistanceIn, int indexTime){
		// this is a one shot code: calculate parms for magic motion, send to SRX and start magic motion
		if (!isDriveMoving) {
			// Right drive train calculations
			double calibratedRgtDistance = indexDistanceIn * SRXDriveBaseCfg.kRgtDistanceCalibration;
			double rightDrvTrainCruiseVelSetPt =  (1.5*(calibratedRgtDistance / indexTime)*60)
											/ SRXDriveBaseCfg.kCalibratedRgtWheelCircum;
			double rightDrvTrainAccelSetPt = ((1.5*(indexDistanceIn / indexTime)*60)
										/ SRXDriveBaseCfg.kCalibratedRgtWheelCircum
										/ (indexTime * .33333));
		    rightDrvTrainTargetPosSetPt = calibratedRgtDistance / SRXDriveBaseCfg.kRgtInchesPerCount;
			
			driveRightMasterMtr.setMotionMagicCruiseVelocity(rightDrvTrainCruiseVelSetPt);
			driveRightMasterMtr.setMotionMagicAcceleration(rightDrvTrainAccelSetPt);
			
			double calibratedLftDistance = indexDistanceIn * SRXDriveBaseCfg.kLftDistanceCalibration;
			
			// Left drive train calcualtions
			double leftDrvTrainCusiseVelSetPt =  (1.5*(calibratedLftDistance / indexTime)*60)
											/ SRXDriveBaseCfg.kCalibratedLftWheelCircum;
			double leftDrvTrainAccelSetPt = ((1.5*(indexDistanceIn / indexTime)*60)
										/ SRXDriveBaseCfg.kCalibratedLftWheelCircum
										/ (indexTime * .33333));
		    leftDrvTrainTargetPosSetPt = calibratedLftDistance / SRXDriveBaseCfg.kLftInchesPerCount;
			driveLeftMasterMtr.setMotionMagicCruiseVelocity(leftDrvTrainCusiseVelSetPt);
			driveLeftMasterMtr.setMotionMagicAcceleration(leftDrvTrainAccelSetPt);
		
		// Start index
			driveRightMasterMtr.changeControlMode(TalonControlMode.MotionMagic);
			driveLeftMasterMtr.changeControlMode(TalonControlMode.MotionMagic);
			driveRightMasterMtr.set(rightDrvTrainTargetPosSetPt); 
			driveLeftMasterMtr.set(leftDrvTrainTargetPosSetPt);
		// set isDriveMoving
			isDriveMoving = true;
		}
		// If drive is moving check for end of move (?? not sure what to check)
		else if ((driveRightMasterMtr.getSpeed() < SRXDriveBaseCfg.kSpeedDeadBand) &&
					(driveLeftMasterMtr.getSpeed() < SRXDriveBaseCfg.kSpeedDeadBand)) {
			
				driveRightMasterMtr.set(0); 
				driveLeftMasterMtr.set(0);
				isDriveMoving = false;
		}
		// resend distance SRX if drive has not reached end
		else {
			driveRightMasterMtr.set(rightDrvTrainTargetPosSetPt); 
			driveLeftMasterMtr.set(leftDrvTrainTargetPosSetPt);
		}
		return isDriveMoving;
	}
	
	public void SRXBaseDriveRotate(double _RotateHeadingValue){
		
	}
		
	public void SRXBaseDriveTurn(double _TurnHeadingValue) {
		
	}
	
	public void SRXBaseDrivePowerMoveToPosition(double _PwrMoveToPositionValue) {
		
	}
	
	public boolean moveInchesAndSpeed(double distance, double speed) {
		boolean moveDone = false;
		double leftEncoderCounts = distance / SRXDriveBaseCfg.kLftInchesPerCount;
		double rightEncoderCounts = distance / SRXDriveBaseCfg.kRgtInchesPerCount;
		if ((Math.abs(driveRightMasterMtr.getEncPosition()) < rightEncoderCounts) &&
				(Math.abs(driveLeftMasterMtr.getEncPosition()) < leftEncoderCounts)) {
			setTurnAndThrottle(0, speed);
		}
		else {
			DebugLogger.log("move is done");
			moveDone = true;
		}
		return moveDone;
		
	}

}