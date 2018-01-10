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
import com.ctre.CANTalon.VelocityMeasurementPeriod;

import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SRXDriveBase {
	private String VersionString = "Release 1, RevC 180108";

	// cheesy or tank motors
	private CANTalon driveRightMasterMtr;
	private CANTalon driveRightFollowerMtr;
	private CANTalon driveLeftMasterMtr;
	private CANTalon driveLeftFollowerMtr;
	// private AHRSSensor robotAngle;

	private RobotDrive driveStyle;
	// private AHRSSensor robotHeading;
	// private UltrasonicSensor distanceSensor;

	private boolean isDriveMoving;
	private double rightDrvTrainTargetPosSetPt;
	private double leftDrvTrainTargetPosSetPt;
	private double leftCmdLevel = 0;
	private double rightCmdLevel = 0;
	private double driveDirectionCorrection;

	// Program flow switches
	private boolean isStallTimerActive = false;
	private boolean isStallTimerTimedOut = false;
	private boolean isVelMoveToPositionActive = false;
	private boolean isRotateToAngleActive = false;
	private boolean isTurnToAngleActive = false;
	private boolean isSRXMagicMoveActive = false;
	private boolean isLowTimeActive = false;
	private boolean isSqWaveFnctStartActive = false;
	private boolean isMovePerpendicularActive = false;
	private double integral = 0;
	private double previousError = 0;
	private double previousTime = 0;
	private double previousTimeSec;

	private double startStallTimerSec = 0;
	// SRXDriveBase Class Constructor
	public SRXDriveBase() {

		// Create CAN SRX motor controller objects
		driveRightMasterMtr = new CANTalon(RobotMap.CAN_ID_1);
		driveRightFollowerMtr = new CANTalon(RobotMap.CAN_ID_2);
		driveLeftMasterMtr = new CANTalon(RobotMap.CAN_ID_3);
		driveLeftFollowerMtr = new CANTalon(RobotMap.CAN_ID_4);

		// robotHeading = new AHRSSensor;
		// distanceSensor = new UltrasonicSensor

		LiveWindow.addActuator("rtM", "RightMaster", driveRightMasterMtr);
		LiveWindow.addActuator("rtF", "RightFollower", driveRightFollowerMtr);
		LiveWindow.addActuator("lftM", "LeftMaster", driveLeftMasterMtr);
		LiveWindow.addActuator("lftF", "LeftFollower", driveLeftFollowerMtr);

		isDriveMoving = false;

		/*
		 * Set right/left masters and right/left followers
		 */
		// Set Right master to percentVbus mode
		driveRightMasterMtr.changeControlMode(TalonControlMode.PercentVbus);
		driveRightMasterMtr.enableControl();
		driveRightMasterMtr.set(0);

		// Set up right follower
		driveRightFollowerMtr.changeControlMode(TalonControlMode.Follower);

		// Set follower motor to follow master
		driveRightFollowerMtr.set(driveRightMasterMtr.getDeviceID());
		driveRightFollowerMtr.enableControl();

		// Set left master to percentVbus mode
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
		setBrakeMode(SRXDriveBaseCfg.isBrakeEnabled);

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
		 * Setup closed-loop velocity and Setup PID values if enabled
		 */
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			setSRXSpeedModeWithFeedback();
		}

		/*
		 * Setup PID values if enabled
		 */
		/*
		 * if (SRXDriveBaseCfg.isPIDEnabled) {
		 * driveRightMasterMtr.setProfile(0);
		 * driveRightMasterMtr.setF(SRXDriveBaseCfg.
		 * kdriveRightMstrFeedForwardGain);
		 * driveRightMasterMtr.setP(SRXDriveBaseCfg.
		 * kdriveRightMstrProportionalGain);
		 * driveRightMasterMtr.setI(SRXDriveBaseCfg.kdriveRightMstrIntegralGain)
		 * ; driveRightMasterMtr.setD(SRXDriveBaseCfg.
		 * kdriveRightMstrDerivativeGain);
		 * 
		 * driveLeftMasterMtr.setProfile(0);
		 * driveLeftMasterMtr.setF(SRXDriveBaseCfg.kdriveLeftMstrFeedForwardGain
		 * ); driveLeftMasterMtr.setP(SRXDriveBaseCfg.
		 * kdriveLeftMstrProportionalGain);
		 * driveLeftMasterMtr.setI(SRXDriveBaseCfg.kdriveLeftMstrIntegralGain);
		 * driveLeftMasterMtr.setD(SRXDriveBaseCfg.kdriveLeftMstrDerivativeGain)
		 * ; }
		 */
		/*
		 * setup IMU if enabled
		 */
		if (SRXDriveBaseCfg.isHeadingModuleEnabled) {
			try {
				// imu = new AHRS(SPI.Port.kOnboardCS0);
			} catch (RuntimeException ex) {
				System.out.println("Error starting the Nav-X");
			}
			// imu.zeroYaw();
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

	/*
	 * ================================================================
	 * SRXDriveBase commands
	 */

	public void setSRXPercentVbusMode() {
		// Set Right master to percentVbus mode
		driveRightMasterMtr.changeControlMode(TalonControlMode.PercentVbus);
		driveRightMasterMtr.enableControl();
		driveRightMasterMtr.set(0);

		// Set left master to percentVbus mode
		driveLeftMasterMtr.changeControlMode(TalonControlMode.PercentVbus);
		driveLeftMasterMtr.enableControl();
		driveLeftMasterMtr.set(0);
	}

	public void setSRXSpeedModeWithFeedback() {
		driveRightMasterMtr.setPID(SRXDriveBaseCfg.kdriveRightMstrProportionalGain,
				SRXDriveBaseCfg.kdriveRightMstrIntegralGain, SRXDriveBaseCfg.kdriveRightMstrDerivativeGain,
				SRXDriveBaseCfg.kdriveRightMstrFeedForwardGain, SRXDriveBaseCfg.kdriveRightMstrRampRate,
				SRXDriveBaseCfg.kdriveRightMstrIzone, SRXDriveBaseCfg.kdriveRightMstrProfile);

		driveLeftMasterMtr.setPID(SRXDriveBaseCfg.kdriveLeftMstrProportionalGain,
				SRXDriveBaseCfg.kdriveLeftMstrIntegralGain, SRXDriveBaseCfg.kdriveLeftMstrDerivativeGain,
				SRXDriveBaseCfg.kdriveLeftMstrFeedForwardGain, SRXDriveBaseCfg.kdriveLeftMstrRampRate,
				SRXDriveBaseCfg.kdriveleftMstrIzone, SRXDriveBaseCfg.kdriveLeftMstrProfile);

		// set controlMode to speed with feedback
		// Note speed is in RPM, thus -1 to 1 input needs to be multiplied by
		// max RPM
		driveRightMasterMtr.changeControlMode(TalonControlMode.Speed);
		driveLeftMasterMtr.changeControlMode(TalonControlMode.Speed);

		// ?? to test should we use this for autonomous
		// driveRightMasterMtr.disableNominalClosedLoopVoltage();
		// The following compensates for battery voltage - 50% output would be
		// %50 of 11 volts
		// driveRightMasterMtr.setNominalClosedLoopVoltage(11.0);

		// cnts / 4096 cnts/rev(magnetic encoder); 100/4096= 2.4%; 8.7 degrees
		driveRightMasterMtr.setAllowableClosedLoopErr(SRXDriveBaseCfg.kClosedLoopErr);
		driveLeftMasterMtr.setAllowableClosedLoopErr(SRXDriveBaseCfg.kClosedLoopErr);

		// Sample period in ms from supported sample periods-default 100ms
		// period/64 sample window
		driveRightMasterMtr.SetVelocityMeasurementPeriod(VelocityMeasurementPeriod.Period_25Ms);
		driveLeftMasterMtr.SetVelocityMeasurementPeriod(VelocityMeasurementPeriod.Period_25Ms);

		// Number if sanples in a rolling average
//		driveRightMasterMtr.SetVelocityMeasurementPeriod(SRXDriveBaseCfg.kSRXVelocitySample);
	//	driveLeftMasterMtr.SetVelocityMeasurementPeriod(SRXDriveBaseCfg.kSRXVelocitySample);

		// Activate closed-Loop velocity
		driveRightMasterMtr.set(0);
		driveLeftMasterMtr.set(0);

	}

	public void setRightPositionToZero() {
		driveRightMasterMtr.setPosition(0);
	}

	public void setLeftPositionToZero() {
		driveLeftMasterMtr.setPosition(0);

	}

	public void setBrakeMode(boolean isBrakeEnabled) {
		driveRightMasterMtr.enableBrakeMode(isBrakeEnabled);
		driveRightFollowerMtr.enableBrakeMode(isBrakeEnabled);
		driveLeftMasterMtr.enableBrakeMode(isBrakeEnabled);
		driveLeftFollowerMtr.enableBrakeMode(isBrakeEnabled);
	}

	/*
	 * ===================================================== SRXDriveBase status
	 * commands
	 */

	// Reads encoder, velocity, current, error, and displays on smartdashboard
	public void UpdateSRXDrive() {

		// Display SRXBaseDrive version
		SmartDashboard.putString("SRXBaseDrive-Version", VersionString);
		// Display SRX module values
		SmartDashboard.putNumber("SRXBaseDrive-Right Bus Voltage", driveRightMasterMtr.getBusVoltage());
		SmartDashboard.putNumber("SRXBaseDrive-Right Output Voltage", driveRightMasterMtr.getOutputVoltage());
		SmartDashboard.putNumber("SRXBaseDrive-Current Right Master", driveRightMasterMtr.getOutputCurrent());
		SmartDashboard.putNumber("SRXBaseDrive-Current Right Follower", driveRightFollowerMtr.getOutputCurrent());

		SmartDashboard.putNumber("SRXBaseDrive-Left Bus Voltage", driveLeftMasterMtr.getBusVoltage());
		SmartDashboard.putNumber("SRXBaseDrive-Left Output Voltage", driveLeftMasterMtr.getOutputVoltage());
		SmartDashboard.putNumber("SRXBaseDrive-Current Left Master", driveLeftMasterMtr.getOutputCurrent());
		SmartDashboard.putNumber("SRXBaseDrive-Current Left Follower", driveRightFollowerMtr.getOutputCurrent());

		if (SRXDriveBaseCfg.isMasterEncodersPresent) {
			SmartDashboard.putNumber("SRXBaseDrive-Right Encoder Count", driveRightMasterMtr.getPosition());
			SmartDashboard.putNumber("SRXBaseDrive-Speed Right", driveRightMasterMtr.getSpeed());
			SmartDashboard.putNumber("SRXBaseDrive-Left Encoder Count", driveLeftMasterMtr.getPosition());
			SmartDashboard.putNumber("SRXBaseDrive-Speed Left", driveLeftMasterMtr.getSpeed());
		}

		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			SmartDashboard.putNumber("SRXBaseDrive-Speed Right ClosedLoopErr",
					driveRightMasterMtr.getClosedLoopError());
			SmartDashboard.putNumber("SRXBaseDrive-Speed Left ClosedLoopErr", driveLeftMasterMtr.getClosedLoopError());
		}

		if (SRXDriveBaseCfg.isHeadingModuleEnabled) {
			// Display Inertial Measurement Unit (IMU) values
			/*
			 * SmartDashboard.putNumber("SRXBaseDrive-ANGLE NAVX",
			 * imu.getAngle()); SmartDashboard.putNumber("SRXBaseDrive-IMU_Yaw",
			 * imu.getYaw()); SmartDashboard.putNumber("SRXBaseDrive-IMU_Pitch",
			 * imu.getPitch());
			 * SmartDashboard.putNumber("SRXBaseDrive-IMU_Roll", imu.getRoll());
			 */
		}
		// check for stall current and motor burnout
		// to do
	}

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

	public double getBusVoltage() {
		return driveLeftMasterMtr.getBusVoltage();
	}

	/*
	 * ========================================================= Teleop Motion
	 * Commands
	 *
	 * Note: left drive is master drive axis for the robot - the right drive
	 * will be modified for driving straight
	 */

	/*
	 * NOTE: Motion command with open loop reflect power levels (-1 to 1) * (the
	 * motor bus voltage). Motion command with closed loop reflect speed level
	 * (-1 to 1) * (top motor RPM)
	 */
	public void SetDriveTrainCmdLevel(double _rightCMDLevel, double _leftCMDLevel) {
		_rightCMDLevel *= SRXDriveBaseCfg.kDriveStraightCorrection;

		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			_rightCMDLevel *= SRXDriveBaseCfg.kTopRPM;
			_leftCMDLevel *= SRXDriveBaseCfg.kTopRPM;
		}
		driveRightMasterMtr.set(_rightCMDLevel);
		driveLeftMasterMtr.set(_leftCMDLevel);
		StallConditionTimeOut();
	}

	/*
	 * WPI throttle and turn commands This method uses WPI library methods to
	 * drive the robot with a throttle and turn input. Drives were set up by:
	 * driveStyle = new RobotDrive(driveRightMasterMtr, driveLeftMasterMtr); The
	 * throttle would be the game controller Y-axis(joystick fwd/rev) and turn
	 * would be game conctroller X-axis(joystick left/right)
	 *
	 * NOTE: WPILib throttleValue and turnValue are open loop power levels (-1
	 * to 1) * (the motor bus voltage). The speed is determined by this power
	 * level and the load to the motor.
	 */
	public void WPISetThrottleTurn(double throttleValue, double turnValue) {
		// Check stall condition
		/*if (StallConditionTimeOut()) {
			throttleValue = 0;
			turnValue = 0;
		}*/
		driveStyle.arcadeDrive(throttleValue, turnValue, false);

		//StallConditionTimeOut();
	}

	/*
	 * setThrottleTurn is both open loop and closed loop control with drive
	 * straight/drive perpendicular correction
	 */
	public void setThrottleTurn(double _throttleValue, double _turnValue, boolean _isDrivingPerpendicular) {

		// Check stall condition
		if (StallConditionTimeOut()) {
			_throttleValue = 0;
			_turnValue = 0;
		}

		// The turn joystick needs to be in its zero position to have drive
		// direction assist
		if ((Math.abs(_turnValue) < SRXDriveBaseCfg.kTurnValueDeadBand)) {
			_turnValue = 0;
		}

		if (SRXDriveBaseCfg.isDriveStraightAssistEnabled && _turnValue == 0) {

			// Calculate cmd level in terms of PercentVbus; range (-1 to 1)
			driveDirectionCorrection = pidDriveCorrection(_isDrivingPerpendicular, _turnValue);
			leftCmdLevel = _throttleValue + _turnValue - driveDirectionCorrection;
			rightCmdLevel = ((_throttleValue - _turnValue) * SRXDriveBaseCfg.kDriveStraightCorrection)
					+ driveDirectionCorrection;

		} else {
			leftCmdLevel = _throttleValue + _turnValue;
			rightCmdLevel = ((_throttleValue - _turnValue) * SRXDriveBaseCfg.kDriveStraightCorrection);
		}

		// This converts cmd to speed in RPM; range -TopRPM to TopRPM
		if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			leftCmdLevel *= SRXDriveBaseCfg.kTopRPM;
			rightCmdLevel *= SRXDriveBaseCfg.kTopRPM;
		}
		// Output left/right command levels
		driveLeftMasterMtr.set(leftCmdLevel);
		driveRightMasterMtr.set(rightCmdLevel);
		StallConditionTimeOut();
	}

	// remove this?
	/* public void setTurnAndThrottle(double rotateValue, double moveValue) {
		double rightMotorSpeed = 0.0;
		double leftMotorSpeed = 0.0;

		if (moveValue > 0.0) { // forward
			if (rotateValue > 0.0) {
				leftMotorSpeed = moveValue - rotateValue;
				rightMotorSpeed = Math.max(moveValue, rotateValue);
			} else {
				leftMotorSpeed = Math.max(moveValue, -rotateValue);
				rightMotorSpeed = moveValue + rotateValue;
			}
		} else { // reverse
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
			DebugLogger.log("rightEncoder:" + driveRightMasterMtr.getPosition() + "leftEncoder:"
					+ driveLeftMasterMtr.getPosition());
			DebugLogger.data(rightMotorSpeed + "," + leftMotorSpeed + "," + driveRightMasterMtr.getSpeed() + ","
					+ driveLeftMasterMtr.getSpeed());
		}
	}
*/
	/*
	 * If robot blocked by another robot Fast turn turns robot 180 deg to escape
	 */
	public void fastTurn() {
		// future TODO
	}

	public double pidDriveCorrection(boolean _isDrivingPerpend, double _turnValue){
		// This method uses: 
		// 1) The ahrs and a PID equation to provide drive straight correction
		// 2) Two ultrasonic sensors and a PID equation to provide drive perpendicular to a surface correction
		if ((Math.abs(_turnValue) >= SRXDriveBaseCfg.kTurnValueDeadBand)) {
			
			// Clear variables
//			robotHeading.resetAngle();
			integral = 0;
			previousError = 0;
			previousTime =0;
		}
		
		// Calculate delta time
		double presentTimeSec = Timer.getFPGATimestamp();
		double error = 0;
		// In iterative Robot timeDiff is aprox 20ms
		double timeDiffSec = presentTimeSec - previousTimeSec;
		if (previousTime == 0) {
			timeDiffSec = 0.02; // set to 20ms
		}
		
		// error = (setpoint - input); our setpoint is zero
		if (_isDrivingPerpend) {
			//error = distanceSensor.getPerpendicularError();
		} else {
			//error = -robotHeading.getAngle();
		} 
		
		integral += error * timeDiffSec;
		double derivative = (error - previousError) / timeDiffSec;
		double PIDOutput = (SRXDriveBaseCfg.kCorrection_Kp * error) 
					+ (SRXDriveBaseCfg.kCorrection_Ki * integral) 
					+ (SRXDriveBaseCfg.kCorrection_Kd * derivative);
		
		// Remember variables for next scan to calculate PID
		previousError = error;
		previousTimeSec = presentTimeSec;
		// return 1/2 of output - output used on left and right side to correct
		return (PIDOutput / 2);
	}

	/*
	 * This method flags a robot in a stall condition longer than stall time
	 */
	public boolean StallConditionTimeOut() {
		// Check if motors are in a stall current state and kill drives if left there too long
		if ((driveRightMasterMtr.getOutputCurrent() > SRXDriveBaseCfg.kStallCurrent) 
			||  (driveLeftMasterMtr.getOutputCurrent() > SRXDriveBaseCfg.kStallCurrent)) {
				if (!isStallTimerActive) {
					startStallTimerSec  = Timer.getFPGATimestamp();
					isStallTimerActive = true;
				} else {
					if ((Timer.getFPGATimestamp() - startStallTimerSec) > SRXDriveBaseCfg.kStallTimeSec) {
						isStallTimerTimedOut = true;
					}
				}
		} else {
			isStallTimerActive = false;
			isStallTimerTimedOut = false;
		}
		return isStallTimerTimedOut;
	}

	/*
	 * =========================================================================
	 * ==== Autonomous motion commands
	 */

	public boolean velMoveToPosition(double _MoveToPositionIn) {
		double moveCounts = 0;
		// This method moves the robot with a predetermined power level and stops at
		// the specified position value. The move will be in brake mode to stop
		// method will check that robot is stopped and set brake mode back to coast and respond
		// that move is done
		if (!isVelMoveToPositionActive) {
			isVelMoveToPositionActive = true;
			setBrakeMode(true);
			moveCounts = (_MoveToPositionIn * SRXDriveBaseCfg.DRIVE_LEFT_ENCODER_CNTS_PER_REV)
							- SRXDriveBaseCfg.kRobotCoastToStopCounts;
			leftCmdLevel = SRXDriveBaseCfg.kMoveToPositionVelCmdLevel;
			rightCmdLevel = SRXDriveBaseCfg.kMoveToPositionVelCmdLevel * SRXDriveBaseCfg.kDriveStraightCorrection;
			if (SRXDriveBaseCfg.isSRXClosedLoopEnabled){
				rightCmdLevel *= SRXDriveBaseCfg.kTopRPM;
				leftCmdLevel *= SRXDriveBaseCfg.kTopRPM;
			}
		} else {
			if (getLeftPosition() >= moveCounts) {
				isVelMoveToPositionActive = false;
				rightCmdLevel = 0;
				leftCmdLevel = 0;
				setBrakeMode(false);
			}
			
		}
		driveRightMasterMtr.set(rightCmdLevel) ;
		driveLeftMasterMtr.set(leftCmdLevel);
		return isVelMoveToPositionActive;
		
	}

	public boolean movePerpendicularToStop() {
		double perpendicularCorrection;
		boolean _isDrivingPerpend = false;
		
		if (!isMovePerpendicularActive) {
			isMovePerpendicularActive = true;
			rightCmdLevel = SRXDriveBaseCfg.kDrivePerpendicularCmdLevel;
			leftCmdLevel = SRXDriveBaseCfg.kDrivePerpendicularCmdLevel;
		} else {
			perpendicularCorrection = pidDriveCorrection(_isDrivingPerpend, 0);
			leftCmdLevel = SRXDriveBaseCfg.kDrivePerpendicularCmdLevel 
					       - pidDriveCorrection(_isDrivingPerpend, 0); //PROFESSOR?
			rightCmdLevel= SRXDriveBaseCfg.kDrivePerpendicularCmdLevel
                           + pidDriveCorrection(_isDrivingPerpend, 0);
	if (SRXDriveBaseCfg.isSRXClosedLoopEnabled)	{
				rightCmdLevel *= SRXDriveBaseCfg.kTopRPM;
				leftCmdLevel *= SRXDriveBaseCfg.kTopRPM;
			}
	}
	
		/*
	if ((ultrasonicLeft.getRangeInches() <= SRXDriveBaseCfg.kDrvePerpendicularStopIn) ||
		(ultrasonicRight.getRangeInches()<=SRXDriveBaseCfg.kDrvePerpendicularStopIn)){
		rightCmdLevel=0;
		leftCmdLevel=0;
		isMovePerpendicularActive=false;
		}
		*/
	driveRightMasterMtr.set(rightCmdLevel);
	driveLeftMasterMtr.set(leftCmdLevel);
	return isMovePerpendicularActive;
	}

	public boolean rotateToAngle(double _angle, boolean _isRotationByEncoder) {
		// direction(true)-rotates right, direction(false)-rotates left
		double powerRight = 0;
		double powerLeft = 0;
		double rotationEncoderCount = 0;
		
		if (!isRotateToAngleActive) {
			powerRight = Math.signum(_angle)*SRXDriveBaseCfg.kRotatePowerLevel; 
			powerLeft = -Math.signum(_angle)*SRXDriveBaseCfg.kRotatePowerLevel;
			isRotateToAngleActive = true;
			//Put a comment here
			rotationEncoderCount = 2*Math.PI*(SRXDriveBaseCfg.kTrackWidthIn / 2) * (_angle / 360); 
		} else {
			if (_isRotationByEncoder) {
				if (driveLeftMasterMtr.getPosition() >= rotationEncoderCount) {
					powerRight = 0;
					powerLeft = 0;
					isRotateToAngleActive = false;
				}
			} else {
				/*if (ahrs.getYaw() >= _angle) {
					powerRight = 0;
					powerLeft = 0;
					isRotateToAngleActive = false;
				}*/
			}			
		}
		driveRightMasterMtr.set(powerRight) ;
		driveLeftMasterMtr.set(powerLeft);
		return isRotateToAngleActive;
	} 

	public boolean turnByEncoderToAngle(double _turnAngleValueDeg, double _turnRadiusIn) {
		double powerCmdLevel = 0;
		double wheelToCenterDistanceIn, ratio = 0, outerDistance = 0, innerDistance = 0;
		if (!isTurnToAngleActive) {
			isTurnToAngleActive = true;
			driveRightMasterMtr.setPosition(0);
			driveLeftMasterMtr.setPosition(0);
			if (SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
				powerCmdLevel = SRXDriveBaseCfg.kturnCmdLevel * SRXDriveBaseCfg.kTopRPM;
			} else {
				powerCmdLevel = SRXDriveBaseCfg.kturnCmdLevel;
			}

			// Calculations
			wheelToCenterDistanceIn = SRXDriveBaseCfg.kTrackWidthIn / 2;
			ratio = (_turnRadiusIn - wheelToCenterDistanceIn) / (_turnRadiusIn + wheelToCenterDistanceIn);
			outerDistance = 2 * Math.PI * ((_turnRadiusIn + wheelToCenterDistanceIn) * _turnAngleValueDeg / 360);
			innerDistance = 2 * Math.PI * ((_turnRadiusIn - wheelToCenterDistanceIn) * _turnAngleValueDeg / 360);

			// Convert distance in inches to encoder counts
			if (_turnAngleValueDeg > 0) {
				outerDistance *= SRXDriveBaseCfg.DRIVE_LEFT_ENCODER_CNTS_PER_REV;
			} else {
				outerDistance *= SRXDriveBaseCfg.DRIVE_RIGHT_ENCODER_CNTS_PER_REV;
			}
		} else {
			if ((_turnAngleValueDeg > 0 && driveLeftMasterMtr.getPosition() > outerDistance)
					|| (_turnAngleValueDeg <= 0 && driveRightMasterMtr.getPosition() > outerDistance)) {
				isTurnToAngleActive = false;
				powerCmdLevel = 0;
				driveRightMasterMtr.setPosition(0);
				driveLeftMasterMtr.setPosition(0);
			}
		}
		if (_turnAngleValueDeg > 0) {
			driveRightMasterMtr.set(powerCmdLevel);
			driveLeftMasterMtr.set(powerCmdLevel * ratio);
		} else {
			driveRightMasterMtr.set(powerCmdLevel * ratio);
			driveLeftMasterMtr.set(powerCmdLevel);
		}
		return isTurnToAngleActive;
	}

	public boolean magicMove(double _rightCruiseVel, double _rightAccel, double _rightDistance, double _leftCruiseVel,
			double _leftAccel, double _leftDistance) {
		// This method performs a SRX magic motion command from user calculated
		// values
		// User should note that the right drive distance needs to be corrected
		// by kDriveStraightCorrection
		if (!isSRXMagicMoveActive) {
			isSRXMagicMoveActive = true;
			driveRightMasterMtr.changeControlMode(TalonControlMode.MotionMagic);
			driveRightMasterMtr.setMotionMagicCruiseVelocity(_rightCruiseVel);
			driveRightMasterMtr.setMotionMagicAcceleration(_rightAccel);

			driveLeftMasterMtr.changeControlMode(TalonControlMode.MotionMagic);
			driveLeftMasterMtr.setMotionMagicCruiseVelocity(_leftCruiseVel);
			driveLeftMasterMtr.setMotionMagicAcceleration(_leftAccel);

			_rightDistance = _rightDistance * SRXDriveBaseCfg.kDriveStraightCorrection;
			_leftDistance = _leftDistance;
		} else {
			if (getRightPosition() >= _leftDistance) {
				isSRXMagicMoveActive = false;
				_rightDistance = 0;
				_leftDistance = 0;
			}
		}
		driveRightMasterMtr.set(_rightDistance);
		driveLeftMasterMtr.set(_leftDistance);
		return isSRXMagicMoveActive;
	}

	public void testMotorSquareWave(boolean _isMtrSquareWaveTestEnabled, boolean _isTestForRightDrive) {
		if (_isMtrSquareWaveTestEnabled && SRXDriveBaseCfg.isSRXClosedLoopEnabled) {
			double startTimeSec = 0;
			// initialize and start at low speed
			if (!isSqWaveFnctStartActive) {
				isLowTimeActive = true;
				isSqWaveFnctStartActive = true;
				startTimeSec = Timer.getFPGATimestamp(); // seconds

				// Start square wave at low speed
				if (_isTestForRightDrive) {
					driveRightMasterMtr.set(SRXDriveBaseCfg.kSquareWaveLowerSpeed);
					driveLeftMasterMtr.set(0);
				} else {
					driveLeftMasterMtr.set(SRXDriveBaseCfg.kSquareWaveLowerSpeed);
					driveRightMasterMtr.set(0);
				}
			}
			if (isLowTimeActive) {

				// Stay at a low speed for klowSQTime ms then switch to high
				// speed
				if (_isTestForRightDrive) {
					driveRightMasterMtr.set(SRXDriveBaseCfg.kSquareWaveLowerSpeed);
					driveLeftMasterMtr.set(0);
				} else {
					driveLeftMasterMtr.set(SRXDriveBaseCfg.kSquareWaveLowerSpeed);
					driveRightMasterMtr.set(0);
				}
				if ((Timer.getFPGATimestamp() - startTimeSec) > SRXDriveBaseCfg.kSquareWaveLowerSpeed) {

					// Stop high speed mode
					isLowTimeActive = false;

					// Set start time for high speed mode
					startTimeSec = Timer.getFPGATimestamp();
				}

			} else {

				// Stay at a high speed for kHighSQTime ms then switch to low
				// speed
				if (_isTestForRightDrive) {
					driveRightMasterMtr.set(SRXDriveBaseCfg.kSquareWaveHigherSpeed);
					driveLeftMasterMtr.set(0);
				} else {
					driveLeftMasterMtr.set(SRXDriveBaseCfg.kSquareWaveHigherSpeed);
					driveRightMasterMtr.set(0);
				}
				if ((Timer.getFPGATimestamp() - startTimeSec) > SRXDriveBaseCfg.kSquareWaveHigherSpeed) {
					// Stop high speed mode
					isLowTimeActive = true;

					// Set start time for low speed mode
					startTimeSec = Timer.getFPGATimestamp();

				}
			}
			SmartDashboard.putNumber("Test Low Speed(RPM)", SRXDriveBaseCfg.kSquareWaveLowerSpeed);
			SmartDashboard.putNumber("Test high Speed(RPM)", SRXDriveBaseCfg.kSquareWaveHigherSpeed);
			if (_isTestForRightDrive) {
				SmartDashboard.putNumber("Test Right Speed", driveRightMasterMtr.getSpeed());
				SmartDashboard.putNumber("Test Right Error", driveRightMasterMtr.getClosedLoopError());
				SmartDashboard.putNumber("Test Left Speed", 0);
				SmartDashboard.putNumber("Test Left Error", 0);
			} else {
				SmartDashboard.putNumber("Test Right Speed", 0);

				SmartDashboard.putNumber("Test Right Error", 0);
				SmartDashboard.putNumber("Test Left Speed", driveLeftMasterMtr.getSpeed());
				SmartDashboard.putNumber("Test Left Error", driveLeftMasterMtr.getClosedLoopError());
			}

		} else {
			// Reset method flags for next call to motorSquareWaveTest method
			isLowTimeActive = false;
			isSqWaveFnctStartActive = false;
			driveLeftMasterMtr.set(0);
			driveRightMasterMtr.set(0);
		}
	}

	/*
	 * This is used to determine the SRXDriveBaseCfg.kDriveStraightCorrection
	 */
	public boolean testMoveWithInchesAndSpeed(double distance, double speed) {
		boolean moveDone = false;
		double leftEncoderCounts = distance / SRXDriveBaseCfg.kLftInchesPerCount;
		double rightEncoderCounts = distance / SRXDriveBaseCfg.kRgtInchesPerCount;
		if ((Math.abs(driveRightMasterMtr.getEncPosition()) < rightEncoderCounts)
				&& (Math.abs(driveLeftMasterMtr.getEncPosition()) < leftEncoderCounts)) {
			//setTurnAndThrottle(0, speed);
		} else {
			DebugLogger.log("move is done");
			moveDone = true;
		}
		return moveDone;

	}

	/**
	 * driveIndexRobot method:
	 * 
	 * @parm indexDistance is in inches
	 * @parm indexTime is in seconds
	 *
	 *       indexRobot uses "Magic Motion" in the Talon SRX modules to index
	 *       the robot. "Magic Motion" SRX method needs cruise velocity(RPM),
	 *       acceleration rate (RPM/Sec), and distance(encoder counts)
	 *
	 *       The following equations for a trapezoid with 1/3 time segments are
	 *       used to determine params for "Magic Motion" move:
	 *
	 *       Velocity = 1.5*(Distance / Time) Accel = Decel = 4.5*(Distance /
	 *       Time2)
	 *
	 *       Velocity(RPM) = (1.5*(Distance(in) / Time(sec))*60(sec/min)) /
	 *       Wheel Circum(in/rev) Acceleration(RPM/sec) = ((1.5*(Distance(in) /
	 *       Time(sec))*60(sec/min)) / Wheel Circum(in/rev)) / Ta(sec) Ta should
	 *       be Time/3 Distance(Encoder Counts) = Distance(in) / (in/count)
	 */
	// RevB, made driveIndexRobot to receive constant calls and return a boolean
	// state
	public boolean SRXBaseDriveIndexRobot(double indexDistanceIn, int indexTime) {
		// this is a one shot code: calculate parms for magic motion, send to
		// SRX and start magic motion
		if (!isDriveMoving) {
			// Right drive train calculations
			double calibratedRgtDistance = indexDistanceIn * SRXDriveBaseCfg.kRgtDistanceCalibration;
			double rightDrvTrainCruiseVelSetPt = (1.5 * (calibratedRgtDistance / indexTime) * 60)
					/ SRXDriveBaseCfg.kCalibratedRgtWheelCircum;
			double rightDrvTrainAccelSetPt = ((1.5 * (indexDistanceIn / indexTime) * 60)
					/ SRXDriveBaseCfg.kCalibratedRgtWheelCircum / (indexTime * .33333));
			rightDrvTrainTargetPosSetPt = calibratedRgtDistance / SRXDriveBaseCfg.kRgtInchesPerCount;

			driveRightMasterMtr.setMotionMagicCruiseVelocity(rightDrvTrainCruiseVelSetPt);
			driveRightMasterMtr.setMotionMagicAcceleration(rightDrvTrainAccelSetPt);

			double calibratedLftDistance = indexDistanceIn * SRXDriveBaseCfg.kLftDistanceCalibration;

			// Left drive train calcualtions
			double leftDrvTrainCusiseVelSetPt = (1.5 * (calibratedLftDistance / indexTime) * 60)
					/ SRXDriveBaseCfg.kCalibratedLftWheelCircum;
			double leftDrvTrainAccelSetPt = ((1.5 * (indexDistanceIn / indexTime) * 60)
					/ SRXDriveBaseCfg.kCalibratedLftWheelCircum / (indexTime * .33333));
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
		else if ((driveRightMasterMtr.getSpeed() < SRXDriveBaseCfg.kSpeedDeadBand)
				&& (driveLeftMasterMtr.getSpeed() < SRXDriveBaseCfg.kSpeedDeadBand)) {

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
	public void stopMotors(){
		driveRightMasterMtr.set(0);
		driveLeftMasterMtr.set(0);
	}
}