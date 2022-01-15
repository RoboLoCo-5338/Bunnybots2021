// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

public class DriveSystem extends PIDSubsystem {
  // create motor controller objects
  public final WPI_TalonSRX leftFront = new WPI_TalonSRX(0); // TODO: change Talon ID
  public final WPI_VictorSPX leftRear = new WPI_VictorSPX(1); // TODO: change Talon ID
  public final WPI_TalonSRX rightFront = new WPI_TalonSRX(2); // TODO: change Talon ID
  public final WPI_VictorSPX rightRear = new WPI_VictorSPX(3); // TODO: change Talon ID

  // constants to find number of encoder ticks in an inch (used for autonomous)
  public static final double TICKS_PER_ROTATION = 4096.0;
	private static final double WHEEL_DIAMETER = 6.0;
	private static final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
  private static final double TICKS_PER_INCH = TICKS_PER_ROTATION / WHEEL_CIRCUMFERENCE;
  private static double targetPosition = 0;

  private final SpeedControllerGroup LEFT_SIDE = new SpeedControllerGroup(this.leftFront, this.leftRear);
  private final SpeedControllerGroup RIGHT_SIDE = new SpeedControllerGroup(this.rightFront, this.rightRear);
  // Creates a drive object that will define how the left and right motor sets are
  // configured (currently as an arcade drive)
  private final DifferentialDrive DRIVE = new DifferentialDrive(this.LEFT_SIDE, this.RIGHT_SIDE);

  public DriveSystem() {
    super(
        // The PIDController used by the subsystem
        // set P, I, and D values here
        new PIDController(0.000213, 0, 0));
    for (final WPI_TalonSRX talon : new WPI_TalonSRX[] { this.leftFront, this.rightFront }) {
      DriveSystem.configureTalon(talon);
    }

    for (final WPI_VictorSPX victor: new WPI_VictorSPX[] {this.leftRear, this.rightRear }) {
      DriveSystem.configureVictor(victor);
    }
    this.leftRear.follow(this.leftFront);
    this.rightFront.follow(this.rightFront);
    this.leftFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
		this.rightFront.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 30);
    leftFront.setInverted(true);
    leftRear.setInverted(true);
    // setDefaultCommand(new drive(this));
  }

  public void setPIDF(double p, double i, double d, double f) {
		// Set PIDF for Left Front controller.
		this.leftFront.config_kP(0, p, 100);
		this.leftFront.config_kI(0, i, 100);
		this.leftFront.config_kD(0, d, 100);
		this.leftFront.config_kF(0, f, 100);

		// Set PIDF for Right Front controller.
		this.rightFront.config_kP(0, p, 100);
		this.rightFront.config_kI(0, i, 100);
		this.rightFront.config_kD(0, d, 100);
		this.rightFront.config_kF(0, f, 100);
  }

    // creates a PID velocity robot. Uses PID settings to determine speeds
    public void tankDriveVelocity(double left, double right) {
      double targetLeft;
		  double targetRight;

		  double targetVelocity = 400;

      targetLeft = left * targetVelocity * 4096 / 600.0;
      targetRight = right * targetVelocity * 4096 / 600.0;

      this.leftFront.set(ControlMode.Velocity, targetLeft);
      this.leftRear.set(ControlMode.Velocity, targetRight);
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Runs the drive in tank mode
  public void drive(OI oi) {
    this.DRIVE.tankDrive(oi.getLeftJoystick('Y'), oi.getRightJoystick('Y'));
  }

  public void autoDrive() {
    this.DRIVE.tankDrive(-0.5, 0.5);
    
  }

  // configure talon properties
  private static void configureTalon(final WPI_TalonSRX talon) {
    talon.configPeakCurrentLimit(100, 0);
    talon.configPeakCurrentDuration(3, 0);
    talon.configContinuousCurrentLimit(80, 0);
    talon.enableCurrentLimit(true);
    talon.configNeutralDeadband(0.001, 0);
    talon.setNeutralMode(NeutralMode.Brake);
  }

  private static void configureVictor(final WPI_VictorSPX victor) {
    victor.configNeutralDeadband(0.001, 0);
    victor.setNeutralMode(NeutralMode.Brake);
  }

  // reset the current position on the encoders
  public void resetPosition() {
		this.rightFront.setSelectedSensorPosition(0);
		this.leftFront.setSelectedSensorPosition(0);
  }
  
  // drive a given distance (inches)
  public void driveDistance(double inches) {
		targetPosition = inches * TICKS_PER_INCH;

		this.leftFront.set(ControlMode.Position, -targetPosition);
		this.rightFront.set(ControlMode.Position, targetPosition);
  }
  
  // get left position
  public double getPosition() {
		return this.leftFront.getSelectedSensorPosition() / TICKS_PER_INCH;
	}

  // get left position
	public double getLeftDistance() {
		return this.leftFront.getSelectedSensorPosition() / TICKS_PER_INCH;
	}

  // get right position
	public double getRightDistance() {
		return this.rightFront.getSelectedSensorPosition() / TICKS_PER_INCH;
  }
  
  // check if the robot reached its desired position
  public boolean reachedPosition() {
    // get motorcontroller positions
		double leftPos = this.leftFront.getSelectedSensorPosition();
		double rightPos = this.rightFront.getSelectedSensorPosition();

    // return true if position is reached
		if (targetPosition > 0) {
			return (leftPos <= targetPosition) && (rightPos <= targetPosition);
		} else if (targetPosition < 0) {
			return (leftPos >= targetPosition) && (rightPos >= targetPosition);
		} else {
			return true;
		}
	}



  @Override
  protected void useOutput(double output, double setpoint) {
    // TODO Auto-generated method stub

  }

  @Override
  protected double getMeasurement() {
    // TODO Auto-generated method stub
    return 0;
  }
}
