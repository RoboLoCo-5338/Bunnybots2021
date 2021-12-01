// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.commands.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class DriveSystem extends PIDSubsystem {

  public final WPI_TalonSRX leftFront = new WPI_TalonSRX(14); // TODO: change Talon ID
  public final WPI_TalonSRX leftRear = new WPI_TalonSRX(2); // TODO: change Talon ID
  public final WPI_TalonSRX rightFront = new WPI_TalonSRX(3); // TODO: change Talon ID
  public final WPI_TalonSRX rightRear = new WPI_TalonSRX(1); // TODO: change Talon ID

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
    for (final WPI_TalonSRX talon : new WPI_TalonSRX[] { this.leftFront, this.leftRear, this.rightFront, this.rightRear }) {
      DriveSystem.configureTalon(talon);
    }
    leftFront.setInverted(true);
    leftRear.setInverted(true);
    setDefaultCommand(new drive(this));
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
  
      // max rpm of wheels desired
      double targetVelocity = 400;
  
      // target speed in encoder units based on joystick position
      targetLeft = left * targetVelocity * 4096 / 600.0;
      targetRight = right * targetVelocity * 4096 / 600.0;
  
      // set target speeds to motors
      this.leftFront.set(ControlMode.Velocity, targetLeft);
      this.rightFront.set(ControlMode.Velocity, targetRight);
    }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // Runs the drive in tank mode
  public void drive(OI oi) {
    this.DRIVE.tankDrive(oi.getLeftJoystick('Y') * 0.5, oi.getRightJoystick('Y') * 0.5);
  }

  private static void configureTalon(final WPI_TalonSRX talon) {
    talon.configPeakCurrentLimit(100, 0);
    talon.configPeakCurrentDuration(3, 0);
    talon.configContinuousCurrentLimit(80, 0);
    talon.enableCurrentLimit(true);
    talon.configNeutralDeadband(0.001, 0);
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
