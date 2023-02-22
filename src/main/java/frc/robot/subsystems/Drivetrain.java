// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private MotorControllerGroup leftDrive;
  private MotorControllerGroup rightDrive;
  private DifferentialDrive robotDrive;
  private Encoder leftEncoder;
  private Encoder rightEncoder;

  private PowerDistribution pdh;

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    var leftTop = new WPI_TalonSRX(3);
    var leftBack = new WPI_TalonSRX(1);
    var leftFront = new WPI_TalonSRX(5);
    leftTop.setInverted(true);
    leftDrive = new MotorControllerGroup(leftTop, leftBack, leftFront);
    leftEncoder = new Encoder(4, 5, false, EncodingType.k4X); //come back to false bit, switch if forward is negative and vise versa
    leftEncoder.setDistancePerPulse( (Math.PI / 3.0) / 2048.0 );

    var rightTop = new WPI_TalonSRX(4);
    var rightBack = new WPI_TalonSRX(2);
    var rightFront = new WPI_TalonSRX(6);
    rightTop.setInverted(true);
    rightDrive = new MotorControllerGroup(rightTop, rightBack, rightFront);
    rightEncoder = new Encoder(6, 7, true, EncodingType.k4X); //come back to false bit, switch if forward is negative and vise versa
    rightEncoder.setDistancePerPulse( (Math.PI / 3.0) / 2048.0 );

    pdh = new PowerDistribution(10 , ModuleType.kRev);


  }

  public void driveRobot(double move, double turn) {
    if(Math.abs(move) < 0.1) { // TODO tune dead zone :)
      move = 0;
    }
    if(Math.abs(turn) < 0.1) {
      turn = 0;
    }
    
      robotDrive.arcadeDrive(-move, -turn);

  }

  public void runRight(){
    rightDrive.set(.5);
  }

  public void runLeft(){
    leftDrive.set(.5);
  }

  public void runBoth(){
    rightDrive.set(.5);
    leftDrive.set(.5);

  }

  public void stop(){
    rightDrive.stopMotor();
    leftDrive.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Right Front A", pdh.getCurrent(0));
    SmartDashboard.putNumber("Right Back A", pdh.getCurrent(0));
    SmartDashboard.putNumber("Right Top A", pdh.getCurrent(0));
    SmartDashboard.putNumber("Left Front A", pdh.getCurrent(0));
    SmartDashboard.putNumber("Left Back A", pdh.getCurrent(0));
    SmartDashboard.putNumber("Left Top A", pdh.getCurrent(0));
  }
}
