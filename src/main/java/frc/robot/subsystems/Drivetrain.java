// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
    var leftTop = new WPI_TalonSRX(7);
    var leftBack = new WPI_TalonSRX(3);
    var leftFront = new WPI_TalonSRX(2);
    leftTop.setInverted(true);
    leftDrive = new MotorControllerGroup(leftTop, leftBack, leftFront);
    leftEncoder = new Encoder(0, 1, false, EncodingType.k4X); //come back to false bit, switch if forward is negative and vise versa
    leftEncoder.setDistancePerPulse( (Math.PI / 3.0) / 2048.0 );

    var rightTop = new WPI_TalonSRX(4);
    var rightBack = new WPI_TalonSRX(6);
    var rightFront = new WPI_TalonSRX(5);
    rightTop.setInverted(true);
    rightDrive = new MotorControllerGroup(rightTop, rightBack, rightFront);
    rightDrive.setInverted(true);
    rightEncoder = new Encoder(2, 3, true, EncodingType.k4X); //come back to false bit, switch if forward is negative and vise versa
    rightEncoder.setDistancePerPulse( (Math.PI / 3.0) / 2048.0 );

    pdh = new PowerDistribution();//This needs to be CAN 1 so one of the motors will need to be changed to another id temporarily
    robotDrive = new DifferentialDrive(leftDrive, rightDrive);

  }

  public Command runConstantSpeed(int choose){
    if(choose == 0){
      //return this.startEnd(() -> rightDrive.set(.5), () -> rightDrive.set(0));
      return this.run(() -> rightDrive.set(.5));
    }
    else if(choose == 1){
      return this.startEnd(() -> leftDrive.set(.5), () -> leftDrive.set(0));
    }
    else if(choose == 2){
      return this.run(() -> driveRobot(1, 0));
    }
  else {
    return this.runOnce(() -> driveRobot(0, 0));
  }
  }

  public void driveRobot(double move, double turn) {
    if(Math.abs(move) < 0.1) { // TODO tune dead zone :)
      move = 0;
    }
    if(Math.abs(turn) < 0.1) {
      turn = 0;
    }
    
      robotDrive.arcadeDrive(move, turn);

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
    SmartDashboard.putNumber("Right Front A", pdh.getCurrent(5));
    SmartDashboard.putNumber("Right Back A", pdh.getCurrent(6));
    SmartDashboard.putNumber("Right Top A", pdh.getCurrent(4));
    SmartDashboard.putNumber("Left Front A", pdh.getCurrent(2));
    SmartDashboard.putNumber("Left Back A", pdh.getCurrent(3));
    SmartDashboard.putNumber("Left Top A", pdh.getCurrent(7));

    SmartDashboard.putNumber("Left Encoder", leftEncoder.getRate());
    SmartDashboard.putNumber("Right Encoder", rightEncoder.getRate());
  }
}
