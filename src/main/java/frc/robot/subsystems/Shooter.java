// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Configs;
import frc.robot.Constants;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterHood;

public class Shooter extends SubsystemBase {

  private SparkFlex shooterFlyWheelLeft;
  private SparkFlex shooterFlyWheelMiddle;
  private SparkFlex shooterFlyWheelRight;

  private RelativeEncoder shooterFlyWheelLeftRelativeEncoder;
  private RelativeEncoder shooterFlyWheelMiddleRelativeEncoder;
  private RelativeEncoder shooterFlyWheelRightRelativeEncoder;

  private SparkFlexConfig shooterFlyWheelLeftConfig;
  private SparkFlexConfig shooterFlyWheelMiddleConfig;
  private SparkFlexConfig shooterFlyWheelRightConfig;

  // Single shared WPILib PID controller for all three flywheels.
  // Since all motors target the same RPM, one controller is sufficient.
  // Tune kP, kI, kD in Constants.ShooterConstants.
  private PIDController shooterFlyWheelPIDController;

  private DriveSubsystem kDrive;
  private ShooterHood kShooterHood;
  private Constants kConstants;
  private double targetRPM;
  private double holdVoltage;
  private double voltage;
  private InterpolatingDoubleTreeMap kFlywheelMap;

  /** Creates a new Shooter. */
  public Shooter(DriveSubsystem kDrive, ShooterHood kShooterHood) {
    kConstants = new Constants();
    this.kDrive = kDrive;
    this.kShooterHood = kShooterHood;


    shooterFlyWheelLeft = new SparkFlex(
        Constants.ShooterConstants.kShooterFlyWheelLeftCanId,
        MotorType.kBrushless);
    shooterFlyWheelLeftRelativeEncoder = shooterFlyWheelLeft.getEncoder();


    shooterFlyWheelMiddle = new SparkFlex(
        Constants.ShooterConstants.kShooterFlyWheelMiddleCanId,
        MotorType.kBrushless);
    shooterFlyWheelMiddleRelativeEncoder = shooterFlyWheelMiddle.getEncoder();


    shooterFlyWheelRight = new SparkFlex(
        Constants.ShooterConstants.kShooterFlyWheelRightCanId,
        MotorType.kBrushless);
    shooterFlyWheelRightRelativeEncoder = shooterFlyWheelRight.getEncoder();


    shooterFlyWheelLeftConfig = Configs.ShooterConfigs.ShooterFlyWheelLeftConfig;
    shooterFlyWheelMiddleConfig = Configs.ShooterConfigs.ShooterFlyWheelMiddleConfig;
    shooterFlyWheelRightConfig = Configs.ShooterConfigs.ShooterFlyWheelRightConfig;

    shooterFlyWheelLeft.configure(
        shooterFlyWheelLeftConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    shooterFlyWheelMiddle.configure(
        shooterFlyWheelMiddleConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);

    shooterFlyWheelRight.configure(
        shooterFlyWheelRightConfig,
        ResetMode.kNoResetSafeParameters,
        PersistMode.kPersistParameters);


    shooterFlyWheelPIDController = new PIDController(
        Constants.ShooterConstants.kFlyWheelP,
        Constants.ShooterConstants.kFlyWheelI,
        Constants.ShooterConstants.kFlyWheelD);

    //SimpleMotorFeedforward feed = new SimpleMotorFeedforward(0,);


    shooterFlyWheelPIDController.setTolerance(Constants.ShooterConstants.kFlyWheelToleranceRPM);
    
  }

  // Set the voltage of the flywheel motors to control the speed of the flywheels
  public void setFlyWheelRPM(double targetRPM) {
    this.targetRPM = targetRPM;
  }

  public double getHoldVoltage(){
      return Constants.ShooterConstants.kFlywheelMap.get(3.0);
  }

  public void setFlyWheelVoltage(double voltage) {
    shooterFlyWheelLeft.setVoltage(voltage);
    shooterFlyWheelMiddle.setVoltage(voltage);
    shooterFlyWheelRight.setVoltage(voltage);
  }

  public void shootAtTarget(Translation2d point){
    double distance = kDrive.getPose().getTranslation().getDistance(point);
    double targetRPM = Constants.ShooterConstants.kFlywheelRPMMap.get(distance);
    double targetHoodAngle = Constants.ShooterConstants.kHoodAngleMap.get(distance);
    kShooterHood.setHoodAngle(targetHoodAngle);
    setFlyWheelRPM(targetRPM);
  }

 
  public boolean isAtSetpoint() {
    return shooterFlyWheelPIDController.atSetpoint();
  }


  // public void setFlyWheelVoltage(double voltage) {
  //   shooterFlyWheelLeft.setVoltage(voltage);
  //   shooterFlyWheelMiddle.setVoltage(voltage);
  //   shooterFlyWheelRight.setVoltage(voltage);
  // }


  public double getFlyWheelVelocity() {
    return (shooterFlyWheelLeft.getEncoder().getVelocity() +
            shooterFlyWheelMiddle.getEncoder().getVelocity() +
            shooterFlyWheelRight.getEncoder().getVelocity()) / 3.0;
  }

  public void shootStationary(double x, double y) {
    Translation2d diff =
        new Translation2d(x, y).minus(kDrive.getPose().getTranslation());

    double distance = diff.getNorm();
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("requested shooter RPM", targetRPM);
    SmartDashboard.putNumber("shooter RPM", getFlyWheelVelocity());
    SmartDashboard.putNumber("shooter votage", voltage);
    if(targetRPM == 0){

      shooterFlyWheelLeft.setVoltage(0);
      shooterFlyWheelMiddle.setVoltage(0);
      shooterFlyWheelRight.setVoltage(0);
      
    } else {
      voltage = Math.max(-12.0, Math.min(
        12.0, shooterFlyWheelPIDController.calculate(
          getFlyWheelVelocity(), targetRPM)));

      holdVoltage = Constants.ShooterConstants.kFlywheelMap.get(targetRPM);

      shooterFlyWheelLeft.setVoltage(voltage + holdVoltage);
      shooterFlyWheelMiddle.setVoltage(voltage + holdVoltage);
      shooterFlyWheelRight.setVoltage(voltage + holdVoltage);
    }
    // This method will be called once per scheduler run
  }
}