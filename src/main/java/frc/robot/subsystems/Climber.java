package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.Configs;

public class Climber extends SubsystemBase {
  // SparkFlexes
  private SparkFlex leftClimber;
  private SparkFlex rightClimber;

  // Sensors
  private DigitalInput leftHallSensor;
  private DigitalInput rightHallSensor;

  // Configs for the SparkFlexes - loaded from Configs
  private SparkFlexConfig leftClimberConfig;
  private SparkFlexConfig rightClimberConfig;
  
  // PID Stuff
  private PIDController positionPid;
  private double targetPosition = 0.0;
  private boolean positionControlEnabled = false;

  // Sensor Status variables
  private boolean leftHallSensorTriggered = false;
  private boolean rightHallSensorTriggered = false;

  /** Creates a new Climber. */
  public Climber() {
    leftHallSensor = new DigitalInput(0);
    rightHallSensor = new DigitalInput(1);


    leftClimber = new SparkFlex(
      Constants.ClimberConstants.kClimberLeftCanId, 
      com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless
    );

    rightClimber = new SparkFlex(
      Constants.ClimberConstants.kClimberRightCanId, 
      com.revrobotics.spark.SparkLowLevel.MotorType.kBrushless
    );


    // Load configs
    leftClimberConfig = Configs.ClimberConfigs.leftClimberConfig;

    rightClimberConfig = Configs.ClimberConfigs.rightClimberConfig;


    //Config motors - Deprecated in 2027
    leftClimber.configure(
      leftClimberConfig, 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters
    );

    rightClimber.configure(
      rightClimberConfig, 
      ResetMode.kNoResetSafeParameters, 
      PersistMode.kPersistParameters
    );


    // Position PID controller (units = encoder position units)
    positionPid = new PIDController(
      Constants.ClimberConstants.kClimberP,
      Constants.ClimberConstants.kClimberI,
      Constants.ClimberConstants.kClimberD
    );

    // Sets how tolerant the PID is to slight errors
    positionPid.setTolerance(Constants.ClimberConstants.kClimberPositionTolerance);

  }


  @Override
  public void periodic() {
      isLeftHallSensorTriggered();
      isRightHallSensorTriggered();

      if (rightHallSensorTriggered) {
        stopRight();
      }

      if (leftHallSensorTriggered) {
        stopLeft();
      }

      // This method will be called once per scheduler run
      if (positionControlEnabled) {

        double leftPos;
        double rightPos;

        try {
          leftPos = leftClimber.getEncoder().getPosition();
          rightPos = rightClimber.getEncoder().getPosition();
        } 

        catch (Exception ex) {
          positionControlEnabled = false;
          return;
        }

        double currentPos = (leftPos + rightPos) / 2.0;
        double output = positionPid.calculate(currentPos, targetPosition);


        // Limit maximum voltage
        double maxV = Constants.ClimberConstants.kClimberMaxVoltage;
        if (output > maxV) {
          output = maxV;
        } else if (output < -maxV) {
          output = -maxV;
        }

        setVoltage(output);
      }
  }


  public void setVoltage(double voltage) {
    leftClimber.setVoltage(voltage);
    rightClimber.setVoltage(voltage);
  }


  public void setTargetPositionRotations(double position) {
    targetPosition = position;
    positionPid.reset();
    positionControlEnabled = true;
  }

  
  public void disablePositionControl() {
    positionControlEnabled = false;
  }

  
  public void enablePositionControl() {
    positionPid.reset();
    positionControlEnabled = true;
  }


  /** Returns true when position PID reports we're at the setpoint. */
  public boolean atSetpoint() {
    return positionPid.atSetpoint();
  }

  
  public void zeroEncoders() {
    try {
      leftClimber.getEncoder().setPosition(0.0);
    } catch (Exception ignore) {
      // If the hardware API doesn't support setPosition, ignore (will be caught at compile/run if needed)
    }
    try {
      rightClimber.getEncoder().setPosition(0.0);
    } catch (Exception ignore) {
    }
  }


  public void stop() {
    setVoltage(0.0);
    disablePositionControl();
  }

  
  public void stopRight() {
    rightClimber.setVoltage(0);
  }

  public void stopLeft() {
    leftClimber.setVoltage(0);
  }

  public boolean isLeftHallSensorTriggered() {
    leftHallSensorTriggered = !leftHallSensor.get(); // Assuming active low
    return leftHallSensorTriggered;
  }


  public boolean isRightHallSensorTriggered() {
    rightHallSensorTriggered = !rightHallSensor.get(); // Assuming active low
    return rightHallSensorTriggered;
  }


}