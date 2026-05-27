// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Delivery;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FieldPoints;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.ShooterDelivery;
import frc.robot.subsystems.ShooterHood;
import frc.robot.subsystems.intakeSpin;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import java.util.jar.Attributes.Name;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  private SendableChooser<String> modeChooser;
  private SendableChooser<Integer> portChooser;
  private SendableChooser<Double> speedChooser;
  
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Intake m_intake = new Intake();
  private final intakeSpin m_intakeSpin = new intakeSpin();
  private final ShooterHood m_shooterHood = new ShooterHood();
  private final Shooter m_shooter = new Shooter(m_robotDrive, m_shooterHood);
  private final Delivery m_delivery = new Delivery();
  private final ShooterDelivery m_shooterDelivery = new ShooterDelivery();
  private final Climber m_climber = new Climber();

  // The driver's controller
  CommandXboxController m_driverController;
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    
    portChooser = new SendableChooser<>();
    portChooser.setDefaultOption("Port 0", 0);
    portChooser.addOption("Port 1", 1);
    SmartDashboard.putData("Port Chooser", portChooser);
    speedChooser = new SendableChooser<>();
    speedChooser.setDefaultOption("100%", 1.0);
    speedChooser.addOption("90%", 0.9);
    speedChooser.addOption("80%", 0.8);
    speedChooser.addOption("70%", 0.7);
    speedChooser.addOption("60%", 0.6);
    speedChooser.addOption("50%", 0.5);
    speedChooser.addOption("40%", 0.4);
    speedChooser.addOption("30%", 0.3);
    speedChooser.addOption("20%", 0.2);
    speedChooser.addOption("10%", 0.1);
    SmartDashboard.putData("Speed Chooser", speedChooser);
    m_driverController = new CommandXboxController(portChooser.getSelected());

    registerNamedCommands();
    // Configure the button bindings
    configureButtonBindings();

    // m_shooter.setDefaultCommand(new RunCommand(() -> m_shooter.setFlyWheelRPM(0), m_shooter));
    // m_delivery.setDefaultCommand(new RunCommand(() -> m_delivery.setDeliveryVoltage(0), m_delivery));
    // m_shooterDelivery.setDefaultCommand(new RunCommand(() -> m_shooterDelivery.setDeliveryVoltage(0), m_shooterDelivery));
    // m_climber.setDefaultCommand(new RunCommand(() -> m_climber.setVoltage(0), m_climber));
    // m_intake.setDefaultCommand(new RunCommand(() -> m_intake.setIntakeVoltage(0), m_intake));
    m_shooterHood.setDefaultCommand(new RunCommand(() -> m_shooterHood.setHoodAngle(10), m_shooterHood));
    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY() * 1, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX() * 1, OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX() * 1, OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
    // For convenience a programmer could change this when going to competition.
    boolean isCompetition = true;

    // Build an auto chooser. This will use Commands.none() as the default option.
    // As an example, this will only show autos that start with "comp" while at
    // competition as defined by the programmer
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      (stream) -> isCompetition
        ? stream.filter(auto -> auto.getName().startsWith(""))
        : stream
    );
    modeChooser = new SendableChooser<>(); 
    modeChooser.setDefaultOption("Just Driver", "Just Driver");
    modeChooser.addOption("Driver & Operator", "Driver & Operator");

    

    StructPublisher<Pose2d> publisher = NetworkTableInstance.getDefault()
      .getStructTopic("MyPose", Pose2d.struct).publish();
    StructArrayPublisher<Pose2d> arrayPublisher = NetworkTableInstance.getDefault()
    .getStructArrayTopic("MyPoseArray", Pose2d.struct).publish();


    SmartDashboard.putData("Auto Chooser", autoChooser);
    SmartDashboard.putData("Controller Mode Chooser", modeChooser);

  }

  private void registerNamedCommands(){
    // NamedCommands.registerCommand("ShootyMcShootFace", new InstantCommand(
    //   () -> m_shooter.shootAtTarget(FieldPoints.getHubPosition()), m_shooter)
    //   .alongWith(new RunCommand(
    //     () -> m_robotDrive.turnToFieldPoint(FieldPoints.getHubPosition(), m_driverController), m_robotDrive))
    //     .until(() -> m_shooter.isAtSetpoint() && m_robotDrive.isAtTurnTarget())
    //     // Cleanly group the feed mechanisms together so they wait for the aim to finish
    //     .andThen(Commands.parallel(
    //       new RunCommand(() -> m_delivery.stutter(Constants.DeliveryConstants.kDeliveryVoltage), m_delivery),
    //       new RunCommand(() -> m_shooterDelivery.setDeliveryVoltage(Constants.ShooterConstants.kShooterDeliveryVoltage), m_shooterDelivery)
    //     ))
    // );

    // //intake out
    // NamedCommands.registerCommand("IntakeOut", new RunCommand(
    //   () -> m_intake.setIntakePosition(Constants.IntakeConstants.IntakePosition.kGround), m_intake));

    //   //intake fuel
    // NamedCommands.registerCommand("IntakeBall", new RunCommand(
    //   () -> m_intakeSpin.setIntakeVoltage(12), m_intakeSpin));

    // //intake in
    // NamedCommands.registerCommand("IntakeIn", new RunCommand(
    //   ()-> m_intake.setIntakePosition(Constants.IntakeConstants.IntakePosition.kStowed), m_intake));

    //   //shooter delivery
    //   NamedCommands.registerCommand("Delivery", new RunCommand(
    //     ()-> m_delivery.stutter(10), m_delivery));

    //   //stop shooter, shooter delivery, and delivery
    //   NamedCommands.registerCommand("StoppyMcStopFace", new RunCommand(
    //     ()-> m_shooter.setFlyWheelVoltage(0), m_shooter).alongWith(new RunCommand(
    //       ()-> m_shooterDelivery.setDeliveryVoltage(0), m_shooterDelivery)).alongWith(new RunCommand(
    //         ()-> m_delivery.stutter(0), m_delivery)));
      
    

    // NamedCommands.registerCommand("IntakeOut", new RunCommand(
    //   () -> m_intake.setIntakePosition(Constants.IntakeConstants.IntakePosition.kGround), m_intake));

    // NamedCommands.registerCommand("IntakeIn", new RunCommand(
    //   () -> m_intake.setIntakePosition(Constants.IntakeConstants.IntakePosition.kStowed), m_intake));

    // NamedCommands.registerCommand("IntakeIntake", new RunCommand(
    //   () -> m_intake.setIntakeVoltage(12), m_intake));

    // NamedCommands.registerCommand("IntakeOuttake", new RunCommand(
    //   () -> m_intake.setIntakeVoltage(-12), m_intake));

    // NamedCommands.registerCommand("ShooterTower", new RunCommand(
    //   () -> m_shooter.setFlyWheelRPM(2850), m_shooter).alongWith(new RunCommand(
    //     () ->m_shooterHood.setHoodAngle(50))
    //   ));

    // NamedCommands.registerCommand("ShooterHub", new RunCommand(
    //   () -> m_shooter.setFlyWheelRPM(2500), m_shooter).alongWith(new RunCommand(
    //     () ->m_shooterHood.setHoodAngle(25))
    //   ));

    // NamedCommands.registerCommand("ShooterDeliveryIn", new RunCommand(
    //   () -> m_shooterDelivery.setDeliveryVoltage(12), m_shooter));

    // NamedCommands.registerCommand("ShooterDeliveryOut", new RunCommand(
    //   () -> m_shooterDelivery.setDeliveryVoltage(-12), m_shooter));

    // NamedCommands.registerCommand("DeliveryIn", new RunCommand(
    //   () -> m_delivery.stutter(10), m_delivery));
    
    // NamedCommands.registerCommand("DeliveryOut", new RunCommand(
    //   () -> m_delivery.stutter(-12), m_delivery));

    // NamedCommands.registerCommand("ShooterReset", new RunCommand(
    //   () -> m_shooter.setFlyWheelRPM(0), m_shooter).alongWith(new RunCommand(
    //     () ->m_shooterHood.setHoodAngle(31.25))
    //   ));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    // m_driverController.a().whileTrue(new RunCommand(
    //   () -> m_robotDrive.turnToFieldPoint(FieldPoints.getHubPosition().getX(), FieldPoints.getHubPosition().getY(), m_driverController),
    //    m_robotDrive));

    // m_driverController.a().onTrue(new RunCommand(
    //   () -> m_intake.setIntakePosition(Constants.IntakeConstants.IntakePosition.kGround), m_intake));

    // m_driverController.y().onTrue(new RunCommand(
    //   () -> m_intake.setIntakePosition(Constants.IntakeConstants.IntakePosition.kStowed), m_intake));

    // m_driverController.leftTrigger().onTrue(new RunCommand(
    //   () -> m_intake.setIntakeVoltage(12), m_intake)).onFalse(new RunCommand(
    //     () -> m_intake.setIntakeVoltage(0), m_intake));
        
    // m_driverController.b().onTrue(new RunCommand(
    //   () -> m_intake.setIntakeVoltage(-12), m_intake)
    //   .alongWith(new RunCommand(
    //   () -> m_shooterDelivery.setDeliveryVoltage(-12), m_shooterDelivery)
    //   ).alongWith(new RunCommand(
    //   () -> m_delivery.setDeliveryVoltage(-12), m_delivery)
    //   ).alongWith(
    //     new RunCommand(
    //   () -> m_shooter.setFlyWheelVoltage(-12), m_shooter)

    //   )).onFalse(new RunCommand(
    //   () -> m_intake.setIntakeVoltage(0), m_intake).alongWith(
    //     new RunCommand(
    //   () -> m_shooterDelivery.setDeliveryVoltage(0), m_shooterDelivery)
    //   ).alongWith(new RunCommand(
    //   () -> m_delivery.setDeliveryVoltage(0), m_delivery))
    //   .alongWith(
    //     new RunCommand(
    //   () -> m_shooter.setFlyWheelRPM(0), m_shooter)
    //   )
    //   );
    
    // m_driverController.leftBumper().onTrue(new RunCommand(
    //   () -> m_shooter.setFlyWheelRPM(2850), m_shooter).alongWith(new RunCommand(
    //     () ->m_shooterHood.setHoodAngle(50))
    //   )).onFalse(new RunCommand(
    //     () -> m_shooter.setFlyWheelRPM(0), m_shooter).alongWith(new RunCommand(
    //     () ->m_shooterHood.setHoodAngle(31.25))
    //   ));


      //   m_driverController.povUp().onTrue(new RunCommand(
      // () -> m_shooter.setFlyWheelRPM(2500), m_shooter).alongWith(new RunCommand(
      //   () ->m_shooterHood.setHoodAngle(25))
      // )).onFalse(new RunCommand(
      //   () -> m_shooter.setFlyWheelRPM(0), m_shooter).alongWith(new RunCommand(
      //   () ->m_shooterHood.setHoodAngle(31.25))
      // ));

    // m_driverController.rightBumper().onTrue(new RunCommand(
    //   () -> m_shooterDelivery.setDeliveryVoltage(12), m_shooter)).onFalse(new RunCommand(
    //     () -> m_shooterDelivery.setDeliveryVoltage(0), m_shooter));
    
    // m_driverController.rightTrigger().onTrue(new RunCommand(
    //   () -> m_delivery.stutter(-10), m_delivery)).onFalse(new RunCommand(
    //     () -> m_delivery.stutter(0), m_delivery));

    // m_driverController.x().whileTrue(new RunCommand(
    //   () -> m_robotDrive.turnToFieldPoint(
    //     FieldPoints.getHubPosition().getX(),
    //      FieldPoints.getHubPosition().getY(), 
    //      m_driverController), m_robotDrive));

    //shoot at hub
//     m_driverController.leftBumper().onTrue(new RunCommand(
//       () -> m_shooter.shootAtTarget(FieldPoints.getHubPosition()), m_shooter)
//       .alongWith(new RunCommand(
//         () -> m_robotDrive.turnToFieldPoint(FieldPoints.getHubPosition(), m_driverController), m_robotDrive)))
//         .onFalse(new RunCommand(
//       () -> m_shooter.setFlyWheelRPM(0), m_shooter)
//       .alongWith(new RunCommand(
//             () -> m_robotDrive.drive(
//                 -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
//                 -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
//                 -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
//                 true),
//             m_robotDrive)));

    m_driverController.rightBumper()
      .onTrue(
        new RunCommand(
          () -> m_delivery.stutter(Constants.DeliveryConstants.kDeliveryVoltage), 
          m_delivery
        )
        .alongWith(
          new RunCommand(
            () -> m_shooterDelivery.setDeliveryVoltage(
              Constants.ShooterConstants.kShooterDeliveryVoltage
            ), 
            m_shooterDelivery
          )
        )
      )
      .onFalse(
        new RunCommand(
          () -> m_delivery.stutter(0), 
          m_delivery
        )
          .alongWith(
            new RunCommand(
              () -> m_shooterDelivery.setDeliveryVoltage(0), 
              m_shooterDelivery
            )
          )
      );

//     //shoot at depot shuttle
//     // m_driverController.rightBumper().whileTrue(new RunCommand(
//     //   () -> m_shooter.shootAtTarget(FieldPoints.getDepotShuttle()), m_shooter)
//     //   .alongWith(new RunCommand(
//     //     () -> m_robotDrive.turnToFieldPoint(FieldPoints.getDepotShuttle(), m_driverController), m_robotDrive))
//     //     .until(
//     //       () -> m_shooter.isAtSetpoint() && m_robotDrive.isAtTurnTarget())
//     //     .andThen(new RunCommand(
//     //       () -> m_delivery.stutter(Constants.DeliveryConstants.kDeliveryVoltage), m_delivery))
//     //       .alongWith(new RunCommand(() -> m_shooterDelivery.setDeliveryVoltage(Constants.ShooterConstants.kShooterDeliveryVoltage), m_shooterDelivery)));

//     // Reset heading
    m_driverController.start().onTrue(Commands.runOnce(
      () -> m_robotDrive.zeroHeading()
      , m_robotDrive));

    // // Put intake out
    //  m_driverController.rightStick().onTrue(new RunCommand(
    //   () -> m_intake.setIntakePosition(Constants.IntakeConstants.IntakePosition.kGround), m_intake));

    //       m_driverController.leftStick().onTrue(new RunCommand(
    //   () -> m_intake.setIntakePosition(Constants.IntakeConstants.IntakePosition.kStowed), m_intake));




// //  m_driverController.a().whileTrue(
// //   new ConditionalCommand(
// //     new RunCommand(() -> m_intake.setIntakePosition(Constants.IntakeConstants.IntakePosition.kGround), m_intake),
// //     new RunCommand(() -> m_intake.setIntakePosition(Constants.IntakeConstants.IntakePosition.kStowed), m_intake),
// //     m_intake::isStowed  // returns true if stowed → deploy, false if deployed → stow
// //   )
// // );

    //intake
    m_driverController.leftTrigger().onTrue(new RunCommand(
      () -> m_intakeSpin.setIntakeVoltage(12), m_intakeSpin))
    .onFalse(new RunCommand(
        () -> m_intakeSpin.setIntakeVoltage(0), m_intakeSpin));
    
    //clear blockage
    m_driverController.b().onTrue(new RunCommand(
      () -> m_intakeSpin.setIntakeVoltage(-12), m_intake)
      .alongWith(new RunCommand(() -> m_delivery.stutter(-12)))
      .alongWith(new RunCommand(() -> m_shooter.setFlyWheelVoltage(-12)))
      .alongWith(new RunCommand(() -> m_shooterDelivery.setDeliveryVoltage(-12))))
    .onFalse(new RunCommand(
      () -> m_intakeSpin.setIntakeVoltage(0), m_intake)
      .alongWith(new RunCommand(() -> m_delivery.stutter(0)))
      .alongWith(new RunCommand(() -> m_shooter.setFlyWheelVoltage(0)))
      .alongWith(new RunCommand(() -> m_shooterDelivery.setDeliveryVoltage(0))));
      // .onFalse(new RunCommand(() -> m_intake.setIntakeVoltage(0), m_intake))
      // .onFalse(new RunCommand(() -> m_delivery.setDeliveryVoltage(0), m_delivery))
      // .onFalse(new RunCommand(() -> m_shooter.setFlyWheelVoltage(0)));

    //put climber up
    // m_driverController.x().onTrue(new RunCommand(
    //   () -> m_climber.setVoltage(4), m_climber)).onFalse(new RunCommand(
    //     () -> m_climber.setVoltage(0), m_climber));

    // //put climber down
    // m_driverController.y().onTrue(new RunCommand(
    //   () -> m_climber.setVoltage(-4), m_climber)).onFalse(new RunCommand(
    //     () -> m_climber.setVoltage(0), m_climber));
    
    // spinny mc spinface
    m_driverController.rightTrigger()
      .onTrue(
        new RunCommand(
          () -> m_shooter.setFlyWheelVoltage(12), 
          m_shooter
        )
      )
      .onFalse(
        new RunCommand(
          () -> m_shooter.setFlyWheelVoltage(0), 
          m_shooter
        )
      );
    


    
      
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
  public void configureController() {
    m_driverController = new CommandXboxController(portChooser.getSelected());
    configureButtonBindings();
  }

}
