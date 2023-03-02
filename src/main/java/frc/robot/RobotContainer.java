// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Arm.ArmGoHome;
import frc.robot.commands.Arm.ArmMove;
import frc.robot.commands.Combos.Score;
import frc.robot.commands.Drive.CmdDrive;
import frc.robot.commands.Drive.ResetSwerveAngs;
import frc.robot.commands.Gripper.GatherCommand;
import frc.robot.subsystems.Arm.Arm;
import frc.robot.subsystems.Arm.ArmCal;
import frc.robot.subsystems.Drive.DriveCal;
import frc.robot.subsystems.Drive.DriveTrain;
import frc.robot.subsystems.Gripper.Gripper;
import frc.robot.subsystems.Gripper.GripperCal;
import frc.robot.subsystems.Inputs.InputCal;
import frc.robot.subsystems.Inputs.Inputs;
import frc.robot.subsystems.Inputs.Lights;
import frc.robot.subsystems.Inputs.Tabs;
import frc.robot.subsystems.Sensors.SensorCal;
import frc.robot.subsystems.Sensors.Sensors;
import frc.robot.util.Vector;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  public Tabs tabs;
  public Inputs inputs;
  public Lights lights;
  public Sensors sensors;
  public DriveTrain driveTrain;
  public Arm arm;
  public Gripper gripper;

  public DriveCal dCal;
  public InputCal iCal;

  public SendableChooser<Integer> specialAutonChooser;

  public SendableChooser<Integer> startPosChooser;
  public SendableChooser<Boolean> secondPieceChooser;
  public SendableChooser<Integer> actionChooser;
  public SendableChooser<Integer> pathChooser;
  public SendableChooser<Integer> pieceChooser;

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    tabs = new Tabs();

    dCal = new DriveCal();
    iCal = new InputCal();
    
    inputs = new Inputs(this, iCal);
    lights = new Lights(this, iCal);
    sensors = new Sensors(this, new SensorCal());
    driveTrain = new DriveTrain(this, dCal);
    arm = new Arm(this, new ArmCal());
    gripper = new Gripper(this, new GripperCal());

    CommandScheduler cs = CommandScheduler.getInstance();
    cs.setDefaultCommand(driveTrain, new CmdDrive(this));
    cs.setDefaultCommand(arm, new Score(this));

    specialAutonChooser = new SendableChooser<>();
    specialAutonChooser.setDefaultOption("No Special Command", 0);
    specialAutonChooser.addOption("Special Command", 1);
    SmartDashboard.putData("Special Chooser", specialAutonChooser);

    //These are technically reversed from what the code is interpreting in inputs for the sake of ease of reading from the driver station
    startPosChooser = new SendableChooser<>();
    startPosChooser.setDefaultOption("Right-Right", 0);
    startPosChooser.addOption("Right-Middle", 1);
    startPosChooser.addOption("Right-Left", 2);
    startPosChooser.addOption("Community-Right", 3);
    startPosChooser.addOption("Community-Middle", 4);
    startPosChooser.addOption("Community-Left", 5);
    startPosChooser.addOption("Left-Right", 6);
    startPosChooser.addOption("Left-Middle", 7);
    startPosChooser.addOption("Left-Left", 8);
    SmartDashboard.putData("Start Position", startPosChooser);

    secondPieceChooser = new SendableChooser<>();
    secondPieceChooser.setDefaultOption("Right", true);
    secondPieceChooser.addOption("Left", false);
    SmartDashboard.putData("Second Piece", secondPieceChooser);

    actionChooser = new SendableChooser<>();
    actionChooser.setDefaultOption("Do Nothing", 0);
    actionChooser.addOption("Drive", 1);
    actionChooser.addOption("1-Score Drive", 2);
    actionChooser.addOption("1-Score Park", 3);
    actionChooser.addOption("2-Score", 4);
    actionChooser.addOption("2-Score Park", 5);
    SmartDashboard.putData("Auton", actionChooser);

    pathChooser = new SendableChooser<>();
    pathChooser.setDefaultOption("Substation", 0);
    pathChooser.addOption("Charge Station", 1);
    pathChooser.addOption("Wall", 2);
    SmartDashboard.putData("Path", pathChooser);

    pieceChooser = new SendableChooser<>();
    pieceChooser.setDefaultOption("Left", 0);
    pieceChooser.addOption("Mid Left", 1);
    pieceChooser.addOption("Mid Right", 2);
    pieceChooser.addOption("Right", 3);
    SmartDashboard.putData("Mid-Field Piece", pieceChooser);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  
  
  private void configureBindings() {
    inputs.resetSwerveZeros.whileTrue(new ResetSwerveAngs(this).ignoringDisable(true));//down on both blue jog doo-hickeys for 5 seconds
    inputs.resetAngle.whileTrue(new InstantCommand(sensors::resetBotAng).ignoringDisable(true));//up on the left blue jog doo-hickey
    inputs.resetPosition.whileTrue(new InstantCommand(sensors::resetBotPos).ignoringDisable(true));//up on the left blue jog doo-hickey
    inputs.resetArm.whileTrue(new InstantCommand(arm::learnArmOffset).ignoringDisable(true));

    inputs.autoGather.whileTrue(GatherCommand.gatherCommand(this));
    inputs.autoGather.onTrue(new InstantCommand(() -> inputs.slowModeTrue()));
    inputs.autoGather.onFalse(new InstantCommand(() -> inputs.slowModeFalse()));
    inputs.autoGather.onFalse(new ArmGoHome(this));

    inputs.jogDown.onTrue(new InstantCommand(arm::jogDown).ignoringDisable(true));
    inputs.jogUp.onTrue(new InstantCommand(arm::jogUp).ignoringDisable(true));
    inputs.jogRight.onTrue(new InstantCommand(arm::jogOut).ignoringDisable(true));
    inputs.jogLeft.onTrue(new InstantCommand(arm::jogIn).ignoringDisable(true));


    Vector armUpVec = Vector.fromDeg(38, 110);
    SmartDashboard.putData("ArmUp", new ArmMove(this, armUpVec));
    SmartDashboard.putData("ArmMid", new ArmMove(this, Vector.addVectors(armUpVec, Vector.fromXY(5, 0))));
    SmartDashboard.putData("ArmGather", new ArmMove(this, Vector.fromDeg(38, 20)));
    SmartDashboard.putData("ArmDown", new ArmMove(this, Vector.fromDeg(34, -5)));
    SmartDashboard.putData("ArmHome", new ArmMove(this, Vector.fromDeg(31, 0)));

    //SmartDashboard.putData("Cone Pickup", new InstantCommand(() -> gripper.setIntakePower(gripper.cals.conePickUpPower)));
    //SmartDashboard.putData("Cube Pickup", new InstantCommand(() -> gripper.setIntakePower(gripper.cals.cubePickUpPower)));
    //SmartDashboard.putData("Stop Gripper", new InstantCommand(() -> gripper.setIntakePower(0)));

    SmartDashboard.putData("Cone Servo Position", new InstantCommand(() -> gripper.close()));
    SmartDashboard.putData("Cube Servo Position", new InstantCommand(() -> gripper.open()));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command autonCommand = null;

  public Command getAutonomousCommand() {
    return autonCommand;
  }
}
