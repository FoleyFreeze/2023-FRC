// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Arm.ArmGoHome;
import frc.robot.commands.Arm.ArmMove;
import frc.robot.commands.Arm.LearnArmOffset;
import frc.robot.commands.Auton.AutonBuilder;
import frc.robot.commands.Auton.MPAutons.AutonCommand;
import frc.robot.commands.Combos.CamCommands;
import frc.robot.commands.Combos.Score;
import frc.robot.commands.Drive.AutoAlign;
import frc.robot.commands.Drive.CmdDrive;
import frc.robot.commands.Drive.DriveToGamePiece;
import frc.robot.commands.Drive.DriveToImageMP;
import frc.robot.commands.Drive.PreMatchAlign;
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
import frc.robot.subsystems.Inputs.Inputs.Level;
import frc.robot.subsystems.Inputs.Inputs.ManScoreMode;
import frc.robot.subsystems.Sensors.SensorCal;
import frc.robot.subsystems.Sensors.Sensors;
import frc.robot.subsystems.Sensors.Vision;
import frc.robot.util.Vector;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
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
  public Vision vision;
  public DriveTrain driveTrain;
  public Arm arm;
  public Gripper gripper;

  public DriveCal dCal;
  public InputCal iCal;

  public SendableChooser<Integer> specialAutonChooser;

  public enum AutonPaths {NOTHING, DRIVE_OUT, SCORE_DRIVE_OUT,
                          SCORE_BALANCE, SCORE_PICKUP_BALANCE,
                          SCORE_PICKUP_BALANCE_TOSS, TWO_SCORE, 
                          TWO_SCORE_BALANCE};
  public SendableChooser<AutonPaths> autonChooser;
  public enum AutonStarts {SUB, SUB_MID, FAR_MID, FAR};
  public SendableChooser<AutonStarts> autonStartPosChooser;

  public SendableChooser<Integer> simpleStartPosChooser;
  public SendableChooser<Boolean> simpleBalanceChooser;
  public SendableChooser<Boolean> driveOutOnlyChooser;

  /*public SendableChooser<Integer> startPosChooser;
  public SendableChooser<Boolean> secondPieceChooser;
  public SendableChooser<Integer> actionChooser;
  public SendableChooser<Integer> pathChooser;
  public SendableChooser<Integer> pieceChooser;*/

  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    tabs = new Tabs();

    dCal = new DriveCal();
    iCal = new InputCal();
    
    inputs = new Inputs(this, iCal);
    lights = new Lights(this, iCal);
    sensors = new Sensors(this, new SensorCal());
    vision = new Vision(this);
    vision.init();
    driveTrain = new DriveTrain(this, dCal);
    arm = new Arm(this, new ArmCal());
    gripper = new Gripper(this, new GripperCal());

    CommandScheduler cs = CommandScheduler.getInstance();
    cs.setDefaultCommand(driveTrain, new CmdDrive(this));

    specialAutonChooser = new SendableChooser<>();
    specialAutonChooser.setDefaultOption("MP Auton", 0);
    specialAutonChooser.setDefaultOption("MP Auton Camera", 1);
    specialAutonChooser.addOption("Simple Auton", 2);
    specialAutonChooser.addOption("3-Piece Clear", 3);
    specialAutonChooser.addOption("3-Piece Bump", 4);
    SmartDashboard.putData("Simplicity Chooser", specialAutonChooser);

    autonChooser = new SendableChooser<>();
    autonChooser.setDefaultOption("Do Nothing", AutonPaths.NOTHING);
    autonChooser.addOption("Drive Out", AutonPaths.DRIVE_OUT);
    autonChooser.addOption("Score and Drive Out", AutonPaths.SCORE_DRIVE_OUT);
    autonChooser.addOption("Score and Balance", AutonPaths.SCORE_BALANCE);
    autonChooser.addOption("Score, Pickup, Balance", AutonPaths.SCORE_PICKUP_BALANCE);
    autonChooser.addOption("Score, Pickup, Balance, Toss", AutonPaths.SCORE_PICKUP_BALANCE_TOSS);
    autonChooser.addOption("Two-Score", AutonPaths.TWO_SCORE);
    autonChooser.addOption("Two-Score and Balance", AutonPaths.TWO_SCORE_BALANCE);
    SmartDashboard.putData("Auton", autonChooser);

    autonStartPosChooser = new SendableChooser<>();
    autonStartPosChooser.setDefaultOption("Substation", AutonStarts.SUB);
    autonStartPosChooser.addOption("Mid-Substation", AutonStarts.SUB_MID);
    autonStartPosChooser.addOption("Mid-Far", AutonStarts.FAR_MID);
    autonStartPosChooser.addOption("Far", AutonStarts.FAR);
    SmartDashboard.putData("Start Position", autonStartPosChooser);

    simpleStartPosChooser = new SendableChooser<>();
    simpleStartPosChooser.setDefaultOption("Middle Far", 0);
    simpleStartPosChooser.addOption("Middle Substation", 1);
    simpleStartPosChooser.addOption("Far", 2);
    simpleStartPosChooser.addOption("Substation", 3);
    SmartDashboard.putData("Simple Start Position", simpleStartPosChooser);

    simpleBalanceChooser = new SendableChooser<>();
    simpleBalanceChooser.setDefaultOption("Drive Out", false);
    simpleBalanceChooser.setDefaultOption("Balance", true);
    SmartDashboard.putData("Balance Or Out", simpleBalanceChooser);

    //These are technically reversed from what the code is interpreting in inputs for the sake of ease of reading from the driver station
    /*startPosChooser = new SendableChooser<>();
    startPosChooser.setDefaultOption("Sub-Cube-Sub", 0);
    startPosChooser.addOption("Sub-Cube-Far", 1);
    startPosChooser.addOption("Sub-Cone", 2);
    startPosChooser.addOption("Mid-Cube-Sub", 3);
    startPosChooser.addOption("Mid-Cube-Far", 4);
    startPosChooser.addOption("Far-Cone", 5);
    startPosChooser.addOption("Far-Cube-Sub", 6);
    startPosChooser.addOption("Far-Cube-Far", 7);
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
    pieceChooser.setDefaultOption("Sub", 0);
    pieceChooser.addOption("Mid Sub", 1);
    pieceChooser.addOption("Mid Far", 2);
    pieceChooser.addOption("Far", 3);
    SmartDashboard.putData("Mid-Field Piece", pieceChooser);*/

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
    inputs.resetArm.onTrue(/*new LearnArmOffset(this).ignoringDisable(false)*/ new InstantCommand(() -> arm.learnArmOffset()).ignoringDisable(true));
    //inputs.resetArm.onTrue(new InstantCommand(() -> arm.learnArmOffset()).ignoringDisable(true));

    //inputs.autoGather.whileTrue(GatherCommand.gatherCommand(this));
    inputs.autoGather.and(inputs.autoScore.negate()).and(inputs.cameraModeTrigger).and(inputs.isShelfTrigger.negate()).whileTrue(CamCommands.AutoPickup(this));
    inputs.autoGather.and(inputs.autoScore.negate()).and(inputs.cameraModeTrigger).and(inputs.isShelfTrigger).whileTrue(CamCommands.AutoDriveToGatherShelf(this));
    inputs.autoGather.and(inputs.autoScore.negate()).and(inputs.cameraModeTrigger.negate()).whileTrue(GatherCommand.gatherCommand(this));

    inputs.autoGather.and(inputs.autoScore.negate()).onTrue(new InstantCommand(() -> inputs.setMode(ManScoreMode.UP)));
    inputs.autoGather.and(inputs.autoScore.negate()).onFalse(new ArmGoHome(this));

    inputs.isCubeTrigger.onTrue(new InstantCommand(gripper::open));
    inputs.isCubeTrigger.onFalse(new InstantCommand(gripper::close));

    //1. move the arm, set slow mode true
    //2. conditional command - check between cube/cone and score mode/up mode
    //3. Change up/score state in inputs
    Command manualScoreCommand = new ArmMove(this, inputs.armScorePos).andThen(new ConditionalCommand(GatherCommand.shootIntake(this, false), new WaitCommand(0), () -> (inputs.isCube() || inputs.selectedLevel == Level.BOTTOM) && inputs.scoreMode == ManScoreMode.SCORE)).andThen(new InstantCommand(() -> inputs.toggleMode()));
    inputs.autoScore.and(inputs.cameraModeTrigger).whileTrue(CamCommands.AutoDriveToScore(this));
    inputs.autoScore.and(inputs.cameraModeTrigger.negate()).onTrue(manualScoreCommand);

    inputs.balanceMode.onTrue(new InstantCommand(() -> inputs.setInchMode(true)));
    inputs.balanceMode.onFalse(new InstantCommand(() -> inputs.setInchMode(false)));

    inputs.parkMode.onTrue(new InstantCommand(() -> driveTrain.setParkMode(true)));
    inputs.parkMode.onFalse(new InstantCommand(() -> driveTrain.setParkMode(false)));

    inputs.alignMode.and(inputs.fieldAlignRight).onTrue(AutoAlign.autoFieldRightAlign(this));
    inputs.alignMode.and(inputs.fieldAlignRight.negate()).onTrue(AutoAlign.autoFieldLeftAlign(this));

    inputs.jogDown.and(inputs.shift.negate()).onTrue(new InstantCommand(arm::jogDown).ignoringDisable(true));
    inputs.jogUp.and(inputs.shift.negate()).onTrue(new InstantCommand(arm::jogUp).ignoringDisable(true));

    inputs.jogDown.and(inputs.shift).onTrue(new InstantCommand(sensors::jogBotDistNegative).ignoringDisable(true));
    inputs.jogUp.and(inputs.shift).onTrue(new InstantCommand(sensors::jogBotDistPositive).ignoringDisable(true));

    inputs.jogRight.and(inputs.shift.negate()).onTrue(new InstantCommand(arm::jogOut).ignoringDisable(true));
    inputs.jogLeft.and(inputs.shift.negate()).onTrue(new InstantCommand(arm::jogIn).ignoringDisable(true));

    inputs.jogRight.and(inputs.shift).onTrue(new InstantCommand(sensors::jogBotAngNegative));
    inputs.jogLeft.and(inputs.shift).onTrue(new InstantCommand(sensors::jogBotAngPositive));

    inputs.gather.onTrue(new ConditionalCommand(GatherCommand.shootIntake(this, false), GatherCommand.shootIntake(this, true), inputs.shift));

    SmartDashboard.putData("PI_Toggle",new InstantCommand(vision::togglePi).ignoringDisable(true));
    SmartDashboard.putData("Align Wheels 4 Auto", new PreMatchAlign(this));

    SmartDashboard.putData("Open Gipper", new InstantCommand(() -> gripper.open()));
    SmartDashboard.putData("Close Gipper", new InstantCommand(() -> gripper.close()));

    SmartDashboard.putData("DemoVisionGather", new SequentialCommandGroup(CamCommands.AutoPickupDemo(this)));

    //These are the "I don't trust Brandon" outputs
    inputs.autoGather.onTrue(new InstantCommand(() -> System.out.println("Gather Trigger")));
    inputs.autoScore.onTrue(new InstantCommand(() -> System.out.println("Score Trigger")));

    //Vector armUpVec = Vector.fromDeg(38, 110);
    //SmartDashboard.putData("ArmUp", new ArmMove(this, armUpVec));
    //SmartDashboard.putData("ArmMid", new ArmMove(this, Vector.addVectors(armUpVec, Vector.fromXY(5, 0))));
    //SmartDashboard.putData("ArmGather", new ArmMove(this, Vector.fromDeg(38, 20)));
    //SmartDashboard.putData("ArmDown", new ArmMove(this, Vector.fromDeg(34, -5)));
    //SmartDashboard.putData("ArmHome", new ArmMove(this, Vector.fromDeg(31, 0)));

    //SmartDashboard.putData("Cone Pickup", new InstantCommand(() -> gripper.setIntakePower(gripper.cals.conePickUpPower)));
    //SmartDashboard.putData("Cube Pickup", new InstantCommand(() -> gripper.setIntakePower(gripper.cals.cubePickUpPower)));
    //SmartDashboard.putData("Stop Gripper", new InstantCommand(() -> gripper.setIntakePower(0)));

    //SmartDashboard.putData("Cone Servo Position", new InstantCommand(() -> gripper.close()));
    //SmartDashboard.putData("Cube Servo Position", new InstantCommand(() -> gripper.open()));

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
