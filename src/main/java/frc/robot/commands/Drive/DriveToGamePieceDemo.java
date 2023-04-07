package frc.robot.commands.Drive;
import frc.robot.RobotContainer;

public class DriveToGamePieceDemo extends DriveToGamePiece{

    double maxRotPwr = 0.1;
    double rotPwr = 0.5;

    double maxDrivePwr = 0.1;
    double drivePwr = 0.1;

    public DriveToGamePieceDemo(RobotContainer r){
        super(r);
        super.maxRotPwr = maxRotPwr;
        super.rotPwr = rotPwr;
        super.maxDrivePwr = maxDrivePwr;
        super.drivePwr = drivePwr;
    }

}
