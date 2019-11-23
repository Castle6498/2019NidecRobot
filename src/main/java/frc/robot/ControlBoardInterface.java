
package frc.robot;


import frc.lib.util.DriveSignal;
import frc.robot.state_machines.BallControlHelper;
//import frc.robot.state_machines.ClimbingHelper;
import frc.robot.state_machines.BallControlHelper.CarryHeight;
import frc.robot.state_machines.BallControlHelper.PickUpHeight;
import frc.robot.state_machines.BallControlHelper.ShootHeight;

/**
 * A basic framework for robot controls that other controller classes implement
 */
public interface ControlBoardInterface {
    // DRIVER CONTROLS
    double getThrottle();

   DriveSignal getDriveSignal();

    double getTurn();

    boolean getLowGear();

    boolean getDriveInverted();

    // OPERATOR CONTROLS
    boolean getHatchPanelCentering();

    boolean getHatchPanelAlignment();

    boolean getPlateHome();

    double getHatchPanelJog();

    boolean getHatchPanelDeploy();

    boolean getHatchHardStops();

    boolean getHatchReset();

    PickUpHeight getBallPickUp();

    ShootHeight getBallShootPosition();

    boolean getBallShoot();

    CarryHeight getCarryBall();

    boolean getBallHome();

    double getLiftJog();

    double getWristJog();

    boolean getSuspensionHome();

   // ClimbingHelper.PreClimbHeight getPreClimbHeight();

    boolean getClimbLift();

    boolean getClimbStow();

    boolean getClimbFullBlast();

    double getClimbVerticleJog();

    double getClimbHorizontalJog();

    public enum Controller {Driver,Operator}
    public enum RumbleSide {left, right, both}
    void setRumble(Controller c, RumbleSide type, double amount);

    void setRumble(double amount);

    void rumbleOff();

    boolean enableMotionProfile();

   

    boolean holdMotionProfile();
  

}
