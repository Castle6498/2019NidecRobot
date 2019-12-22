
package frc.robot;


import frc.lib.util.DriveSignal;
//import frc.robot.state_machines.BallControlHelper;
//import frc.robot.state_machines.ClimbingHelper;
//import frc.robot.state_machines.BallControlHelper.CarryHeight;
//import frc.robot.state_machines.BallControlHelper.PickUpHeight;
//import frc.robot.state_machines.BallControlHelper.ShootHeight;

/**
 * A basic framework for robot controls that other controller classes implement
 */
public interface ControlBoardInterface {
    // DRIVER CONTROLS
    double getThrottle();

   DriveSignal getDriveSignal();

    double getTurn();

   

    boolean getDriveInverted();

    boolean getStartIntake();

    boolean getStopIntake();

    boolean getUnfoldIntake();

    double getHoodAngle();

    boolean getUpdateHoodAngle();

    boolean getStartShoot();

    boolean getStopShoot();

    public enum Controller {Driver,Operator}
    public enum RumbleSide {left, right, both}
    void setRumble(Controller c, RumbleSide type, double amount);

    void setRumble(double amount);

    void rumbleOff();
  

}
