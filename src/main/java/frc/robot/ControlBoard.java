package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import frc.lib.util.DriveSignal;
import frc.robot.CameraVision.StreamMode;
//import frc.robot.state_machines.ClimbingHelper;
import frc.robot.state_machines.BallControlHelper.CarryHeight;


import frc.robot.state_machines.BallControlHelper.PickUpHeight;
import frc.robot.state_machines.BallControlHelper.ShootHeight;
//import frc.robot.state_machines.ClimbingHelper.PreClimbHeight;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Contains the button mappings for the competition control board. Like the
 * drive code, one instance of the ControlBoard object is created upon startup,
 * then other methods request the singleton ControlBoard instance. Implements
 * the ControlBoardInterface.
 * 
 * @see ControlBoardInterface.java
 */
public class ControlBoard implements ControlBoardInterface {
    private static ControlBoardInterface mInstance = null;
    

    public static ControlBoardInterface getInstance() {
        if (mInstance == null) {
            //if (kUseGamepad) {
               // mInstance = new GamepadControlBoard();
            //} else {
                mInstance = new ControlBoard();
            //}
        }
        return mInstance;
    }
    private final XboxController mDriver;
    private final XboxController mOperator;
    

    protected ControlBoard() {
        mOperator = new XboxController(1);
        mDriver =new XboxController(0);
    }

    //DRIVING ---------------------------------------------------------------------------
        @Override
        public double getThrottle() {
            driverArcadeDrive();
            return throttle;
        }
        

        @Override
        public double getTurn() {
            driverArcadeDrive();
            return turn;
        }

        boolean driveReduction=false;
        double driveReductionAmount = .7; //remember a higher number means less reduction
        
        
        
        double throttle=0;
        double turn=0;
        
        public void driverArcadeDrive() {
            throttle=0;
            turn=mDriver.getX(Hand.kLeft)*Constants.regularTurnReduction;
            if(mDriver.getTriggerAxis(Hand.kRight)>.05) {
                throttle=mDriver.getTriggerAxis(Hand.kRight);			
            }else if(mDriver.getTriggerAxis(Hand.kLeft)>.05) {
                throttle=-mDriver.getTriggerAxis(Hand.kLeft);
                turn=-turn;
            }else {
                throttle=0;
                turn=turn*Constants.kDriveSwivelReduction;
            }
            if(getDriveInverted()&&throttle!=0){
                //turn=turn;
                throttle=-throttle;
            }
            

            //System.out.println("turn: "+turn+" throttle: "+throttle);
        }

        @Override
        public DriveSignal getDriveSignal() {
            boolean squareInputs=true;
            double xSpeed;
            double zRotation;
            
            driverArcadeDrive();
            

            xSpeed=throttle;
        
            zRotation = turn;
            
        
            // Square the inputs (while preserving the sign) to increase fine control
            // while permitting full power.
            if (squareInputs) {
                xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
                zRotation = Math.copySign(zRotation * zRotation, zRotation);
            }
            
            

            double leftMotorOutput;
            double rightMotorOutput;
        
            double maxInput = Math.copySign(Math.max(Math.abs(xSpeed), Math.abs(zRotation)), xSpeed);
        
            if (xSpeed >= 0.0) {
                // First quadrant, else second quadrant
                if (zRotation >= 0.0) {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
                } else {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
                }
            } else {
                // Third quadrant, else fourth quadrant
                if (zRotation >= 0.0) {
                leftMotorOutput = xSpeed + zRotation;
                rightMotorOutput = maxInput;
                } else {
                leftMotorOutput = maxInput;
                rightMotorOutput = xSpeed - zRotation;
                }
            }
            double m_rightSideInvertMultiplier = -1.0;

            leftMotorOutput=(limit(leftMotorOutput) * 1);
            rightMotorOutput=(limit(rightMotorOutput) * 1 * m_rightSideInvertMultiplier);

            
        // System.out.println("Rot:"+turn+" xSpeed: "+xSpeed+" Left: "+leftMotorOutput+ " right: "+rightMotorOutput);
            return new DriveSignal(leftMotorOutput,rightMotorOutput,false);
            
        }

        boolean driveInverted=true;
        @Override
        public boolean getDriveInverted() {
            if(mDriver.getStickButtonReleased(Hand.kLeft)){
                driveInverted=!driveInverted;
                if(driveInverted)CameraVision.setStreamMode(StreamMode.LimeMain);
                else CameraVision.setStreamMode(StreamMode.USBMain);
            }
        
        
            return driveInverted;


        }

        protected double limit(double value) {
            if (value > 1.0) {
            return 1.0;
            }
            if (value < -1.0) {
            return -1.0;
            }
            return value;
        }

        
        boolean gear=false;
            @Override
            public boolean getLowGear() {
                if(mDriver.getBumper(Hand.kRight)){
                    gear=false;
                }else if(mDriver.getBumper(Hand.kLeft))gear=true;

                return gear;
            }

    //HATCH ----------------------------------------------------------------------
    
        @Override
        public boolean getHatchPanelCentering() {
            return mOperator.getXButton();
        }

        @Override
        public boolean getHatchPanelAlignment() {
            return mOperator.getYButton();
        }

        @Override
        public double getHatchPanelJog() {
            double speed=0;
            double left = mOperator.getTriggerAxis(Hand.kLeft);
            double right = mOperator.getTriggerAxis(Hand.kRight);
            if(right>.1) {
                speed=right;			
            }else if(left>.1) {
                speed=-left;
            }else {
                speed=0;
            }
            speed*=.1;
            return -speed;
        }

        @Override
        public boolean getHatchPanelDeploy() {
            return mDriver.getXButton();
        }

        @Override
        public boolean getPlateHome() {
            return mOperator.getRawButton(8);
        }

        boolean hardStops=false;
        @Override
        public boolean getHatchHardStops() {
            if(mDriver.getYButtonReleased())hardStops=!hardStops;
            return hardStops;
        }

        @Override
        public boolean getHatchReset() {
            return false;
        }


    //BALLS -------------------------------------------------------------------------

        @Override
        public PickUpHeight getBallPickUp() {
            if(mOperator.getAButton())return PickUpHeight.FLOOR;
            else if (mOperator.getBButton()) return PickUpHeight.LOADING_STATION;
            else return null;
        }

        @Override
        public ShootHeight getBallShootPosition() {
            int pos = mOperator.getPOV();
            if(pos==270)return ShootHeight.CARGO_SHIP;
            else if(pos==180) return ShootHeight.ROCKET_ONE;
            else if (pos==90) return ShootHeight.ROCKET_TWO;
            else return null;
        }

        @Override
        public boolean getBallShoot() {
            return mDriver.getAButton();
        }

        @Override
        public CarryHeight getCarryBall() {
            if(mOperator.getPOV()==0) return CarryHeight.LOW;
            else return null;
        }

        @Override
        public boolean getBallHome() {
            return mOperator.getRawButtonReleased(7);
        }

        @Override
        public double getLiftJog() {
            double speed=mOperator.getY(Hand.kLeft);
            if(Math.abs(speed)<=.1){
                speed=0;
            }

            speed*=.1;
            return -speed;
        }

        @Override
        public double getWristJog() {
            double speed=mOperator.getY(Hand.kRight);
            if(Math.abs(speed)<=.1){
                speed=0;
            }

            speed*=.75;
            return speed;
        }


   

    //RUMBLE ------------------------------------------------------------------------------

        @Override
        public void setRumble(Controller c, RumbleSide type, double amount) {
            XboxController controller;
            if(c==Controller.Driver) controller=mDriver;
            else controller = mOperator;

            switch(type){
                case left:
                    controller.setRumble(RumbleType.kLeftRumble,amount);              
                break;
                case right:
                   controller.setRumble(RumbleType.kRightRumble,amount);
                break;
                case both: 
                   controller.setRumble(RumbleType.kLeftRumble, amount);
                   controller.setRumble(RumbleType.kRightRumble,amount);
                break;
            }
            
        }

        @Override
        public void rumbleOff() {
            setRumble(Controller.Driver, RumbleSide.both, 0);
            setRumble(Controller.Operator, RumbleSide.both, 0);
        }

        @Override
        public void setRumble(double amount) {
            setRumble(Controller.Driver, RumbleSide.both, amount);
            setRumble(Controller.Operator, RumbleSide.both, amount);
        }

   //Climbing ---------------------------------------------------------------------------------

        @Override
        public boolean getSuspensionHome() {
            return false;//mDriver.getRawButtonReleased(7);
        }

      //  @Override
       // public PreClimbHeight getPreClimbHeight() {
      //      if(mDriver.getPOV()==270) return ClimbingHelper.PreClimbHeight.MIDDLE;
      //      else if(mDriver.getPOV()==0) return ClimbingHelper.PreClimbHeight.HIGH;
      //      else return null;
     //   }

        @Override
        public boolean getClimbLift() {
            return mDriver.getPOV()==90;
        }

        @Override
        public boolean getClimbStow() {
            return mDriver.getPOV()==180;
        }

        @Override
        public boolean getClimbFullBlast() {
            return mDriver.getStickButtonReleased(Hand.kRight);
        }

        @Override
        public double getClimbVerticleJog() {
            double speed=mDriver.getY(Hand.kRight);
                if(Math.abs(speed)<=.1){
                    speed=0;
                }

                speed*=.03;
                return -speed;
        }

        @Override
        public double getClimbHorizontalJog() {
            return mDriver.getX(Hand.kRight)*.5;
        }

    //MOTION PROFILE -----------------------------------------------------------------------------
        @Override
        public boolean enableMotionProfile() {
            return mDriver.getRawButton(7); //left one;
        }

        
        
        @Override
        public boolean holdMotionProfile() {
            return mDriver.getRawButton(8); //right one;
        }

   

   

  

    
}
