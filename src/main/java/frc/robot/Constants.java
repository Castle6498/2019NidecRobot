package frc.robot;

import frc.lib.util.ConstantsBase;
import frc.lib.util.InterpolatingDouble;
import frc.lib.util.InterpolatingTreeMap;
import frc.lib.util.math.PolynomialRegression;

/**
 * A list of constants used by the rest of the robot code. This include physics
 * constants as well as constants determined through calibrations.
 */
public class Constants extends ConstantsBase {
    public static double kLooperDt = 0.005;
    
       
    // Drive
        
        //Sparks
            public static final int kDriveLeftMasterSparkPort=5; //TODO: Set drive ports
            public static final int kDriveRightMasterSparkPort=2;

            public static final int kDriveLeftSlaveSparkPort=4; //TODO: Set drive ports
            public static final int kDriveRightSlaveSparkPort=3;

            public static final double regularTurnReduction=.85;

            public static final double kDriveSwivelReduction=1;
      

    
    //Flywheel

        public static double kShooterMinVelocity = 670;

         // Shooter tuning parameters
            public static boolean kIsShooterTuning = false; //SET TRUE if you want to tune shooter basically starts at tuning floor and steps by set increment
            public static double kShooterTuningRpmFloor = 2900;   //min tuning rpm
            public static double kShooterTuningRpmCeiling = 3500;  //max tuning rpm
            public static double kShooterTuningRpmStep = 50; //tuning increment
            public static double kShooterTuningRpm = 3500.0; //this doesn't appear to be used ever, probably an old setting
        // Shooter constants
            public static final int kShooterId = 1;//TODO: Change this
        // Shooter gains
            public static double kShooterTalonKP = 0.16;
            public static double kShooterTalonKI = 0.00008;
            public static double kShooterTalonKD = 0.0;
            public static double kShooterTalonKF = 0.035;
            public static double kShooterRampRate = 60.0/1000; //TODO: Think it's in milli

            public static double kShooterTalonHoldKP = 0.0;
            public static double kShooterTalonHoldKI = 0.0;
            public static double kShooterTalonHoldKD = 0.0;

            public static double kShooterHoldRampRate = 720.0/1000; //TODO: Think it's in milli

            public static int kShooterTalonIZone = 1000;// 73 rpm
            public static int kShooterOpenLoopCurrentLimit = 35;

            public static double kShooterSetpointDeadbandRpm = 1.0;

        // Used to determine when to switch to hold profile.
            public static double kShooterMinTrackStability = 0.25; //minimum stability required to shoot the  percentage of the goal tracking time during which this goal has
                                                                        // been observed (0 to 1)
            public static double kShooterStartOnTargetRpm = 50.0;
            public static double kShooterStopOnTargetRpm = 150.0;
            public static int kShooterKfBufferSize = 20;
            public static int kShooterMinOnTargetSamples = 20; // Should be <= kShooterKvBufferSize because it will never get more than the buffer can hold
                                                                //The minimum kf samples required before it can go to hold
            public static int kShooterJamBufferSize = 30;
            public static double kShooterDisturbanceThreshold = 25;
            public static double kShooterJamTimeout = 1.5; // In secs
            public static double kShooterUnjamDuration = 0.5; // In secs
            public static double kShooterMinShootingTime = 1.0; // In secs

            public static double kShooterSpinDownTime = 0.25; //how long we have to wait for shooter to be slow 
                                                            //enough/stop to do normal tasks INCLUDING DRIVING


         /* AIM PARAMETERS */

            public static double kDefaultShootingDistanceInches = 95.8;
            public static double kDefaultShootingRPM = 2950.0;
            public static boolean kUseFlywheelAutoAimPolynomial = true; // Change to 'true' to use the best-fit polynomial
                                                                        // instead. if false, uses linear interpolation based on calculated points
            public static InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> kFlywheelAutoAimMap = new InterpolatingTreeMap<>();
            public static PolynomialRegression kFlywheelAutoAimPolynomial;

            public static double kShooterOptimalRange = 100.0; //if the target isn't stable, defaulted to this distance
            public static double kShooterOptimalRangeFloor = 95.0;   //if the drive base is outside of absolut range, how far forward does it need to go
            public static double kShooterOptimalRangeCeiling = 105.0; 

            public static double kShooterAbsoluteRangeFloor = 90.0; //minimum shooting distance
            public static double kShooterAbsoluteRangeCeiling = 130.0; //maximum shooting distance

            public static double[][] kFlywheelDistanceRpmValues = {
                    // At champs 4/27
                    { 90.0, 2890.0 },
                    { 95.0, 2940.0 },
                    { 100.0, 2990.0 }, //these were calculated with the tuning constants, setting min and max and step is 50
                    { 105.0, 3025.0 }, //then going through a sequence, setting the robot at a distance that works based on rpm
                    { 110.0, 3075.0 },
                    { 115.0, 3125.0 },
                    { 120.0, 3175.0 },
                    { 125.0, 3225.0 },
                    { 130.0, 3275.0 },
            };

            static {
                for (double[] pair : kFlywheelDistanceRpmValues) {
                    kFlywheelAutoAimMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
                }
                kDefaultShootingRPM = kFlywheelAutoAimMap
                        .getInterpolated(new InterpolatingDouble(Constants.kDefaultShootingDistanceInches)).value;

                kFlywheelAutoAimPolynomial = new PolynomialRegression(kFlywheelDistanceRpmValues, 2); //values, degree of polynomial
            }

    //Intake -----------------------------------------------------------
    
        //Sparks
        public static final int kIntakeSparkPort=1;
        public static final int kLinearActuatorSparkPort=0;

        public static final double kIntakePickUpSpeed=.45;

        //Linear Actuator Settings
        public static final int kIntakeMinLimitPort = 2;
        public static final int kIntakeMaxLimitPort = 3;

        public static final double kIntakeActuationSpeed = .1;
    
    //Hood
        
        public static final int kHoodLinearActuatorPort=6;

        public static final double kHoodStroke = 140; //mm
        public static final double kHoodSpeed = 7; //mm/s
        public static final double kHoodRadius = 120.65; //mm
        public static final double kHoodMaxElevation=(kHoodStroke/kHoodRadius)*(180/Math.PI);
    
    //Feeder

        //Ports
        public static final int kFeederLeftPWM = 7;
        public static final int kFeederLeftDIO = 0;
        
        public static final int kFeederRightPWM = 8;
        public static final int kFeederRightDIO = 1;

        public static final double kFeederSpeed = 1;
    

        //Photoeye
        //public static final int kFeederSensorPort = 0; //TODO: get this mounted

        //public static final double kFeederBallRequiredTime = .2;
    



    


   
        
   


            
 
    //LED -------------------------------------------------------------------------------

        
        // Digital Outputs
            public static int kGreenLEDId = 9;//TODO: change led ports
            public static int kRangeLEDId = 8;
    

    

    @Override
    public String getFileLocation() {
        return "~/constants.txt";
    }

   
}
