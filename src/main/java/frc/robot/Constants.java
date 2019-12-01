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
            public static final int kDriveLeftSparkPort=0; //TODO: Set drive ports
            public static final int kDriveRightSparkPort=1;

            public static final double regularTurnReduction=.85;

            public static final double kDriveSwivelReduction=1;
      

    
    //Flywheel
        // Shooter constants
            public static final int kShooterMasterId = 2;//TODO: Change this
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
            public static int kShooterMinOnTargetSamples = 20; // Should be <= kShooterKvBufferSize

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
    
        //Talon
        public static final int kIntakeTalonID=5;

        public static final double kIntakePickUpSpeed=.8;
        public static final double kIntakeShootSpeed=-1;

        public static final double kIntakeBallRequiredTime=.55;
        public static final double kIntakeShootPause = 1; 

        public static final double kIntakeCurrentThreshold=1.5;

        //Photoeye
        public static final int kIntakeSensorPort = 0;
         
    



    


   
        
   


            
 
    //LED -------------------------------------------------------------------------------

        public static final int[] kPlateLEDPorts = {4,5,6};
    

    

    @Override
    public String getFileLocation() {
        return "~/constants.txt";
    }

   
}
