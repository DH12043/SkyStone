package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

import static java.lang.Thread.sleep;

@Autonomous (name= "AutoREDPrimaryRelativeCoordinates", group= "None")
public class AutoREDPrimaryRelativeCoordinates extends SkystoneVuforiaNew {

    DcMotor verticalRight, verticalLeft, horizontal;

    String verticalLeftEncoderName = "FrontLeft";
    String verticalRightEncoderName = "BackRight";
    String horizontalEncoderName = "BackLeft";

    private File startingXpositionFile = AppUtil.getInstance().getSettingsFile("startingXposition.txt");
    private File startingYpositionFile = AppUtil.getInstance().getSettingsFile("startingYposition.txt");
    private File startingθpositionFile = AppUtil.getInstance().getSettingsFile("startingθposition.txt");

    Thread positionThread;
    OdometryGlobalCoordinatePosition globalPositionUpdate;

    private ModernRoboticsI2cRangeSensor OrientationSensor;
    private ModernRoboticsI2cRangeSensor StonePresenceSensor;

    private int SkystoneXPosition;
    private int SkystoneYPosition;
    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor BackLeft;
    private DcMotor IntakeMotor;
    private DcMotor LiftMotor;
    private Servo lFoundationator;
    private Servo rFoundationator;
    private Servo OrientationServoLeft;
    private Servo OrientationServoRight;

    final double COUNTS_PER_INCH = 307.699557;
    private boolean goToNewPosition = true;
    private static int PreFoundationXPosition = 36;
    private static int PreFoundationYPosition = 95;
    private static int FoundationXPosition = 48;
    private static int FoundationYPosition = 107;
    private static int BuildSiteXPosition = 9;
    private static int BuildSiteYPosition = 111;
    private static int ParkLineXPosition = 9;
    private static int ParkLineYPosition = 72;

    private double RobotXPosition;
    private double RobotYPosition;
    private double RobotRotation;
    private double startTime;
    private double currentTime;

    private int autoState = 0;
    private int lastAutoState = -1;

    // ORIENTER STUFF

    private String stoneOrientation = "empty";
    private String lCurrentPosition = "lDisengage";
    private String rCurrentPosition = "rDisengage";
    private double foundationatorPosition = .335;
    private double orientDistance = 0;
    private double lDisengage = .2;
    private double rDisengage = .9;
    private double lEngage = .85;
    private double rEngage = .25;
    private double startRightTime = 0;
    private double currentRightTime = 0;
    private double actualRightTime = 1.5;
    private double lastActualRightTime = 0;
    private double rightOrientCheck = 1;
    private double startLeftTime = 0;
    private double currentLeftTime = 0;
    private double actualLeftTime = 1.5;
    private double waitTime = 1;
    private double lastActualLeftTime = 0;
    private double targetTime = .5;
    private double stoneDistance = 0;
    private boolean readyToGrab = false;
    private boolean stoneFullyInStraightener = false;
    private boolean straightenerBusy = false;
    private boolean firstRightRun = true;
    private boolean firstLeftRun = true;

    @Override
    public void init() {
        super.init();
        msStuckDetectStart = 300000;
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
//        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
//        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
        LiftMotor = hardwareMap.dcMotor.get("LiftMotor");
        lFoundationator = hardwareMap.servo.get("lFoundationator");
        rFoundationator = hardwareMap.servo.get("rFoundationator");
        verticalLeft = hardwareMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = hardwareMap.dcMotor.get(verticalRightEncoderName);
        horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);
        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);
        StartingXPosition = (0);
        StartingYPosition = (0);
        StartingRotation = (0);
    }

    @Override
    public void start() {
        //Create and start GlobalCoordinatePosition thread to constantly update the global coordinate positions\
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();




    }

    @Override
    public void loop() {
        currentTime = getRuntime();

        RobotXPosition = (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH) + StartingXPosition;
        RobotYPosition = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH) + StartingYPosition;
        RobotRotation = (globalPositionUpdate.returnOrientation()) + StartingRotation;

        telemetry.addData ("RobotX", RobotXPosition);
        telemetry.addData ("RobotY", RobotYPosition);
        telemetry.addData ("RobotTheta", RobotRotation);

        // Drop intake
        // Turn on intake
        if (autoState == 0) {
            // TODO setPosition of the servo
            IntakeMotor.setPower(1);

            autoState++;
            lastAutoState = 0;
        }

        // Drive to stone / pick it up
        else if (autoState == 1) {
            // Will only happen upon entering state 1
            if (lastAutoState == 0) {
                startTime = getRuntime();
                if (positionSkystone.equals("Left")) {
                    telemetry.addData("Skystone Location Left", "");
                    SkystoneXPosition = (600);
                    SkystoneYPosition = (47);
                } else if (positionSkystone.equals("Center")) {
                    telemetry.addData("Skystone Location Center", "");
                    SkystoneXPosition = (36);
                    SkystoneYPosition = (47);
                } else if (positionSkystone.equals("Right")) {
                    telemetry.addData("Skystone Location Right", "");
                    SkystoneXPosition = (28);
                    SkystoneYPosition = (47);
                } else {
                    telemetry.addData("Skystone Location Error", "");
                    SkystoneXPosition = (8);
                    SkystoneYPosition = (47);
                }
            }
            if (startTime > currentTime - 5) {
                telemetry.addData("startTime", startTime);
                telemetry.addData("currentTime", currentTime);
                telemetry.addData("Time Left", currentTime - (startTime + 5));

                goToTarget(SkystoneXPosition, SkystoneYPosition, 45);
            }
            else {
                autoState++;
                stopMotors();
            }
            lastAutoState = 1;
        }
        else if(autoState == 2) {
            if(lastAutoState == 1) {
                IntakeMotor.setPower(0);
            }
            lastAutoState = 2;
        }
//        DriveToFoundation(globalPositionUpdate);
//
//        HookOntoFoundation(globalPositionUpdate);
//
//        MoveFoundation(globalPositionUpdate);
//
//        Park(globalPositionUpdate);

        telemetry.update();

    }
    private void goToTarget (double x, double y, double Θ) {
       if (goToNewPosition) {
           StartingXPosition = (-x);
           StartingYPosition = (-y);
           StartingRotation = (-Θ);

           goToNewPosition = false;
       }
       else {
           driveToScoringPosition();
       }
    }

    private void driveToScoringPosition() {
        java.lang.String turningDirection;
        double distanceFromOrientation;

        if (RobotRotation > 2 && RobotRotation < 180) {
            turningDirection = "left";
        }
        else {
            turningDirection = "right";
        }
        telemetry.addData("TurningDirection", turningDirection);
        if (RobotRotation > 2 && RobotRotation < 358) {
            telemetry.addData("RobotCurrentOrientation", RobotRotation);
            telemetry.addData("RobotTargetOrientation", "0");
            if (turningDirection.equals("right")) {
                distanceFromOrientation = 360 - RobotRotation;
                if (distanceFromOrientation > 120) {
                    Drive(0, 0, 1);
                }
                else if (distanceFromOrientation > 45) {
                    Drive(0,0,.35);
                }
                else {
                    Drive(0,0,.15);
                }
            }
            else {
                distanceFromOrientation = RobotRotation;
                if (distanceFromOrientation > 120) {
                    Drive(0, 0, -1);
                }
                else if (distanceFromOrientation > 45) {
                    Drive(0,0,-.35);
                }
                else {
                    Drive(0,0,-.15);
                }
            }
            telemetry.addData("DistanceFromOrientation", distanceFromOrientation);
        }
        else if (RobotYPosition < -.5 || RobotYPosition > .5) {
            if (RobotYPosition < 0) {
                if (RobotYPosition < -6) {
                    Drive(-.3,0,0);
                }
                else {
                    Drive(-.1, 0, 0);
                }
            }
            else {
                if (RobotYPosition > 6) {
                    Drive(.3,0,0);
                }
                else {
                    Drive(.1,0,0);
                }
            }
        }
        else if (RobotXPosition < -.5 || RobotXPosition > .5) {
            if (RobotXPosition < 0) {
                if (RobotXPosition < -6) {
                    Drive(0,.5,0);
                }
                else {
                    Drive(0, .2, 0);
                }
            }
            else {
                if (RobotXPosition > 6) {
                    Drive(0,-.5,0);
                }
                else {
                    Drive(0,-.2,0);
                }
            }
        }
        else {
            Drive(0,0,0);
        }
    }

    private void stopMotors() { Drive (0,0,0); }

    public void Drive(double DZForwardButton, double DZSidewaysButton, double DZSpinningButton) {
        DZSpinningButton = -DZSpinningButton;
        FrontRight.setPower(-DZSidewaysButton - DZForwardButton + DZSpinningButton);
        FrontLeft.setPower(-DZSidewaysButton + DZForwardButton + DZSpinningButton);
        BackRight.setPower(DZSidewaysButton - DZForwardButton + DZSpinningButton);
        BackLeft.setPower(DZSidewaysButton + DZForwardButton + DZSpinningButton);
    }

    @Override
    public void stop() {
        globalPositionUpdate.stop(); //Maybe comment this out?
//        OdometryGlobalCoordinatePosition position = null;
//        ReadWriteFile.writeFile(startingXpositionFile, String.valueOf(position.returnXCoordinate()));
//        ReadWriteFile.writeFile(startingYpositionFile, String.valueOf(position.returnYCoordinate()));
//        ReadWriteFile.writeFile(startingθpositionFile, String.valueOf(position.returnOrientation()));
    }

    double getDistanceFromCoordinates(double x, double y, OdometryGlobalCoordinatePosition position) {
        double deltaX = x - position.returnXCoordinate();
        double deltaY = y - position.returnYCoordinate();

//        telemetry.addData("deltaX",deltaX);
//        telemetry.addData("deltaY",deltaY);
//        telemetry.update();

        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    double[] calculateMotorSpeeds(double deltaX, double deltaY) {
        double[] speeds = new double[4];
        speeds[0] = -deltaY + deltaX;
        speeds[1] = deltaY - deltaX;
        speeds[2] = -deltaY - deltaX;
        speeds[3] = deltaY + deltaX;


        if (speeds[0] > 1) {
            speeds[0] = 1;
        }
        else if (speeds[0] < -1) {
            speeds[0] = -1;
        }
        if (speeds[1] > 1) {
            speeds[1] = 1;
        }
        else if (speeds[1] < -1) {
            speeds[1] = -1;
        }
        if (speeds[2] > 1) {
            speeds[2] = 1;
        }
        else if (speeds[2] < -1) {
            speeds[2] = -1;
        }
        if (speeds[3] > 1) {
            speeds[3] = 1;
        }
        else if (speeds[3] < -1) {
            speeds[3] = -1;
        }
//        double maxSpeed = 0;
//        for(double speed : speeds) {
//            maxSpeed = Math.max(maxSpeed, Math.abs(speed));
//        }
//
//        if(maxSpeed > 1.0) {
//            for(int i = 0; i < 4; ++i) {
//                speeds[i] = maxSpeed;
//            }
//        }
//
        return speeds;
    }

    private void driveToSkystonePosition(OdometryGlobalCoordinatePosition position) {

        if(RobotXPosition < SkystoneXPosition) {
            if(RobotYPosition < SkystoneYPosition) {
                FrontRight.setPower(.3);
                FrontLeft.setPower(.3);
                BackRight.setPower(-.3);
                BackLeft.setPower(-.3);
                telemetry.addData("Strafing","Forward Right");
                telemetry.update();
            }
            else if(RobotYPosition > SkystoneYPosition) {
                FrontRight.setPower(-.3);
                FrontLeft.setPower(-.3);
                BackRight.setPower(.3);
                BackLeft.setPower(.3);
                telemetry.addData("Strafing", "Forward Left");
            }
            else {
                FrontRight.setPower(.15);
                FrontLeft.setPower(-.15);
                BackRight.setPower(.15);
                BackLeft.setPower(-.15);
                telemetry.addData("Straight", "Forward");
                telemetry.addData("RobotX", RobotXPosition);
                telemetry.addData("RobotY", RobotYPosition);
                telemetry.addData("StartingRotation", RobotRotation);
            }
            RobotXPosition = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH) + StartingXPosition;
            RobotYPosition = -(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH) + StartingYPosition;
            RobotRotation = (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH) + StartingRotation;
            telemetry.addData("SkystoneXPosition", SkystoneXPosition / COUNTS_PER_INCH);
            telemetry.addData("SkystoneYPosition", SkystoneYPosition / COUNTS_PER_INCH);
            telemetry.update();
        }
        while (position.returnXCoordinate() > SkystoneXPosition - 20) {
            FrontRight.setPower(-15);
            FrontLeft.setPower(.15);
            BackRight.setPower(-.15);
            BackLeft.setPower(.15);
        }
        FrontRight.setPower(0);
        FrontLeft.setPower(0);
        BackRight.setPower(0);
        BackLeft.setPower(0);
//        while(distanceAway > 18.5) {
//            double deltaX = SkystoneXPosition - position.returnXCoordinate();
//            double deltaY = SkystoneYPosition - position.returnYCoordinate();
//
//            double[] motorSpeeds = calculateMotorSpeeds(deltaX, deltaY);
//
//            FrontRight.setPower(motorSpeeds[2]);
//            FrontLeft.setPower(motorSpeeds[1]);
//            BackRight.setPower(motorSpeeds[1]);
//            BackLeft.setPower(motorSpeeds[2]);
//
//            distanceAway = getDistanceFromCoordinates(SkystoneXPosition, SkystoneYPosition, position);
//        }
//        while(distanceAway <= 11) {
//            double deltaX = SkystoneXPosition - position.returnXCoordinate();
//            double deltaY = SkystoneYPosition - position.returnYCoordinate();
//
//            double[] motorSpeeds = calculateMotorSpeeds(deltaX, deltaY);
//
//            FrontRight.setPower(motorSpeeds[2]);
//            FrontLeft.setPower(motorSpeeds[1]);
//            BackRight.setPower(motorSpeeds[1]);
//            BackLeft.setPower(motorSpeeds[2]);
//
//            distanceAway = getDistanceFromCoordinates(SkystoneXPosition, SkystoneYPosition, position);
//        }
    }

    private void DriveToFoundation(OdometryGlobalCoordinatePosition position) {
        double distanceAway = getDistanceFromCoordinates(PreFoundationXPosition, PreFoundationYPosition, position);

        while (distanceAway > 9.5) {
            double deltaX = PreFoundationXPosition - position.returnXCoordinate();
            double deltaY = PreFoundationYPosition - position.returnYCoordinate();

            double[] motorSpeeds = calculateMotorSpeeds(deltaX, deltaY);

            FrontRight.setPower(motorSpeeds[2]);
            FrontLeft.setPower(motorSpeeds[1]);
            BackRight.setPower(motorSpeeds[1]);
            BackLeft.setPower(motorSpeeds[2]);

            distanceAway = getDistanceFromCoordinates(PreFoundationXPosition, PreFoundationYPosition, position);
        }
        while (distanceAway != 0) {
            double deltaX = PreFoundationXPosition - position.returnXCoordinate();
            double deltaY = PreFoundationYPosition - position.returnYCoordinate();

            double[] motorSpeeds = calculateMotorSpeeds(deltaX, deltaY);

            FrontRight.setPower(motorSpeeds[2]);
            FrontLeft.setPower(motorSpeeds[1]);
            BackRight.setPower(motorSpeeds[1]);
            BackLeft.setPower(motorSpeeds[2]);

            distanceAway = getDistanceFromCoordinates(PreFoundationXPosition, PreFoundationYPosition, position);
        }
        while (StartingRotation < 90) {
            FrontRight.setPower(.5);
            FrontLeft.setPower(-.5);
            BackRight.setPower(.5);
            BackLeft.setPower(-.5);
        }
    }
    private void HookOntoFoundation(OdometryGlobalCoordinatePosition position) {
        double distanceAway = getDistanceFromCoordinates(FoundationXPosition, FoundationYPosition, position);

        while(distanceAway > 9.5) {
            double deltaX = FoundationXPosition - position.returnXCoordinate();
            double deltaY = FoundationYPosition - position.returnYCoordinate();

            double[] motorSpeeds = calculateMotorSpeeds(deltaX, deltaY);

            FrontRight.setPower(motorSpeeds[0]);
            FrontLeft.setPower(motorSpeeds[2]);
            BackRight.setPower(motorSpeeds[2]);
            BackLeft.setPower(motorSpeeds[0]);

            distanceAway = getDistanceFromCoordinates(FoundationXPosition, FoundationYPosition, position);
        }
        while(distanceAway != 0) {
            double deltaX = FoundationXPosition - position.returnXCoordinate();
            double deltaY = FoundationYPosition - position.returnYCoordinate();

            double[] motorSpeeds = calculateMotorSpeeds(deltaX, deltaY);

            FrontRight.setPower(motorSpeeds[0]);
            FrontLeft.setPower(motorSpeeds[2]);
            BackRight.setPower(motorSpeeds[2]);
            BackLeft.setPower(motorSpeeds[0]);

            distanceAway = getDistanceFromCoordinates(FoundationXPosition, FoundationYPosition, position);
        }

        lFoundationator.setPosition(.27);
        rFoundationator.setPosition(0);
    }

    private void MoveFoundation(OdometryGlobalCoordinatePosition position) {
        double distanceAway = getDistanceFromCoordinates(BuildSiteXPosition, BuildSiteYPosition, position);

        while(distanceAway > 9.5) {
            double deltaX = BuildSiteXPosition - position.returnXCoordinate();
            double deltaY = BuildSiteYPosition - position.returnYCoordinate();

            double[] motorSpeeds = calculateMotorSpeeds(deltaX, deltaY);

            FrontRight.setPower(motorSpeeds[0]);
            FrontLeft.setPower(motorSpeeds[2]);
            BackRight.setPower(motorSpeeds[2]);
            BackLeft.setPower(motorSpeeds[0]);

            distanceAway = getDistanceFromCoordinates(BuildSiteXPosition, BuildSiteYPosition, position);
        }
        while(distanceAway != 0) {
            double deltaX = BuildSiteXPosition - position.returnXCoordinate();
            double deltaY = BuildSiteYPosition - position.returnYCoordinate();

            double[] motorSpeeds = calculateMotorSpeeds(deltaX, deltaY);

            FrontRight.setPower(motorSpeeds[0]);
            FrontLeft.setPower(motorSpeeds[2]);
            BackRight.setPower(motorSpeeds[2]);
            BackLeft.setPower(motorSpeeds[0]);

            distanceAway = getDistanceFromCoordinates(BuildSiteXPosition, BuildSiteYPosition, position);
        }

        lFoundationator.setPosition(0);
        rFoundationator.setPosition(.27);

        while (StartingRotation < 90) {
            FrontRight.setPower(.5);
            FrontLeft.setPower(-.5);
            BackRight.setPower(.5);
            BackLeft.setPower(-.5);
        }
    }

    private void Park(OdometryGlobalCoordinatePosition position) {
        double distanceAway = getDistanceFromCoordinates(ParkLineXPosition, ParkLineYPosition, position);

        while(distanceAway > 9.5) {
            double deltaX = ParkLineXPosition - position.returnXCoordinate();
            double deltaY = ParkLineYPosition - position.returnYCoordinate();

            double[] motorSpeeds = calculateMotorSpeeds(deltaX, deltaY);

            FrontRight.setPower(motorSpeeds[2]);
            FrontLeft.setPower(motorSpeeds[1]);
            BackRight.setPower(motorSpeeds[1]);
            BackLeft.setPower(motorSpeeds[2]);

            distanceAway = getDistanceFromCoordinates(ParkLineXPosition, ParkLineYPosition, position);
        }
        while(distanceAway != 0) {
            double deltaX = ParkLineXPosition - position.returnXCoordinate();
            double deltaY = ParkLineYPosition - position.returnYCoordinate();

            double[] motorSpeeds = calculateMotorSpeeds(deltaX, deltaY);

            FrontRight.setPower(motorSpeeds[2]);
            FrontLeft.setPower(motorSpeeds[1]);
            BackRight.setPower(motorSpeeds[1]);
            BackLeft.setPower(motorSpeeds[2]);

            distanceAway = getDistanceFromCoordinates(ParkLineXPosition, ParkLineYPosition, position);
        }
    }


    //
    // Straightener
    //

    private void initializeStraightener() {
        OrientationSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "OrientationSensor");
        StonePresenceSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor .class, "StonePresenceSensor");
        OrientationServoLeft = hardwareMap.get(Servo.class, "OrientationServoLeft");
        OrientationServoRight = hardwareMap.get(Servo.class, "OrientationServoRight");
        OrientationServoLeft.setPosition(lDisengage);
        OrientationServoRight.setPosition(rDisengage);
    }

    private void checkStraightener() {
        stoneDistance = StonePresenceSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("Stone Distance", stoneDistance);
        stoneFullyInStraightener = stoneDistance < 1.5;

        orientStone();
        //manualOverride();
    }


    private void senseOrientation() {
        //orientDistance = OrientationSensor.getDistance(DistanceUnit.INCH);
        orientDistance = OrientationSensor.cmOptical()/2.54;
        telemetry.addData("Orient Distance", orientDistance);

        // StoneOrientation is assigned relative to the side of the robot that the studs of the stone
        // are on when looking at the robot from the back.
        // this is true for the servos as well; left and right are assigned relative to the back

        if(!straightenerBusy) {
            /*if (orientDistance > .25 && orientDistance <= .55) {
                stoneOrientation = "left";
            }
            else if (orientDistance > .75 && orientDistance <= 2.3) {
                stoneOrientation = "center";
            }
            else if (orientDistance > 4) {
                stoneOrientation = "empty";
            }
            else if (orientDistance > 2.5 && orientDistance < 3){
                stoneOrientation = "right";
            }*/

            if (orientDistance > .35 && orientDistance <= .6) {
                stoneOrientation = "left";
            }
            else if (orientDistance > 1.2 && orientDistance <= 1.8) {
                stoneOrientation = "center";
            }
            else if (orientDistance > 3) {
                stoneOrientation = "empty";
            }
            else if (orientDistance > 2 && orientDistance < 2.4){
                stoneOrientation = "right";
            }
            else {
                stoneOrientation = "notInThreshold";
            }
        }
        telemetry.addData("orientPosition", stoneOrientation);
        telemetry.addData("StraightenerBusy", straightenerBusy);
    }

    private void orientStone() {
        senseOrientation();

        if (!straightenerBusy) {
            OrientationServoLeft.setPosition(lDisengage);
            OrientationServoRight.setPosition(rDisengage);
        }

        if (stoneFullyInStraightener) {
            if (stoneOrientation.equals("right")) {
                runLeftServo();
                readyToGrab = false;
            }
            else if (stoneOrientation.equals("left")) {
                runRightServo();
                readyToGrab = false;
            }
            else if (stoneOrientation.equals("empty")) {
                readyToGrab = false;
                OrientationServoLeft.setPosition(lDisengage);
                OrientationServoRight.setPosition(rDisengage);
            }
            else if(stoneOrientation.equals("center")) {
                readyToGrab = true;
                OrientationServoLeft.setPosition(lDisengage);
                OrientationServoRight.setPosition(rDisengage);
            }
            else if (stoneOrientation.equals("notInThreshold")) {
                readyToGrab = false;
                OrientationServoLeft.setPosition(lDisengage);
                OrientationServoRight.setPosition(rDisengage);
            }
            telemetry.addData("actualLeftTime", actualLeftTime);
            telemetry.addData("actualRightTime", actualRightTime);
            telemetry.addData("target time", targetTime);
            telemetry.addData("orientPosition", stoneOrientation);
            telemetry.addData("StraightenerBusy", straightenerBusy);
        }
        else {
            readyToGrab = false;
            if (!straightenerBusy) {
                stoneOrientation = "empty";
            }
        }
    }

    private void runRightServo() {
        if(firstRightRun && !straightenerBusy) {
            OrientationServoRight.setPosition(rEngage);
            startRightTime = getRuntime();
            firstRightRun = false;
            straightenerBusy = true;
        }
        currentRightTime = getRuntime();
        actualRightTime = (currentRightTime-startRightTime);

        if(actualRightTime > targetTime) {
            OrientationServoRight.setPosition(rDisengage);
            firstRightRun = true;
        }

        if (actualRightTime > targetTime*2) {
            OrientationServoRight.setPosition(rDisengage);
            firstRightRun = true;
            straightenerBusy = false;
        }
        else {
            straightenerBusy = true;
        }
    }

    private void runLeftServo() {
        if(firstLeftRun && !straightenerBusy) {
            OrientationServoLeft.setPosition(lEngage);
            startLeftTime = getRuntime();
            firstLeftRun = false;
            straightenerBusy = true;
        }
        currentLeftTime = getRuntime();
        actualLeftTime = (currentLeftTime-startLeftTime);

        if(actualLeftTime > targetTime) {
            OrientationServoLeft.setPosition(lDisengage);
            firstLeftRun = true;
        }

        if (actualLeftTime > targetTime*2) {
            OrientationServoLeft.setPosition(lDisengage);
            firstLeftRun = true;
            straightenerBusy = false;
        }
        else {
            straightenerBusy = true;
        }
    }
}