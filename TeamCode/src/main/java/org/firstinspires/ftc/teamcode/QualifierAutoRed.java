package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import android.view.View;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.sql.Date;
import java.sql.Time;
import java.util.TimeZone;

@Autonomous(name= "QualifierAutoRed", group= "None")
public class QualifierAutoRed extends OpMode {

    DcMotor verticalRight, verticalLeft, horizontal;

    String verticalLeftEncoderName = "FrontLeft";
    String verticalRightEncoderName = "BackRight";
    String horizontalEncoderName = "BackLeft";

    Thread positionThread;
    OdometryGlobalCoordinatePosition globalPositionUpdate;

    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor BackLeft;
    private DcMotor IntakeMotor;
    private DcMotor LiftMotor;
    private DcMotor IntakeAssistMotor;
    private Servo IntakeReleaseServo;
    private Servo lFoundationator;
    private Servo rFoundationator;
    private Servo OrientationServoLeft;
    private Servo OrientationServoRight;
    private Servo GrabberServo;
    private Servo OrientStoneServo;
    private Servo MoveArmServo;

    private ModernRoboticsI2cRangeSensor OrientationSensor;
    private ModernRoboticsI2cRangeSensor StonePresenceSensor;

    private VoltageSensor batteryVoltageSensor;
    private double batteryVoltage;
    private double DEFAULT_MOVEMENT_SPEED = 1.0;
    private double DEFAULT_TURN_SPEED = 1.0;
    private static final double BATTERY_VOLTAGE_COMP_X1 = 13.0;
    private static final double BATTERY_VOLTAGE_COMP_X2 = 14.0;
    private static final double BATTERY_VOLTAGE_COMP_Y1 = 1.0;
    private static final double BATTERY_VOLTAGE_COMP_Y2 = 0.8;

    final double COUNTS_PER_INCH = 307.699557;
    private double lastDistanceToTarget = 0;
    private double foundationatorPosition = .335;
    private double StartingXPosition = 0;
    private double StartingYPosition = 0;
    private double StartingRotation = 0;
    private double RobotXPosition;
    private double RobotYPosition;
    private double RobotRotation;
    private double SkyStonePosition;
    private double GrabRotateTime;
    private double startTime;
    private double currentTime;
    private double movement_x;
    private double movement_y;
    private double movement_turn;
    private int autoState = INIT_STATE;
    private int lastAutoState = NO_STATE;
    private static final double DECELERATION_START_POINT = 48;
    private static final double DECELERATION_ZERO_POINT = -6;   // -6
    private static final double TURNING_DECELERATION_START_POINT = 180;
    private static final double TURNING_DECELERATION_ZERO_POINT = -5; // -5
    private static final double X_SPEED_MULTIPLIER = 1;
    private static final int NO_STATE = -2;
    private static final int INIT_STATE = -1;
    private static final int FIRST_MOVE_TO_SKYSTONE_STATE = 0;
    private static final int FIRST_SKYSTONE_PLACE = 1;
    private static final int ALIGN_FOUNDATION_STATE = 2;
    private static final int FOUNDATION_STATE = 3;
    private static final int SECOND_FOUNDATION_STATE = 4;
    private static final int BUILD_SITE_STATE = 5;
    private static final int BUILD_SITE_STATE2 = 6;
    private static final int SECOND_SKYSTONE_STATE = 7;
    private static final int SECOND_SKYSTONE_TAKE = 8;
    private static final int SECOND_SKYSTONE_TAKE2 = 9;
    private static final int SECOND_SKYSTONE_TAKE3 = 10;
    private static final int SECOND_SKYSTONE_PLACE = 11;
    private static final int SECOND_SKYSTONE_PLACE2 = 12;
    private static final int SECOND_SKYSTONE_PLACE3 = 13;
    private static final int PARK_STATE = 14;
    private long lastUpdateTime = 0;

    static final double countsPerMotor          = 383.6;
    static final double gearReduction           = 1.0 ;
    static final double wheelDiameter           = 1.7; //1.771653543307087
    static final double countsPerInch           = (countsPerMotor * gearReduction) / (wheelDiameter * Math.PI);
    static final double liftOffset = (2.5 * countsPerInch);

    private boolean liftUpCommand;
    private boolean liftDownCommand;
    private boolean GrabStart = false;
    private boolean readyToGrab = true;
    private int liftHeight = -1;

    private String stoneOrientation = "empty";
    private String lCurrentPosition = "lDisengage";
    private String rCurrentPosition = "rDisengage";
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
    private boolean stoneFullyInStraightener = false;
    private boolean straightenerBusy = false;
    private boolean isSkyStoneInView = false;
    private boolean LiftShouldBeUp = false;
    private boolean firstRightRun = true;
    private boolean firstLeftRun = true;
    private boolean stoneGrab = true;

    private double startGrabberTime;
    private double currentGrabberTime;
    private int liftGrabberState = 0;
    private int grabberReturnState = 0;
    private int emergencyStoneEjectState = 0;
    private int capstoneState = 0;
    private double grabberOpenPosition = .4;
    private double grabberClosedPosition = 0;
    private double armInsidePosition = 1;
    private double armOutsidePosition = 0;
    private double programStart;
    private boolean grabRotateStoneCommand = false;
    private boolean releaseStoneCommand;
    private NormalizedColorSensor SkyStoneSensor;

    View relativeLayout;

    private float redValues = 0;
    private float blueValues = 0;
    private float greenValues = 0;

    private boolean red = false;
    private boolean blue = false;
    private boolean green = false;

    @Override
    public void init() {
        //super.init();
        msStuckDetectStart = 300000;
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
        IntakeAssistMotor = hardwareMap.dcMotor.get("IntakeAssistMotor");
        IntakeReleaseServo = hardwareMap.get(Servo.class, "IntakeReleaseServo");
        lFoundationator = hardwareMap.servo.get("lFoundationator");
        rFoundationator = hardwareMap.servo.get("rFoundationator");
        FrontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        BackLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
        StartingXPosition = 49;
        StartingYPosition = 9;
        StartingRotation = 0;
        batteryVoltageSensor = hardwareMap.voltageSensor.get("Expansion Hub 2");
        initializeStraightener();
        initializeSkyStoneColorSensor();
        initializeVerticalLift();
        initializeGrabber();
    }

    @Override
    public void start() {
        startVerticalLift();
        startGrabber();
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();

        checkOdometry();
        programStart = getRuntime();

        lFoundationator.setPosition(0);
        rFoundationator.setPosition(foundationatorPosition);
        autoState = INIT_STATE;
        lastAutoState = NO_STATE;

        batteryVoltage = batteryVoltageSensor.getVoltage();

        // Max motor power is decreased if battery voltage is greater than a certain threshold,
        // and is linearly decreased at a larger amount as voltage increases.
        if(batteryVoltage > BATTERY_VOLTAGE_COMP_X1) {
            DEFAULT_MOVEMENT_SPEED = ((BATTERY_VOLTAGE_COMP_Y2 - BATTERY_VOLTAGE_COMP_Y1)
                    / (BATTERY_VOLTAGE_COMP_X2 - BATTERY_VOLTAGE_COMP_X1))
                    * (batteryVoltage - BATTERY_VOLTAGE_COMP_X1) + BATTERY_VOLTAGE_COMP_Y1;
            DEFAULT_TURN_SPEED = DEFAULT_MOVEMENT_SPEED;
        } else {
            DEFAULT_MOVEMENT_SPEED = 1.0;
            DEFAULT_TURN_SPEED = 1.0;
        }
    }

    private void checkOdometry() {
        RobotXPosition = (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH) + StartingXPosition;
        RobotYPosition = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH) + StartingYPosition;
        RobotRotation = (globalPositionUpdate.returnOrientation()) + StartingRotation;
    }

    @Override
    public void loop() {
        telemetry.addData("LiftShouldBeUp", LiftShouldBeUp);
        telemetry.addData("LastAutoState", lastAutoState);
        telemetry.addData("CurrentState?", autoState);
        checkStraightener();
        skyStoneCheck();
        if (LiftShouldBeUp) {
            checkVerticalLift();
            checkGrabber();
        }
        checkOdometry();
        currentTime = getRuntime();
        IntakeAssistMotor.setPower(-1);
        IntakeReleaseServo.setPosition(.6);
        LiftFinishDown();

        //skyStoneCheck();
        goToPositionByTime(StartingXPosition, StartingYPosition + 10, StartingRotation, .5, INIT_STATE, FIRST_MOVE_TO_SKYSTONE_STATE);
        lowerLiftDuringState(INIT_STATE);
        IntakeOn();
        goToPositionByTime(SkyStonePosition, 41, .2, .5, 0, 2.3, FIRST_MOVE_TO_SKYSTONE_STATE, FIRST_SKYSTONE_PLACE);   //Slowing Down to Grab Stone
        goToPositionByTime(SkyStonePosition, 28, DEFAULT_MOVEMENT_SPEED, DEFAULT_TURN_SPEED, 280, .7, FIRST_SKYSTONE_PLACE, ALIGN_FOUNDATION_STATE);
        goToPositionByTime(118, 33, DEFAULT_MOVEMENT_SPEED, DEFAULT_TURN_SPEED, 270, 1.3, ALIGN_FOUNDATION_STATE, FOUNDATION_STATE);
        lowerLiftDuringState(ALIGN_FOUNDATION_STATE);
        goToPositionByTime(128, 20, DEFAULT_MOVEMENT_SPEED, DEFAULT_TURN_SPEED, 180, .9, FOUNDATION_STATE, SECOND_FOUNDATION_STATE);
        FoundationDown();
        goToPositionByTime(128, 50, DEFAULT_MOVEMENT_SPEED, DEFAULT_TURN_SPEED, 180, 1, SECOND_FOUNDATION_STATE, BUILD_SITE_STATE);
        grabRotateStoneAtBeginningOfState(BUILD_SITE_STATE);
        goToPositionByTime(114, 15, DEFAULT_MOVEMENT_SPEED, DEFAULT_TURN_SPEED, 270, 3.5, BUILD_SITE_STATE, BUILD_SITE_STATE2);
        releaseStoneAtBeginningOfState(BUILD_SITE_STATE2);
        goToPositionByTime(124, 0, DEFAULT_MOVEMENT_SPEED, DEFAULT_TURN_SPEED, 270, 2.5, BUILD_SITE_STATE2, SECOND_SKYSTONE_STATE);
        FoundationUp();
        goToPositionByTime(124, 38, DEFAULT_MOVEMENT_SPEED, DEFAULT_TURN_SPEED, 270, 1.8, SECOND_SKYSTONE_STATE, PARK_STATE);           //TODO CHANGE TO SECOND_SKYSTONE_TAKE Instead of PARK_STATE
        lowerLiftDuringState(SECOND_SKYSTONE_TAKE);
        goToPositionByTime(SkyStonePosition - 10, 35, .7, .7, 270, 2, SECOND_SKYSTONE_TAKE, SECOND_SKYSTONE_TAKE2);
        goToPositionByTime(SkyStonePosition - 22, 35, DEFAULT_MOVEMENT_SPEED, DEFAULT_TURN_SPEED, 330, .7, SECOND_SKYSTONE_TAKE2, SECOND_SKYSTONE_TAKE3);
        goToPositionByTime(SkyStonePosition - 22, 47, .2, .5, 330, 2, SECOND_SKYSTONE_TAKE3, PARK_STATE);
        goToPositionByTime(76,35, .5, .7, 270, 1.5, PARK_STATE, PARK_STATE);
//        goToPositionByTime(20, 0, DEFAULT_MOVEMENT_SPEED, DEFAULT_TURN_SPEED, 90, 1.2, SECOND_SKYSTONE_PLACE, SECON
        lowerLiftDuringState(PARK_STATE);
    }

    private void LiftFinishDown() {
        if (currentTime > programStart + 29) {
            LiftMotor.setTargetPosition((int)(0 * countsPerInch));
        }
    }

    private void lowerLiftDuringState (int State) {
        if (autoState == State) {
            LiftShouldBeUp = false;
            LiftMotor.setTargetPosition(0);
        }
        else if (lastAutoState == State) {
            LiftShouldBeUp = true;
            LiftMotor.setTargetPosition((int)(3 * countsPerInch));
        }
    }

    private void skyStoneCheck() {
//        if (autoState == INIT_STATE) {
//            if (positionSkystone.equals("left")) {
//                SkyStonePosition = 96;
//            } else if (positionSkystone.equals("Center")) {
//                SkyStonePosition = 88;
//            } else if (positionSkystone.equals("Right")) {
//                SkyStonePosition = 82;
//            } else {
                SkyStonePosition = 49;
//            }
        }
//    }
    private void IntakeOn() {
        IntakeMotor.setPower(1);
    }

    private void FoundationDown() {
        if (autoState == SECOND_FOUNDATION_STATE) {
            lFoundationator.setPosition(foundationatorPosition);
            rFoundationator.setPosition(0);
        }
    }
    private void releaseStoneAtBeginningOfState(int state) {
        if (autoState == state) {
            if (lastAutoState != state) {
                releaseStoneCommand = true;
            }
            else {
                releaseStoneCommand = false;
            }
        }
    }

    private void grabRotateStoneAtBeginningOfState(int state) {
        if (autoState == state) {
            if(RobotXPosition > 76) {
                if(stoneGrab) {
                    grabRotateStoneCommand = true;
                    stoneGrab = false;
                }
                else {
                    grabRotateStoneCommand = false;
                }
            }
//        }
//        else if (lastAutoState == state) {
//            grabRotateStoneCommand = false;
//            stoneGrab = true;
        }
    }

    private void FoundationUp() {
        if (autoState == SECOND_SKYSTONE_STATE) {
            lFoundationator.setPosition(0);
            rFoundationator.setPosition(foundationatorPosition);
        }
    }

    private void goToPositionByTime(double x, double y, double preferredAngle, double timeout, int thisState, int nextState) {
        double maxMovementSpeed = 1.0;
        double maxTurnSpeed = 1.0;

        goToPositionByTime(x, y, maxMovementSpeed, maxTurnSpeed, preferredAngle, timeout, thisState, nextState);
    }

    private void goToPositionByTime(double x, double y, double maxMovementSpeed, double maxTurnSpeed, double preferredAngle, double timeout, int thisState, int nextState) {
        // exit method if auto is not in this state
        if (autoState != thisState) {
            return;
        }

        if (lastAutoState != thisState) {
            startTime = getRuntime();
            lastAutoState = thisState;
        }

        // setup timer, set startTime variable


        if (currentTime - timeout > startTime) {
            autoState = nextState;
        }
        goToPositionMrK(x, y, maxMovementSpeed, maxTurnSpeed, preferredAngle);
    }

    private void goToPositionMrK(double x, double y, double maxMovementSpeed, double maxTurnSpeed, double preferredAngle) {
        double distanceToTarget = Math.hypot(x-RobotXPosition, y-RobotYPosition);
        double absoluteAngleToTarget = Math.atan2(y-RobotYPosition, x-RobotXPosition);
        double relativeAngleToPoint = AngleWrap(-absoluteAngleToTarget
                - Math.toRadians(RobotRotation) + Math.toRadians(90));

        double relativeXToPoint = 2 * Math.sin(relativeAngleToPoint);
        double relativeYToPoint = Math.cos(relativeAngleToPoint);

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        double yDecelLimiter = Range.clip(Math.abs((distanceToTarget - DECELERATION_ZERO_POINT)
                / (DECELERATION_START_POINT - DECELERATION_ZERO_POINT)), 0, 1);
        double xDecelLimiter = Range.clip(yDecelLimiter * X_SPEED_MULTIPLIER, 0, 1);

        double relativeTurnAngle = AngleWrap(Math.toRadians(preferredAngle)-Math.toRadians(RobotRotation));
        double turnDecelLimiter = Range.clip((Math.abs(Math.toDegrees(relativeTurnAngle)) - TURNING_DECELERATION_ZERO_POINT)
                / (TURNING_DECELERATION_START_POINT - TURNING_DECELERATION_ZERO_POINT), 0, 1);

        movement_x = movementXPower * Range.clip(maxMovementSpeed, -xDecelLimiter, xDecelLimiter);
        movement_y = movementYPower * Range.clip(maxMovementSpeed, -yDecelLimiter, yDecelLimiter);

        if (distanceToTarget < 1) {
            movement_turn = 0;
        } else {
            //movement_turn = Range.clip(Range.clip(relativeTurnAngle / Math.toRadians(30),
            //        -1, 1) * maxTurnSpeed, -turnDecelLimiter, turnDecelLimiter);
            movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(TURNING_DECELERATION_START_POINT), -1, 1) * maxTurnSpeed;
        }
        telemetry.addData("relativeTurnAngle", relativeTurnAngle);
        telemetry.addData("turnDecelLimiter", turnDecelLimiter);
        telemetry.addData("relativeXToPoint", relativeXToPoint);
        telemetry.addData("relativeYToPoint", relativeYToPoint);
        telemetry.addData("X Movement", movement_x);
        telemetry.addData("Y Movement", movement_y);
        telemetry.addData("Turn Movement", movement_turn);

        lastDistanceToTarget = distanceToTarget;

        applyMovement();
    }

    private void applyMovement() {
        long currTime = SystemClock.uptimeMillis();
        if(currTime - lastUpdateTime < 16){
            return;
        }
        lastUpdateTime = currTime;

        //double fl_power_raw = movement_y-movement_turn+movement_x;
        //double bl_power_raw = movement_y-movement_turn-movement_x;
        //double br_power_raw = -movement_y-movement_turn-movement_x;
        //double fr_power_raw = -movement_y-movement_turn+movement_x;

        double fl_power_raw = movement_y+movement_turn+movement_x;
        double bl_power_raw = movement_y+movement_turn-movement_x;
        double br_power_raw = -movement_y+movement_turn-movement_x;
        double fr_power_raw = -movement_y+movement_turn+movement_x;

        //find the maximum of the powers
        double maxRawPower = Math.abs(fl_power_raw);
        if(Math.abs(bl_power_raw) > maxRawPower){ maxRawPower = Math.abs(bl_power_raw);}
        if(Math.abs(br_power_raw) > maxRawPower){ maxRawPower = Math.abs(br_power_raw);}
        if(Math.abs(fr_power_raw) > maxRawPower){ maxRawPower = Math.abs(fr_power_raw);}

        //if the maximum is greater than 1, scale all the powers down to preserve the shape
        double scaleDownAmount = 1.0;
        if(maxRawPower > 1.0){
            //when max power is multiplied by this ratio, it will be 1.0, and others less
            scaleDownAmount = 1.0/maxRawPower;
        }
        fl_power_raw *= scaleDownAmount;
        bl_power_raw *= scaleDownAmount;
        br_power_raw *= scaleDownAmount;
        fr_power_raw *= scaleDownAmount;

        //now we can set the powers ONLY IF THEY HAVE CHANGED TO AVOID SPAMMING USB COMMUNICATIONS
        FrontLeft.setPower(-fl_power_raw);
        BackLeft.setPower(-bl_power_raw);
        BackRight.setPower(-br_power_raw);
        FrontRight.setPower(-fr_power_raw);
    }
    private static double AngleWrap(double angle){

        while(angle < -Math.PI){
            angle += 2*Math.PI;
        }
        while(angle > Math.PI){
            angle -= 2*Math.PI;
        }

        return angle;
    }private void initializeStraightener() {
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
        if (stoneDistance < 1.5) {
            IntakeMotor.setPower(0);
        }

        orientStone();
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
        senseSkyStone();

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
            else if(stoneOrientation.equals("center") || isSkyStoneInView) {
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
            OrientationServoLeft.setPosition(lDisengage);
            OrientationServoRight.setPosition(rDisengage);
            readyToGrab = false;
            if (!straightenerBusy) {
                stoneOrientation = "empty";
            } else {
                straightenerBusy = false;
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

    private void initializeSkyStoneColorSensor(){
        SkyStoneSensor =  hardwareMap.get(NormalizedColorSensor.class, "SkyStoneSensor");
    }

    private void senseSkyStone() {
        NormalizedRGBA SkyStoneValues = SkyStoneSensor.getNormalizedColors();

        redValues = SkyStoneValues.red;
        blueValues = SkyStoneValues.blue;
        greenValues = SkyStoneValues.green;

        telemetry.addData("red", SkyStoneValues.red);
        telemetry.addData("green", SkyStoneValues.green);
        telemetry.addData("blue", SkyStoneValues.blue);
        telemetry.addData("skystone", isSkyStoneInView);
        telemetry.addData("red bool", red);

        if (redValues > .008 && redValues < .01) {
            isSkyStoneInView = true;
        }
        else {
            isSkyStoneInView = false;
        }
    }

    //
    // VERTICAL LIFT
    //

    private void initializeVerticalLift() {
        LiftMotor = hardwareMap.dcMotor.get("LiftMotor");
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setPower(1);
        LiftMotor.setTargetPosition(0);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setDirection(DcMotor.Direction.FORWARD);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    private void startVerticalLift() {
        LiftMotor.setTargetPosition((int)(0 * countsPerInch));
    }

    private void checkVerticalLift() {
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setPower(1);

        if (liftUpCommand) {
            if (liftHeight >= 10) {
                liftHeight = 10;
            }
            else {
                liftHeight++;
            }
            LiftMotor.setPower(1);
            LiftMotor.setTargetPosition((int)(liftHeight * (4 * countsPerInch) + liftOffset));
            liftUpCommand = false;
        }
        else if (liftDownCommand) {
            LiftMotor.setPower(.5);
            LiftMotor.setTargetPosition((int)(3 * countsPerInch));
            liftDownCommand = false;
        }

        telemetry.addData("Lift Height", liftHeight);
    }

    //
    // Grabber Arm
    //

    private void initializeGrabber() {
        GrabberServo =  hardwareMap.servo.get("GrabberServo");
        OrientStoneServo = hardwareMap.servo.get("OrientStoneServo");
        MoveArmServo = hardwareMap.servo.get("MoveArmServo");
    }

    private void startGrabber() {
        MoveArmServo.setPosition(armInsidePosition);
        GrabberServo.setPosition(grabberOpenPosition);
    }

    private void checkGrabber() {

        if (grabRotateStoneCommand) {
            LiftMotor.setTargetPosition(0);
            liftGrabberState = 1;
            startGrabberTime = getRuntime();
        }
        grabRotateRaiseStone();

        if (releaseStoneCommand) {
            GrabberServo.setPosition(grabberOpenPosition);
            liftGrabberState = 0;
            grabberReturnState = 1;
            startGrabberTime = getRuntime();
            releaseStoneCommand = false;
        }
        returnGrabberArm();
    }

    private void grabRotateRaiseStone() {
        if(liftGrabberState == 1) {
            if (LiftMotor.getCurrentPosition() < (int)(.25*countsPerInch)) {
                startGrabberTime = getRuntime();
                GrabberServo.setPosition(grabberClosedPosition);
                liftGrabberState++;
            }
        }
        if(liftGrabberState == 2) {
            currentGrabberTime = getRuntime();
            if (currentGrabberTime - startGrabberTime > .5) {
                liftGrabberState++;
                startGrabberTime = getRuntime();
            }
        }
        if(liftGrabberState == 3) {
            currentGrabberTime = getRuntime();
            if (liftHeight <= 1) {
                LiftMotor.setPower(1);
                LiftMotor.setTargetPosition((int)((10 * countsPerInch) + liftOffset));
                if(LiftMotor.getCurrentPosition() > (int)((9 * countsPerInch) + liftOffset)) {
                    MoveArmServo.setPosition(armOutsidePosition);
                    if (currentGrabberTime - startGrabberTime > 2) {
                        liftUpCommand = true;
                        liftGrabberState = 0;
                    }
                }
            }
            else {
                liftUpCommand = true;
                liftGrabberState++;
                startGrabberTime = getRuntime();
            }
        }
        if(liftGrabberState == 4) {
            currentGrabberTime = getRuntime();
            if (LiftMotor.getCurrentPosition() > (int)(liftHeight * (4 * countsPerInch) + (liftOffset-(.5 * countsPerInch)))) {
                liftGrabberState++;
            }
        }
        else if(liftGrabberState == 5) {
            MoveArmServo.setPosition(armOutsidePosition);
            liftGrabberState = 0;
        }
    }

    private void returnGrabberArm() {
        if (grabberReturnState == 1) {
            LiftMotor.setPower(.5);
            LiftMotor.setTargetPosition((int)((liftHeight * (4 * countsPerInch)) + liftOffset - (2 * countsPerInch)));
            if (LiftMotor.getCurrentPosition() < (int)(liftHeight * (4 * countsPerInch) + liftOffset - (1.5 * countsPerInch))) {
                GrabberServo.setPosition(grabberOpenPosition);
                grabberReturnState++;
                startGrabberTime = getRuntime();
            }
        }
        if (grabberReturnState == 2) {
            LiftMotor.setPower(1);
            LiftMotor.setTargetPosition((int)((liftHeight * (4 * countsPerInch)) + liftOffset + (1 * countsPerInch)));
            if (LiftMotor.getCurrentPosition() > (int)(liftHeight * (4 * countsPerInch) + liftOffset + (.5 * countsPerInch))) {
                grabberReturnState++;
                startGrabberTime = getRuntime();
            }
        }
        if (grabberReturnState == 3) {
            currentGrabberTime = getRuntime();

            if(currentGrabberTime - startGrabberTime > .5) {
                if (liftHeight <= 1) {
                    LiftMotor.setPower(1);
                    LiftMotor.setTargetPosition((int)((6 * countsPerInch) + liftOffset));
                    if(LiftMotor.getCurrentPosition() > (int)((5 * countsPerInch) + liftOffset)) {
                        MoveArmServo.setPosition(armInsidePosition);
                        grabberReturnState++;
                        startGrabberTime = getRuntime();
                    }
                }
                else {
                    MoveArmServo.setPosition(armInsidePosition);
                    grabberReturnState++;
                    startGrabberTime = getRuntime();
                }
            }

        }
        else if (grabberReturnState == 4) {
            currentGrabberTime = getRuntime();
            if(currentGrabberTime - startGrabberTime > 1.5) {
                liftDownCommand = true;
                grabberReturnState = 0;
            }
        }
    }
}
