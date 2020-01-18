package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Autonomous(name= "PickStoneBlue", group= "None")
public class PickRandomStone extends SkystoneVuforiaNew {

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
    private Servo IntakeReleaseServo;
    private CRServo IntakeAssistServo;
    private Servo lFoundationator;
    private Servo rFoundationator;

    final double COUNTS_PER_INCH = 307.699557;
    private double RobotXPosition;
    private double RobotYPosition;
    private double RobotRotation;
    private double startTime;
    private double currentTime;
    private double intakeNotReleased = .6;
    private double intakeReleased = .15;
    private double lastDistanceToTarget = 0;
    private double StartingXPosition = 9;
    private double StartingYPosition = 36;
    private double StartingRotation = 90;
    private double foundationatorPosition = .335;
    private double movement_x;
    private double movement_y;
    private double movement_turn;
    private int autoState = INIT_STATE;
    private int lastAutoState = NO_STATE;
    private boolean autoComplete = false;
    private boolean readyToGrab = true;
    private boolean LiftFall = true;
    private static final double DECELERATION_START_POINT = 48;
    private static final double DECELERATION_ZERO_POINT = -6;   // -6
    private static final double TURNING_DECELERATION_START_POINT = 180;
    private static final double TURNING_DECELERATION_ZERO_POINT = -5; // -5
    private static final double X_SPEED_MULTIPLIER = 1;
    private static final int NO_STATE = -1;
    private static final int INIT_STATE = 0;
    private static final int FOUDNATION_STATE = 1;
    private static final int LIFT_STATE = 2;
    private static final int PARK_STATE = 3;
    private long lastUpdateTime = 0;

    @Override
    public void init() {
        super.init();
        msStuckDetectStart = 300000;
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
        IntakeReleaseServo = hardwareMap.get(Servo.class, "IntakeReleaseServo");
        IntakeAssistServo = hardwareMap.crservo.get("IntakeAssistServo");
        lFoundationator = hardwareMap.servo.get("lFoundationator");
        rFoundationator = hardwareMap.servo.get("rFoundationator");
//        IntakeReleaseServo.setPosition(intakeNotReleased);
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
        StartingXPosition = 0;
        StartingYPosition = 0;
        StartingRotation = 0;
    }

    @Override
    public void start() {
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();

        checkOdometry();

        lFoundationator.setPosition(0);
        rFoundationator.setPosition(foundationatorPosition);

//        RobotXPosition = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH) + StartingXPosition;
//        RobotYPosition = -(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH) + StartingYPosition;
//        RobotRotation = (globalPositionUpdate.returnOrientation() / COUNTS_PER_INCH) + StartingRotation;
    }


    private void checkOdometry() {
        RobotXPosition = (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH) + StartingXPosition;
        RobotYPosition = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH) + StartingYPosition;
        RobotRotation = (globalPositionUpdate.returnOrientation()) + StartingRotation;
    }

    @Override
    public void loop() {
        currentTime = getRuntime();
        IntakeAssistServo.setPower(-1);
        positionSkystone.equals("Left");
        if (autoState == INIT_STATE) {
            if (lastAutoState == NO_STATE) {
                lastAutoState = INIT_STATE;
                startTime = getRuntime();
                IntakeReleaseServo.setPosition(.6);
                IntakeAssistServo.setPower(0);
            }
            else {
                if (startTime > currentTime - 32) {           //TODO CHANGE
                    IntakeMotor.setPower(1);
 //                   if (positionSkystone.equals("Left")) {
                    if (startTime > currentTime - 1.5) {
                        driveToDrop();
                    }
                    else if (startTime > currentTime - 4) {
                        driveToSkystoneLeft();
                    }
                    else if (startTime > currentTime - 7){
                        driveNearFoundation();
                    }
//                    }
//                    else if (positionSkystone.equals("Center")) {
//                        if (startTime > currentTime - 5) {
//                            driveToSkystoneCenter();
//                        }
//                    }
//                    else if (positionSkystone.equals("Right")) {
//                        if (startTime > currentTime - 5) {
//                            driveToSkystoneRight();
//                        }
//                    }
                }
                else {
                    autoState = FOUDNATION_STATE;
                }
            }
        }
        else if (autoState == FOUDNATION_STATE) {
            if (lastAutoState == INIT_STATE) {
                lastAutoState = FOUDNATION_STATE;
                startTime = getRuntime();
                lFoundationator.setPosition(foundationatorPosition);
                rFoundationator.setPosition(0);
            }
            else {
                if (startTime > currentTime - 1.5) {
                    driveNearFoundation();
                }
                else if ( startTime < (currentTime - 1.5) && startTime > (currentTime - 3.5) ) {
                    driveToFoundation();
                }
                else if (startTime < (currentTime - 3.5) && startTime > (currentTime - 6)) {
                    driveToBuildSite();
                }
                else if (startTime < (currentTime - 6) && startTime > (currentTime - 7.5)) {
                    driveFoundationInBuildSite();
                }
                else if (startTime < (currentTime - 7.6) && startTime > (currentTime - 8.6)) {
                    shimmyForwardFromBuildSite();
                }
                else {
                    lFoundationator.setPosition(0);
                    rFoundationator.setPosition(foundationatorPosition);
                    driveToPark();
                }
            }
        }

        checkOdometry();
    }

    private void driveToSkystoneLeft() {
        goToPositionMrK(-3, 30, .4, .5, 0);
    }
    private void driveToSkystoneCenter() {
        goToPositionMrK(8, 30, .7, .5, 0);
    }
    private  void driveToSkystoneRight() {
        goToPositionMrK(19, 30, .7, .5, 0);
    }
    private void driveNearFoundation() {
        goToPositionMrK(-72, 9, .7, .5, -90);
    }
    public void driveToFoundation() {
        goToPositionMrK(-52,43,.7, .5,-180);
    }
    public void driveToDrop(){goToPositionMrK(-3,8,.7,.5,0);}
    public void driveToBuildSite () {
        goToPositionMrK(-33, 7, .7, .5, 60); //TODO change if problems
    }
    public void driveFoundationInBuildSite () {
        goToPositionMrK(-39, 9, .7, .5, 90);  //TODO change if problems
    }
    public void shimmyForwardFromBuildSite () {
        goToPositionMrK(-16, 9, .7, .5, 90);  //TODO change if problems
    }
    private void driveToPark() {
        goToPositionMrK(-72, 9, .7, .5, 90);
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




    /**
     * Converts movement_y, movement_x, movement_turn into motor powers.
     *
     * author: Original code by FTC Team 7571 Alumineers, modified by Travis Kuiper
     * date: 2020/01/01
     *
     * movement_turn: Positive value to turn clockwise, negative for counterclockwise
     * movement_x: Positive value indicates strafe to the right proportionally
     * movement_y: Positive value indicates drive forward proportionally
     */
    // Code comes from 11115 Peter and 7571 Alumineers
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
    }

}