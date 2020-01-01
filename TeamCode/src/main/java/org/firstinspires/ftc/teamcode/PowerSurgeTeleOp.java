package org.firstinspires.ftc.teamcode;

import android.os.SystemClock;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import com.qualcomm.robotcore.util.Range;

import java.io.File;
import java.util.ArrayList;


@TeleOp(name = "PowerSurgeTeleOp")
public class PowerSurgeTeleOp extends OpMode {

    // DRIVETRAIN AND ODOMETRY

    public static final double DEADZONE = 0.15;

    DcMotor verticalRight, verticalLeft, horizontal;

    String verticalLeftEncoderName = "FrontLeft";
    String verticalRightEncoderName = "BackRight";
    String horizontalEncoderName = "BackLeft";

    private double RobotXPosition;
    private double RobotYPosition;
    private double RobotRotation;

    private double StartingXPosition;
    private double StartingYPosition;
    private double StartingRotation;

    private File startingXpositionFile = AppUtil.getInstance().getSettingsFile("startingXposition.txt");
    private File startingYpositionFile = AppUtil.getInstance().getSettingsFile("startingYposition.txt");
    private File startingθpositionFile = AppUtil.getInstance().getSettingsFile("startingθposition.txt");

    final double COUNTS_PER_INCH = 307.699557;
    private static final int PreFoundationXPosition = 36;
    private static final int PreFoundationYPosition = 95;
    private static final int FoundationXPosition = 48;
    private static final int FoundationYPosition = 107;
    private static final int BuildSiteXPosition = 9;
    private static final int BuildSiteYPosition = 111;
    private static final int ParkLineXPosition = 9;
    private static final int ParkLineYPosition = 72;

    private int autoDrivingStage = 0;
    private int autoDrivingTimes = 0;

    private double movement_x;
    private double movement_y;
    private double movement_turn;

    private double rightBackupDistance;
    private double leftBackupDistance;

    OdometryGlobalCoordinatePosition globalPositionUpdate;
    Thread positionThread;

    // MOTORS SERVOS SENSORS

    private DcMotor FrontRight;
    private DcMotor FrontLeft;
    private DcMotor BackRight;
    private DcMotor BackLeft;
    private DcMotor IntakeMotor;
    private DcMotor LiftMotor;
    private Servo lFoundationator;
    private Servo rFoundationator;
    private Servo GrabberServo;
    private Servo OrientStoneServo;
    private Servo MoveArmServo;
    private Servo OrientationServoLeft;
    private Servo OrientationServoRight;
    private CRServo IntakeAssistServo;
    private Servo IntakeReleaseServo;

    private ModernRoboticsI2cRangeSensor OrientationSensor;
    private ModernRoboticsI2cRangeSensor StonePresenceSensor;
    private ModernRoboticsI2cRangeSensor RightBackupSensor;
    private ModernRoboticsI2cRangeSensor LeftBackupSensor;

    // FIRST PRESS AND RUN

    private boolean firstLiftUpButton;
    private boolean firstLiftDownButton;
    private boolean liftUpCommand;
    private boolean liftDownCommand;
    private boolean outputButton;
    private boolean intakeButton;
    private boolean firstPressDpadUp = true;
    private boolean firstPressBumpers = true;
    private boolean liftEncoderState = true;
    private boolean firstPressy = true;
    private boolean firstPressb = true;
    private boolean firstPressx = true;
    private boolean firstPressx2 = true;
    private boolean firstPressb2 = true;
    private boolean firstPressDown = true;
    private boolean firstPressUp = true;
    private boolean firstLeftRun = true;
    private boolean firstPressy2 = true;
    private boolean firstPressRightTrigger = true;
    private boolean firstPressLeftTrigger = true;

    // WAFFLE STUFF

    private boolean lastWaffleState = false;
    private boolean isWaffleStateRaised = false;

    // INTAKE STUFF

    private int intakeState = 0;
    private int intakeReleaseState = 1;
    private int liftHeight = -1;

    // LIFT STUFF

    static final double countsPerMotor          = 383.6;
    static final double gearReduction           = 1.0 ;
    static final double wheelDiameter           = 1.771653543307087 ;
    static final double countsPerInch           = (countsPerMotor * gearReduction) / (wheelDiameter * Math.PI);
    static final double liftOffset = (1.5 * countsPerInch);     //TODO Change

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
    private double targetTime = .5 + waitTime;
    private double stoneDistance = 0;
    private boolean readyToGrab = false;
    private boolean stoneFullyInStraightener = false;
    private boolean straightenerBusy = false;
    private boolean firstRightRun = true;

    // GRABBER STUFF

    private double startGrabberTime;
    private double currentGrabberTime;
    private int liftGrabberState = 0;
    private int grabberReturnState = 0;
    private int emergencyStoneEjectState = 0;
    private double grabberOpenPosition = 0;
    private double grabberClosedPosition = .4;

    //last update time
    private long lastUpdateTime = 0;


    @Override
    public void init() {
        telemetry.addData("Version Number", "1-1-20 100pm");
        initializeVerticalLift();
        initializeFoundationator();
        initializeGrabber();
        initializeDriveTrain();
        initializeOdometry();
        initializeIntakeMechanism();
        initializeStraightener();
        telemetry.addData("Status", "Init Complete");
        telemetry.update();
    }

    @Override
    public void start() {
        startFoundationator();
        startIntakeMechanism();
        startGrabber();
        startOdometry();
        telemetry.addData("Status", "Odometry System has started");
        telemetry.update();
    }

    @Override
    public void loop() {
        checkDriveTrain();
        checkOdometry();
        checkVerticalLift();
        checkFoundationator();
        checkIntakeMechanism();
        checkStraightener();
        checkGrabber();
        telemetry.update();
    }

    //
    // VERTICAL LIFT
    //

    public void initializeVerticalLift() {
        LiftMotor = hardwareMap.dcMotor.get("LiftMotor");
        LiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LiftMotor.setPower(0);
        LiftMotor.setTargetPosition(0);
        LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        LiftMotor.setDirection(DcMotor.Direction.FORWARD);
        LiftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void checkVerticalLift() {
        double LiftUpButton = gamepad1.right_trigger;
        double LiftDownButton = gamepad1.left_trigger;
        boolean LiftManualToggleButton = gamepad2.y;
        boolean LiftOverideDownButton = gamepad2.dpad_down;
        boolean LiftOverideUpButton = gamepad2.dpad_up;

        if (LiftManualToggleButton) {
            if (firstPressy2) {
                liftEncoderState =! liftEncoderState;
                firstPressy2 = false;
            }
        }
        else {
            firstPressy2 = true;
        }

        if (LiftOverideDownButton) {
            if (firstPressDown) {
                liftHeight--;
                firstPressDown = false;
            }
        }
        else {
            firstPressDown = true;
        }

        if (LiftOverideUpButton) {
            if (firstPressUp) {
                liftHeight++;
                firstPressUp = false;
            }
        }
        else {
            firstPressUp = true;
        }

        if (liftEncoderState) {
            LiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            LiftMotor.setPower(1);
            /*telemetry.addData("LiftMotorDistance",LiftMotor.getCurrentPosition() - LiftMotor.getTargetPosition());
            //setting the Target position if we press the right bumper
            if (LiftUpButton > .5) {
                if (firstLiftUpButton) {
                    liftUpCommand = true;
                    firstLiftUpButton = false;
                }
                else {
                    liftUpCommand = false;
                }
            }
            else {
                firstLiftUpButton = true;
            }

            if (LiftDownButton > .5) {
                if (firstLiftDownButton) {
                    liftDownCommand = true;
                    firstLiftDownButton = false;
                }
                else {
                    liftDownCommand = false;
                }
            }
            else {
                firstLiftDownButton = true;
            }*/

            if (liftUpCommand) {
                if (liftHeight >= 10) {
                    liftHeight = 10;
                }
                else {
                    liftHeight++;
                }

                LiftMotor.setTargetPosition((int)(liftHeight * (4 * countsPerInch) + liftOffset));
                liftUpCommand = false;
            }
            else if (liftDownCommand) {
                LiftMotor.setTargetPosition(0);
                liftDownCommand = false;
            }

            telemetry.addData("Lift Height", liftHeight);

            /*if (liftUpCommand) {
                LiftMotor.setTargetPosition((int)(LiftMotor.getTargetPosition() + (4 * countsPerInch)));
            }
            //setting the Target position if we press the left Bumper
            else if (liftDownCommand) {
                LiftMotor.setTargetPosition((int)(LiftMotor.getTargetPosition() + (-4 * countsPerInch)));
            }*/
        }
        else {
            LiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            LiftMotor.setPower(LiftUpButton - LiftDownButton);
        }
    }

    //
    // FOUNDATIONATOR
    //

    public void initializeFoundationator() {
        lFoundationator = hardwareMap.servo.get("lFoundationator");
        rFoundationator = hardwareMap.servo.get("rFoundationator");
    }

    public void startFoundationator() {
        lFoundationator.setPosition(0);
        rFoundationator.setPosition(foundationatorPosition);
    }

    public void checkFoundationator() {
        boolean currentWaffleState = gamepad1.a;

        if (lastWaffleState == currentWaffleState){
            return;
        }
        else if(!lastWaffleState && currentWaffleState){
            if(isWaffleStateRaised) {
                lowerFoundationator();
            } else {
                raiseFoundationator();
            }
        }
        lastWaffleState = currentWaffleState;

        if (isWaffleStateRaised) {
            telemetry.addData("Foundationator is", "Raised");
        }
        else {
            telemetry.addData("Foundationator is", "Lowered");
        }
    }

    private void raiseFoundationator() {
        lFoundationator.setPosition(0);
        rFoundationator.setPosition(foundationatorPosition);
        isWaffleStateRaised = true;
    }

    private void lowerFoundationator() {
        lFoundationator.setPosition(foundationatorPosition);
        rFoundationator.setPosition(0);
        isWaffleStateRaised = false;
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
        MoveArmServo.setPosition(1);
        GrabberServo.setPosition(grabberOpenPosition);
    }

    private void checkGrabber() {
        if (gamepad1.right_trigger > .5) {
            if (firstPressRightTrigger) {
                GrabberServo.setPosition(grabberClosedPosition);
                liftGrabberState = 1;
                startGrabberTime = getRuntime();
                firstPressRightTrigger = false;
            }
        }
        else {
            firstPressRightTrigger = true;
        }
        grabRotateRaiseStone();

        if(gamepad1.left_trigger > .5) {
            if(firstPressLeftTrigger) {
                GrabberServo.setPosition(grabberOpenPosition);
                liftGrabberState = 0;
                grabberReturnState = 1;
                startGrabberTime = getRuntime();
                firstPressLeftTrigger = false;
            }
        }
        else {
            firstPressLeftTrigger = true;
        }
        returnGrabberArm();

        if(gamepad1.y) {
            if(firstPressy) {
                emergencyStoneEjectState = 1;
                GrabberServo.setPosition(grabberClosedPosition);
                startGrabberTime = getRuntime();
                firstPressy = false;
            }
        }
        else {
            firstPressy = true;
        }
        emergencyStoneEject();
    }

    private void grabRotateRaiseStone() {
        if(liftGrabberState == 1) {
            currentGrabberTime = getRuntime();
            if (currentGrabberTime - startGrabberTime > .5) {
                liftGrabberState++;
                startGrabberTime = getRuntime();
            }
        }
        else if(liftGrabberState == 2) {
            currentGrabberTime = getRuntime();
            if (liftHeight <= 0) {
                LiftMotor.setTargetPosition((int)((9 * countsPerInch) + liftOffset));
                if(LiftMotor.getCurrentPosition() > (int)((8 * countsPerInch) + liftOffset)) {
                    MoveArmServo.setPosition(0);
                    if (currentGrabberTime - startGrabberTime > 1.5) {
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
        else if(liftGrabberState == 3) {
            currentGrabberTime = getRuntime();
            if (LiftMotor.getCurrentPosition() > (int)(liftHeight * (4 * countsPerInch) + (liftOffset-(.5 * countsPerInch)))) {
                liftGrabberState++;
            }
        }
        else if(liftGrabberState == 4) {
            MoveArmServo.setPosition(0);
            liftGrabberState = 0;
        }
    }

    private void returnGrabberArm() {
        if (grabberReturnState == 1) {
            currentGrabberTime = getRuntime();
            GrabberServo.setPosition(grabberOpenPosition);

            if(currentGrabberTime - startGrabberTime > .5) {
                if (liftHeight <= 0) {
                    LiftMotor.setTargetPosition((int)((5 * countsPerInch) + liftOffset));
                    if(LiftMotor.getCurrentPosition() > (int)((4 * countsPerInch) + liftOffset)) {
                        MoveArmServo.setPosition(1);
                        grabberReturnState++;
                        startGrabberTime = getRuntime();
                    }
                }
                else {
                    MoveArmServo.setPosition(1);
                    grabberReturnState++;
                    startGrabberTime = getRuntime();
                }
            }

        }
        else if (grabberReturnState == 2) {
            currentGrabberTime = getRuntime();
            if(currentGrabberTime - startGrabberTime > 1) {
                liftDownCommand = true;
                grabberReturnState = 0;
            }
        }
    }

    private void emergencyStoneEject() {
        if(emergencyStoneEjectState == 1) {
            currentGrabberTime = getRuntime();
            if (currentGrabberTime - startGrabberTime > .5) {
                LiftMotor.setTargetPosition((int)((8 * countsPerInch) + liftOffset));
                if(LiftMotor.getCurrentPosition() > (int)((7 * countsPerInch) + liftOffset)) {
                    MoveArmServo.setPosition(0);
                    startGrabberTime = getRuntime();
                    emergencyStoneEjectState++;
                }
            }
        }
        else if(emergencyStoneEjectState == 2) {
            currentGrabberTime = getRuntime();
            if (currentGrabberTime - startGrabberTime > 1) {
                GrabberServo.setPosition(grabberOpenPosition);
                MoveArmServo.setPosition(1);
                startGrabberTime = getRuntime();
                emergencyStoneEjectState++;
            }
        }
        else if(emergencyStoneEjectState == 3) {
            currentGrabberTime = getRuntime();
            if (currentGrabberTime - startGrabberTime > 1) {
                liftDownCommand = true;
                emergencyStoneEjectState = 0;
            }
        }
    }

    /*private void grabStoneFromStraightener() {
        GrabberServo.setPosition(.5);
        // close grabber servo
        return;
    }
    //this.moveArmToScorePosition("horizontal");
    private void moveArmToScorePosition(String orientation) {
        if(orientation.contentEquals("long")) {
        }
        else if(orientation.contentEquals("wide")) {
        }
        // move arm to scoring position
        // orient the grabber to prepare to place the stone
        return;
    }

    private void releaseStoneFromGrabber() {
        GrabberServo.setPosition(0);
        // open grabber servo to release stone
        return;
    }*/

    //
    // DRIVE TRAIN
    //

    public void initializeDriveTrain() {
        RightBackupSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor .class, "RightBackupSensor");
        LeftBackupSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor .class, "LeftBackupSensor");
        FrontRight = hardwareMap.dcMotor.get("FrontRight");
        FrontLeft = hardwareMap.dcMotor.get("FrontLeft");
        BackRight = hardwareMap.dcMotor.get("BackRight");
        BackLeft = hardwareMap.dcMotor.get("BackLeft");
        FrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void checkDriveTrain() {
        rightBackupDistance = RightBackupSensor.getDistance(DistanceUnit.INCH);
        leftBackupDistance = LeftBackupSensor.getDistance(DistanceUnit.INCH);

        telemetry.addData("Right Backup Distance", rightBackupDistance);
        telemetry.addData("Left Backup Distance", leftBackupDistance);

        if (gamepad1.right_bumper && gamepad1.left_bumper) {
            if (firstPressBumpers) {
                //autoDrivingStage = 0;
                //autoDrivingTimes = 0;
                firstPressBumpers = false;
            }
            goToPositionMrK(0,0,.5,.5, 0);
        } else {
            firstPressBumpers = true;

            double forwardButton = gamepad1.left_stick_y;
            double sidewaysButton = gamepad1.left_stick_x;
            double spinningButton = gamepad1.right_stick_x;

            FrontRight.setDirection(DcMotor.Direction.REVERSE);
            BackLeft.setDirection(DcMotor.Direction.REVERSE);

            forwardButton = DeadModifier(forwardButton);
            sidewaysButton = DeadModifier(sidewaysButton);
            spinningButton = DeadModifier(spinningButton);

            Drive(forwardButton, sidewaysButton, spinningButton);
        }
    }

    // From Gluten Free
    public void goToPosition(double x, double y, double movementSpeed, double turnSpeed, double preferredAngle) {
        double distanceToTarget = Math.hypot(x-RobotXPosition, y-RobotYPosition);

        double absoluteAngleToTarget = Math.atan2(y-RobotYPosition, x-RobotXPosition);
        double relativeAngleToPoint = AngleWrap(absoluteAngleToTarget
                - (Math.toRadians(RobotRotation)-Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint) * distanceToTarget;
        double relativeYToPoint = Math.sin(relativeAngleToPoint) * distanceToTarget;

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;

        double relativeTurnAngle = relativeAngleToPoint - Math.toRadians(180) + preferredAngle;
        if (distanceToTarget < 3) {
            movement_turn = 0;
        } else {
            movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;
        }
        if (distanceToTarget < 1) {
            movement_x = 0;
            movement_y = 0;
        }

        Drive(.3*movement_y, -movement_x, .3*movement_turn);

        telemetry.addData("Distance to Target", distanceToTarget);

        telemetry.addData("absoluteAngleToTarget", absoluteAngleToTarget);
        telemetry.addData("relativeTurnAngle", relativeTurnAngle);

        telemetry.addData("MovementX", movement_x);
        telemetry.addData("MovementY", movement_y);
        telemetry.addData("MovementTurn", movement_turn);
        
        applyMovement();
    }

    // From mrKuiper
    public void goToPositionMrK(double x, double y, double movementSpeed, double turnSpeed, double preferredAngle) {
        double distanceToTarget = Math.hypot(x-RobotXPosition, y-RobotYPosition);
        double absoluteAngleToTarget = Math.atan2(y-RobotYPosition, x-RobotXPosition);
        double relativeAngleToPoint = AngleWrap(-absoluteAngleToTarget
                - (Math.toRadians(RobotRotation)+Math.toRadians(90)));

        double relativeXToPoint = Math.cos(relativeAngleToPoint);
        double relativeYToPoint = Math.sin(relativeAngleToPoint);

        double movementXPower = relativeXToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));
        double movementYPower = relativeYToPoint / (Math.abs(relativeXToPoint) + Math.abs(relativeYToPoint));

        movement_x = movementXPower * movementSpeed;
        movement_y = movementYPower * movementSpeed;

        double relativeTurnAngle = AngleWrap(Math.toRadians(preferredAngle)-Math.toRadians(RobotRotation);
        if (distanceToTarget < 3) {
            movement_turn = 0;
        } else {
            movement_turn = Range.clip(relativeTurnAngle / Math.toRadians(30), -1, 1) * turnSpeed;
        }

        telemetry.addData("X Movement", movement_x);
        telemetry.addData("Y Movement", movement_y);
        telemetry.addData("Turn Movement", movement_turn);

        applyMovement();
    }

    /**converts movement_y, movement_x, movement_turn into motor powers */
    // Code comes from 11115 Peter and 7571 Alumineers
    public void applyMovement() {
        long currTime = SystemClock.uptimeMillis();
        if(currTime - lastUpdateTime < 16){
            return;
        }
        lastUpdateTime = currTime;


        double fl_power_raw = movement_y-movement_turn+movement_x*1.5;
        double bl_power_raw = movement_y-movement_turn- movement_x*1.5;
        double br_power_raw = -movement_y-movement_turn-movement_x*1.5;
        double fr_power_raw = -movement_y-movement_turn+movement_x*1.5;

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
        FrontLeft.setPower(fl_power_raw);
        BackLeft.setPower(bl_power_raw);
        BackRight.setPower(br_power_raw);
        FrontRight.setPower(fr_power_raw);
    }

    /**
     * wrap angle to -180 to 180
     * @param angle angle to be wrapped in radians
     * @return new angle in radians
     */

    public static double AngleWrap(double angle){

        while(angle < -Math.PI){
            angle += 2*Math.PI;
        }
        while(angle > Math.PI){
            angle -= 2*Math.PI;
        }


        return angle;
    }

    /*public ArrayList<Point> lineCircleIntersection(Point circleCenter, double radius, Point linePoint1, Point linePoint2) {
        if(Math.abs(linePoint1.y - linePoint2.y) < .003) {
            linePoint1.y = linePoint2.y + .003;
        }
        if(Math.abs(linePoint1.x - linePoint2.x) < .003) {
            linePoint1.x = linePoint2.x + .003;
        }
    }*/

    // From Wizards

    /*private void goToPosition(double targetXPosition, double targetYPosition, double targetOrientation, double power, double allowableDistanceError) {
        double distanceToXTarget = (targetXPosition*COUNTS_PER_INCH) - globalPositionUpdate.returnXCoordinate();
        double distanceToYTarget = (targetYPosition*COUNTS_PER_INCH) - globalPositionUpdate.returnYCoordinate();

        double distance = Math.hypot(distanceToXTarget, distanceToYTarget);

        double robotMovementAngle = Math.toDegrees(Math.atan2(distanceToXTarget, distanceToYTarget));

        double robot_movement_x_component = calculateX(robotMovementAngle, power);
        double robot_movement_y_component = calculateY(robotMovementAngle, power);
        double pivotCorrection = targetOrientation - globalPositionUpdate.returnOrientation();

        if (distance < allowableDistanceError) {
            Drive(0,0,0);
        }
        else {
//            FrontRight.setPower(-DZSidewaysButton - DZForwardButton - DZSpinningButton);
//            FrontLeft.setPower(-DZSidewaysButton + DZForwardButton - DZSpinningButton);
//            BackRight.setPower(DZSidewaysButton - DZForwardButton - DZSpinningButton);
//            BackLeft.setPower(DZSidewaysButton + DZForwardButton - DZSpinningButton);

            telemetry.addData("robot_movement_x_component", robot_movement_x_component);
            telemetry.addData("robot_movement_y_component", robot_movement_y_component);
            telemetry.addData("pivotCorrection", pivotCorrection);
            telemetry.addData("robotMovementAngle", robotMovementAngle);
        }
    }*/

    private double calculateX(double desiredAngle, double speed) {
        return Math.sin(Math.toRadians(desiredAngle)) * speed;
    }
    private double calculateY(double desiredAngle, double speed) {
        return Math.cos(Math.toRadians(desiredAngle)) * speed;
    }

    // By DH

    public void driveToScoringPosition() {
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

    public void smoothDriveToScoringPosition() {
        java.lang.String turningDirection;
        double distanceFromOrientation;

        if (RobotRotation > 2 && RobotRotation < 180) {
            turningDirection = "left";
        } else {
            turningDirection = "right";
        }
        telemetry.addData("TurningDirection", turningDirection);

        if (autoDrivingStage == 0) {
            if (RobotRotation > 5 && RobotRotation < 355) {
                if (turningDirection.equals("right")) {
                    distanceFromOrientation = 360 - RobotRotation;
                    if (distanceFromOrientation > 120) {
                        Drive(0, 0, .6);
                    } else if (distanceFromOrientation > 45) {
                        Drive(0, 0, .35);
                    } else {
                        Drive(0, 0, .15);
                    }
                } else {
                    distanceFromOrientation = RobotRotation;
                    if (distanceFromOrientation > 120) {
                        Drive(0, 0, -.6);
                    } else if (distanceFromOrientation > 45) {
                        Drive(0, 0, -.35);
                    } else {
                        Drive(0, 0, -.15);
                    }
                }
                telemetry.addData("DistanceFromOrientation", distanceFromOrientation);
            }
            else {
                autoDrivingStage++;
            }
        }
        else if (autoDrivingStage == 1) {
            if (RobotYPosition < -.5 || RobotYPosition > .5) {
                if (RobotYPosition < 0) {
                    if (RobotYPosition < -12) {
                        Drive(-.3, 0, 0);
                    } else {
                        Drive(-.1, 0, 0);
                    }
                } else {
                    if (RobotYPosition > 12) {
                        Drive(.3, 0, 0);
                    } else {
                        Drive(.1, 0, 0);
                    }
                }
            }
            else {
                autoDrivingStage++;
            }
        }
        else if (autoDrivingStage == 2) {
            if (RobotXPosition < -.5 || RobotXPosition > .5) {
                if (RobotXPosition < 0) {
                    if (RobotXPosition < -6) {
                        Drive(0, .5, 0);
                    } else {
                        Drive(0, .2, 0);
                    }
                } else {
                    if (RobotXPosition > 6) {
                        Drive(0, -.5, 0);
                    } else {
                        Drive(0, -.2, 0);
                    }
                }
            } else {
                if (autoDrivingTimes == 0) {
                    autoDrivingTimes = 1;
                    autoDrivingStage = 0;
                }
                else {
                    Drive(0, 0, 0);
                }
            }
        }
    }

    public void oneMotionDriveToScoringPosition() {
        java.lang.String turningDirection;
        double distanceFromOrientation;
        int inversingPowerRotationMultiplier;

        if (RobotRotation > 2 && RobotRotation < 180) {
            turningDirection = "left";
        } else {
            turningDirection = "right";
        }
        telemetry.addData("TurningDirection", turningDirection);
        if (turningDirection.equals("right")) {
            distanceFromOrientation = 360 - RobotRotation;
            inversingPowerRotationMultiplier = 1;
        }
        else {
            distanceFromOrientation = RobotRotation;
            inversingPowerRotationMultiplier = -1;
        }
        telemetry.addData("DistanceFromOrientation", distanceFromOrientation);
    }

//    public double DeadModifier(double joystickValue) {
//        forwardButton = DeadModifier(forwardButton);
//        sidewaysButton = DeadModifier(sidewaysButton);
//        spinningButton = DeadModifier(spinningButton);
//
//        Drive(forwardButton, sidewaysButton, spinningButton);
//    }

    // By Paul

    public double  DeadModifier(double joystickValue) {
        if(joystickValue < DEADZONE && joystickValue > -DEADZONE)
            return 0;
        else {
            return joystickValue;
        }
    }

    public void Drive(double DZForwardButton, double DZSidewaysButton, double DZSpinningButton) {
        DZSpinningButton = -DZSpinningButton;
        FrontRight.setPower(-DZSidewaysButton - DZForwardButton + DZSpinningButton);
        FrontLeft.setPower(-DZSidewaysButton + DZForwardButton + DZSpinningButton);
        BackRight.setPower(DZSidewaysButton - DZForwardButton + DZSpinningButton);
        BackLeft.setPower(DZSidewaysButton + DZForwardButton + DZSpinningButton);
    }

    //
    // Odometry
    //

    public void initializeOdometry() {
        verticalLeft = hardwareMap.dcMotor.get(verticalLeftEncoderName);
        verticalRight = hardwareMap.dcMotor.get(verticalRightEncoderName);
        horizontal = hardwareMap.dcMotor.get(horizontalEncoderName);

        verticalRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        horizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        verticalLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        verticalRight.setDirection(DcMotorSimple.Direction.REVERSE);
        horizontal.setDirection(DcMotorSimple.Direction.REVERSE);

        verticalRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        verticalLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        horizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //StartingXPosition = Double.parseDouble(ReadWriteFile.readFile(startingXpositionFile).trim());
        //StartingYPosition = Double.parseDouble(ReadWriteFile.readFile(startingYpositionFile).trim());
        //StartingRotation = Double.parseDouble(ReadWriteFile.readFile(startingθpositionFile).trim());
        StartingXPosition = 0;
        StartingYPosition = 0;
        StartingRotation = 0;
    }

    public void startOdometry() {
        globalPositionUpdate = new OdometryGlobalCoordinatePosition(verticalLeft, verticalRight, horizontal, COUNTS_PER_INCH, 75);
        positionThread = new Thread(globalPositionUpdate);
        positionThread.start();

        globalPositionUpdate.reverseRightEncoder();
    }

    public void checkOdometry() {
        RobotXPosition = (globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH) + StartingXPosition;
        RobotYPosition = (globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH) + StartingYPosition;
        RobotRotation = (globalPositionUpdate.returnOrientation()) + StartingRotation;

        if (gamepad2.b) {
            if (firstPressb2) {
                firstPressb2 = false;
                StartingXPosition = -(globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
                StartingYPosition = -(globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
                StartingRotation = -(globalPositionUpdate.returnOrientation());
            }
        }
        else {
            firstPressb2 = true;
        }

        if (RobotRotation < 0){
            RobotRotation += 360;
        }

        telemetry.addData("X", RobotXPosition);
        telemetry.addData("Y", RobotYPosition);
        telemetry.addData("θ", RobotRotation);

        //Display Global (x, y, theta) coordinates
        telemetry.addData("X Position", globalPositionUpdate.returnXCoordinate() / COUNTS_PER_INCH);
        telemetry.addData("Y Position", globalPositionUpdate.returnYCoordinate() / COUNTS_PER_INCH);
        telemetry.addData("θ (Degrees)", globalPositionUpdate.returnOrientation());

        telemetry.addData("Vertical Left Encoder", verticalLeft.getCurrentPosition());
        telemetry.addData("Vertical Right Encoder", verticalRight.getCurrentPosition());
        telemetry.addData("Horizontal Encoder", horizontal.getCurrentPosition());

        telemetry.addData("Thread Active", positionThread.isAlive());
    }

    double getDistanceFromCoordinates(double x, double y, OdometryGlobalCoordinatePosition position) {
        double deltaX = x - position.returnXCoordinate();
        double deltaY = y - position.returnYCoordinate();

        return Math.sqrt(deltaX * deltaX + deltaY * deltaY);
    }

    //
    // INTAKE
    //

    public void initializeIntakeMechanism() {
        IntakeMotor = hardwareMap.dcMotor.get("IntakeMotor");
        IntakeAssistServo = hardwareMap.crservo.get("IntakeAssistServo");
        IntakeReleaseServo = hardwareMap.servo.get("IntakeReleaseServo");
        IntakeMotor.setPower(0);
        IntakeAssistServo.setPower(0);
    }
    public void startIntakeMechanism() {
        IntakeReleaseServo.setPosition(.6);
    }

    public void checkIntakeMechanism() {
        intakeButton = gamepad1.dpad_up;
        outputButton = gamepad1.dpad_down;

        intake(intakeButton);

        if (gamepad2.x) {
            if (firstPressx2) {
                if (intakeReleaseState == 1) {
                    intakeReleaseState = 0;
                }
                else {
                    intakeReleaseState = 1;
                }
                firstPressx2 = false;
            }
        }
        else {
            firstPressx2 = true;
        }

        if (intakeReleaseState == 0) {
            IntakeReleaseServo.setPosition(.15);
        }
        else {
            IntakeReleaseServo.setPosition(.6);
        }


    }

    public void intake(boolean intakeButton) {
        if (intakeButton) {
            if (firstPressDpadUp) {
                if (intakeState == 1) {
                    intakeState = 0;
                }
                else {
                    intakeState = 1;
                }
                firstPressDpadUp = false;
            }
        }
        else {
            firstPressDpadUp = true;
        }
        if (outputButton) {
            IntakeMotor.setPower(-1);
            IntakeAssistServo.setPower(1);
        }
        else {
            if (intakeState == 1) {
                IntakeMotor.setPower(1);
                IntakeAssistServo.setPower(-1);
            }
            else if (intakeState == 0) {
                IntakeMotor.setPower(0);
                IntakeAssistServo.setPower(0);
            }
        }
    }

    //
    // Straightener
    //

    public void initializeStraightener() {
        OrientationSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor .class, "OrientationSensor");
        StonePresenceSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor .class, "StonePresenceSensor");
        OrientationServoLeft = hardwareMap.get(Servo.class, "OrientationServoLeft");
        OrientationServoRight = hardwareMap.get(Servo.class, "OrientationServoRight");
        OrientationServoLeft.setPosition(lDisengage);
        OrientationServoRight.setPosition(rDisengage);
    }

    public void checkStraightener() {
        stoneDistance = StonePresenceSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("Stone Distance", stoneDistance);
        if (stoneDistance < 1.5) {
            stoneFullyInStraightener = true;
        }
        else {
            stoneFullyInStraightener = false;
        }

        orientStone();
        manualOverride();
    }

    private void senseOrientation() {
        orientDistance = OrientationSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("Orient Distance", orientDistance);

        // StoneOrientation is assigned relative to the side of the robot that the studs of the stone
        // are on when looking at the robot from the back.
        // this is true for the servos as well; left and right are assigned relative to the back

        if(!straightenerBusy) {
            if (orientDistance > .25 && orientDistance <= .55) {
                stoneOrientation = "left";
            } else if (orientDistance > .75 && orientDistance <= 2.3) {
                stoneOrientation = "center";
            } else if (orientDistance > 4) {
                stoneOrientation = "empty";
            } else {
                stoneOrientation = "right";
            }
            telemetry.addData("orientPosition:  ", stoneOrientation);
        }
    }

    private void orientStone() {
        senseOrientation();

        if (stoneFullyInStraightener) {
            if (stoneOrientation.equals("right")) {
                runLeftServo();
            }

            if (stoneOrientation.equals("left")) {
                runRightServo();
            }

            if (stoneOrientation.equals("center")) {
                readyToGrab = true;
            }
            else {
                readyToGrab = false;
            }
        }
    }

    private void manualOverride() {
        if (gamepad2.dpad_right) {
            runLeftServo();
        }
        if (gamepad2.dpad_left) {
            runRightServo();
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
        telemetry.addData("actualRightTime", actualRightTime);
        telemetry.addData("target time", targetTime);

        if(actualRightTime > targetTime) {
            OrientationServoRight.setPosition(rDisengage);
            telemetry.addData("is ready for return", "yes");
            firstRightRun = true;
        }
        if (actualRightTime > targetTime + .5) {
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
        telemetry.addData("actualLeftTime", actualLeftTime);
        telemetry.addData("target time", targetTime);

        if(actualLeftTime > targetTime) {
            OrientationServoLeft.setPosition(lDisengage);
            telemetry.addData("is ready for return", "yes");
            firstLeftRun = true;
        }

        if (actualLeftTime > targetTime + .5) {
            firstLeftRun = true;
            straightenerBusy = false;
        }
        else {
            straightenerBusy = true;
        }
    }
}