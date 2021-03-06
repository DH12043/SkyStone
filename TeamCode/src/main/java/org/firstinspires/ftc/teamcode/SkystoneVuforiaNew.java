package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.YZX;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection.BACK;

@TeleOp(name = "New Test Vuforia")
public abstract class SkystoneVuforiaNew extends OpMode {

    private TouchSensor AllianceSwitch;

    // IMPORTANT:  For Phone Camera, set 1) the camera source and 2) the orientation, based on how your phone is mounted:
    // 1) Camera Source.  Valid choices are:  BACK (behind screen) or FRONT (selfie side)
    // 2) Phone Orientation. Choices are: PHONE_IS_PORTRAIT = true (portrait) or PHONE_IS_PORTRAIT = false (landscape)

    private static final VuforiaLocalizer.CameraDirection CAMERA_CHOICE = BACK;
    private static final boolean PHONE_IS_PORTRAIT = false;

    private static final String VUFORIA_KEY = "ARFWHLf/////AAABmQYBi+Yt9UfXhQeC8EeUpylMd+Pha6aQHw+i5Pyw28Fs7CaBACsFzXFNGv7p2jfwf9sn5zAO1CNrRa6XilwVANqg6g+mkwciKF38WPZGG6j88PDkkTkH6Sq6RM/VeeYCf+BikiEWGjM/BS5u8FlfvYERSQ9En9Hn8ootiBHXeNZrGl4BZwIfpt0LachcG2DAadZSZbtGV6evUlpC++Sx6JvuERscDOFVo1YdV2MHovW82LVRgp8Xctfsdr5euTXVkCT7d2C9I1X7D+y4mjjZSd4N6VMkuQYAXTsIU+RSW+OeSXtRRkFdZ7O5fCM6bUNgGUdyT7vSRUWh3A3qVTGLNT+exhPciVkD9yW/xW5yMvMk";

    // Since ImageTarget trackables use mm to specifiy their dimensions, we must use mm for all the physical dimension.
    // We will define some constants and conversions here
    private static final float mmPerInch = 25.4f;
    private static final float mmTargetHeight = (6) * mmPerInch;          // the height of the center of the target image above the floor

    // Constant for Stone Target
    private static final float stoneZ = 2.00f * mmPerInch;

    // Constants for the center support targets
    private static final float bridgeZ = 6.42f * mmPerInch;
    private static final float bridgeY = 23 * mmPerInch;
    private static final float bridgeX = 5.18f * mmPerInch;
    private static final float bridgeRotY = 59;                                 // Units are degrees
    private static final float bridgeRotZ = 180;

    // Constants for perimeter targets
    private static final float halfField = 72 * mmPerInch;
    private static final float quadField = 36 * mmPerInch;

    // Class Members
    private OpenGLMatrix lastLocation = null;
    private VuforiaLocalizer vuforia = null;
    private boolean targetVisible = false;
    private float phoneXRotate = 0;
    private float phoneYRotate = 0;
    private float phoneZRotate = 0;

    protected double StartingXPosition;
    protected double StartingYPosition;
    protected double StartingRotation;
    protected String positionSkystone = "";
    private VuforiaTrackables targetsSkyStone;
    private List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();

    private int cameraMonitorViewId;
    private float xPosition;
    private float yPosition;
    private float zPosition;
    OpenGLMatrix robotFromCamera;
    VuforiaLocalizer.Parameters parameters;

    private NormalizedColorSensor VuforiaSensor1;
    private NormalizedColorSensor VuforiaSensor0;

    public float redVuforiaValues1 = 0;
    public float blueVuforiaValues1 = 0;
    public float greenVuforiaValues1 = 0;
    public float alphaVuforiaValues1 = 0;

    public float redVuforiaValues0 = 0;
    public float blueVuforiaValues0 = 0;
    public float greenVuforiaValues0 = 0;
    public float alphaVuforiaValues0 = 0;

    float hsvValues1[] = {0F, 0F, 0F};

    final float values1[] = hsvValues1;

    float hsvValues0[] = {0F, 0F, 0F};

    final float values0[] = hsvValues0;

    final double SCALE_FACTOR = 255;

    public SkystoneVuforiaNew() {
        super();

        msStuckDetectInitLoop = 20000;
        msStuckDetectInit = 20000;
    }

    int numberOfTimesUpdated = 0;

    private String robotAlliance = "null";

    @Override
    public void init() {
        AllianceSwitch = hardwareMap.touchSensor.get("AllianceSwitch");

        cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CAMERA_CHOICE;
        parameters.useExtendedTracking = true;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        targetsSkyStone = this.vuforia.loadTrackablesFromAsset("Skystone");

        VuforiaTrackable stoneTarget = targetsSkyStone.get(0);
        stoneTarget.setName("Stone Target");
        VuforiaTrackable blueRearBridge = targetsSkyStone.get(1);
        blueRearBridge.setName("Blue Rear Bridge");
        VuforiaTrackable redRearBridge = targetsSkyStone.get(2);
        redRearBridge.setName("Red Rear Bridge");
        VuforiaTrackable redFrontBridge = targetsSkyStone.get(3);
        redFrontBridge.setName("Red Front Bridge");
        VuforiaTrackable blueFrontBridge = targetsSkyStone.get(4);
        blueFrontBridge.setName("Blue Front Bridge");
        VuforiaTrackable red1 = targetsSkyStone.get(5);
        red1.setName("Red Perimeter 1");
        VuforiaTrackable red2 = targetsSkyStone.get(6);
        red2.setName("Red Perimeter 2");
        VuforiaTrackable front1 = targetsSkyStone.get(7);
        front1.setName("Front Perimeter 1");
        VuforiaTrackable front2 = targetsSkyStone.get(8);
        front2.setName("Front Perimeter 2");
        VuforiaTrackable blue1 = targetsSkyStone.get(9);
        blue1.setName("Blue Perimeter 1");
        VuforiaTrackable blue2 = targetsSkyStone.get(10);
        blue2.setName("Blue Perimeter 2");
        VuforiaTrackable rear1 = targetsSkyStone.get(11);
        rear1.setName("Rear Perimeter 1");
        VuforiaTrackable rear2 = targetsSkyStone.get(12);
        rear2.setName("Rear Perimeter 2");

        allTrackables.addAll(targetsSkyStone);

        /**
         * In order for localization to work, we need to tell the system where each target is on the field, and
         * where the phone resides on the robot.  These specifications are in the form of <em>transformation matrices.</em>
         * Transformation matrices are a central, important concept in the math here involved in localization.
         * See <a href="https://en.wikipedia.org/wiki/Transformation_matrix">Transformation Matrix</a>
         * for detailed information. Commonly, you'll encounter transformation matrices as instances
         * of the {@link OpenGLMatrix} class.
         *
         * If you are standing in the Red Alliance Station looking towards the center of the field,
         *     - The X axis runs from your left to the right. (positive from the center to the right)
         *     - The Y axis runs from the Red Alliance Station towards the other side of the field
         *       where the Blue Alliance Station is. (Positive is from the center, towards the BlueAlliance station)
         *     - The Z axis runs from the floor, upwards towards the ceiling.  (Positive is above the floor)
         *
         * Before being transformed, each target image is conceptually located at the origin of the field's
         *  coordinate system (the center of the field), facing up.
         */

        // Set the position of the Stone Target.  Since it's not fixed in position, assume it's at the field origin.
        // Rotated it to to face forward, and raised it to sit on the ground correctly.
        // This can be used for generic target-centric approach algorithms
        stoneTarget.setLocation(OpenGLMatrix
                .translation(0, 0, stoneZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //Set the position of the bridge support targets with relation to origin (center of field)
        blueFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, bridgeRotZ)));

        blueRearBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, bridgeRotZ)));

        redFrontBridge.setLocation(OpenGLMatrix
                .translation(-bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, -bridgeRotY, 0)));

        redRearBridge.setLocation(OpenGLMatrix
                .translation(bridgeX, -bridgeY, bridgeZ)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 0, bridgeRotY, 0)));

        //Set the position of the perimeter targets with relation to origin (center of field)
        red1.setLocation(OpenGLMatrix
                .translation(quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        red2.setLocation(OpenGLMatrix
                .translation(-quadField, -halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 180)));

        front1.setLocation(OpenGLMatrix
                .translation(-halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        front2.setLocation(OpenGLMatrix
                .translation(-halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 90)));

        blue1.setLocation(OpenGLMatrix
                .translation(-quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        blue2.setLocation(OpenGLMatrix
                .translation(quadField, halfField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, 0)));

        rear1.setLocation(OpenGLMatrix
                .translation(halfField, quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        rear2.setLocation(OpenGLMatrix
                .translation(halfField, -quadField, mmTargetHeight)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, XYZ, DEGREES, 90, 0, -90)));

        //
        // Create a transformation matrix describing where the phone is on the robot.
        //
        // NOTE !!!!  It's very important that you turn OFF your phone's Auto-Screen-Rotation option.
        // Lock it into Portrait for these numbers to work.
        //
        // Info:  The coordinate frame for the robot looks the same as the field.
        // The robot's "forward" direction is facing out along X axis, with the LEFT side facing out along the Y axis.
        // Z is UP on the robot.  This equates to a bearing angle of Zero degrees.
        //
        // The phone starts out lying flat, with the screen facing Up and with the physical top of the phone
        // pointing to the LEFT side of the Robot.
        // The two examples below assume that the camera is facing forward out the front of the robot.

        // We need to rotate the camera around it's long axis to bring the correct camera forward.
        if (CAMERA_CHOICE == BACK) {
            phoneYRotate = -90;
        } else {
            phoneYRotate = 90;
        }

        // Rotate the phone vertical about the X axis if it's in portrait mode
        if (PHONE_IS_PORTRAIT) {
            phoneXRotate = 90;
        }

        // Next, translate the camera lens to where it is on the robot.
        // In this example, it is centered (left to right), but forward of the middle of the robot, and above ground level.
        final float CAMERA_FORWARD_DISPLACEMENT = 9.0f * mmPerInch;   // eg: Camera is 9 Inches in front of robot center
        final float CAMERA_VERTICAL_DISPLACEMENT = 2.0f * mmPerInch;   // eg: Camera is 2 Inches above ground
        final float CAMERA_LEFT_DISPLACEMENT = 0;     // eg: Camera is ON the robot's center line

        robotFromCamera = OpenGLMatrix
                .translation(CAMERA_FORWARD_DISPLACEMENT, CAMERA_LEFT_DISPLACEMENT, CAMERA_VERTICAL_DISPLACEMENT)
                .multiplied(Orientation.getRotationMatrix(EXTRINSIC, YZX, DEGREES, phoneYRotate, phoneZRotate, phoneXRotate));

        com.vuforia.CameraDevice.getInstance().setFlashTorchMode(true);

        com.vuforia.CameraDevice.getInstance().setField("opti-zoom", "opti-zoom-on");
        com.vuforia.CameraDevice.getInstance().setField("zoom", "30");

        VuforiaSensor1 =  hardwareMap.get(NormalizedColorSensor.class, "VuforiaSensor1");
        VuforiaSensor0 =  hardwareMap.get(NormalizedColorSensor.class, "VuforiaSensor0");

    }

    @Override
    public void init_loop() {
        NormalizedRGBA VuforiaValues0 = VuforiaSensor0.getNormalizedColors();
        NormalizedRGBA VuforiaValues1 = VuforiaSensor1.getNormalizedColors();

        redVuforiaValues0 = VuforiaValues0.red;
        blueVuforiaValues0 = VuforiaValues0.blue;
        greenVuforiaValues0 = VuforiaValues0.green;
        alphaVuforiaValues0 = VuforiaValues0.alpha;

        Color.RGBToHSV((int) (redVuforiaValues0 * SCALE_FACTOR),
                (int) (greenVuforiaValues0 * SCALE_FACTOR),
                (int) (blueVuforiaValues0 * SCALE_FACTOR),
                hsvValues0);

        telemetry.addData("red0", redVuforiaValues0);
        telemetry.addData("green0", greenVuforiaValues0);
        telemetry.addData("blue0", blueVuforiaValues0);
        telemetry.addData("alpha0", alphaVuforiaValues0);
        telemetry.addData("Hue0", hsvValues0[0]);
        telemetry.addData("Saturation0", hsvValues0[1]);
        telemetry.addData("Value0", hsvValues0[2]);


        redVuforiaValues1 = VuforiaValues1.red;
        blueVuforiaValues1 = VuforiaValues1.blue;
        greenVuforiaValues1 = VuforiaValues1.green;
        alphaVuforiaValues1 = VuforiaValues1.alpha;

        Color.RGBToHSV((int) (redVuforiaValues1 * SCALE_FACTOR),
                (int) (greenVuforiaValues1 * SCALE_FACTOR),
                (int) (blueVuforiaValues1 * SCALE_FACTOR),
                hsvValues1);

        telemetry.addData("red1", redVuforiaValues1);
        telemetry.addData("green1", greenVuforiaValues1);
        telemetry.addData("blue1", blueVuforiaValues1);
        telemetry.addData("alpha1", alphaVuforiaValues1);
        telemetry.addData("Hue1", hsvValues1[0]);
        telemetry.addData("Saturation1", hsvValues1[1]);
        telemetry.addData("Value1", hsvValues1[2]);

        VuforiaTrackable stoneTrackable = targetsSkyStone.get(0);
        VuforiaTrackableDefaultListener listener = (VuforiaTrackableDefaultListener) stoneTrackable.getListener();
        listener.setPhoneInformation(robotFromCamera, parameters.cameraDirection);


        targetsSkyStone.activate();
        targetVisible = false;

        if (listener.isVisible()) {
            telemetry.addData("Visible Target", stoneTrackable.getName());
            targetVisible = true;

            OpenGLMatrix robotLocationTransform = listener.getUpdatedRobotLocation();
            if (robotLocationTransform != null) {
                lastLocation = robotLocationTransform;
                telemetry.addData("# of times changed: %d", ++numberOfTimesUpdated);
            }
        }

        // Provide feedback as to where the robot is located (if we know).
        if (targetVisible && lastLocation != null) {
            // express position (translation) of robot in inches.
            VectorF translation = lastLocation.getTranslation();
            telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                    translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

            xPosition = translation.get(0) / mmPerInch;
            yPosition = translation.get(1) / mmPerInch;
            zPosition = translation.get(2) / mmPerInch;

//            if (yPosition >= 1.5) {
//                positionSkystone = "Right";
//            } else if (yPosition <= 1.5 && yPosition >= -2) {
//                positionSkystone = "Center";
//            } else if (yPosition <= -2) {
//                positionSkystone = "Left";

                if (AllianceSwitch.isPressed()) {
                    telemetry.addData("Switch Position", "Red");
                    robotAlliance = "Red";
                } else {
                    telemetry.addData("Switch Position", "Blue");
                    robotAlliance = "Blue";
                }

            if (hsvValues1[0] == 0 && (hsvValues0[0] == 60 || hsvValues0[0] == 120)) {
                //sensor 1 is greater than skystone threshold and sensor one is greater than sensor 0
                // = left
                positionSkystone = "Left";
            }
            else if (hsvValues0[0] == 0 && (hsvValues1[0] == 60 || hsvValues1[0] == 120)) {
                //sensor 0 is greater than skystone threshold and sensor one is greater than sensor 1
                // = center
                positionSkystone = "Center";
            }
            else {
                // neither sensor is greater than skyStone sensor or both are equal, in which case it will default to third stone
                // = right
                positionSkystone = "Right";
            }

            // express the rotation of the robot in degrees.
            Orientation rotation = Orientation.getOrientation(lastLocation, EXTRINSIC, XYZ, DEGREES);
            telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);
        } else {
            telemetry.addData("Visible Target", "none");
        }

        lastLocation = null;
        telemetry.addData("Skystone Position", positionSkystone);
        telemetry.addData("Version", "1.5");
        telemetry.update();
    }

    @Override
    public void loop() {
        telemetry.addData("Skystone Position", positionSkystone);
    }
}
