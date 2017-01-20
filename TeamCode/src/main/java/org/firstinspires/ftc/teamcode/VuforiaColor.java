package org.firstinspires.ftc.teamcode;

import android.graphics.Bitmap;

import org.firstinspires.ftc.robotcontroller.internal.PIController;
import org.firstinspires.ftc.robotcontroller.internal.RobotMovement;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.MatrixF;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.Matrix34F;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Tool;
import com.vuforia.Vec2F;
import com.vuforia.Vec3F;
import com.vuforia.Vuforia;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.firstinspires.ftc.teamcode.VuforiaLocation.TAG;

/**
 * Created by Winston on 10/8/2016.
 */
@Autonomous(name="Vuforia Color", group="Vuforia")
public class VuforiaColor extends LinearOpMode {

    private VuforiaLocalizer vuforia;
    private ColorBlobDetector colorDetector;
    private OpenGLMatrix lastLocation = null;

    private DcMotor frontRightMotor;
    private DcMotor backRightMotor;
    private DcMotor frontLeftMotor;
    private DcMotor backLeftMotor;
    private DcMotor rollerMotor;

    private PIController piController;
    private boolean hitRed;
    private String directionToHit;
    private boolean isButtonHit;
    private int directionFoundInARow;

    public void runOpMode() throws InterruptedException
    {
//        frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
//        backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
//        frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
//        backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
//        rollerMotor = hardwareMap.dcMotor.get("rollerMotor");
//
//        backRightMotor.setDirection(DcMotor.Direction.REVERSE);
//        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);

        colorDetector = new ColorBlobDetector();

        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "ATjJBiD/////AAAAGc0JoedLjk5flVb2gExO3UVJCpOq5U4cyH9czcMyX5C8h+1AWXo7A0CU24r/IVeoC+7Te9zwJkX6IjHv5c77UNqrsyerM7pbjywj6/2NlzSUwb3jtEd9APhY5cOoSibb5NDRFM9beUWt0k4HuFMaw5OIZRs5YWge7KaJt5SzhqEFMQ6Loo8eugB9BBbPfuV3d7u4sQZBAKeRsR9mmnfvFJTUHHgcPlALU/rJBgw40AeFFvChjzNhwtlWYymeM/0173jH7JB2dyhoNtn/9byIUQzMw8KtaXbD3IfFJySLgJWmYjaA7cKdboL0nvkOoZNFMm2yqenbUDe/CEIMkhAsKjS3sgX4t6Fq+8gkhSnOS/Vd";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB565, true); //enables RGB565 format for the image
        vuforia.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time
        piController = new PIController(.0016, 0.00013, 0.00023, 0.000012);

        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 1);
        VuforiaTrackables visionTargets = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        VuforiaTrackable wheelsTarget = visionTargets.get(0);
        wheelsTarget.setName("Wheels");  // Wheels

        VuforiaTrackable toolsTarget  = visionTargets.get(1);
        toolsTarget.setName("Tools");  // Tools

        VuforiaTrackable legosTarget = visionTargets.get(2);
        legosTarget.setName("Legos");  // Legos

        VuforiaTrackable gearsTarget  = visionTargets.get(3);
        gearsTarget.setName("Gears");  // Gears

        /** For convenience, gather together all the trackable objects in one easily-iterable collection */
        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(visionTargets);

        /**
         * We use units of mm here because that's the recommended units of measurement for the
         * size values specified in the XML for the ImageTarget trackables in data sets. E.g.:
         *      <ImageTarget name="stones" size="247 173"/>
         * You don't *have to* use mm here, but the units here and the units used in the XML
         * target configuration files *must* correspond for the math to work out correctly.
         */
        float mmPerInch        = 25.4f;
        float mmBotLength      = 16 * mmPerInch;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels
        float mmVisionTargetZOffset = 5.75f * mmPerInch;
        float mmPhoneZOffset = 5.5f * mmPerInch;

        OpenGLMatrix gearsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/2, -mmFTCFieldWidth/12, mmVisionTargetZOffset)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        gearsTarget.setLocation(gearsTargetLocationOnField);
        RobotLog.ii(TAG, "Gears Target=%s", format(gearsTargetLocationOnField));

        OpenGLMatrix toolsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the RED WALL. Our translation here
                is a negative translation in X.*/
                .translation(-mmFTCFieldWidth/2, mmFTCFieldWidth/4, mmVisionTargetZOffset)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X, then 90 in Z */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 90, 0));
        toolsTarget.setLocation(toolsTargetLocationOnField);
        RobotLog.ii(TAG, "Tools Target=%s", format(toolsTargetLocationOnField));

       /*
        * To place the Wheels and Legos Targets on the Blue Audience wall:
        * - First we rotate it 90 around the field's X axis to flip it upright
        * - Finally, we translate it along the Y axis towards the blue audience wall.
        */
        OpenGLMatrix wheelsTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(mmFTCFieldWidth/12, mmFTCFieldWidth/2, mmVisionTargetZOffset)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        wheelsTarget.setLocation(wheelsTargetLocationOnField);
        RobotLog.ii(TAG, "Wheels Target=%s", format(wheelsTargetLocationOnField));

        OpenGLMatrix legosTargetLocationOnField = OpenGLMatrix
                /* Then we translate the target off to the Blue Audience wall.
                Our translation here is a positive translation in Y.*/
                .translation(-mmFTCFieldWidth/4, mmFTCFieldWidth/2, mmVisionTargetZOffset)
                .multiplied(Orientation.getRotationMatrix(
                        /* First, in the fixed (field) coordinate system, we rotate 90deg in X */
                        AxesReference.EXTRINSIC, AxesOrder.XZX,
                        AngleUnit.DEGREES, 90, 0, 0));
        legosTarget.setLocation(legosTargetLocationOnField);
        RobotLog.ii(TAG, "Legos Target=%s", format(legosTargetLocationOnField));

        /**
         * Create a transformation matrix describing where the phone is on the robot. Here, we
         * put the phone on the right hand side of the robot with the screen facing in (see our
         * choice of BACK camera above) and in landscape mode. Starting from alignment between the
         * robot's and phone's axes, this is a rotation of -90deg along the Y axis.
         *
         * When determining whether a rotation is positive or negative, consider yourself as looking
         * down the (positive) axis of rotation from the positive towards the origin. Positive rotations
         * are then CCW, and negative rotations CW. An example: consider looking down the positive Z
         * axis towards the origin. A positive rotation about Z (ie: a rotation parallel to the the X-Y
         * plane) is then CCW, as one would normally expect from the usual classic 2D geometry.
         */
        OpenGLMatrix phoneLocationOnRobot = OpenGLMatrix
                .translation(mmBotWidth/2,0,mmPhoneZOffset)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, 0, 180, 0));
        RobotLog.ii(TAG, "phone=%s", format(phoneLocationOnRobot));

        ((VuforiaTrackableDefaultListener)visionTargets.get(0).getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)visionTargets.get(1).getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)visionTargets.get(2).getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)visionTargets.get(3).getListener()).setPhoneInformation(phoneLocationOnRobot, parameters.cameraDirection);

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start tracking");
        telemetry.addData("OpenCV", Core.NATIVE_LIBRARY_NAME);
        telemetry.update();
        waitForStart();

        /** Start tracking the data sets we care about. */
        visionTargets.activate();

        hitRed = true;
        isButtonHit = false;
        directionFoundInARow = 0;
        directionToHit = "";

        telemetry.addData("Loop", "Out");
        telemetry.update();
        while(opModeIsActive()) {
            String visibleTarget = "";
            Mat img = null;
            Mat croppedImg = null;
            Point beaconImageCenter = null;

            VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take();
            telemetry.update();
            if(frame != null) {
                Image rgb = null;
                long numImages = frame.getNumImages();

                for (int i = 0; i < numImages; i++) {
                    if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB565) {
                        rgb = frame.getImage(i);
                        break;
                    }//if
                }//for

                if(rgb != null) {
                    Bitmap bmp = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.RGB_565);
                    bmp.copyPixelsFromBuffer(rgb.getPixels());

                    img = new Mat();
                    Utils.bitmapToMat(bmp, img);
                    telemetry.addData("Img", "Converted");
                    telemetry.update();
                }
            }

            for(VuforiaTrackable beacon : allTrackables) {
                // Add beacon to telemetry if visible
                if(((VuforiaTrackableDefaultListener)beacon.getListener()).isVisible()) {
                    visibleTarget = beacon.getName();
                    telemetry.addData(visibleTarget, "Visible");
                }

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)beacon.getListener()).getUpdatedRobotLocation();
                if (robotLocationTransform != null) {
                    lastLocation = robotLocationTransform;
                }

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener)beacon.getListener()).getRawPose();

                if(pose != null) {
                    Matrix34F rawPose = new Matrix34F();
                    float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
                    rawPose.setData(poseData);

                    // Corners of beacon image in camera image
                    Vec2F upperLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127,92,0));
                    Vec2F upperRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(127,92,0));
                    Vec2F lowerLeft = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(-127,-92,0));
                    Vec2F lowerRight = Tool.projectPoint(vuforia.getCameraCalibration(), rawPose, new Vec3F(127,-92,0));

                    VectorF translation = pose.getTranslation();
                    /** First argument is get(1) if phone is vertical
                     First argument is get(0) if phone is horizontal */

                    if(img != null && !isButtonHit) {
                        telemetry.addData(beacon.getName() + "-Translation", translation);

                        // Vectors are stored (y,x).  Coordinate system starts in top right
                        int height = (int)(upperLeft.getData()[0] - lowerLeft.getData()[0]);
                        int width = (int)(upperRight.getData()[1] - upperLeft.getData()[1]);

                        int rowStart = (int)upperRight.getData()[0] - height < 0 ? 1 : (int)upperRight.getData()[0] - height;
                        int rowEnd = rowStart + height > img.rows() ? img.rows() - 1 : rowStart + height;
                        int colStart = (int)upperRight.getData()[1] < 0 ? 1 : (int)upperRight.getData()[1];
                        int colEnd = colStart + width > img.cols() ? img.cols() - 1 : colStart + width;

                        telemetry.addData("Target Location", "");
                        telemetry.addData("[" + upperLeft.getData()[0] + "," + upperLeft.getData()[1] + "]", "[" + upperRight.getData()[0] + "," + upperRight.getData()[1] + "]");
                        telemetry.addData("[" + lowerLeft.getData()[0] + "," + lowerLeft.getData()[1] + "]", "[" + lowerRight.getData()[0] + "," + lowerRight.getData()[1] + "]");

                        telemetry.addData(colStart + "", rowStart);
                        telemetry.addData(colEnd + "", rowEnd);
                        telemetry.addData(img.rows() + "", img.cols());
                        telemetry.update();

                        // Crop the image to look only at the beacon
                        // TODO Verify beacon is in cropped image
                        // NEED TO CHECK BEACON HEIGHT FOR INCLUSION IN CROPPED IMAGE
                        croppedImg = img.submat(rowStart, rowEnd, colStart, colEnd);
                    }
                }
            }

            // Process the rgb image
            if(croppedImg != null && !isButtonHit) {
                // Find the color of the beacon you need to hit
                if(hitRed) {
                    colorDetector.setHsvColor(new Scalar(230,75,255)); // Red detector, needs verification with beacon
                } else {
                    colorDetector.setHsvColor(new Scalar(130,150,255)); // Blue detector, needs verification with beacon
                }
                colorDetector.process(croppedImg);

                // Calculate the center of the blob detected
                Point beaconToHitCenter = null;
                List<Moments> blueMu = new ArrayList<>(colorDetector.getContours().size());
                for (int i = 0; i < colorDetector.getContours().size(); i++) {
                    blueMu.add(Imgproc.moments(colorDetector.getContours().get(i), false));
                    Moments p = blueMu.get(i);
                    int x = (int) (p.get_m10() / p.get_m00());
                    int y = (int) (p.get_m01() / p.get_m00());
                    beaconToHitCenter = new Point(x, y);
                }

                // Find the color of the beacon you are not hitting
                if(hitRed) {
                    colorDetector.setHsvColor(new Scalar(130,150,255)); // Blue detector, needs verification with beacon
                } else {
                    colorDetector.setHsvColor(new Scalar(230,75,255)); // Red detector, needs verification with beacon
                }
                colorDetector.process(croppedImg);

                // Calculate the center of the blob detected
                Point secondReferenceCenter = null;
                List<Moments> redMu = new ArrayList<>(colorDetector.getContours().size());
                for (int i = 0; i < colorDetector.getContours().size(); i++) {
                    redMu.add(Imgproc.moments(colorDetector.getContours().get(i), false));
                    Moments p = redMu.get(i);
                    int x = (int) (p.get_m10() / p.get_m00());
                    int y = (int) (p.get_m01() / p.get_m00());
                    secondReferenceCenter = new Point(x, y);
                }

                // Use the two centers of the blobs to determine which direction to hit
                if (beaconToHitCenter != null && secondReferenceCenter != null && !isButtonHit) {
                    // (!isButtonHit) Only hit the button once
                    // (!needToTurn) Do not hit the button if the robot is not straight centered
//                    hitBeaconButton(isLeft(center, beaconImageCenter));
                    if(isLeft(beaconToHitCenter, secondReferenceCenter)) {
                        if(!directionToHit.equals("Left")) {
                            directionFoundInARow = 0;
                        }
                        directionFoundInARow++;
                        directionToHit = "Left";
                    } else {
                        if(!directionToHit.equals("Right")) {
                            directionFoundInARow = 0;
                        }
                        directionFoundInARow++;
                        directionToHit = "Right";
                    }
                }

                // Find the color five times in a row before hitting it
                if(directionFoundInARow >= 3) {
                    isButtonHit = true;
                }
            }

            if(isButtonHit) {
                telemetry.addData("Hit Button-", directionToHit);
            }

//            if(needToTurn) {
//                turn(degreesToTurn);
//                telemetry.addData("Turn-", degreesToTurn);
//            }

            /**
             * Provide feedback as to where the robot was last located (if we know).
             */
            if (lastLocation != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                telemetry.addData("Pos", myFormat(lastLocation));

                if(!visibleTarget.equals("")) {
                    telemetry.addData("Move", piController.processLocation(lastLocation, visibleTarget));
                }
            } else {
                telemetry.addData("Pos", "Unknown");
            }

            telemetry.update();
            idle();
        }
    }

    /**
     * Returns if point 1 is to the left of point 2
     * Assume x coordinates increase from left to right
     * @param p1 a point
     * @param p2 a point
     * @return whether point 1 is to the left of point 2
     */
    private boolean isLeft(Point p1, Point p2)
    {
        return p1.x > p2.x;
    }

    /**
     * Hit the beacon button that is on the left
     * @param isLeft should the bot hit the left button
     */
    public void hitBeaconButton(boolean isLeft)
    {
        isButtonHit = true;

        // TODO Tweak loop length and motor power
        // Move forward
        for(int i = 0; i < 100; i++) {
            frontRightMotor.setPower(1);
            backRightMotor.setPower(1);
            frontLeftMotor.setPower(1);
            backLeftMotor.setPower(1);
        }

    }

    /**
     * A simple utility that extracts positioning information from a transformation matrix
     * and formats it in a form palatable to a human being.
     */
    private String format(OpenGLMatrix transformationMatrix) { return transformationMatrix.formatAsTransform(); }

    private String myFormat(OpenGLMatrix transformationMatrix)
    {
        VectorF translation = transformationMatrix.getTranslation();

        Orientation orientation = Orientation.getOrientation(transformationMatrix, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

        return String.format("%n  %s%n  %s", orientation.toString(), translation.toString());
    }
}