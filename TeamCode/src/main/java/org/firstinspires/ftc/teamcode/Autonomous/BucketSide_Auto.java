package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.drive.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.subsytems.ArmControl;
import org.firstinspires.ftc.teamcode.subsytems.Gripper;
import org.firstinspires.ftc.teamcode.subsytems.SliderControl;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous
public class BucketSide_Auto extends LinearOpMode {

    private ElapsedTime AutoTimer = new ElapsedTime();
    private ArmControl armControl;
    private Gripper gripper;
    private SliderControl sliderControl;
    private double speedFactor = 0.65;


    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize Mecanum Drive
        MecanumDriveBase drive = new MecanumDriveBase(hardwareMap);
        drive.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        // - - - Setting up Arm motors - - - //
        armControl = new ArmControl(this);
        armControl.init(hardwareMap);

        // - - - Setting up Slider motors - - - //
        sliderControl = new SliderControl(this);
        sliderControl.init(hardwareMap);

        // - - - Initialize gripper to starting position - - - //
        gripper = new Gripper(this);
        gripper.init(hardwareMap);
        gripper.gripperStopped();
        gripper.setAnglerDown();


        // Initialize telemetry
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Define starting position
        Pose2d startPos = new Pose2d(8, 87, Math.toRadians(0));
        drive.setPoseEstimate(startPos);

        Pose2d SpecimenDropoffPos = new Pose2d(33, 74, Math.toRadians(0));
        Pose2d SampleDropoffPos1 = new Pose2d(13, 125, Math.toRadians(-45));
        Pose2d SampleDropoffPos2 = new Pose2d(16, 126, Math.toRadians(-45));
        Pose2d PushPos1 = new Pose2d(56, 100, Math.toRadians(0));
        Pose2d PushPos2 = new Pose2d(56, 120, Math.toRadians(0));

        // Define the trajectory sequence
        TrajectorySequence StageRedBucket = drive.trajectorySequenceBuilder(startPos)

                // Step 1: Set angler down
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setDesArmPosDeg(60);})
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{gripper.setAnglerUP();})

                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> {sliderControl.setDesSliderLen(1);})
                .waitSeconds(0.1)
                .lineToLinearHeading(SpecimenDropoffPos)
                .UNSTABLE_addTemporalMarkerOffset(0.0,()->{gripper.setAnglerDown();})


                //.waitSeconds(1)
                .forward(7)
                // Step 5: Set Arm to the Deposit angle

                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setDesArmPosDeg(45);})
                // Step 6: Slider motor to the Intake length
                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> {sliderControl.setDesSliderLen(4);})
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> gripper.gripperForward(0.3))
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {gripper.gripperStopped();})
                .back(5)
                //.waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> gripper.gripperReverse(-0.3))
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {gripper.gripperStopped();})


                .back(10)
                .strafeLeft(30)
                .forward(31)
                // strafe left to prepare for pushing the sample to SampleDropoffPos1
                .strafeLeft(12)

                // push sample to SampleDropoffPos1
                .lineToLinearHeading(SampleDropoffPos1)

                /*
                // Go to PushPos2 for the second sample
                .lineToLinearHeading(PushPos2)
                .strafeLeft(5)

                // push sample to SampleDropoffPos2
                .lineToLinearHeading(SampleDropoffPos2)
                */

                // Go Back to the position PushPos1 for level 1 ascent
                .lineToLinearHeading(PushPos1)
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {sliderControl.setDesSliderLen(8);})
                .turn(Math.toRadians(-90))
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setDesArmPosDeg(28);})
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setArmPower(0);})


                .waitSeconds(0.5)


                //final build
                .build();

        // Wait for start signal
        waitForStart();


        // Execute the trajectory sequence
        drive.followTrajectorySequence(StageRedBucket);



    }
}