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
        //Gripper closed state
        gripper.setGripperClosed();
        //Gripper holder to the side
        gripper.setGripperHolderParallel();
        gripper.setAnglerSide();

        // Initialize telemetry
        //telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Define starting position
        Pose2d startPos = new Pose2d(8, 87, Math.toRadians(0));
        drive.setPoseEstimate(startPos);

        Pose2d SpecimenDropoffPos = new Pose2d(33, 79, Math.toRadians(0));
        Pose2d SampleDropoffPos1 = new Pose2d(19, 120, Math.toRadians(135));
        Pose2d SampleDropoffPos2 = new Pose2d(16, 126, Math.toRadians(135));
        Pose2d PushPos1 = new Pose2d(56, 100, Math.toRadians(0));
        Pose2d PushPos2 = new Pose2d(56, 120, Math.toRadians(0));
        Pose2d SamplePickUpPos1 = new Pose2d(29.3, 120, Math.toRadians(0));

        // Define the trajectory sequence
        TrajectorySequence StageRedBucket = drive.trajectorySequenceBuilder(startPos)

                // Step 1: Set
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setDesArmPosDeg(74);})
                .UNSTABLE_addTemporalMarkerOffset(0.1,()->{gripper.setAnglerForward();})
                .UNSTABLE_addTemporalMarkerOffset(0.2,()->{gripper.setGripperHolderPerpendicular();})


                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> {sliderControl.setDesSliderLen(1);})
                .waitSeconds(0.5)
                .lineToLinearHeading(SpecimenDropoffPos)


                //.waitSeconds(1)
                .forward(5.8)
                // Step 5: Set Arm to the Deposit angle

                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setDesArmPosDeg(40);})
                // Step 6: Slider motor to the Intake length
                //.UNSTABLE_addTemporalMarkerOffset(0.5, () -> {sliderControl.setDesSliderLen(4);})
                .waitSeconds(0.2)
                .back(7.5)
                //.waitSeconds(1)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> gripper.setGripperOpen())
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> armControl.setDesArmPosDeg(-20))

                .waitSeconds(0.6)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> armControl.setArmPower(0))

                // goes to sample 1
                .lineToLinearHeading(SamplePickUpPos1)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> sliderControl.setDesSliderLen(6))
                .UNSTABLE_addTemporalMarkerOffset(1.5, () -> gripper.setGripperClosed())
                .waitSeconds(1.8)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> sliderControl.setDesSliderLen(0))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> armControl.setArmDeposit())
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> sliderControl.setSliderDeposit())
                .waitSeconds(1)
                .turn(Math.toRadians(135))
                .waitSeconds(1)


                // Go to PushPos2 for the second sample
                .lineToLinearHeading(SampleDropoffPos1)
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> gripper.setGripperHolderParallel())
                .UNSTABLE_addTemporalMarkerOffset(0.5, () -> armControl.setDesArmPosDeg(74))
                .waitSeconds(0.8)
                .forward(2.7)
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> gripper.setGripperOpen())
                .UNSTABLE_addTemporalMarkerOffset(0.8, () -> armControl.setDesArmPosDeg(80))
                .UNSTABLE_addTemporalMarkerOffset(1.0, () -> gripper.setGripperClosed())
                .UNSTABLE_addTemporalMarkerOffset(1.2, () -> {sliderControl.setDesSliderLen(8);})

                // Go Back to the position PushPos1 for level 1 ascent
                .lineToLinearHeading(PushPos1)
                .turn(-Math.toRadians(90))

                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setDesArmPosDeg(28);})
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.0, () -> {armControl.setArmPower(-0.4);})
                .waitSeconds(2)



                //final build
                .build();

        // Wait for start signal
        waitForStart();


        // Execute the trajectory sequence
        drive.followTrajectorySequence(StageRedBucket);



    }
}