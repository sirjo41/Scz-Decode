package org.firstinspires.ftc.teamcode.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Autonomous(name = "Blue Test", group = "Testing", preselectTeleOp = "Drive")
public class blueTest extends OpMode {

    private Follower follower;
    private Timer pathTimer, opmodeTimer;
    private int pathState;

    // Poses - Extracted from User's Path Data
    private final Pose startPose = new Pose(21.234, 123.514, Math.toRadians(54)); // Start from Path1
    private final Pose shootPose = new Pose(42.000, 102.000, Math.toRadians(136));
    private final Pose intake1Pose = new Pose(42.000, 83.000, Math.toRadians(180));
    private final Pose feed1Pose = new Pose(23.000, 83.000, Math.toRadians(180));
    private final Pose intake2Pose = new Pose(42.000, 60.000, Math.toRadians(180));
    private final Pose feed2Pose = new Pose(23.000, 60.000, Math.toRadians(180));

    // Paths
    private PathChain toShoot1, intake1, feed1, toShoot2, intake2, feed2, toShoot3;

    // PatternDefault

    public void buildPaths() {
        // Path1: Start -> Shoot 1
        toShoot1 = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(54), Math.toRadians(136))
                .build();

        // Path2: Shoot 1 -> Intake 1
        intake1 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake1Pose))
                .setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(180))
                .build();

        // Path3: Intake 1 -> Feed 1
        feed1 = follower.pathBuilder()
                .addPath(new BezierLine(intake1Pose, feed1Pose))
                .setTangentHeadingInterpolation()
                .build();

        // Path4: Feed 1 -> Shoot 2
        toShoot2 = follower.pathBuilder()
                .addPath(new BezierLine(feed1Pose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(136))
                .build();

        // Path5: Shoot 2 -> Intake 2
        intake2 = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, intake2Pose))
                .setLinearHeadingInterpolation(Math.toRadians(136), Math.toRadians(180))
                .build();

        // Path6: Intake 2 -> Feed 2
        feed2 = follower.pathBuilder()
                .addPath(new BezierLine(intake2Pose, feed2Pose))
                .setTangentHeadingInterpolation()
                .build();

        // Path7: Feed 2 -> Shoot 3
        toShoot3 = follower.pathBuilder()
                .addPath(new BezierLine(feed2Pose, shootPose))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(136))
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0: // Start -> Shoot 1
                follower.followPath(toShoot1);
                setPathState(1);
                break;

            case 1: // Moving to Shoot 1
                if (!follower.isBusy()) {
                    /* Start Shooting Sequence */

                    setPathState(2);
                }
                break;

            case 2: // Shooting 1 (Wait 2s)
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(intake1, true);
                    setPathState(3);
                }
                break;

            case 3: // Moving to Intake 1
                if (!follower.isBusy()) {
                    /* At Intake 1 */
                    setPathState(35); // Wait at Intake
                }
                break;

            case 35: // Wait at Intake 1 (2s)
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(feed1, true);
                    setPathState(4);
                }
                break;

            case 4: // Moving to Feed 1
                // We keep intake running
                if (!follower.isBusy()) {
                    follower.followPath(toShoot2, true);
                    setPathState(5);
                }
                break;

            case 5: // Moving to Shoot 2
                if (!follower.isBusy()) {
                    /* Start Shooting Sequence 2 */
                    setPathState(6);
                }
                break;

            case 6: // Shooting 2
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    // Start next cycle: Intake 2
                    follower.followPath(intake2, true);
                    setPathState(7);
                }
                break;

            case 7: // Moving to Intake 2
                if (!follower.isBusy()) {
                    setPathState(75); // Wait at Intake
                }
                break;

            case 75: // Wait at Intake 2 (2s)
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    follower.followPath(feed2, true);
                    setPathState(8);
                }
                break;

            case 8: // Moving to Feed 2
                if (!follower.isBusy()) {
                    follower.followPath(toShoot3, true);
                    setPathState(9);
                }
                break;

            case 9: // Moving to Shoot 3
                if (!follower.isBusy()) {
                    /* Start Shooting Sequence 3 */
                    setPathState(10);
                }
                break;

            case 10: // Shooting 3
                if (pathTimer.getElapsedTimeSeconds() > 2) {
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();

        telemetry.addData("Path State", pathState);
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        // Initialize Follower
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

    }

    @Override
    public void init_loop() {
        telemetry.addLine("play");
        telemetry.update();
    }

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

}