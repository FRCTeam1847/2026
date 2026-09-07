package frc.robot.simulation;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Translation2d;
import swervelib.simulation.ironmaple.simulation.SimulatedArena;
import swervelib.simulation.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import swervelib.simulation.ironmaple.simulation.seasonspecific.rebuilt2026.RebuiltFuelOnField;

public class FieldSimulation {

    private SimulatedArena arena = SimulatedArena.getInstance();

    public FieldSimulation(SwerveDriveSimulation swerveDriveSimulation) {
        arena.resetFieldForAuto();

        arena.addGamePiece(
            new RebuiltFuelOnField(
                new Translation2d(2, 2)
            ));
    }

    public void periodic() {
        Logger.recordOutput("FieldSimulation/Fuel",
                arena.getGamePiecesArrayByType("Fuel"));
        arena.simulationPeriodic();
    }
}