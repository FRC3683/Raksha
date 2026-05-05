package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config;
import frc.robot.simulation.mech2d;
import frc.robot.utils.configurable;

public class spindexer extends SubsystemBase implements configurable {
    public static spindexer instance = new spindexer();

    // private TalonFX top;
    private TalonFX bottom;
    public static AngularVelocity topspeed;
    public static AngularVelocity bottomspeed;

    private VelocityTorqueCurrentFOC foc_request = new VelocityTorqueCurrentFOC(0);
    private DutyCycleOut gogogo = new DutyCycleOut( 1 ).withEnableFOC( true );

    private spindexer() {
        // top = new TalonFX( config.topspindexer, config.main_canbus );
        bottom = new TalonFX( config.bottomspindexer, config.main_canbus );
        // SmartDashboard.putNumber( "Top Agitator Speed", 0.125 );
        // SmartDashboard.putNumber( "Bottom Feed Speed", 0.5 );
        topspeed = RPM.of( 160 );//SmartDashboard.getNumber( "Top Agitator Speed", 0 );

        SmartDashboard.putNumber("bottomsped", 0);
        bottomspeed = RotationsPerSecond.of( -2 );//SmartDashboard.getNumber( "Bottom Feed Speed", 0 );

        bottom.optimizeBusUtilization();
        setDefaultCommand(BackAndThenIdle());
    }

    private Command go( boolean spin, boolean slow ) {
        return run(() -> {
            double gogogo_speed = 1;
            if( slow ) {
                gogogo_speed = 0.2;
            }
            bottomspeed = RotationsPerSecond.of( SmartDashboard.getNumber("bottomsped", 0) );
            AngularVelocity speed = spin ? bottomspeed : RPM.zero();
            if( neck.instance.jammed() ) {
                speed = speed.unaryMinus();
                gogogo_speed = -1;
            }
            // top.setControl(foc_request.withVelocity( agitate ? topspeed : RPM.zero() ) );
            // bottom.setControl(foc_request.withVelocity( spin ? speed : RPM.zero() ) );
            // top.set( agitate ? topspeed : 0 );
            bottom.setControl( gogogo.withOutput( spin ? gogogo_speed : 0 ) );
        }).ignoringDisable(false);
    }

    public Command SpinSlow() {
        return go( true, true );
    }

    public Command Agitate() {
        return go( false, true );
    }

    public Command Spin() {
        return go( true, false );
    }

    public Command Idle() {
        return go( false, false );
    }

    public Command SpinBackwards() {
        return run(() -> {
            // top.set( 0 );
            bottom.set( -1 );
        }).ignoringDisable(false);
    }

    public Command BackAndThenIdle() {
        return Commands.sequence(
            SpinBackwards()
                .withTimeout(0.4),
            Idle()
        );
    }

    @Override
    public void simulationPeriodic() {
        double bottomcurrentangle = 0;
        double topcurrentangle = 0;

        bottomcurrentangle = mech2d.bottomspinner.getAngle();
        // mech2d.bottomspinner.setAngle(bottomcurrentangle + bottomspeed);

        topcurrentangle = mech2d.agitator.getAngle();
        // mech2d.agitator.setAngle(topcurrentangle + topspeed);
    }

    @Override
    public void periodic() {
    }


    @Override
    public void configure() {
        // top.getConfigurator().apply( config.spindexer_cfg.agitator_cfg );
        bottom.getConfigurator().apply( config.spindexer_cfg.indexer_cfg );
    }
}

