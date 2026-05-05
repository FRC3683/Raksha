package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.config;
import frc.robot.utils.configurable;

public class neck extends SubsystemBase implements configurable {
    public static neck instance = new neck();

    private Debouncer jam_debouncer = new Debouncer( 0.70067, DebounceType.kBoth );
    private boolean is_jammed = false;

    private boolean running = false;

    private TalonFX nmotor;

    private final VelocityVoltage m_request = new VelocityVoltage(0).withSlot(0).withEnableFOC(true);
    private final VelocityTorqueCurrentFOC torque_req = new VelocityTorqueCurrentFOC(0).withSlot(1);
    private final DutyCycleOut gogogo = new DutyCycleOut( 1 );

    private final StatusSignal<AngularVelocity> speed_signal;
    private final StatusSignal<Current> stator_signal;

    private neck() {
       nmotor = new TalonFX( config.neck_motor, config.main_canbus );
       SmartDashboard.putNumber( "Neck Motor Speed", 0.5 );

       speed_signal = nmotor.getVelocity();
       stator_signal = nmotor.getStatorCurrent();

       speed_signal.setUpdateFrequency( 50 );
       stator_signal.setUpdateFrequency( 50 );

       nmotor.optimizeBusUtilization();
       setDefaultCommand( idle_cmd() );
    }

    @Override
    public void periodic() {
        BaseStatusSignal.refreshAll( speed_signal, stator_signal );

        is_jammed = DriverStation.isAutonomous() && jam_debouncer.calculate( running && speed_signal.getValue().in(RPM) > 0.750067 * 5800 && stator_signal.getValue().in(Amps) < 20 );
    }

    public boolean jammed() {
        return is_jammed;
    }

    public Command idle_cmd() {
        return run( () -> {
            running = false;
            nmotor.stopMotor();
        });
    }

    private void set( double speed_percent ) {
        running = true;
        speed_percent = MathUtil.clamp( speed_percent, -1, 1 );
        var speed = RPM.of( 5800 * speed_percent );
        nmotor.setControl( torque_req.withVelocity( speed ) );

        // nmotor.set( speed.in( RPM ) );
        // nmotor.setControl( gogogo.withOutput( speed_percent ) );
    }

    public Command SpinNeck() {
        return run(() -> {
            if( jammed() ) {
                set( -0.850067 );//SmartDashboard.getNumber("Neck Motor Speed", 0) );
            } else {
                set( 0.850067 );//SmartDashboard.getNumber("Neck Motor Speed", 0) );
            }
        }).ignoringDisable(false);
    }

    public Command BackwardsSpinNeck() {
        return run(() -> {
            set( -0.80067 );//-SmartDashboard.getNumber("Neck Motor Speed", 0) );
        }).ignoringDisable(false);
    }

    @Override
    public void configure() {
        nmotor.getConfigurator().apply( config.neck.roller_cfg );
    }
}
