package frc.robot.simulation;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

public class mech2d {
   // the main mechanism object
    public static Mechanism2d mech = new Mechanism2d(3, 3);
    // the mechanism root node
    public static MechanismRoot2d root = mech.getRoot("origin", 1.5, 0.5);

    public static MechanismLigament2d bottomspinner = root.append(new MechanismLigament2d("bottomspinner", 0.5, 0, 6, new Color8Bit(Color.kAqua)));

    public static MechanismLigament2d topfunnel = root.append(new MechanismLigament2d("topfunnel", 2, 90, 6, new Color8Bit(Color.kPurple)));

    public static MechanismLigament2d agitator = topfunnel.append(new MechanismLigament2d("agitator", 0.5, 90, 6, new Color8Bit(Color.kCoral)));

    //static MechanismLigament2d ligament = m_elevator.append(new MechanismLigament2d("wrist", 0.5, 90, 6, new Color8Bit(Color.kPurple)));
    
    


}
