package frc.robot;

import com.ctre.phoenix6.controls.MusicTone;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.AnalogTriggerOutput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.swerve;
import frc.robot.utils.LL_connected_check;
import frc.robot.utils.commandable_flag;
import frc.robot.utils.dave_led;
import frc.robot.utils.match_timer;
import frc.robot.utils.oi;
import frc.robot.utils.voltage_warning;
import frc.robot.utils.match_timer.phase;

public class leds extends dave_led {

    public static final LL_connected_check LL_connected_turret = new LL_connected_check(config.LL_turret_name);
    // private final LL_connected_check LL_connected_right = new LL_connected_check(config.LL_right_name);
    private final swerve swerve;

    public final commandable_flag flag_configuring = new commandable_flag();

    public final commandable_flag flag_operator_flash = new commandable_flag();
    public final commandable_flag flag_flash_zero = new commandable_flag();
    public static final commandable_flag flag_nevk_feeding = new commandable_flag();
    public static final commandable_flag flag_sotm_throttle = new commandable_flag();
    public static final commandable_flag flag_unjamming = new commandable_flag();

// this chunk is all stuff for the LED game
    private boolean game_playing = false;
    Timer loopTimer = new Timer();
    Timer gameTimer = new Timer();
    private int LED_game_score = 0;
    private int Long_note_score = 0;
    private int Levelselect = 0;
    private boolean blobberdob = true;
    private boolean blueLongnotefail = false;
    private boolean yellowLongnotefail = false;
    private boolean yellowLongnoteactive = false;
    private boolean Bluehit = false;
    private boolean yellowhit = false;
    private boolean Purblehit = false;
    private boolean doubletime = false;
    private boolean Purbleinput = false;
    private boolean Bluelongpause = false;
    private boolean yellowlong_note_pause = false;
    private boolean Colourblindmode = false;
    private double realsecondspernote = 0;
    private int Hardmode = 0;
    private int purblecondition = 0;
    private int glooby = 0;
    private int evilglooby = 0;
    private int BlueWrongcount = 0;
    private int yellowwrongcount = 0;
    private boolean gamemenu = false;
    private int menuselect = 0;
    private int settingsellect =0;
    private boolean Good_hit_blue= false;
    private boolean Good_hit_purb = false;
    private boolean goodhityellow = false;
    private boolean blueLongnoteactive = false;
    private boolean bothlongnotesactive = false;
    private double Songlength = 24;
    private int[] blue_pattern  = {};
    private int[] yellowpattern = {};
    private int[] purble_pattern = {};
    private int[] Long_bluepattern = {};
    private int[] longyellow_pattern = {};

    public leds(TimedRobot robot, swerve swerve) {
        super(config.pwm_leds, config.leds.length, robot);
        this.swerve = swerve;

        robot.addPeriodic(this::periodic, constants.control_dts);
    } 

    private void handle_flags() {
        if(flag_flash_zero.get()) {
            set_fast_flicker(clr_cyan);
            return;
        }

        if(flag_operator_flash.get()) {
            set_fast_flicker(clr_yellow);
            return;
        }

        if(DriverStation.isDisabled()) {
            return;
        }

        Color8Bit state = clr_green;

        if( !match_timer.is_hub_active() ) {
            state = clr_red;
        }

        if( match_timer.get_current_phase() == phase.ENDGAME ) {
            state = clr_purple;
        }

        if( bindings.xkeys_always_shoot.get() ) {
            state = clr_cyan;
        }

        if( flag_nevk_feeding.get() ) {
            state = clr_orange;
        }

        if( flag_unjamming.get() && DriverStation.isAutonomous() ) {
            state = clr_purple;
        }

        // if( flag_sotm_throttle.get() ) {
        //     state = clr_blue;
        // }

        set_color( state );

        if( match_timer.hub_active_preview( 3 ) != match_timer.is_hub_active() ) {
            set_bloom( clr_white, (int)Math.round( timer.get() * length * 3.5 ) % length );
        }
        else if( match_timer.hub_active_preview() != match_timer.is_hub_active() ) {
            set_bloom( clr_white, (int)Math.round( timer.get() * length * 2 ) % length );
        }
    }
    public void LEDLeveldata(double secondspernote, double secondspernote2, int lengthofsong, int lengthofsong2, String songname, 
    String songname2, int bluenotes[],int yellownotes[], int purpblenotes[], int longbluenotes[], int longyellownotes[]){
        if(doubletime){
            realsecondspernote = secondspernote2;
            Songlength = lengthofsong2;
            frc.robot.robot.m_orchestra.loadMusic(songname2+".chrp");
        }
        else{
            realsecondspernote = secondspernote;
            frc.robot.robot.m_orchestra.loadMusic(songname+".chrp");
            Songlength =lengthofsong;
        }
        blue_pattern = bluenotes;
        yellowpattern = yellownotes;
        purble_pattern = purpblenotes;
        Long_bluepattern = longbluenotes;
        longyellow_pattern = longyellownotes;
        loopTimer.reset();
        gameTimer.reset();
    }
    public void LEDgamecontrols(boolean notehit, int[] notepattern, boolean notegoodhit, boolean notelongnoteactive, boolean buttoninput){
        if(notehit == false){
            
            for(int i = 0; i<notepattern.length; i++){
                if(notepattern[i] == 18 && buttoninput){
                    LED_game_score++;
                    glooby++;
                    notegoodhit = true;
                    if(Purbleinput){
                        LED_game_score = LED_game_score + Hardmode;
                    }
                    }
                }
            for(int i = 0; i<notepattern.length; i++){ 
                if(!(notepattern[i] == 18) && buttoninput && !(notegoodhit) && notelongnoteactive==false){
                    LED_game_score = LED_game_score -Hardmode;
                    evilglooby++;
                }
            }
            if(buttoninput && !(notelongnoteactive)){
                notehit = true;
                purblecondition++;
            }
        }
    } 
    public void Thisonemakesthecolourblindmodeworkwithouttoomuchbloat(Color8Bit colorbloo, Color8Bit colooryellow, Color8Bit coluorpuple, Color8Bit colirerornage, Color8Bit cloorcyen){
        for(int i = 0; i<blue_pattern.length; i++){
        set_single(blue_pattern[i], colorbloo);
        }
        for(int i = 0; i<yellowpattern.length; i++){
        set_single(yellowpattern[i], colooryellow);
        }
        for(int i = 0; i<purble_pattern.length; i++){
        set_single(purble_pattern[i], coluorpuple);
        }
        for(int i = 0; i<longyellow_pattern.length; i++){
        set_single(longyellow_pattern[i], colirerornage);
        }
        for(int i = 0; i<Long_bluepattern.length; i++){
        set_single(Long_bluepattern[i], cloorcyen);
        }
    }

    @Override
    protected void periodic() {
        boolean disabled = DriverStation.isDisabled();
        if(flag_configuring.get()) {
            set_color(0, length/2, clr_blue);
            set_color(length/2, length, clr_purple);
            return;
        }
        if(!disabled){
            game_playing = false;
            gamemenu = false;
            blobberdob = true;
            LED_game_score = 0;
            Long_note_score = 0;
        }
        if(disabled) {
            if(!(game_playing) && !(gamemenu) && disabled){
            set_color(0, length, clr_off);

            Color8Bit disabled_color = clr_purple;
            if(DriverStation.isDSAttached() && DriverStation.getAlliance().isPresent()) {
                switch(DriverStation.getAlliance().get()) {
                    case Blue: disabled_color = clr_blue; break;
                    case Red:  disabled_color = clr_red; break;
                }
            }
            if(DriverStation.isTest()) {
                rainbow(5);
            } else {
                set_trail(disabled_color, 0, length-1);
            }
            if(oi.cmd_driver.getHID().getStartButtonPressed()){
                gamemenu = true;
                menuselect = 0;
            }
            }

            if(gamemenu && disabled){
                if(menuselect == 0){

                    set_color(-1, 22, clr_off);
                    if(Levelselect == 0){
                        set_color(4, 7, clr_white);
                        int BlueHotCrossBuns[] =  {-33,-35,-37,-39};   
                        int yellow_hotcross_buns[] = {-41,-43,-45,-47}; 
                        int purble_hotcrossbuns[] = {};
                        int emptylongblue[] = {-1,-2,-3,-4,-9,-10,-11,-12,-21,-22,-23,-24,-49,-50,-51,-52,-57,-58,-59,-60};
                        int emptylon_gyellow[] = {-5,-6,-7,-8,-17,-18,-19,-20,-25,-26,-27,-28,-53,-54,-55,-56};                        
                        LEDLeveldata(0.25, 0.125, 24, 14, "HotCrossBuns","2XHotCrossBuns",BlueHotCrossBuns,yellow_hotcross_buns,purble_hotcrossbuns,emptylongblue,emptylon_gyellow); 
                    }
                    else if(Levelselect == 1){
                        set_color(9, 12, clr_white);
                        int spook_Blue[] = {-1,-2,-3,-9,-10,-11,-14,-15,-20,-21,-22,-23,-28,-29,-42,-44,};
                        int spookyellow[] = {-4,-5,-6,-7,-12,-13,-17,-18,-19,-25,-26,-27,-30,-31,-41,-43,-45};
                        int spookpurble[] = {3,4,-46,-47};
                        int spooklongyellow[] = {-37,-38,-39};
                        int spooklongblue[] = {-33,-34,-35};
                        LEDLeveldata(0.5, 0.25, 36, 20, "Spooktune","2XSpooktune" , spook_Blue, spookyellow, spookpurble, spooklongblue, spooklongyellow);
                    }
                    else{
                        set_color(14, 17, clr_white);
                        int TTFAFblue[] = {-1,-3,-5,-7,-9,-11,-13,-15,-17,-19,-21,-23,-25,-27,-29,-31,-97,-99,-101,-103,-105,-107,-109,-111,-113,-115,-117,-119,-121,-123,-125,-127,-193,-195,-197,-199,-201,-203,-205,-207,-209,-211,-213,-215,-217,-219,-221,-223,-480,-481,-482,-483,-484,-485,-486,-517,-519,-521,-533,-535,-537,-543,-545,-547,-559,-561,-563,-569,-571,-573,-581,-583,-585,-587,-589,-597,-599,-616,-617,-618,-619,-620,-621,-626,-627,-628,-630};
                        int TTFAFyellow[] = {-33,-35,-37,-39,-41,-43,-45,-47,-49,-51,-53,-55,-57,-59,-61,-63,-129,-131,-133,-135,-137,-139,-141,-143,-145,-147,-149,-151,-153,-155,-157,-159,-488,-489,-490,-491,-492,-493,-494,-495,-511,-513,-515,-523,-525,-527,-529,-531,-539,-541,-549,-551,-553,-555,-557,-565,-567,-575,-577,-579,-591,-593,-595,-622,-623,-624};
                        int purble_place[] = {-65,-67,-69,-71,-73,-75,-77,-79,-81,-83,-85,-87,-89,-91,-93,-95,-161,-163,-165,-167,-169,-171,-173,-175,-177,-179,-181,-183,-185,-187,-189,-191,-225,-227,-229,-231,-233,-235,-237,-239,-240,-241,-242,-243,-244,-245,-246,-247,-248,-249,-250,-251,-252,-253,-254,-255,-368,-369,-370,-371,-372,-373,-374,-375,-600,-601,-602,-603,-604,-605,-606,-607,-608,-609,-610,-611,-612,-613,-614,-615,-631,-655,-657,-659};
                        int TTFAFbluelong[] = {-256,-257,-258,-259,-262,-263,-264,-265,-268,-269,-272,-273,-274,-275,-278,-279,-280,-281,-284,-285,-288,-289,-290,-291,-294,-295,-296,-297,-300,-301,-304,-305,-306,-307,-310,-311,-312,-313,-316,-317,-320,-321,-322,-323,-326,-327,-328,-329,-332,-333,-336,-337,-338,-339,-342,-343,-344,-345,-348,-349,-352,-353,-354,-355,-358,-359,-360,-361,-364,-365,-380,-381,-382,-383,-384,-385,-386,-387,-390,-391,-392,-393,-396,-397,-400,-401,-402,-403,-406,-407,-408,-409,-412,-413,-416,-417,-418,-419,-422,-423,-424,-425,-428,-429,-432,-433,-434,-435,-438,-439,-440,-441,-444,-445,-448,-449,-450,-451,-454,-454,-455,-456,-457,-460,-461,-464,-465,-466,-467,-470,-471,-472,-473,-476,-477};
                        int TTFAFlongyellow[] = {-260,-261,-266,-267,-270,-271,-276,-277,-282,-283,-286,-287,-292,-293,-298,-299,-302,-303,-308,-309,-314,-315,-318,-319,-324,-325,-330,-331,-334,-335,-340,-341,-346,-347,-350,-351,-356,-357,-362,-363,-366,-367,-378,-379,-388,-389,-394,-395,-398,-399,-404,-405,-410,-411,-414,-415,-420,-421,-426,-427,-430,-431,-436,-437,-442,-443,-446,-447,-452,-453,-458,-459,-462,-463,-468,-469,-474,-475,-478,-479};
                        LEDLeveldata(0.075, 0.0375, 60, 31, "ThroughFireandFlames", "2XThroughFireandFlames", TTFAFblue, TTFAFyellow, purble_place, TTFAFbluelong, TTFAFlongyellow);
                    }
                    set_single(5, clr_green);
                    set_single(10, clr_yellow);
                    set_single(15, clr_red);
                    if(oi.cmd_driver.getHID().getAButtonPressed()){
                        game_playing = true;
                        gamemenu = false;
                        blobberdob = true;
                        frc.robot.robot.m_orchestra.play();
                    }
                    if(oi.cmd_driver.getHID().getBButtonPressed()){
                        Levelselect++;
                    }
                    if (Levelselect > 2) {
                        Levelselect = 0;
                    }
                }
                if(menuselect == 1){
                    set_color(-1, 22, clr_off);
                    set_single(5, clr_blue);
                    set_single(10, clr_purple);
                    set_single(15, clr_orange);
                    if(oi.cmd_driver.getHID().getBButtonPressed()){
                        settingsellect++;
                    }
                    if(settingsellect > 2){
                        settingsellect = 0;
                    }
                    if(Colourblindmode){
                        set_single(11, clr_green);
                    }
                    else{
                        set_single(11, clr_red);
                    }
                    if(Hardmode == 0){
                        set_single(6, clr_red);
                    }
                    else{
                        set_single(6, clr_green);
                    }
                    if(doubletime){
                        set_single(16, clr_green);
                    }
                    else{
                        set_single(16, clr_red);
                    }
                    if(settingsellect == 0){
                        set_single(4, clr_white);
                        if(oi.cmd_driver.getHID().getAButtonPressed()){
                            if(Hardmode == 0){
                                Hardmode = 1;
                            }
                            else{
                                Hardmode = 0;
                            }
                        }
                    }
                    if(settingsellect == 1){
                        set_single(9, clr_white);
                        if(oi.cmd_driver.getHID().getAButtonPressed()){
                            if(Colourblindmode){
                                Colourblindmode = false;
                            }
                            else{
                                Colourblindmode = true;
                            }
                        }
                    }
                    if(settingsellect == 2){
                        set_single(14, clr_white);
                        if(oi.cmd_driver.getHID().getAButtonPressed()){
                            if(doubletime){
                                doubletime = false;
                            }
                            else{
                                doubletime = true;
                            }
                        }
                    }
                }

                if(oi.cmd_driver.getHID().getYButtonPressed()){
                    menuselect++;
                }
                if(menuselect > 1){
                    menuselect = 0;
                }
            }
            
            // System.out.println(Long_note_score);
            // System.out.println(blueLongnoteactive);
            if(game_playing && disabled){    
                loopTimer.start();
                gameTimer.start();
                
            if(blobberdob){ //makes sure that the white dot only stays on during gameplay
                set_single(17, clr_white); 
            }

                if(loopTimer.get() > realsecondspernote && gameTimer.get() <= Songlength -1){
                    loopTimer.reset();


                    //various variables that need to be reset every beat
                    Bluehit = false; 
                    yellowhit = false;
                    Purblehit = false;
                    Bluelongpause = false;
                    yellowlong_note_pause = false;
                    purblecondition = 0;
                    Good_hit_blue = false;
                    goodhityellow = false;
                    Good_hit_purb = false;
                    glooby = 0;
                    evilglooby = 0;

                    set_color(-1, 17, clr_off);//disables the lights before they are re-lit
                    set_color(18, 21, clr_off);
                    //this sets all the lights that need to be lit as lit
                    if(Colourblindmode){
                        Thisonemakesthecolourblindmodeworkwithouttoomuchbloat(clr_white, clr_white, clr_white, clr_white, clr_white);
                    }
                    else{
                        Thisonemakesthecolourblindmodeworkwithouttoomuchbloat(clr_blue,clr_yellow,clr_purple,clr_orange,clr_cyan);
                    }
                    //increments each element of blue_pattern, yellowpattern, longyellow_pattern, and Long_bluepattern by one
                    for(int i = 0; i<blue_pattern.length; i++){
                    blue_pattern[i]++;
                    }
                    for(int i = 0; i<yellowpattern.length; i++){
                    yellowpattern[i]++;
                    }
                    for(int i = 0; i<purble_pattern.length; i++){
                    purble_pattern[i]++;
                    }
                    for(int i = 0; i<longyellow_pattern.length; i++){
                    longyellow_pattern[i]++;
                    }
                    for(int i = 0; i<Long_bluepattern.length; i++){
                    Long_bluepattern[i]++;
                    }

                }
                // System.out.println(LED_game_score);
                //prevents you from getting more than one point from a note
                if((glooby > 1) && !(blueLongnoteactive) && !(yellowLongnoteactive)){
                    LED_game_score = LED_game_score - (glooby - 1) -Hardmode;
                    glooby = 1;
                }
                //prevents you from losing all your points in a single mistake
                if((evilglooby > 1) && !(blueLongnoteactive) && !(yellowLongnoteactive) && Hardmode == 1){
                    LED_game_score = LED_game_score + (evilglooby - 1);
                    evilglooby = 1;
                }
                //so you can't get a negative score if Hardmode = 1
                if (LED_game_score < 0) {
                    LED_game_score = 0;
                }
                //this section is all the junk that lets you hit blue long notes
                //oi.cmd_driver.getHID().getRightBumperButton()
                for(int i = 0; i<Long_bluepattern.length; i++){
                    if(Long_bluepattern[i] == 18 && oi.cmd_driver.getHID().getRightBumperButton() && !(Bluelongpause) && !(blueLongnotefail)){
                        Long_note_score++;
                        blueLongnoteactive = true;
                        Bluelongpause = true;
                    }
                }
                BlueWrongcount=Long_bluepattern.length;
                for(int i = 0; i<Long_bluepattern.length; i++){
                    if(Long_bluepattern[i] == 18){
                        BlueWrongcount--;
                    }
                }
                if(BlueWrongcount==Long_bluepattern.length){
                        blueLongnotefail = false;
                        blueLongnoteactive = false;
                }
                for(int i = 0; i<Long_bluepattern.length; i++){
                    if(Long_bluepattern[i] == 19 && !(oi.cmd_driver.getHID().getRightBumperButton()) && blueLongnoteactive){
                        blueLongnotefail = true;
                        blueLongnoteactive = false;   
                    }
                }
                //this section is all the junk that lets you hit yellow long notes
                 //oi.cmd_driver.getHID().getLeftBumperButton()
                for(int i = 0; i<longyellow_pattern.length; i++){
                    if(longyellow_pattern[i] == 18 && oi.cmd_driver.getHID().getLeftBumperButton() && !(yellowlong_note_pause) && !(yellowLongnotefail)){
                        Long_note_score++;
                        yellowLongnoteactive = true;
                        yellowlong_note_pause = true;
                    }
                }
                yellowwrongcount=longyellow_pattern.length;
                for(int i = 0; i<longyellow_pattern.length; i++){
                    if(longyellow_pattern[i] == 18){
                        yellowwrongcount--;
                    }
                }
                if(yellowwrongcount==longyellow_pattern.length){
                        yellowLongnotefail = false;
                        yellowLongnoteactive = false;
                }
                for(int i = 0; i<longyellow_pattern.length; i++){
                    if(longyellow_pattern[i] == 19 && !(oi.cmd_driver.getHID().getLeftBumperButton()) && yellowLongnoteactive){
                        yellowLongnotefail = true;
                        yellowLongnoteactive = false;   
                    }
                }
                // System.out.println(blueLongnotefail);
                //this bit lets you hit blue short notes
                LEDgamecontrols(Bluehit,blue_pattern,Good_hit_blue,blueLongnoteactive, oi.cmd_driver.getHID().getRightBumperButtonPressed());//.getRightBumperButtonPressed()

                //this bit lets you hit yellow short notes
                LEDgamecontrols(yellowhit,yellowpattern, goodhityellow, yellowLongnoteactive, oi.cmd_driver.getHID().getLeftBumperButtonPressed());//oi.cmd_driver.getHID().getLeftBumperButtonPressed()
                
                //this section lets you hit purple notes
                if(purblecondition == 2){
                    Purbleinput = true;
                }
                else{
                    Purbleinput = false;
                }
                if(blueLongnoteactive && yellowLongnoteactive){
                    bothlongnotesactive = true;
                }
                else{
                    bothlongnotesactive = false;
                }
                LEDgamecontrols(Purblehit,purble_pattern,Good_hit_purb,bothlongnotesactive, Purbleinput);
                //this part is all the post-level stuff
            if(gameTimer.get() >= Songlength && gameTimer.get() <= Songlength + 8){
                //this blobberdob section is for calc'ing the score and turning off the white dot; i couldn't think of a good name for it
                if(blobberdob){
                    LED_game_score = LED_game_score + Long_note_score;
                    blobberdob = false;
                    set_color(-1, 19, clr_off);
                }
                //displays F in morse code (..-.)
                int Final_led_maxscore = yellowpattern.length + blue_pattern.length + longyellow_pattern.length + Long_bluepattern.length + purble_pattern.length;
                if(LED_game_score <= (Final_led_maxscore)/6){ 
                    set_single( 3, clr_red);
                    set_single(5, clr_red);
                    set_color(7, 9, clr_red);
                    set_single(10, clr_red);
                }
                //displays D in morse code (-..)
                else if(LED_game_score <= (Final_led_maxscore)-((Final_led_maxscore)/4)*3){ 
                    set_color(3, 5, clr_orange);
                    set_single(6, clr_orange);
                    set_single(8, clr_orange);
                }
                //displays C in morse code (-.-.)
                else if(LED_game_score <= (Final_led_maxscore)/2){ 
                    set_color(3, 5, clr_yellow);
                    set_single(6, clr_yellow);
                    set_color(8, 10, clr_yellow);
                    set_single(11, clr_yellow);
                }
                //displays B in morse code (-...)
                else if(LED_game_score <= (Final_led_maxscore)-(Final_led_maxscore)/4){ 
                    set_color(3, 5, clr_green);
                    set_single(6, clr_green);
                    set_single(8, clr_green);
                    set_single(10, clr_green);
                }
                //displays A in morse code (.-)
                else if(LED_game_score <= Final_led_maxscore-1){ 
                    set_single(3, clr_cyan);
                    set_color(5, 7, clr_cyan);
                }
                //displays S in morse code (...)
                else{
                    set_single(3, clr_white);
                    set_single(5, clr_white);
                    set_single(7, clr_white);
                }
            }
            if(gameTimer.get() >= Songlength + 9){
                game_playing = false;
                loopTimer.reset();
                gameTimer.reset();
                blobberdob = true;
                LED_game_score = 0;
                Long_note_score = 0;
            }

        }

        handle_flags();

        if(bindings.manual_intake) {
            set_color(length-5, length, clr_orange);
        }


        // most important, override other led codes
        if(RobotBase.isReal()) {
            boolean error = false;
            error = error || !voltage_warning.check();
            SmartDashboard.putBoolean("estopped", !DriverStation.isEStopped());
            error = error || DriverStation.isEStopped();
            boolean joysticks = DriverStation.isJoystickConnected(0) && DriverStation.isJoystickConnected(1);
            SmartDashboard.putBoolean("joysticks", joysticks);
            error = error || !joysticks;
            boolean LL_left = LL_connected_turret.connected();
            // boolean LL_right = LL_connected_right.connected(); // make sure both are updated
            boolean LL_connected =  LL_left;// && LL_right;
            error = error || !LL_connected;
            boolean toasty = swerve.toasty_motors();
            error = error || toasty;
            error = error || !SmartDashboard.getBoolean( "close_enough", true );
            if(error) {
                set_checker(0, length / 4, clr_cyan, clr_orange);
            }
        }
    }
}
}