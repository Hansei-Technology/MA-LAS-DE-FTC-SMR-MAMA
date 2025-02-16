package htech.mechanism;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import htech.config.RobotSettings;
import htech.config.Sensors;

@Config
public class LimeLightWrapper {
    LLResult result;
    public double[] pythonOutput;
    public boolean valid;
    Limelight3A limelight;
    public static int pipeLine = 0;
    public double X, Y, HEADING;


    public static final int[][] referenceMatrix = {
            {50, 425}, {70, 425}, {90, 425}, {110, 425}, {130, 425},
            {150, 425}, {170, 380}, {200, 335}, {210, 310}, {220, 290},
            {240, 275}, {260, 240}, {280, 210}, {300, 190}, {320, 170},
            {340, 150}, {360, 123}, {380, 100}, {400, 65}, {420, 40}
    };

    public LimeLightWrapper(HardwareMap map) {
        limelight = map.get(Limelight3A.class, "limelight");
        //limelight.pipelineSwitch(pipeLine);
    }

    public void start() {
        limelight.start();
    }

    public void update() {
        result = limelight.getLatestResult();
        pythonOutput = result.getPythonOutput();

        valid = (pythonOutput[0] == 1);

        if (valid) {
            if(pythonOutput[2] > 300) {
                X = (pythonOutput[1] - 318) * 0.05;
            } else if(pythonOutput[2] > 200){
                X = (pythonOutput[1] - 318) * 0.063;
            } else {
                X = (pythonOutput[1] - 318) * 0.07;
            }
            //X = (pythonOutput[1] - 318) * RobotSettings.limeLightXMultiplyer;
            Y = getIntervalValue(pythonOutput[2]);
            HEADING = pythonOutput[3];
        }
    }


    public static int getIntervalValue(double pixelDistance) {
        for (int i = 0; i < referenceMatrix.length - 1; i++) {
            int px1 = referenceMatrix[i][0];
            int px2 = referenceMatrix[i + 1][0];
            int realValue = referenceMatrix[i][1];

            if (pixelDistance >= px1 && pixelDistance < px2) {
                return realValue;
            }
        }

        // Dacă distanța depășește ultimul interval, returnăm ultima valoare
        return referenceMatrix[referenceMatrix.length - 1][1];
    }


}
