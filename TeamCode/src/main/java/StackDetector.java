import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

public class StackDetector extends OpenCvPipeline {
    public StackDetector(Telemetry t){
        telemetry = t;
        Mat mat  = new Mat();
    }

    @Override
    public Mat processFrame(Mat input){
        Imgproc.cvtColor(input, mat, Imgproc.COLOR);
    }
}
