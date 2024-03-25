package frc.robot.subsystems.vision;

import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.cscore.VideoSource;
import edu.wpi.first.wpilibj.DriverStation;

@Deprecated
public class SwitchableDriverCam {
    private final VideoSink server;
    private final VideoSource[] sources;
    private int index = -1;

    @SafeVarargs
    public SwitchableDriverCam(VideoSink server, VideoSource... sources) {
        if (sources.length == 0) {
            throw new IllegalArgumentException("Must include at least 1 camera");
        }
        this.server = server;
        this.sources = sources;
        switchCam();
    }

    public void switchCam() {
        server.setSource(sources[++index % sources.length]); // overflow on index doesn't matter, mod loops right back around
                                                             // also takes 1.36 years so there's that
    }

    public void setStreamToIndex(int index) {
        if (index == this.index) return;
        if (index >= sources.length) {
            DriverStation.reportError("Index out of bounds", Thread.currentThread().getStackTrace());
        } else {
            this.index = index;
            server.setSource(sources[index]);
        }
    }
}
