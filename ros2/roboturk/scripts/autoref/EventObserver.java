import edu.tigers.autoref.AutoReferee;
import edu.tigers.autoreferee.engine.EAutoRefMode;
import edu.tigers.autoreferee.module.AutoRefModule;
import edu.tigers.sumatra.model.SumatraModel;
import edu.tigers.sumatra.wp.AWorldPredictor;
import edu.tigers.sumatra.wp.IWorldFrameObserver;
import edu.tigers.sumatra.wp.data.WorldFrameWrapper;
import java.util.Base64;
import java.util.concurrent.atomic.AtomicLong;

/** Print the original protobuf for each event the unmodified TIGERs engine reports. */
public final class EventObserver {
    public static void main(String[] args) {
        AutoReferee.main(new String[]{"--headless", "--ci"});
        var module = SumatraModel.getInstance().getModule(AutoRefModule.class);
        var timestamp = new AtomicLong();
        SumatraModel.getInstance().getModule(AWorldPredictor.class).addObserver(new IWorldFrameObserver() {
            @Override public void onNewWorldFrame(WorldFrameWrapper frame) {
                timestamp.set(frame.getTimestamp());
            }
        });
        module.addObserver(event -> System.out.println("SSL_EVENT " + timestamp.get() + " " +
            Base64.getEncoder().encodeToString(event.toProtobuf().toByteArray())));
        module.changeMode(EAutoRefMode.valueOf(args[0]));
        System.out.println("SSL_READY " + module.getMode());
    }
}
