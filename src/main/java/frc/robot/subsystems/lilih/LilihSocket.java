// /**
//  * ChatClientEndpoint.java
//  * http://programmingforliving.com
//  */
package frc.robot.subsystems.lilih;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.DeserializationFeature;
import com.fasterxml.jackson.databind.ObjectMapper;
import frc.robot.utilities.LimelightHelpers;
import frc.robot.utilities.WebsocketListener;
import java.net.URI;
import java.net.http.HttpClient;

import org.littletonrobotics.junction.Logger;

public class LilihSocket {

    private WebsocketListener listener;
    private ObjectMapper objectMapper;
    private int ip;

    public LilihSocket(int ip) {
        this.ip = ip;

        createSocket();

        objectMapper = new ObjectMapper();
        objectMapper.configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
    }

    private void createSocket() {
        HttpClient httpClient = HttpClient.newHttpClient();
        listener = new WebsocketListener();
        httpClient
                .newWebSocketBuilder()
                .buildAsync(URI.create("ws://10.43.29." + ip + ":5806"), listener)
                .thenRun(() -> Logger.recordOutput("connection established", true));
    }

    public boolean isConnected() {
        return listener.isReceivingMessages();
    }

    public LimelightHelpers.LimelightResults getResults() {
        if (!isConnected()) {
            return new LimelightHelpers.LimelightResults();
        }

        try {
            return objectMapper.readValue(listener.getOutput(), LimelightHelpers.LimelightResults.class);
        } catch (JsonProcessingException e) {
            System.err.println("lljson error: " + e.getMessage());
            return new LimelightHelpers.LimelightResults();
        }
    }
}
