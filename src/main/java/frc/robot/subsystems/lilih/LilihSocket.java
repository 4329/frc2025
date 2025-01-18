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
import java.util.concurrent.ExecutionException;

public class LilihSocket {

  private WebsocketListener listener;
  private ObjectMapper objectMapper;
  private boolean webSocketConnected;

  public LilihSocket() {
    HttpClient httpClient = HttpClient.newHttpClient();
    listener = new WebsocketListener();
    new Thread(
            () -> {
              try {
                httpClient
                    .newWebSocketBuilder()
                    .buildAsync(URI.create("ws://10.43.29.11:5806"), listener)
                    .get();
                webSocketConnected = true;
              } catch (InterruptedException | ExecutionException e) {
                System.out.println(e.getMessage());
              }
            })
        .start();
    objectMapper = new ObjectMapper();
    objectMapper.configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);
  }

  public LimelightHelpers.Results getResults() {
    if (!webSocketConnected) return new LimelightHelpers.Results();

    try {
      String asdfs = listener.getOutput();
      return objectMapper.readValue(asdfs, LimelightHelpers.Results.class);
    } catch (JsonProcessingException e) {
      System.err.println("lljson error: " + e.getMessage());
      return new LimelightHelpers.Results();
    }
  }
}
