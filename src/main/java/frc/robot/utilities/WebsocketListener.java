package frc.robot.utilities;

import java.net.http.WebSocket;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CompletableFuture;
import java.util.concurrent.CompletionStage;

public class WebsocketListener implements WebSocket.Listener {
  List<CharSequence> parts = new ArrayList<>();
  private List<CharSequence> outParts = new ArrayList<>();
  CompletableFuture<?> accumulatedMessage = new CompletableFuture<>();

  public CompletionStage<?> onText(WebSocket webSocket, CharSequence message, boolean last) {
    parts.add(message);
    webSocket.request(1);
    if (last) {
      processWholeText(parts);
      parts = new ArrayList<>();
      accumulatedMessage.complete(null);
      CompletionStage<?> cf = accumulatedMessage;
      accumulatedMessage = new CompletableFuture<>();
      return cf;
    }
    return accumulatedMessage;
  }

  private void processWholeText(List<CharSequence> parts) {
    outParts = parts;
  }

  public String getOutput() {
    return outParts.stream().map((c) -> c.toString()).reduce("", String::concat).toString();
  }
}
