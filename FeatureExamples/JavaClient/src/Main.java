import java.io.IOException;

public class Main {
    public static void main(String []args) throws IOException {
        Client AkulaApp = new Client("127.0.0.1", 50000, 32);
        AkulaApp.sendRequest();
    }
}

