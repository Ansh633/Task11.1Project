#include <WiFiNINA.h>
#include <PubSubClient.h>
#include <DHT.h>

// Pin definitions
#define DHTPIN 2            // Pin connected to the DHT11 sensor
#define DHTTYPE DHT22       // DHT sensor type (DHT22)
#define MQ2PIN A0           // Analog pin connected to the MQ2 sensor

DHT dht(DHTPIN, DHTTYPE);   // Initialize DHT sensor with pin and type

// WiFi and MQTT credentials
char ssid[] = "Ansh";                     // WiFi SSID
char pass[] = "123456789";                // WiFi password
const char* broker = "broker.hivemq.com"; // MQTT broker address
const char* topic = "Home/SensorData";    // MQTT topic for sending sensor data

// WiFi and MQTT client initialization
WiFiClient wifiClient;
PubSubClient client(wifiClient);

// Thresholds for validating sensor readings
const float TEMP_MIN = -10.0;    // Minimum valid temperature
const float TEMP_MAX = 50.0;     // Maximum valid temperature
const float HUMIDITY_MIN = 0.0;  // Minimum valid humidity
const float HUMIDITY_MAX = 100.0; // Maximum valid humidity
const float SMOKE_MIN = 0;       // Minimum valid smoke level
const float SMOKE_MAX = 100;     // Maximum valid smoke level

void setup() {
  // Start serial communication for debugging
  Serial.begin(9600);
  dht.begin();  // Start the DHT sensor

  // Connect to WiFi
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Set MQTT broker and connect
  client.setServer(broker, 1883);
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    if (client.connect("ArduinoNano")) {
      Serial.println("Connected to MQTT broker");
    } else {
      Serial.print("Failed, rc=");
      Serial.print(client.state());
      delay(2000);  // Wait before retrying
    }
  }
}

void loop() {
  client.loop();  // Process incoming and outgoing MQTT messages

  // Read humidity and temperature from DHT sensor
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  // Read smoke level from MQ2 sensor
  int mq2Value = analogRead(MQ2PIN);
  float smokeLevel = map(mq2Value, 0, 1023, 0, 100);  // Convert to a percentage scale

  // Check if DHT readings are valid and within a sensible range
  if (isnan(humidity) || isnan(temperature) ||
      temperature < TEMP_MIN || temperature > TEMP_MAX ||
      humidity < HUMIDITY_MIN || humidity > HUMIDITY_MAX) {
    Serial.println("DHT sensor reading out of range or failed!");
    return;  // Exit if temperature or humidity is invalid
  }

  // Check if smoke level is within a valid range
  if (smokeLevel < SMOKE_MIN || smokeLevel > SMOKE_MAX) {
    Serial.println("MQ2 sensor reading out of range!");
    return;  // Exit if smoke level is invalid
  }

  // Prepare the data message with valid temperature, humidity, and smoke level
  String dataMessage = "Temperature: " + String(temperature) + 
                       ", Humidity: " + String(humidity) + 
                       ", Smoke Level: " + String(smokeLevel);

  // Publish the sensor data to the MQTT broker
  if (client.publish(topic, dataMessage.c_str())) {
    Serial.println("Sensor data sent successfully!");
    Serial.println(dataMessage);  // Print the data for confirmation
  } else {
    Serial.println("Failed to send sensor data");
  }

  delay(5000);  // Delay before the next reading
}
