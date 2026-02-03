#include <DHT.h>  // ← SOLO ESTE include

#define DHTPIN 4
#define DHTTYPE DHT22

DHT dht(DHTPIN, DHTTYPE);

void setup() {
  Serial.begin(74880);
  delay(2000);
  Serial.println("\n=== DHT22 Iniciado ===");
  dht.begin();
  delay(1000);
  Serial.println("Ready to read data...\n");
}

void loop() {
  delay(1000);
  
  float hum = dht.readHumidity();
  float temp = dht.readTemperature();
  
  if (isnan(hum) || isnan(temp)) {
    Serial.println("ERROR: Fail reading DHT22");
    return;
  }
  
  Serial.print("{temperature:");
  Serial.print(temp);
  Serial.print(", humidity:");
  Serial.print(hum);
  Serial.println("}");
}
