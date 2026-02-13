#include <SPI.h>
#include <LoRa.h>

#define SS 5
#define RST 14
#define DIO0 2

#define BUZZER_PIN 26
#define LED_PIN 25

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Initialisation broches LED et buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(LED_PIN, OUTPUT);

  digitalWrite(BUZZER_PIN, LOW);
  digitalWrite(LED_PIN, LOW);

  // LoRa init
  LoRa.setPins(SS, RST, DIO0);

  if (!LoRa.begin(433E6)) {
    Serial.println("❌ Erreur init LoRa !");
    while (1);
  }

  Serial.println("✅ LoRa récepteur prêt !");
}

void loop() {
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    String message = "";

    while (LoRa.available()) {
      message += (char)LoRa.read();
    }

    Serial.print("📩 Reçu : ");
    Serial.println(message);

    if (message == "CHUTE:1") {
      Serial.println("🚨 ALARME : Chute détectée !");

      for (int i = 0; i < 3; i++) {
        // LED ON + Buzzer bip
        digitalWrite(LED_PIN, HIGH);
        tone(BUZZER_PIN, 2000); // 2kHz
        delay(200);

        // LED OFF + stop buzzer
        digitalWrite(LED_PIN, LOW);
        noTone(BUZZER_PIN);
        delay(200);
      }
    }
  }
}