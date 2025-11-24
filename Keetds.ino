const int TDS_PIN = 34;  // AOUT TDS модуля

void setup() {
	Serial.begin(115200);

	// Налаштування АЦП ESP32
	analogReadResolution(12);              // 0..4095
	analogSetAttenuation(ADC_11db);        // до ~3.3V
}

void loop() {
	const int samples = 20;
	long sum = 0;

	for (int i = 0; i < samples; i++) {
		sum += analogRead(TDS_PIN);
		delay(10);
	}
	int raw = sum / samples;

	// Переводимо в напругу (приблизно)
	float voltage = (raw / 4095.0) * 3.3;  // Вольти на піні

	// Дуже груба оцінка TDS (залежить від модуля!):
	// часто беруть коефіцієнт  (voltage * 1000…700)
	float tds = voltage * 500.0;          // заглушка, потім підженеш калібровкою


	float tds_rel = tds - 215;

	Serial.print("RAW=");
	Serial.print(raw);
	Serial.print("  U=");
	Serial.print(voltage, 3);
	Serial.print(" V  TDS~");
	Serial.print(tds, 1);
	Serial.print(" ppm ");
	Serial.print("TDS-bouillon~");
	Serial.print(tds_rel, 1);
	Serial.println(" ppm");

	delay(1000);
}
