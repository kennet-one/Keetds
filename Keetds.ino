#include <OneWire.h>
#include <math.h>

// ---- Піни ----
const int TDS_PIN      = 34;  // AOUT TDS модуля (ADC)
const int ONE_WIRE_PIN = 4;   // DATA DS18B20

// ---- OneWire / DS18B20 ----
OneWire oneWire(ONE_WIRE_PIN);

byte dsAddr[8];        // адреса датчика
bool dsFound = false;  // чи знайшли датчик у setup()

// ---- Параметри TDS ----
const float VREF      = 3.3;    // опорна напруга АЦП
const float TDS_K     = 500.0;  // тимчасовий коеф. для "псевдо ppm"
const float TDS_BASE  = 215.0;  // фон від кип’яченої води
const float ALPHA_T   = 0.02;   // 2%/°C температурний коеф.

// ======================= DS18B20 =======================

float readTemperatureC() {
	if (!dsFound) {
		// Якщо датчик не знайшли в setup()
		return NAN;
	}

	byte data[9];

	// 1) Запускаємо вимір
	oneWire.reset();
	oneWire.select(dsAddr);
	oneWire.write(0x44, 1); // 0x44 - команда "конверсія", 1 = parasite power

	// Час конверсії для 12 біта ≈ 750 мс
	delay(750);

	// 2) Читаємо scratchpad
	oneWire.reset();
	oneWire.select(dsAddr);
	oneWire.write(0xBE); // 0xBE - читання scratchpad

	for (int i = 0; i < 9; i++) {
		data[i] = oneWire.read();
	}

	// Перетворення в температуру (DS18B20)
	int16_t raw = (data[1] << 8) | data[0];
	float celsius = (float)raw / 16.0;

	return celsius;
}

// ======================= SETUP =======================

void setup() {
	Serial.begin(115200);
	delay(200);

	// ---- Налаштування АЦП ESP32 ----
	analogReadResolution(12);       // 0..4095
	analogSetAttenuation(ADC_11db); // до ~3.3V на вході

	// ---- Пошук DS18B20 один раз ----
	oneWire.reset_search();
	if (oneWire.search(dsAddr)) {
		// Перевіряємо CRC адреси
		if (OneWire::crc8(dsAddr, 7) == dsAddr[7]) {
			Serial.print("DS18B20 знайдено. ROM: ");
			for (int i = 0; i < 8; i++) {
				if (dsAddr[i] < 16) Serial.print("0");
				Serial.print(dsAddr[i], HEX);
				Serial.print(" ");
			}
			Serial.println();
			dsFound = true;
		} else {
			Serial.println("CRC адреси DS18B20 не співпадає, датчик ігноруємо.");
			dsFound = false;
		}
	} else {
		Serial.println("Датчики DS18B20 не знайдені на шині OneWire.");
		dsFound = false;
	}
}

// ======================= LOOP =======================

void loop() {
	// ---- 1. Читання TDS (сирі значення) ----
	const int samples = 20;
	long sum = 0;

	for (int i = 0; i < samples; i++) {
		sum += analogRead(TDS_PIN);
		delay(10);
	}
	int raw = sum / samples;

	// Переводимо в напругу (приблизно)
	float voltage = (raw / 4095.0) * VREF;  // Вольти на піні

	// Дуже груба оцінка TDS (псевдо ppm)
	float tds_raw = voltage * TDS_K;

	// "ППМ бульйона" – скільки поверх базової води
	float tds_broth_raw = tds_raw - TDS_BASE;
	if (tds_broth_raw < 0) tds_broth_raw = 0; // не даємо йти в мінус

	// ---- 2. Читання температури ----
	float temperature = readTemperatureC();

	// ---- 3. Температурна компенсація до 25°C ----
	float tds_25      = tds_raw;
	float tds_broth25 = tds_broth_raw;

	if (!isnan(temperature)) {
		float factor = 1.0 + ALPHA_T * (temperature - 25.0);
		if (factor <= 0.0) factor = 1.0; // на всякий випадок від дивних значень

		tds_25      = tds_raw      / factor;
		tds_broth25 = tds_broth_raw / factor;
	}

	// ---- 4. Вивід ----
	Serial.println("--------------");
	Serial.print("RAW=");
	Serial.print(raw);
	Serial.print("  U=");
	Serial.print(voltage, 3);
	Serial.print(" V");

	Serial.print("  TDS_raw~");
	Serial.print(tds_raw, 1);
	Serial.print(" ppm");

	Serial.print("  TDS25~");
	Serial.print(tds_25, 1);
	Serial.print(" ppm");

	Serial.print("  TDS_broth_raw~");
	Serial.print(tds_broth_raw, 1);
	Serial.print(" ppm");

	Serial.print("  TDS_broth25~");
	Serial.print(tds_broth25, 1);
	Serial.println(" ppm");

	if (!isnan(temperature)) {
		Serial.print("Температура: ");
		Serial.print(temperature, 2);
		Serial.println(" °C");
	} else {
		Serial.println("Температура: помилка / датчик не знайдено");
	}

	delay(1000);
}
