#include <OneWire.h>
#include <math.h>

#include <painlessMesh.h>
#include "mash_parameter.h"
#include "CRC.h"   // sendB(), receivedCallback(), qPop(...)

// ==== Mesh ====
Scheduler    userScheduler;
painlessMesh mesh;

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
const float TDS_BASE  = 180.0;  // фон від кип’яченої води
const float ALPHA_T   = 0.02;   // 2%/°C температурний коеф.

// ======================= DS18B20: неблокуюча стейт-машина =======================

enum DsState : uint8_t { DS_IDLE, DS_CONVERTING };

DsState   dsState           = DS_IDLE;
float     lastTempC         = NAN;
uint32_t  dsConvStartMs     = 0;
uint32_t  dsLastReadMs      = 0;
const uint32_t DS_PERIOD_MS = 5000;    // як часто оновлювати температуру (5 сек)

// один крок обробки DS18B20, викликати часто з loop()
void updateDs18b20() {
	if (!dsFound) return;

	uint32_t now = millis();

	switch (dsState) {
		case DS_IDLE:
			// час запускати нову конверсію?
			if (now - dsLastReadMs >= DS_PERIOD_MS) {
				oneWire.reset();
				oneWire.select(dsAddr);
				oneWire.write(0x44, 1); // start conversion, parasite power
				dsConvStartMs = now;
				dsState = DS_CONVERTING;
			}
			break;

		case DS_CONVERTING:
			// чекаємо ≥750 мс не блокуючи
			if (now - dsConvStartMs >= 750) {
				byte data[9];
				oneWire.reset();
				oneWire.select(dsAddr);
				oneWire.write(0xBE); // read scratchpad

				for (int i = 0; i < 9; i++) {
					data[i] = oneWire.read();
				}

				int16_t raw = (data[1] << 8) | data[0];
				lastTempC   = (float)raw / 16.0;
				dsLastReadMs = now;
				dsState      = DS_IDLE;
			}
			break;
	}
}

// ======================= TDS: неблокуюче вимірювання по запиту =======================

const int      TDS_SAMPLES            = 20;    // скільки семплів на одне вимірювання
const uint32_t TDS_SAMPLE_INTERVAL_MS = 50;    // інтервал між семплами

long     tdsSum          = 0;
int      tdsSampleCount  = 0;
uint32_t tdsLastSampleMs = 0;

bool     tdsActive       = false;    // зараз йде цикл вимірювання TDS
bool     sendOnComplete  = false;    // чи слати результат в mesh після завершення

// збережені результати (для логів / дебагу)
int   lastRaw           = 0;
float lastVoltage       = 0;
float lastTdsRaw        = 0;
float lastTds25         = 0;
float lastTdsBrothRaw   = 0;
float lastTdsBroth25    = 0;

// раз на годину
const uint32_t MEASURE_PERIOD_MS = 3600000UL;    // 1 година
uint32_t lastPeriodicMeasureMs   = 0;

// старт вимірювання TDS
void startTdsMeasurement(bool sendAfter) {
	if (tdsActive) {
		// якщо вже міряємо — просто гарантуємо, що після завершення буде відправка
		if (sendAfter) sendOnComplete = true;
		return;
	}
	tdsActive       = true;
	sendOnComplete  = sendAfter;
	tdsSum          = 0;
	tdsSampleCount  = 0;
	tdsLastSampleMs = 0;   // щоб перший семпл пройшов одразу
}

// один крок TDS: читаємо по одному семплу, раз у 50 мс
void updateTds() {
	if (!tdsActive) return;

	uint32_t now = millis();

	if (now - tdsLastSampleMs < TDS_SAMPLE_INTERVAL_MS) return;
	tdsLastSampleMs = now;

	int raw = analogRead(TDS_PIN);
	tdsSum += raw;
	tdsSampleCount++;

	if (tdsSampleCount < TDS_SAMPLES) return;

	// є повний набір семплів — рахуємо
	int avgRaw = tdsSum / tdsSampleCount;

	tdsSum         = 0;
	tdsSampleCount = 0;
	tdsActive      = false;

	// Переводимо в напругу (приблизно)
	float voltage = (avgRaw / 4095.0f) * VREF;  // Вольти на піні

	// Дуже груба оцінка TDS (псевдо ppm)
	float tds_raw = voltage * TDS_K;

	// "ППМ бульйона" – скільки поверх базової води
	float tds_broth_raw = tds_raw - TDS_BASE;
	if (tds_broth_raw < 0) tds_broth_raw = 0; // не даємо йти в мінус

	// Температурна компенсація до 25°C
	float tds_25      = tds_raw;
	float tds_broth25 = tds_broth_raw;

	if (!isnan(lastTempC)) {
		float factor = 1.0f + ALPHA_T * (lastTempC - 25.0f);
		if (factor <= 0.0f) factor = 1.0f;
		tds_25      = tds_raw       / factor;
		tds_broth25 = tds_broth_raw / factor;
	}

	// зберігаємо результати
	lastRaw         = avgRaw;
	lastVoltage     = voltage;
	lastTdsRaw      = tds_raw;
	lastTds25       = tds_25;
	lastTdsBrothRaw = tds_broth_raw;
	lastTdsBroth25  = tds_broth25;

	// ---- Вивід у Serial (для дебагу) ----
	Serial.println("--------------");
	Serial.print("RAW=");
	Serial.print(avgRaw);
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

	if (!isnan(lastTempC)) {
		Serial.print("Температура: ");
		Serial.print(lastTempC, 2);
		Serial.println(" °C");
	} else {
		Serial.println("Температура: помилка / датчик не знайдено");
	}

	// ---- Відправка в Mesh через CRC-шар (якщо треба) ----
	if (sendOnComplete) {
		String msg = "TDSB" + String((float)round(lastTdsBroth25));
		String ttds = "ttds" + String((float)round(lastTempC));
		String tds = "TDS" + String((float)round(tds_25));

		sendB(msg); 
		sendB(ttds);   
		sendB(tds);   
		sendOnComplete = false;
	}
}

// ======================= Обробка команд із mesh =======================

// тут прилітає вже "чисте" тіло без "*CRC"
void handleBody(const String &body) {
	String cmd = body;
	cmd.trim();
	if (!cmd.length()) return;

	// команда на оновлення вимірювання й негайну відправку
	if (cmd == "readtds") {
		Serial.println("[CMD] readtds -> startTdsMeasurement");
		startTdsMeasurement(true);   // позачерговий замір + відправка після завершення
		return;
	}

	// тут можна додати інші команди, якщо згодом захочеш
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

	// ---- Mesh + CRC ----
	mesh.setDebugMsgTypes(ERROR | STARTUP);
	mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);
	mesh.onReceive(&receivedCallback);  // з CRC.h

	// щоб перший замір був майже одразу після старту:
	lastPeriodicMeasureMs = millis() - MEASURE_PERIOD_MS;
}

// ======================= LOOP =======================
void loop() {
	// 1) Обов'язково оновлюємо mesh
	mesh.update();

	// 2) Обробляємо вхідну mesh-чергу (CRC.h: qPop)
	for (uint8_t i = 0; i < 4; ++i) {
		String body;
		if (!qPop(body)) break;
		handleBody(body);
	}

	// 3) Оновлюємо температуру
	updateDs18b20();

	// 4) Підтримуємо TDS-вимірювання (якщо активне)
	updateTds();

	// 5) Періодичний (раз на годину) запуск вимірювання
	uint32_t now = millis();
	if (!tdsActive && (now - lastPeriodicMeasureMs >= MEASURE_PERIOD_MS)) {
		lastPeriodicMeasureMs = now;
		Serial.println("[AUTO] hourly TDS measurement");
		startTdsMeasurement(true);   // раз в годину міряємо й шлемо результат
	}
}
