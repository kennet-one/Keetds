#pragma once
#include <painlessMesh.h>
#include <stdlib.h>   // strtol
#include <stdio.h>    // snprintf

// УВАГА: реальний екземпляр оголошуй у .ino (див. garland.ino)
extern painlessMesh mesh;

// ===================== CRC-8 Dallas =====================
// poly 0x31 (reflected 0x8C), init 0x00
static inline uint8_t crc8_dallas(const uint8_t* data, size_t len){
	uint8_t crc = 0;
	while (len--){
		uint8_t inbyte = *data++;
		for (uint8_t i = 8; i; --i){
			uint8_t mix = (crc ^ inbyte) & 0x01;
			crc >>= 1;
			if (mix) crc ^= 0x8C;
			inbyte >>= 1;
		}
	}
	return crc;
}

// '*XX' уже є у кінці?
static inline bool hasStarCrc(const String& s){
	int p = s.lastIndexOf('*');
	if (p < 0 || p + 3 != (int)s.length()) return false;
	auto isHex = [](char c){
		return (c>='0'&&c<='9')||(c>='A'&&c<='F')||(c>='a'&&c<='f');
	};
	return isHex(s[p+1]) && isHex(s[p+2]);
}

// Додати "*XX" (HEX uppercase) у кінець
static inline String addCrcStar(const String& s){
	const uint8_t c = crc8_dallas((const uint8_t*)s.c_str(), s.length());
	char buf[3]; // "XX"+'\0'
	snprintf(buf, sizeof(buf), "%02X", c);
	return s + "*" + String(buf);
}

// Зняти та перевірити CRC; out = тіло БЕЗ "*XX"
static inline bool stripAndVerifyCrcStar(const String& in, String& out){
	int p = in.lastIndexOf('*');
	if (p < 0 || p+3 != (int)in.length()){ out = in; return false; }
	String hex = in.substring(p+1);
	char* endp = nullptr;
	long v = strtol(hex.c_str(), &endp, 16);
	if (!hex.length() || endp==nullptr || *endp!='\0' || v<0 || v>255){ out = in; return false; }
	String body = in.substring(0,p);
	const uint8_t calc = crc8_dallas((const uint8_t*)body.c_str(), body.length());
	out = body;
	return calc == (uint8_t)v;
}

// ===================== Безпечні send-обгортки =====================
static inline void sendB(const String& s){
	const String out = hasStarCrc(s) ? s : addCrcStar(s);
	mesh.sendBroadcast(out);
}
static inline void sendS(uint32_t to, const String& s){
	const String out = hasStarCrc(s) ? s : addCrcStar(s);
	mesh.sendSingle(to, out);
}

// ===================== Діагностика CRC =====================
static volatile uint32_t CRC_OK = 0, CRC_BAD = 0, CRC_NOSTAR = 0;
static uint32_t          CRC_LAST_FROM = 0;
static String            CRC_LAST_BAD  = "";
static String            CRC_LAST_NOST = "";

// ===================== Дефер-черга (з адресою) =====================
static constexpr uint8_t QN = 8;   // елементів
static constexpr uint8_t QL = 64;  // макс. довжина (вкл. '\0')

struct QItem {
	uint32_t from;
	char     body[QL];
};
static QItem          inQ[QN];
static volatile uint8_t qh = 0, qt = 0;

static inline uint8_t qInc(uint8_t i){ return (uint8_t)((i+1) % QN); }
static inline bool     qEmpty(){ return qh == qt; }

static inline void qPush2(uint32_t from, const String& s){
	const uint8_t n = qInc(qh);
	if (n == qt) qt = qInc(qt); // overwrite oldest
	size_t len = s.length(); if (len > QL-1) len = QL-1;
	inQ[qh].from = from;
	memcpy(inQ[qh].body, s.c_str(), len);
	inQ[qh].body[len] = 0;
	qh = n;
}
static inline bool qPop2(uint32_t& from, String& out){
	if (qEmpty()) return false;
	from = inQ[qt].from;
	out  = String(inQ[qt].body);
	qt = qInc(qt);
	return true;
}
// Сумісність (тільки тіло)
static inline bool qPop(String& out){
	uint32_t dummy; return qPop2(dummy, out);
}

// ===================== Колбек прийому (строгий) =====================
static inline void receivedCallback(uint32_t from, String &msg){
	if (from == mesh.getNodeId()) return; // ігнор власних
	String body;
	if (stripAndVerifyCrcStar(msg, body)){
		CRC_OK++; CRC_LAST_FROM = from;
		qPush2(from, body);
		return;
	}
	// Діагностика
	if (msg.indexOf('*') < 0){
		CRC_NOSTAR++; CRC_LAST_NOST = msg;
	} else {
		CRC_BAD++;    CRC_LAST_BAD  = msg;
	}
	// некоректні пакети — дроп
}
