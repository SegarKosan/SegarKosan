// Fallback to older MQ135 wrapper implementation
#ifndef MQ135_WRAPPER_H
#define MQ135_WRAPPER_H

#include <Arduino.h>
#include <MQUnifiedsensor.h>

// Configuration defaults (override these before including this header if needed)
#ifndef MQ135_BOARD
#define MQ135_BOARD "Arduino MEGA 2560"
#endif

#ifndef MQ135_VOLTAGE_RESOLUTION
#define MQ135_VOLTAGE_RESOLUTION 5.0f
#endif

#ifndef MQ135_ADC_BIT_RESOLUTION
#define MQ135_ADC_BIT_RESOLUTION 10
#endif

#ifndef MQ135_ANALOG_PIN
#define MQ135_ANALOG_PIN A0
#endif

#ifndef MQ135_DIGITAL_PIN
#define MQ135_DIGITAL_PIN 3
#endif

#ifndef MQ135_RATIO_CLEAN_AIR
#define MQ135_RATIO_CLEAN_AIR 3.6f // Rs/R0 for clean air (typical)
#endif

class MQ135Sensor {
private:
    MQUnifiedsensor mq;
    int digitalPin;
    bool calibrated;

public:
    MQ135Sensor()
        : mq(MQ135_BOARD, MQ135_VOLTAGE_RESOLUTION, MQ135_ADC_BIT_RESOLUTION, MQ135_ANALOG_PIN, "MQ-135"),
          digitalPin(MQ135_DIGITAL_PIN),
          calibrated(false) {}

    // Keep the sensor energized in clean air so the baseline stabilizes before calibration
    // Optional callback is called on every loop iteration (approx every updateIntervalMs)
    void preheat(unsigned long durationMs = 30000, unsigned long updateIntervalMs = 50, void (*callback)(int remainingSec) = nullptr) {
        unsigned long start = millis();
        
        while (millis() - start < durationMs) {
            mq.update();
            
            if (callback) {
                int remaining = (durationMs - (millis() - start)) / 1000;
                if (remaining < 0) remaining = 0;
                callback(remaining);
                if (remaining == 0) {
                    break;
                }
            }
            
            delay(updateIntervalMs);
        }
    }

    // Perform basic initialization and calibration. Place the sensor in clean air.
    void begin(unsigned long calibrationSamples = 10, unsigned long sampleIntervalMs = 100) {
        pinMode(digitalPin, INPUT);
        mq.setRegressionMethod(1); // Exponential curve
        mq.init();

        float r0 = 0.0f;
        for (unsigned long i = 0; i < calibrationSamples; i++) {
            mq.update();
            r0 += mq.calibrate(MQ135_RATIO_CLEAN_AIR);
            delay(sampleIntervalMs);
        }
        r0 /= (float)calibrationSamples;
        mq.setR0(r0);
        calibrated = true;
    }

    // Update the sensor reading (reads the analog voltage internally)
    void update() {
        mq.update();
    }

    // Convenience getters for different target gases (ppm). Choose the one you need.
    float readCO2() {
        // Common MQ-135 CO2 approximation curve
        mq.setA(110.47f); mq.setB(-2.862f);
        return mq.readSensor();
    }

    float readNH3() {
        mq.setA(102.2f); mq.setB(-2.473f);
        return mq.readSensor();
    }

    float readAlcohol() {
        mq.setA(77.255f); mq.setB(-3.18f);
        return mq.readSensor();
    }

    float readCO() {
        mq.setA(605.18f); mq.setB(-3.937f);
        return mq.readSensor();
    }

    float readToluene() {
        mq.setA(44.947f); mq.setB(-3.445f);
        return mq.readSensor();
    }

    float readAcetone() {
        mq.setA(34.668f); mq.setB(-3.369f);
        return mq.readSensor();
    }

    // Digital output from the module's onboard comparator (threshold adjustable via potentiometer)
    bool isAboveThreshold() const {
        return digitalRead(digitalPin) == HIGH;
    }

    bool isCalibrated() const { return calibrated; }
    float getR0() { return mq.getR0(); }

    // --- FUNGSI ANALISIS KUALITAS UDARA BARU ---

    /**
     * Menghitung Skor Bau (0-100) berdasarkan 4 parameter.
     * Weights: CO2(40%), Temp(20%), Hum(20%), Raw(20%)
     */
    int calculateOdorScore(float co2, float temp, float hum, int raw_mq) {
        // 1. CO2 Score (40%)
        // Baseline 400ppm (Fresh) -> 2000ppm (Bad)
        int co2Score = map(constrain((int)co2, 400, 2000), 400, 2000, 0, 100);

        // 2. Temperature Deviation Score (20%)
        // Ideal 25C. Deviation > 7 (i.e., <18 or >32) is Bad (100)
        float tempDiff = abs(temp - 25.0);
        int tempScore = map(constrain((int)(tempDiff * 10), 0, 70), 0, 70, 0, 100);

        // 3. Humidity Deviation Score (20%)
        // Ideal 50%. Deviation > 20 (i.e., <30 or >70) is Bad (100)
        float humDiff = abs(hum - 50.0);
        int humScore = map(constrain((int)humDiff, 0, 20), 0, 20, 0, 100);

        // 4. Raw Sensor Score (20%)
        // Raw < 200 (Clean), Raw > 700 (Polluted)
        int rawScore = map(constrain(raw_mq, 200, 700), 200, 700, 0, 100);

        // Weighted Average
        // CO2 (40%) + Temp (20%) + Hum (20%) + Raw (20%)
        int finalScore = (co2Score * 0.40) + (tempScore * 0.20) + (humScore * 0.20) + (rawScore * 0.20);
        
        return constrain(finalScore, 0, 100);
    }

    /**
     * Mengembalikan interpretasi teks berdasarkan Odor Score.
     * Range: 0-30 (Fresh), 31-50 (Good), 51-70 (Moderate), 71-90 (Poor), 91-100 (Critical)
     */
    String getScoreDescription(int score) {
        if (score <= 30) return "Fresh air, excellent quality 🟢";
        if (score <= 50) return "Good air quality, comfortable 🟡";
        if (score <= 70) return "Moderate, consider ventilation 🟠";
        if (score <= 90) return "Poor quality, ventilation needed 🔴";
        return "Critical, immediate action required ⚠️";
    }

    /**
     * Menentukan jenis bau/kondisi dominan (Source detection).
     */
    String detectOdorType(int raw_mq, float hum) {
        // Prioritas deteksi berdasarkan Raw MQ (Gas) dan Humidity
        if (raw_mq > 650) return "SAMPAH/BAHAYA"; // Polutan tinggi
        if (raw_mq > 450) return "ASAP";          // Indikasi asap
        
        // Deteksi kelembaban tinggi jika udara relatif bersih
        if (hum > 75 && raw_mq < 350) return "LEMBAB"; 
        
        if (raw_mq < 250) return "FRESH";         // Udara bersih
        
        return "NORMAL";
    }
};

#endif // MQ135_WRAPPER_H