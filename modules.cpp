//battery
//PION
//ESP32
//StepDown

//BNO055
//BMP280
//NRF24L01
//SD Module

#include "BNO055ESP32.h";
#include "freertos/FreeRTOS.h";
#include "freertos/task.h";
#include "servoControl.h"

extern "C" void app_main(){
    
    //Variável de estado do servo (abertura do paraquedas)
    myServo.write(i);
	TaskDelay(10 / portTICK_RATE_MS);

    //Definindo o servo
    servoControl myServo;
	myServo.attach(GPIO_NUM_23);
    myServo.write(0);
	vTaskDelay(1000 / portTICK_RATE_MS);

    // Calibrando o BBNO:
    // bno055_offsets_t storedOffsets;
    // storedOffsets.accelOffsetX = 29;
    // storedOffsets.accelOffsetY = 24;
    // storedOffsets.accelOffsetZ = 16;
    // storedOffsets.magOffsetX = -243;
    // storedOffsets.magOffsetY = -420;
    // storedOffsets.magOffsetZ = -131;
    // storedOffsets.gyroOffsetX = 1;
    // storedOffsets.gyroOffsetY = -1;
    // storedOffsets.gyroOffsetZ = 0;
    // storedOffsets.accelRadius = 0;
    // storedOffsets.magRadius = 662;

    //Relizar comunicação com BNO: Conferir se é GPIO ou UART

    //inicializando o BNO e retornando erros caso não seja possível
    try{
        bno.begin(); // Inicia no modo de configuração
        bno.enableExternalCrystal();
        bno.setOprModeNdof();
        ESP_LOGI(TAG, "Configuração concluída");
    } catch(BNO055BaseException& ex) {
        ESP_LOGE(TAG, "Falha na configuração, Erro: %s", ex.what());
        return;
    } catch(std::exception& ex){
        ESP_LOGE(TAG, "Falha na configuração, Erro %s", ex.what());
        return; //Ver possibilidade de juntar em um catch só
    }

    //Registrando dados do BNO
     try {
        int8_t temperature = bno.getTemp();
        ESP_LOGI(TAG, "TEMP: %d°C", temperature);

        int16_t sw = bno.getSWRevision();
        uint8_t bl_rev = bno.getBootloaderRevision();
        ESP_LOGI(TAG, "SW rev: %d, bootloader rev: %u", sw, bl_rev);

        bno055_self_test_result_t res = bno.getSelfTestResult();
        ESP_LOGI(TAG, "Self-Test Results: MCU: %u, GYR:%u, MAG:%u, ACC: %u", res.mcuState, res.gyrState, res.magState,
                 res.accState);
    } catch (BNO055BaseException& ex) {  // see BNO055ESP32.h for more details about exceptions
        ESP_LOGE(TAG, "Something bad happened: %s", ex.what());
        return;
    } catch (std::exception& ex) {
        ESP_LOGE(TAG, "Something bad happened: %s", ex.what());
        return;
    }

    // Equivalente ao void loop (uma rotina ativa durante toda a execução)
    while (1) {

        if(abrir_paraquedas){
        	myServo.write(90);
			vTaskDelay(10 / portTICK_RATE_MS);
        }

        //Checando dados de calibragem do BNO
        try {
            // Calibration 3 = fully calibrated, 0 = not calibrated
            bno055_calibration_t cal = bno.getCalibration();
            bno055_vector_t v = bno.getVectorEuler();
            ESP_LOGI(TAG, "Euler: X: %.1f Y: %.1f Z: %.1f || Calibration SYS: %u GYRO: %u ACC:%u MAG:%u", v.x, v.y, v.z, cal.sys,
                     cal.gyro, cal.accel, cal.mag);
        } catch (BNO055BaseException& ex) {
            ESP_LOGE(TAG, "Something bad happened: %s", ex.what());
            return;
        } catch (std::exception& ex) {
            ESP_LOGE(TAG, "Something bad happened: %s", ex.what());
        }
        vTaskDelay(100 / portTICK_PERIOD_MS);  // in fusion mode max output rate is 100hz (actual rate: 100ms (10hz))
    }

}

