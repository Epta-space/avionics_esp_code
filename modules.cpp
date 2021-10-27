// ---------------------------------------------------------------------------------------------------------Incluindo bibliotecas------------------------------------------------------------------------------------------------
//Bibliotecas do Cartão SD
// #include <SPI.h>
// #include <SD.h>

//Bibliotecas padrão C 
#include <stdio.h> //
#include <stdlib.h> //
#include <string.h> //Manipulação de strings
#include <time.h>

// Biblioteca NRF24L01
#include "mirf.h"

// Biblioteca de gerenciamento de tarefas - freeRTOS
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Biblioteca para logs (prints de informação no console)
#include "esp_log.h"

// Biblioteca do BNO055
#include "BNO055ESP32.h"

// Biblioteca Servo
#include "servoControl.h"

// Bibliotecas BMP280
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>


//* ------------------------------------------------------------------------------------------------------Definindo algumas variáveis------------------------------------------------------------------------------------------
//Criando um arquivo para salvar os dados
File myFile;

//Definindo Pinos BMP
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10

//Definindo classe MYDATA_t para utilização no rádio
typedef union {
  uint8_t value[4];
  unsigned long now_time;
} MYDATA_t;

//Criando uma variável do tipo MYDATA_t
MYDATA_t mydata;

TaskHandle_t xHandle;

//*----------------------------------------------------------------------------------------------Código rádio -----------------------------------------------------------------------------------------------------------------
//Configuração Receiver
if(CONFIG_RECEIVER){
void receiver(void *pvParameters)
{
	NRF24_t dev;

	ESP_LOGI(pcTaskGetTaskName(0), "Start");
	ESP_LOGI(pcTaskGetTaskName(0), "CONFIG_CE_GPIO=%d",CONFIG_CE_GPIO);
	ESP_LOGI(pcTaskGetTaskName(0), "CONFIG_CSN_GPIO=%d",CONFIG_CSN_GPIO);
	spi_master_init(&dev, CONFIG_CE_GPIO, CONFIG_CSN_GPIO, CONFIG_MISO_GPIO, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO);

	Nrf24_setRADDR(&dev, (uint8_t *)"FGHIJ");
	uint8_t payload = sizeof(mydata.value);
	uint8_t channel = 90;
	Nrf24_config(&dev, channel, payload);
	Nrf24_printDetails(&dev);
	ESP_LOGI(pcTaskGetTaskName(0), "Listening...");

	while(1) {
		if (Nrf24_dataReady(&dev)) { //When the program is received, the received data is output from the serial port
			Nrf24_getData(&dev, mydata.value);
			ESP_LOGI(pcTaskGetTaskName(0), "Got data:%lu", mydata.now_time);
		}
		vTaskDelay(1);
	}
}
}

//Confiração transmitter
if(CONFIG_TRANSMITTER){
void transmitter(void *pvParameters)
{
	NRF24_t dev;

	ESP_LOGI(pcTaskGetTaskName(0), "Start");
	ESP_LOGI(pcTaskGetTaskName(0), "CONFIG_CE_GPIO=%d",CONFIG_CE_GPIO);
	ESP_LOGI(pcTaskGetTaskName(0), "CONFIG_CSN_GPIO=%d",CONFIG_CSN_GPIO);
	spi_master_init(&dev, CONFIG_CE_GPIO, CONFIG_CSN_GPIO, CONFIG_MISO_GPIO, CONFIG_MOSI_GPIO, CONFIG_SCLK_GPIO);

	Nrf24_setRADDR(&dev, (uint8_t *)"ABCDE");
	uint8_t payload = sizeof(mydata.value);
	uint8_t channel = 90;
	Nrf24_config(&dev, channel, payload);
	Nrf24_printDetails(&dev);

	while(1) {
		mydata.now_time = xTaskGetTickCount();
		Nrf24_setTADDR(&dev, (uint8_t *)"FGHIJ");			//Set the receiver address
		Nrf24_send(&dev, mydata.value);				   //Send instructions, send random number value
		vTaskDelay(1);
		ESP_LOGI(pcTaskGetTaskName(0), "Wait for sending.....");
		if (Nrf24_isSend(&dev)) {
			ESP_LOGI(pcTaskGetTaskName(0),"Send success:%lu", mydata.now_time);
		} else {
			ESP_LOGI(pcTaskGetTaskName(0),"Send fail:");
		}
		vTaskDelay(1000/portTICK_PERIOD_MS);
	}
}
}

//Executando tarefas do rádio
#define tag "MIRF"
void app_main(void)
{
    if (CONFIG_RECEIVER){
        xTaskCreate(receiver, "RECV", 1024*2, NULL, 2, NULL);
    }

    if (CONFIG_TRANSMITTER){
        xTaskCreate(transmitter, "TRANS", 1024*2, NULL, 2, NULL);

    }
}

void waitFor(unsigned int secs) {
    unsigned int retTime = time(0) + secs;   
    while (time(0) < retTime);             
}

void dispara_paraquedas() {
    myServo.write(90);
	vTaskDelay(10 / portTICK_RATE_MS);
}

//*-------------------------------------------------------------------------------------------Criação de rotina ---------------------------------------------------------------------------------------------------------
extern "C" void app_main(){
    
    bool start_flag, sensor_flag;

    ESP_LOGI(TAG, "Etapa de boot iniciada");
    start_execution = clock();
    while(start_flag == false){
        if((long int)(clock() - start_execution) > 60000 || telemetry_command()){
            start_flag = true;
        }
    }
    ESP_LOGI(TAG, "Etapa de boot concluída");
    ESP_LOGI(TAG, "Etapa de testes individuais dos componentes iniciada");
    //Iniciando testes dos sensores e equipamentos
    //* -----SD CARD --------
    //inicializando SD Card
    ESP_LOGI(TAG, "Initializing SD card...");
    if (!SD.begin(10)) {
    ESP_LOGI(TAG, "initialization failed!");
    while (1);
    }
    ESP_LOGI("initialization done.");

    //Abrindo o arquivo para salvar infos
    myFile = SD.open("test.txt", FILE_WRITE);

    // Se o arquivo for inicializado, faz-se o seguinte teste
    if (myFile) {
    ESP_LOGI(TAG, "Writing to test.txt...");
    myFile.println("testing 1, 2, 3.");
    }


    //* ----- SERVO -----
    //Variável de estado do servo (abertura do paraquedas)
    myServo.write(i);
	TaskDelay(10 / portTICK_RATE_MS);

    //Definindo o servo
    servoControl myServo;
	myServo.attach(GPIO_NUM_23);
    myServo.write(0);
	vTaskDelay(1000 / portTICK_RATE_MS);


    //* ----- BNO055 --------
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
    
     
    if (!bme.begin()) {
        ESP_LOGI(TAG, "Não foi possível encontrar uma conexão válida");
    }

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

    ESP_LOGI(TAG, "Etapa de testes individuais dos componentes concluídos");

//*------------------------------------------------------------------------------------------------Tarefas em loop ----------------------------------------------------------------------------------------------------
    // Equivalente ao void loop (uma rotina ativa durante toda a execução)

     void reset_listener(void * parameters){
        for(;;){
            if(soft_reset()){
            vTaskSuspend(xHandle);
            xTaskCreate(general_execution, "EXC", 1024*2, NULL, 1, NULL);
            }
        }
    }
    

    void general_execution(void * parameters){
        for(;;){
            //Coletando dados do BMP
            temp = bme.readTemperature();
            press = bme.readPressure();
            alt = bme.readAltitude(1013.25);

            //adquirindo dados do BNO e validando seus dados
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

            estado = abrir_paraquedas();
            if(estado == true){
                try{
                    dispara_paraquedas();
                } catch {
                    ESP_LOGE(TAG, "Recovery not worked, Retrying...");
                    dispara_paraquedas();
                }
        	    
            }

            myFile.println(informações);//Salvando informações no cartão

            vTaskDelay(100 / portTICK_PERIOD_MS);  // in fusion mode max output rate is 100hz (actual rate: 100ms (10hz))

            ESP_LOGI(TAG, "TODAS AS INFORMAÇõES JUNTAS");

            if(fim_do_voo){
                myFile.close();  // fecha o arquivo e salva as informações
                break;
            }
        }
    }

    xTaskCreate(general_execution, "EXC", 1024*2, NULL, 1, xHandle);
    xTaskCreate(reset_listener, "RES", 1024*2, NULL, 1, NULL);
    
}

