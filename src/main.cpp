/*
*****************************************************
*
*       Программа управления модулем подсветки
*         на ESP32 c датчиком жестов PAJ7620
*
*               (с) 2023, by Dr@Cosha              
*
*****************************************************
*/

#include <Arduino.h>
#include <WiFi.h>

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

#include <AsyncMqttClient.h>
#include <paj7620.h>

// ----------- режим компиляции для отладки с выводом в порт -----------
#define DEBUG_IN_SERIAL
// ---------------------------------------------------------------------

#define WIFI_SSID "iot_ls"                          // SSID нашей локальной сети  
#define WIFI_PASSWORD "vvssoft40"                   // пароль к нашей локальной сети
#define MQTT_USER "mqtt_user"                       // имя пользователя для подключения к MQTT серверу
#define MQTT_PWD "vvssoft40"                        // пароль для подключения к MQTT серверу
#define MQTT_HOST IPAddress(192, 168, 10, 100)      // адрес нашего Mosquito MQTT сервера

#define LWT_TOPIC   "diy/blm32_kitchen/LWT"         // топик публикации доступности устройства
#define SET_TOPIC   "diy/blm32_kitchen/set"         // топик публикации команд для устройства
#define STATE_TOPIC "diy/blm32_kitchen/state"       // топик публикации состояния устройства

#define _SCL 22                                     // назначение выводов линии i2c
#define _SDA 21                                     //

// создаем объекты для управления MQTT-клиентом и WiFi соединением
AsyncMqttClient mqttClient;                         // MQTT клиент
TimerHandle_t mqttReconnectTimer;                   // таймер повторной попытки установки MQTT соединения
TimerHandle_t wifiReconnectTimer;                   // таймер повторной попытки установки WiFi соединения

// назначаем GPIO контакты для устройств
const int ledInd = 2;                               // выход управления индикаторным светодиодом
const int buttonPin = 19;                           // вход внешней кнопки
const int ledCh1 = 5;                               // выход на канал управления LED1
const int ledCh2 = 4;                               // выход на канал управления LED2

// набор обработчиков событий для MQTT клиента 
void connectToWifi() {
#ifdef DEBUG_IN_SERIAL                            
  Serial.println("Try to connect Wi-Fi...");
#endif
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);             // запуск соединения с WiFi
}

void connectToMqtt() {
#ifdef DEBUG_IN_SERIAL                            
  Serial.println("Try to connect MQTT server...");
#endif
  mqttClient.connect();                             // запуск соединения с MQTT 
}

void WiFiEvent(WiFiEvent_t event) {
#ifdef DEBUG_IN_SERIAL                              
  Serial.printf("[WiFi-event] event: %d\n", event);
#endif  
  switch(event) {                                   // обработка событий WiFi соединения             
    case SYSTEM_EVENT_STA_GOT_IP:                   // если получили IP:
#ifdef DEBUG_IN_SERIAL                                  
      Serial.println("WiFi connected");  
      Serial.println("IP address: ");  
      Serial.println(WiFi.localIP());
#endif        
      connectToMqtt();                              // соединяемся с MQTT
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:             // если произошел разрыв соединения
#ifdef DEBUG_IN_SERIAL                                  
      Serial.println("WiFi lost connection");
#endif              
                                                    // делаем так, чтобы ESP32 не переподключалась к MQTT во время переподключения к WiFi:      
      xTimerStop(mqttReconnectTimer, 0);            // останавливаем таймер переподключения к MQTT
      xTimerStart(wifiReconnectTimer, 0);           // запускаем таймер переподключения к WiFi
      break;
  }
}

// --- в этом фрагменте добавляем топики, на которые будет подписываться ESP32: SET_TOPIC
void onMqttConnect(bool sessionPresent) { 
#ifdef DEBUG_IN_SERIAL                                    
  Serial.println("Connected to MQTT.");  //  "Подключились по MQTT."
  Serial.print("Session present: ");  //  "Текущая сессия: "
  Serial.println(sessionPresent);
#endif                
                                                                     // далее подписываем ESP32 на набор необходимых для управления топиков:
  uint16_t packetIdSub = mqttClient.subscribe(SET_TOPIC, 0);         // подписываем ESP32 на топик SET_TOPIC

#ifdef DEBUG_IN_SERIAL                                      
  Serial.print("Subscribing at QoS 0, packetId: ");
  Serial.println(packetIdSub);
  Serial.print("Topic: ");
  Serial.println(SET_TOPIC);
#endif                  
}

void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
#ifdef DEBUG_IN_SERIAL                                        
  Serial.println("Disconnected from MQTT.");                        // если отключились от MQTT
#endif                               
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);                             // запускаем таймер переподключения к MQTT
  }
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
#ifdef DEBUG_IN_SERIAL   
  Serial.println("Subscribe acknowledged.");                        // подписка подтверждена
  Serial.print("  packetId: ");                                     // 
  Serial.println(packetId);                                         // выводим ID пакета
  Serial.print("  qos: ");                                          // 
  Serial.println(qos);                                              // выводим значение QoS
#endif                   
}

void onMqttUnsubscribe(uint16_t packetId) {
#ifdef DEBUG_IN_SERIAL     
  Serial.println("Unsubscribe acknowledged.");                      // отписка подтверждена
  Serial.print("  packetId: ");                                     //
  Serial.println(packetId);                                         // выводим ID пакета
#endif                     
}

void onMqttPublish(uint16_t packetId) {
#ifdef DEBUG_IN_SERIAL     
  Serial.println("Publish acknowledged.");                          // публикация подтверждена
  Serial.print("  packetId: ");                                     //
  Serial.println(packetId);                                         // выводим ID пакета
#endif                     
}


// TODO: --------------------------- дальше правим


// этой функцией управляется то, что происходит
// при получении того или иного сообщения в топике «esp32/led»;
// (если хотите, можете ее отредактировать):
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  String messageTemp;
  for (int i = 0; i < len; i++) {
    //Serial.print((char)payload[i]);
    messageTemp += (char)payload[i];
  }
/*  
  // проверяем, получено ли MQTT-сообщение в топике «esp32/led»:
  if (strcmp(topic, "esp32/led") == 0) {
    // если светодиод выключен, включаем его (и наоборот):
    if (ledState == LOW) {
      ledState = HIGH;
    } else {
      ledState = LOW;
    }
    // задаем светодиоду значение из переменной «ledState»:
    digitalWrite(ledPin, ledState);
  }
 */

  Serial.println("Publish received.");
             //  "Опубликованные данные получены."
  Serial.print("  message: ");  //  "  сообщение: "
  Serial.println(messageTemp);
  Serial.print("  topic: ");  //  "  топик: "
  Serial.println(topic);
  Serial.print("  qos: ");  //  "  уровень обслуживания: "
  Serial.println(properties.qos);
  Serial.print("  dup: ");  //  "  дублирование сообщения: "
  Serial.println(properties.dup);
  Serial.print("  retain: ");  //  "сохраненные сообщения: "
  Serial.println(properties.retain);
  Serial.print("  len: ");  //  "  размер: "
  Serial.println(len);
  Serial.print("  index: ");  //  "  индекс: "
  Serial.println(index);
  Serial.print("  total: ");  //  "  суммарно: "
  Serial.println(total);
}



void setup() {  // --- процедура начальной инициализации устройства ---
#ifdef DEBUG_IN_SERIAL                              // условная компиляция при выводе отладки в порт
  // инициализация консольного порта 
  Serial.begin(115200); 
#endif



}

void loop() {
  // put your main code here, to run repeatedly:
}