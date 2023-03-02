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
#include <GyverButton.h>
#include "RevEng_PAJ7620.h"
#include <ArduinoJson.h>

// ----------- режим компиляции для отладки с выводом в порт -----------
#define DEBUG_IN_SERIAL
// ---------------------------------------------------------------------

#define WIFI_SSID "iot_ls"                          // SSID нашей локальной сети  
#define WIFI_PASSWORD "vvssoft40"                   // пароль к нашей локальной сети
#define MQTT_USER "mqtt_user"                       // имя пользователя для подключения к MQTT серверу
#define MQTT_PWD "vvssoft40"                        // пароль для подключения к MQTT серверу
#define MQTT_HOST IPAddress(192, 168, 10, 100)      // адрес нашего Mosquito MQTT сервера
#define MQTT_PORT 1883                              // порт нашего Mosquito MQTT сервера

#define LWT_TOPIC   "diy/blm32_kitchen/LWT"         // топик публикации доступности устройства
#define SET_TOPIC   "diy/blm32_kitchen/set"         // топик публикации команд для устройства
#define STATE_TOPIC "diy/blm32_kitchen/state"       // топик публикации состояния устройства

#define _SCL 22                                     // назначение выводов линии i2c
#define _SDA 21                                     //

#define BUTTON_PIN 19                               // вывод назначенный для кнопки управления

// определение JSON тегов для обмена 
#define C_STATE "state"                             // тег состояния устройства  
#define C_GESTURE "gesture"                         // тег последнего определенного жеста
#define C_BRIGHTNESS "brightness"                   // тег текущей яркости для устройства

// создаем объекты для управления MQTT-клиентом и WiFi соединением
AsyncMqttClient mqttClient;                         // MQTT клиент
TimerHandle_t mqttReconnectTimer;                   // таймер повторной попытки установки MQTT соединения
TimerHandle_t wifiReconnectTimer;                   // таймер повторной попытки установки WiFi соединения

// создаем обработчик кнопки
GButton ctrl_butt(BUTTON_PIN, HIGH_PULL, NORM_OPEN);  // инициализируем кнопку управления
// HIGH_PULL - кнопка подключена к GND, пин подтянут к VCC (PIN --- КНОПКА --- GND)
// LOW_PULL  - кнопка подключена к VCC, пин подтянут к GND
// NORM_OPEN - нормально-разомкнутая кнопка
// NORM_CLOSE - нормально-замкнутая кнопка

// создаем объект - сенсор движений
RevEng_PAJ7620 gestureSensor = RevEng_PAJ7620();

// создаем объект - JSON документ для приема/передачи данных через MQTT
StaticJsonDocument<200> doc;                        // создаем json докумкент с буфером в 200 байт 

// назначаем GPIO контакты для устройств
const int ledInd = 2;                               // выход управления индикаторным светодиодом
const int buttonPin = 19;                           // вход внешней кнопки
const int ledCh1 = 5;                               // выход на канал управления LED1
const int ledCh2 = 4;                               // выход на канал управления LED2

// объявляем глобальные переменные 
bool MQTTConnected = false;                         // соединение MQTT установлено




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
      MQTTConnected=false;
      break;

    default:                                        // обработка прочих кейсов
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

  mqttClient.publish(LWT_TOPIC, 0, true, "online");                 // публикуем в топик LWT_TOPIC событие о своей жизнеспособности

  #ifdef DEBUG_IN_SERIAL                                      
    Serial.print("Publishing LWT state in [");
    Serial.print(LWT_TOPIC); 
    Serial.println("]. QoS 0. "); 
  #endif                    
  
  MQTTConnected=true;
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
    Serial.println("Publish acknowledged.");                                              // публикация подтверждена
    Serial.print("  packetId: ");                                                         //
    Serial.println(packetId);                                                             // выводим ID пакета
  #endif                     
}


// в этой функции обрабатываем события получения данных в управляющем топике SET_TOPIC
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  String messageTemp;

  #ifdef DEBUG_IN_SERIAL         
    Serial.print("Get message: [");
  #endif                         
  for (int i = 0; i < len; i++) {                                                       // преобразуем полученные в сообщении данные в строку
    #ifdef DEBUG_IN_SERIAL         
      Serial.print((char)payload[i]);
    #endif                         
    messageTemp += (char)payload[i];
  }
  #ifdef DEBUG_IN_SERIAL         
    Serial.println("]");
  #endif                         

  // проверяем, в каком именно топике получено MQTT сообщение
  if (strcmp(topic, SET_TOPIC) == 0) {
    // проводим действия согласно полученным командам
    // TODO:

    
  }
 
  #ifdef DEBUG_IN_SERIAL         
    Serial.println("Publish received.");                                                 //  выводим на консоль данные из топика
    Serial.print("  topic: ");                                                           //  "  топик: "
    Serial.println(topic);                                                               // название топика 
    Serial.print("  message: ");                                                         //  "  сообщение: "
    Serial.println(messageTemp);                                                         //  сообщение 
  #endif                         

}


void setup() {  // --- процедура начальной инициализации устройства ---
  
  #ifdef DEBUG_IN_SERIAL                                                                 // условная компиляция при выводе отладки в порт
    // инициализация консольного порта 
    Serial.begin(115200); 
  #endif

  // настраиваем входы и выходы контроллера

  // TODO:
  // -----------------------------------------------------------------
  // -----------------------------------------------------------------
  // -----------------------------------------------------------------
  // -----------------------------------------------------------------
 // -----------------------------------------------------------------
  // -----------------------------------------------------------------
  // -----------------------------------------------------------------
  // -----------------------------------------------------------------

  // инициализируем датчик жестов на линии I2C
  if( !gestureSensor.begin() )              // инициализируем датчик жестов - если return = 0 , то успех
  {
    #ifdef DEBUG_IN_SERIAL                                                               // условная компиляция при выводе отладки в порт   
      Serial.println("PAJ7620 I2C error - halting");
    #endif
  } else {
    #ifdef DEBUG_IN_SERIAL                                                                 // условная компиляция при выводе отладки в порт    
      Serial.println("Gesture sensor PAJ7620 - Init OK");    
    #endif      
  }

  // настраиваем параметры кнопки
  ctrl_butt.setDebounce(50);                // настройка антидребезга (по умолчанию 80 мс)
  ctrl_butt.setTimeout(300);                // настройка таймаута на удержание (по умолчанию 500 мс)
  ctrl_butt.setClickTimeout(600);           // настройка таймаута между кликами (по умолчанию 300 мс)

  // настраиваем WiFi клиента
  WiFi.onEvent(WiFiEvent);

  // настраиваем MQTT клиента
  mqttClient.setCredentials(MQTT_USER,MQTT_PWD);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  // создаем таймеры, которые будут устанавливать и переустанавливать соединение с WiFi и MQ
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(2000), pdFALSE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));  

  // запускаем подключение к WiFi
  connectToWifi();

}

void loop() {  // --- основной цикл исполняемого кода устройства

  Gesture gesture;                  // Gesture is an enum type from RevEng_PAJ7620.h
  gesture = gestureSensor.readGesture();   // Read back current gesture (if any) of type Gesture

  switch (gesture)
  {
    case GES_FORWARD:
      {
        Serial.print("GES_FORWARD");
        break;
      }

    case GES_BACKWARD:
      {
        Serial.print("GES_BACKWARD");
        break;
      }

    case GES_LEFT:
      {
        Serial.print("GES_LEFT");
        break;
      }

    case GES_RIGHT:
      {
        Serial.print("GES_RIGHT");
        break;
      }

    case GES_UP:
      {
        Serial.print("GES_UP");
        break;
      }

    case GES_DOWN:
      {
        Serial.print("GES_DOWN");
        break;
      }

    case GES_CLOCKWISE:
      {
        Serial.print("GES_CLOCKWISE");
        break;
      }

    case GES_ANTICLOCKWISE:
      {
        Serial.print("GES_ANTICLOCKWISE");
        break;
      }

    case GES_WAVE:
      {
        Serial.print("GES_WAVE");
        break;
      }

    case GES_NONE:
      {
        break;
      }
  }

  if( gesture != GES_NONE )
  {
    Serial.print(", Code: ");
    Serial.println(gesture);


    // если нет соединения с MQTT - ничего не генерим, ждем соединения
    if (mqttClient.connected()) {                                       // если есть соединение с MQTT - выкладываем статус устройства 
       Serial.println("Пишем в MQTT");
       doc.clear();   
       doc[C_STATE] = "ON";
       doc[C_GESTURE] = gesture;
       String payload;
       serializeJson(doc, payload);
       // публикуем в топик STATE_TOPIC серилизованный json через буфер buffer
       char buffer[ payload.length()+1 ];
       payload.toCharArray(buffer, sizeof(buffer));   
       mqttClient.publish(STATE_TOPIC, 0, true, buffer );

    }
  }


  delay(100);

}