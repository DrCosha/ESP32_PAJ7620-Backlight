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


// ------------------------------------------
// ----------- режимы компиляции  -----------
// ------------------------------------------
//#define DEBUG_IN_SERIAL                             // компиляция в отладочном режиме с выводом в порт 
#define ONLY_BRIGHTNESS_MODE                        // компилируем прошивку в режиме - только регулирование яркостью иначе - яркость+цветовая температура
//#define EXTRA_MQTT_REPORT                           // компилируем прошивку в режиме передачи расширенных данных через MQTT
// ------------------------------------------
// ------------------------------------------
// ------------------------------------------


#define WIFI_SSID "iot_ls"                          // SSID нашей локальной сети  
#define WIFI_PASSWORD "vvssoft40"                   // пароль к нашей локальной сети
#define MQTT_USER "mqtt_user"                       // имя пользователя для подключения к MQTT серверу
#define MQTT_PWD "vvssoft40"                        // пароль для подключения к MQTT серверу
#define MQTT_HOST IPAddress(192, 168, 10, 100)      // адрес нашего Mosquito MQTT сервера
#define MQTT_PORT 1883                              // порт нашего Mosquito MQTT сервера

#define LWT_TOPIC   "diy/blm32_kitchen/LWT"         // топик публикации доступности устройства
#define SET_TOPIC   "diy/blm32_kitchen/set"         // топик публикации команд для устройства
#define STATE_TOPIC "diy/blm32_kitchen/state"       // топик публикации состояния устройства

// назначаем GPIO контакты для устройств
#define _SCL 22                                     // назначение выводов линии i2c
#define _SDA 21                                     //
#define BUTTON_PIN 19                               // вывод назначенный для кнопки управления
#define LED_IND 2                                   // выход управления индикаторным светодиодом
#define LED_PWR1 5                                  // выход на канал управления LED1
#define LED_PWR2 4                                  // выход на канал управления LED2

// константа определяющая промежуток времени между событиями публикации статуса в MQTT
#define MQTT_STATUS_LAG 60000                       // сколько миллисекунд должно пройти между выводом состояния устройства в MQTT  

// определение JSON тегов для обмена 
#define CN_STATE "state"                            // тег состояния устройства  
#define CN_GESTURE "gesture"                        // тег последнего определенного жеста
#define CN_BUTTON "button"                          // тег последней команды от кнопки
#define CN_BRIGHTNESS "brightness"                  // тег текущей яркости для устройства
#define CN_COLOR_TEMP "color_temp"                  // тег значения текущей цветовой температуры

// определяем минимальные и максимальные константы яркости и цветовой температуры
#define MIN_LED_BRIGHTNESS 1                        // минимальная яркость
#define MAX_LED_BRIGHTNESS 255                      // максимальная яркость
#define MIN_COLOR_TEMP 153                          // минимальная цветовая температура
#define MAX_COLOR_TEMP 500                          // максимальная цветовая температура

// константы шагов регулирования яркости/цветовой температуры
#define STEP_DELTA_BR 20                            // задержка для следующего шага инкремента/декремента при регулировке яркости
#define STEP_DELTA_CT 20                            // задержка для следующего шага инкремента/декремента при регулировке цвета
#define CT_DELTA 1                                  // шаг изменения цвета через кнопки
#define BR_DELTA 5                                  // шаг изменения яркости через кнопки  
#define CT_DELTA_BG 20                              // шаг изменения цвета через жесты
#define BR_DELTA_BG 50                              // шаг изменения яркости через жесты  

// определяем команды, которые могут быть получены устройством для исполнения
enum UserCommand {
  UCMD_NONE = 0,                                    // нет команды 
  UCMD_ON,                                          // включить устройство
  UCMD_OFF,	                                        // выключить устройство
  UCMD_BRGH_UP,                                     // повысить яркость  
  UCMD_BRGH_DOWN,                                   // понизить яркость
  UCMD_COLORTEMP_UP,                                // изменить цветовую температуру - холоднее
  UCMD_COLORTEMP_DOWN                               // изменить цветовую температуру - теплее
};

// определяем команды, которые могут быть получены устройством для исполнения
enum ButtonState {
  BTTN_NONE = 0,                                    // нет команды от кнопки
  BTTN_PRESS,                                       // кнопка нажата
  BTTN_HOLD,	                                      // кнопка удерживается
  BTTN_RELEASE,	                                    // кнопка отпущена
  BTTN_SINGLE,	                                    // однократное нажатие на кнопку
  BTTN_DBL_PRESS                                    // двойное нажатие на кнопку
};

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
RevEng_PAJ7620 gestureSensor = RevEng_PAJ7620();    // создаем объект - сенсор
Gesture curr_Gesture;                               // данные полученные от PAJ7620 - код жеста

// создаем объект - JSON документ для приема/передачи данных через MQTT
StaticJsonDocument<255> doc;                        // создаем json документ с буфером в 255 байт 

// назначаем параметры ШИМ для каналов LED
const int PWM_Freq = 5000;                          // базовая частота PWM
const int PWM_Led1Channel = 0;                      // канал счетчика для LED1
const int PWM_Led2Channel = 1;                      // канал счетчика для LED2
const int PWM_Resolution = 16;                      // разрешение управляющих каналов 

// переменные управляющие PWM каналами 
uint32_t DutyCycleLED1 = 0;                         // заполнение цикла для LED1
uint32_t DutyCycleLED2 = 0;                         // заполнение цикла для LED2

// переменные управляющие текущей яркостью и текущей цветовой температурой
uint16_t curr_Brightness = MAX_LED_BRIGHTNESS;                                     // текущее значение яркости  - диапазон от MIN_LED_BRIGHTNESS до MAX_LED_BRIGHTNESS
uint16_t curr_ColorTemp = ((MAX_COLOR_TEMP-MIN_COLOR_TEMP) >> 1) + MIN_COLOR_TEMP; // текущее значение цветовой температуры - диапазон от MIN_COLOR_TEMP до MAX_COLOR_TEMP - значение по умолчанию - середина диапазона

// текущие команда цикла
UserCommand curr_Command;                           // текущая команда в цикле исполнения
ButtonState curr_Button_State;                      // текущее состояние кнопки

// текущие переменные программы
bool HasChanges = false;                            // есть ли изменения необходимые к отработке/отображению 
bool DeviceON = false;                              // текущее состояние устройства     
bool ColorMode = false;                             // режим регулирования цвет/яркость
bool Direction = true;                              // направление изменения яркости/цвета
unsigned long LastStepInMills = 0;                  // последний отсчет времени при регулировании яркости/цветовой температуры       
unsigned long LastReportToMQTT = 0;                 // время последнего отчета в MQTT
bool InCycleChanges = false;                        // если мы в циклическом изменении регулирования яркости/цветовой температуры
bool Has_MQTT_Command = false;                      // флаг получения MQTT команды 


// -------------------------- вспомогательные процедуры ----------------------------
bool StrCheck(const char *str1, const char *str2){
  int i = 0;
  while(str1[i] != '\0' && str2[i] != '\0'){
    if(str1[i] != str2[i])
      return false;
    i++;
  }
  //Если мы вышли из цикла значит одну из строк мы перебрали до конца
  if(str1[i] == '\0' && str2[i] == '\0')
    return true;
  return false;
}

// ------------------ набор обработчиков событий для MQTT клиента -----------------
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
      digitalWrite(LED_IND, HIGH);                  // зажигаем индикаторный светодиод при обрыве связи с MQTT и WiFi
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
  
}


void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  #ifdef DEBUG_IN_SERIAL                                        
    Serial.println("Disconnected from MQTT.");                      // если отключились от MQTT
  #endif                               
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);                             // запускаем таймер переподключения к MQTT
  }
  // зажигаем индикаторный светодиод при обрыве связи  с MQTT
  digitalWrite(LED_IND, HIGH);                                      // включение через подачу 1
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
  messageTemp[len] = '\0';  

  #ifdef DEBUG_IN_SERIAL         
    Serial.println("]");
  #endif                         

  // проверяем, в каком именно топике получено MQTT сообщение
  if (strcmp(topic, SET_TOPIC) == 0) {
    // разбираем MQTT сообщение и подготавливаем буфер с изменениями для формирования команд
    deserializeJson(doc, messageTemp);                  // десерилизуем сообщение и взводим признак готовности к обработке
    Has_MQTT_Command = true;                            // взводим флаг получения команды по MQTT

  }
 
  #ifdef DEBUG_IN_SERIAL         
    Serial.println("Publish received.");                //  выводим на консоль данные из топика
    Serial.print("  topic: ");                          //  "  топик: "
    Serial.println(topic);                              // название топика 
    Serial.print("  message: ");                        //  "  сообщение: "
    Serial.println(messageTemp);                        //  сообщение 
  #endif                         

}

uint32_t CalcPWMCh1() {  // вычисляем текущую яркость, канала LED1 - канал "холодного" белого цвета
  double _calc_BR = 0;           // расчётная общая яркость
  double _calc_CT_corr = 0;      // расчётная цветовая коррекция "теплого" (Ch2)
  _calc_BR = (65535/MAX_LED_BRIGHTNESS)*curr_Brightness;                                                    // расчитываем общую яркость суммы каналов
  _calc_CT_corr = (_calc_BR/(MAX_COLOR_TEMP-MIN_COLOR_TEMP))*(curr_ColorTemp-MIN_COLOR_TEMP);               // расчитываем канал "теплого" 
  return int32_t(_calc_BR-_calc_CT_corr);                                                                   // расчитываем значение канала "холодного" как разницу между общей яркостью и "теплым" каналом
}

uint32_t CalcPWMCh2() {  // вычисляем текущую яркость, канала LED2 - канал "теплого" белого цвета
  double _calc_BR = 0;           // расчётная общая яркость
  _calc_BR = (65535/MAX_LED_BRIGHTNESS)*curr_Brightness;                                                    // расчитываем общую яркость суммы каналов
  return int32_t((_calc_BR/(MAX_COLOR_TEMP-MIN_COLOR_TEMP))*(curr_ColorTemp-MIN_COLOR_TEMP));               // расчитываем значение канала "теплого"  
}

bool NextStep() { // нужен ли нам следующий шаг для регулирования
int tmpStep = 0;  // шаг изменений зависит от того, какую величину регулируем
  if (ColorMode) tmpStep = STEP_DELTA_CT;
    else tmpStep = STEP_DELTA_BR;  
  if ((millis()-LastStepInMills)>tmpStep)
    {
      LastStepInMills = millis();   
      return true;  
    }
   else return false;  
}

void get_button_command() {  // --- процедура получения управляющих команд от кнопки ---

  curr_Button_State = BTTN_NONE;                        // сбрасываем состояние кнопки
  ctrl_butt.tick();                                     // опрашиваем кнопку в цикле

  if (ctrl_butt.isSingle()) {                           // если у нас одинарный клик - включение/выключение устройства
    if (DeviceON) curr_Command = UCMD_OFF;              // включаем LED каналы
      else curr_Command = UCMD_ON;                      // выключаем LED каналы
    HasChanges = true;                                  // есть изменения
    curr_Button_State = BTTN_SINGLE;                    // кнопка нажата однократно
    InCycleChanges = false;                             // выходим из режима циклического изменения    
  }

  if (ctrl_butt.isDouble()){                            // если у нас двойной клик - переключаем режим работы с яркостью/цветовой температурой
  #ifdef ONLY_BRIGHTNESS_MODE  // если компилируем прошивку в режиме - только регулирование яркостью
    ColorMode = false;                                  // всегда только яркость  
  #else 
    if (!DeviceON) curr_Command = UCMD_ON;              // если светильник выключен - включаем перед регулированием         
    ColorMode = !ColorMode;            
  #endif    
    HasChanges = true;
    InCycleChanges = false;                             // выходим из режима циклического изменения
  } 

  if (ctrl_butt.isRelease()) {                          // если кнопка отпускается - переключаем направление управления inc/dec
    if (!DeviceON) return ;            
    Direction = !Direction;                             // при отпускании кнопки проводим смену направления изменения ++ или -- 
    if (InCycleChanges) {
      InCycleChanges = false;                           // выходим из режима циклического изменения
      HasChanges = true;                                // при отпускании кнопки при выходе из Hold-а даем сигнал о фиксации изменений
    }
  }

  if (ctrl_butt.isPress()) {                            // нажатие на кнопку - сброс последнего отсчета
    LastStepInMills = millis();    
    InCycleChanges = true;                              // входим в режим циклического изменения   
  } 

  if ( ctrl_butt.isHold() and DeviceON and NextStep() ){   // кнопка удерживается при этом светильник включен и наступило время для следующего шага - проводим изменение
    if (ColorMode){ // если это режим изменения цвета  
      if (Direction){ // определяем направление изменений - увеличение цветовой температуры (холоднее)
        if ((curr_ColorTemp+CT_DELTA) > MAX_COLOR_TEMP) curr_ColorTemp = MAX_COLOR_TEMP;
          else {
            curr_ColorTemp = curr_ColorTemp+CT_DELTA;
          }
        curr_Command = UCMD_COLORTEMP_UP;               // взводим команду на изменение ЦТ
        } 
      else { // уменьшение цветовой температуры (теплее)
        if ((curr_ColorTemp-CT_DELTA) < MIN_COLOR_TEMP) curr_ColorTemp = MIN_COLOR_TEMP;
          else {
            curr_ColorTemp = curr_ColorTemp-CT_DELTA;
          }          
        curr_Command = UCMD_COLORTEMP_DOWN;            // взводим команду на изменение ЦТ
        }        
      } 
    else { // это режим изменения яркости
      if (Direction){ // определяем направление изменений - увеличение яркости
        if ((curr_Brightness+BR_DELTA) > MAX_LED_BRIGHTNESS) curr_Brightness = MAX_LED_BRIGHTNESS;
          else {
            curr_Brightness = curr_Brightness+BR_DELTA;           
          }          
        curr_Command = UCMD_BRGH_UP;                  // взводим команду на изменение яркости                                    
        } 
      else { // уменьшение яркости
        if ((curr_Brightness-BR_DELTA) < MIN_LED_BRIGHTNESS) curr_Brightness = MIN_LED_BRIGHTNESS;
          else {
            curr_Brightness = curr_Brightness-BR_DELTA;
          }                    
        curr_Command = UCMD_BRGH_DOWN;               // взводим команду на изменение яркости                                              
      }          
    }
    InCycleChanges = true;                           // находимся в режиме циклического изменения   
    HasChanges = true;                               // поднимаем флаг изменений    
  }
}

void get_sensor_command() { // --- процедура получения управляющих команд от сенсора ---

  curr_Gesture = gestureSensor.readGesture();    // Read back current gesture (if any) of type Gesture
  switch (curr_Gesture)
  {
    case GES_LEFT:
      {
        if (!DeviceON) {  // если устройство выключено - включаем его
          HasChanges = true;
          curr_Command = UCMD_ON;
        }
        break;
      }

    case GES_RIGHT:
      {
        if (DeviceON) {  // если устройство включено - выключаем его
          HasChanges = true;
          curr_Command = UCMD_OFF;
        }     
        break;
      }

    case GES_DOWN:          // если жест - движение назад или вперед - это режим переключения регулирования яркость/цветовая температура
    case GES_FORWARD:       // 
      {         
        #ifdef ONLY_BRIGHTNESS_MODE  // если компилируем прошивку в режиме - только регулирование яркостью
          ColorMode = false;                                  // всегда только яркость  
        #else 
          ColorMode = !ColorMode;            
        #endif    
        break;
      }

    case GES_CLOCKWISE:
      {  // увеличиваем цветовую температуру или яркость
        if (ColorMode) { // если это режим изменения цвета - увеличиваем цветовую температуру (холоднее)  
            if ((curr_ColorTemp+CT_DELTA_BG) > MAX_COLOR_TEMP) curr_ColorTemp = MAX_COLOR_TEMP;
              else curr_ColorTemp = curr_ColorTemp+CT_DELTA_BG; 
            curr_Command = UCMD_COLORTEMP_UP;           // взводим команду на изменение ЦТ  
          } 
        else { // это добавляем яркости
            if ((curr_Brightness+BR_DELTA_BG) > MAX_LED_BRIGHTNESS) curr_Brightness = MAX_LED_BRIGHTNESS;
            else curr_Brightness = curr_Brightness+BR_DELTA_BG;           
            curr_Command = UCMD_BRGH_UP;                // взводим команду на изменение яркости              
          }
        HasChanges = true;  
        break;
      }

    case GES_ANTICLOCKWISE:
      {  // уменьшаем цветовую температуру или яркость
        if (ColorMode) { // если это уменьшение цветовой температуры (теплее)
            if ((curr_ColorTemp-CT_DELTA_BG) < MIN_COLOR_TEMP) curr_ColorTemp = MIN_COLOR_TEMP;
              else curr_ColorTemp = curr_ColorTemp-CT_DELTA_BG;                    
            curr_Command = UCMD_COLORTEMP_DOWN;         // взводим команду на изменение ЦТ                
          }        
        else { // это уменьшение  яркости
          if ((curr_Brightness-BR_DELTA_BG) < MIN_LED_BRIGHTNESS) curr_Brightness = MIN_LED_BRIGHTNESS;
            else curr_Brightness = curr_Brightness-BR_DELTA_BG;
            curr_Command = UCMD_BRGH_DOWN;             // взводим команду на изменение яркости                          
          }                
        HasChanges = true;            
        break;
      }

    case GES_NONE:
      break;

    default:
      break;      
      
  }

}

void get_mqtt_command() { // --- процедура получения управляющих команд по каналу MQTT ---
  if (!Has_MQTT_Command) return;                              // если флага о полученной команде от MQTT нет - выходим  

    //--- обрабатываем полученную MQTT команду ---
    // обработка тега STATE
    if (doc.containsKey(CN_STATE)) { // есть тег state
      if (doc[CN_STATE] == "ON") curr_Command =  UCMD_ON;     // если есть ON - даем команду на включение LED каналов
        else curr_Command =  UCMD_OFF;
      HasChanges = true;        
    }

    // обработка тега BRIGHTNESS
    if (doc.containsKey(CN_BRIGHTNESS)) { // есть тег brightness
      curr_Brightness = doc[CN_BRIGHTNESS];                   // присваиваем текущее значение яркости из тега яркости
      // нормализуем значение яркости
      if (curr_Brightness < MIN_LED_BRIGHTNESS) curr_Brightness = MIN_LED_BRIGHTNESS;   // снизу
      if (curr_Brightness > MAX_LED_BRIGHTNESS) curr_Brightness = MAX_LED_BRIGHTNESS;   // сверху
      HasChanges = true;      
    }

    #ifndef ONLY_BRIGHTNESS_MODE
    // обработка тега COLOR_TEMP
    if (doc.containsKey(CN_COLOR_TEMP)) { // есть тег color_temp
      curr_ColorTemp = doc[CN_COLOR_TEMP];                    // присваиваем текущее значение цветовой температуры 
      // нормализуем значение яркости
      if (curr_ColorTemp < MIN_COLOR_TEMP) curr_ColorTemp = MIN_COLOR_TEMP;             // снизу
      if (curr_ColorTemp > MAX_COLOR_TEMP) curr_ColorTemp = MAX_COLOR_TEMP;             // сверху
      HasChanges = true;
    }
    #endif

  doc.clear();                                                // очищаем документ
  Has_MQTT_Command = false;                                   // сбрасываем флаг об MQTT команде  
}

void get_command() {  // --- процедура получения управляющих команд ---
  curr_Command = UCMD_NONE;                                   // сбрасываем предыдущую команду  
  HasChanges = false;                                         // обнуляем флаг изменений    
  get_button_command();                                       // процедура получения управляющих команд от кнопки
  get_sensor_command();                                       // процедура получения управляющих команд от сенсора
  get_mqtt_command();                                         // процедура получения управляющих команд по каналу MQTT
}

void applay_changes() { // --- применяем команды/изменения ---
  if (!HasChanges) return;                                    // если нечего применять - выходим без обработки
  switch (curr_Command)
  {
  case UCMD_ON:                 // включить LED каналы
    {  // обработка команды включения устройства     
      ledcWrite(PWM_Led1Channel, CalcPWMCh1());               // выставляем значения PWM сигнала для канала 1
      ledcWrite(PWM_Led2Channel, CalcPWMCh2());               // ... для канала 2     
      DeviceON = true;                                        // LED-ы включены
      ColorMode = false;                                      // вкл режим регулирования яркости
      break;
    }
  case UCMD_OFF:                // выключить LED каналы
    {  // обработка команды включения устройства
      ledcWrite(PWM_Led1Channel, 0);                          // выставляем значения PWM сигнала для канала 1
      ledcWrite(PWM_Led2Channel, 0);                          // ... для канала 2     
      DeviceON = false;                                       // 
      break;
    }
  case UCMD_BRGH_UP:            // повысить яркость  
  case UCMD_BRGH_DOWN:          // понизить яркость
  case UCMD_COLORTEMP_UP:       // изменить цветовую температуру - холоднее
  case UCMD_COLORTEMP_DOWN:     // изменить цветовую температуру - теплее
    {  // обработка команд изменения яркости/температуры
      ledcWrite(PWM_Led1Channel, CalcPWMCh1());               // выставляем значения PWM сигнала для канала 1
      ledcWrite(PWM_Led2Channel, CalcPWMCh2());               // ... для канала 2     
      break;
    }
  default:  // обработка команд по умолчанию
    break;
  }  
}

void report_to_asyncPort() { // --- пишем события и состояние в асинхронный порт ---
  if (!HasChanges) return;                                    // если нет изменений - просто выходим без обработки
  Serial.println();
  Serial.print("State: ");
  if (DeviceON) { Serial.println("ON"); }                     // пишем cостояние устройства в целом
    else { Serial.println("OFF"); }
  Serial.print("Gesture code: ");    
  Serial.println(curr_Gesture);                               // пишем cостояние датчика жестов 
  Serial.print("Button state: ");    
  Serial.println(curr_Button_State);                          // пишем cостояние кнопки
  Serial.print("Brightness: ");                             
  Serial.println(curr_Brightness);                            // выводим значение текущей яркости     
  Serial.print("Color temperature: ");    
  Serial.println(curr_ColorTemp);                             // выводим значение текущей цветовой температуры

}

void report_to_topicMQTT() { // --- пишем события и состояние в MQTT топик ---
  // если нет соединения с MQTT - ничего не генерим, ждем соединения  
  if ((HasChanges or (MQTT_STATUS_LAG < (millis()-LastReportToMQTT))) and !InCycleChanges) { // если есть изменения или наступил таймаут по публикации статуса в MQTT и при этом мы не в циклических изменениях
    if (mqttClient.connected()) {                               // если есть соединение с MQTT - выкладываем статус устройства в MQTT
        // зажигаем индикаторный светодиод при начале процедуры обмена с MQTT и гасим в конце
        // тем самым мигаем светодиодом при обмене с MQTT + гасим светодиод после начального включения и установления связи с MQTT сервером
        digitalWrite(LED_IND, HIGH);                              // включение через подачу 1
        // создаем JSON сообщение
        doc.clear();   
        if (DeviceON) { doc[CN_STATE] = "ON"; }                   // пишем текущее состояние устройства
          else { doc[CN_STATE] = "OFF"; }                
      // если работаем в режиме расширенных данных по MQTT
      #ifdef EXTRA_MQTT_REPORT
        doc[CN_GESTURE] = curr_Gesture;                           // пишем последнюю команду от датчика жестов
        doc[CN_BUTTON] = curr_Button_State;                       // пишем последнюю команду от кнопки
      #endif  
        doc[CN_BRIGHTNESS] = curr_Brightness;                     // выводим значение текущей яркости
      // если режим двух каналов без цветовой температуры
      #ifndef ONLY_BRIGHTNESS_MODE      
        doc[CN_COLOR_TEMP] = curr_ColorTemp;                      // выводим значение текущей цветовой температуры
      #endif  
        String payload;
        serializeJson(doc, payload);
        // публикуем в топик STATE_TOPIC серилизованный json через буфер buffer
        char buffer[ payload.length()+1 ];
        payload.toCharArray(buffer, sizeof(buffer));   
        mqttClient.publish(STATE_TOPIC, 0, true, buffer );
        // и сбрасываем счетчик лага
        LastReportToMQTT = millis();        
        // гасим индикаторный светодиод 
        digitalWrite(LED_IND, LOW);                                // выключение через подачу 0
      }
    }
}

void report_state() { // --- сообщаем об изменении состояния в порт и топик MQTT ---
#ifdef DEBUG_IN_SERIAL                                        // если стоит признак отладки через порт
  report_to_asyncPort();                                      // пишем события и состояние в асинхронный порт
#endif  
  report_to_topicMQTT();                                      // пишем события и состояние в MQTT топик
}


void setup() {  // --- процедура начальной инициализации устройства ---
  
  #ifdef DEBUG_IN_SERIAL                                                                 // условная компиляция при выводе отладки в порт
    // инициализация консольного порта 
    Serial.begin(115200); 
  #endif

  // настраиваем входы и выходы контроллера
  // инициализация входов и выходов  
  pinMode(LED_IND, OUTPUT);                                   // инициализируем pin индикаторного светодиода
  pinMode(LED_PWR1, OUTPUT);                                  // инициализируем pin силового выхода LED1
  pinMode(LED_PWR2, OUTPUT);                                  // инициализируем pin силового выхода LED2 

  // зажигаем индикаторный светодиод 
  digitalWrite(LED_IND, HIGH);                                // включение через подачу 1

  // инициализируем датчик жестов на линии I2C
  if( !gestureSensor.begin() )                                // инициализируем датчик жестов - если return = 0 , то успех
  {
    #ifdef DEBUG_IN_SERIAL                                    // условная компиляция при выводе отладки в порт   
      Serial.println("PAJ7620 I2C error - halting");
    #endif
  } else {
    #ifdef DEBUG_IN_SERIAL                                    // условная компиляция при выводе отладки в порт    
      Serial.println("Gesture sensor PAJ7620 - Init OK");    
    #endif      
  }

  // настраиваем параметры кнопки
  ctrl_butt.setDebounce(50);                // настройка антидребезга (по умолчанию 80 мс)
  ctrl_butt.setTimeout(400);                // настройка таймаута на удержание (по умолчанию 500 мс)
  ctrl_butt.setClickTimeout(300);           // настройка таймаута между кликами (по умолчанию 300 мс)

  // настраиваем WiFi клиента
  WiFi.onEvent(WiFiEvent);

  // настраиваем параметры ШИМ для каналов LED1 и LED2
  ledcSetup(PWM_Led1Channel, PWM_Freq, PWM_Resolution);       // назначаем каналы для PWM, частоты и разрешение сигнала управления
  ledcSetup(PWM_Led2Channel, PWM_Freq, PWM_Resolution);       // 
  ledcAttachPin(LED_PWR1, PWM_Led1Channel);                   // привязываем GPIO к каналам PWM 
  ledcAttachPin(LED_PWR2, PWM_Led2Channel);                   //
  DutyCycleLED1 = 0;                                          // заполнение цикла для LED1
  DutyCycleLED2 = 0;                                          // заполнение цикла для LED2
  ledcWrite(PWM_Led1Channel, DutyCycleLED1);                  // обнуляем заполнение PWM сигнала
  ledcWrite(PWM_Led2Channel, DutyCycleLED2);                  //

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

  // начальная инициализация текущего распознанного жеста
  curr_Gesture = GES_NONE;                  // нет текущего жеста                               
  curr_Command = UCMD_NONE;                 // нет текущей команды

  // инициализируем данные по последнему отчету в MQTT так, что бы сразу при старте выдать очередной отчет
  LastReportToMQTT = MQTT_STATUS_LAG + millis();

}

void loop() {  // --- основной цикл исполняемого кода устройства

  get_command();                            // получаем текущую команду
  applay_changes();                         // применяем команды/изменения
  report_state();                           // сообщаем об изменении состояния в порт и топик MQTT
 
}