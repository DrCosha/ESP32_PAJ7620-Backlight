//
//          Файл содержит локальные "секреты"  проекта
//


#define WIFI_SSID "iot_ls"                          // SSID нашей локальной сети  
#define WIFI_PASSWORD "vvssoft40"                   // пароль к нашей локальной сети
#define MQTT_USER "mqtt_user"                       // имя пользователя для подключения к MQTT серверу
#define MQTT_PWD "vvssoft40"                        // пароль для подключения к MQTT серверу
#define MQTT_HOST IPAddress(192, 168, 10, 100)      // адрес нашего Mosquito MQTT сервера
#define MQTT_PORT 1883                              // порт нашего Mosquito MQTT сервера

#define LWT_TOPIC   "diy/blm32_kitchen/LWT"         // топик публикации доступности устройства
#define SET_TOPIC   "diy/blm32_kitchen/set"         // топик публикации команд для устройства
#define STATE_TOPIC "diy/blm32_kitchen/state"       // топик публикации состояния устройства