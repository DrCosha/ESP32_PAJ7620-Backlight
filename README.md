# ESP32_PAJ7620-Backlight
Firware for ESP32s module with gesture sensor PAJ7620.  

Модуль управления построенный на ESP32 c двумя каналами LED (warm & cold). Которые могут работать как в синхронном режиме - управление только яркостью, так и в режиме яркость + цветовая температура.

Управление осуществляется по тремя различными способами:
1. Управление кнопкой на плате (опторазвязка).
2. Управление через датчик жестов PAJ7620
3. Управление через топики MQTT сервера.

![20230317_091343](https://user-images.githubusercontent.com/80087552/225835447-27c02a59-03d8-4f5d-88e4-4ac7fa8e6db7.jpg)
