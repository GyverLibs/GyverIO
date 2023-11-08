[![latest](https://img.shields.io/github/v/release/GyverLibs/GyverIO.svg?color=brightgreen)](https://github.com/GyverLibs/GyverIO/releases/latest/download/GyverIO.zip)
[![Foo](https://img.shields.io/badge/Website-AlexGyver.ru-blue.svg?style=flat-square)](https://alexgyver.ru/)
[![Foo](https://img.shields.io/badge/%E2%82%BD$%E2%82%AC%20%D0%9D%D0%B0%20%D0%BF%D0%B8%D0%B2%D0%BE-%D1%81%20%D1%80%D1%8B%D0%B1%D0%BA%D0%BE%D0%B9-orange.svg?style=flat-square)](https://alexgyver.ru/support_alex/)
[![Foo](https://img.shields.io/badge/README-ENGLISH-blueviolet.svg?style=flat-square)](https://github-com.translate.goog/GyverLibs/GyverIO?_x_tr_sl=ru&_x_tr_tl=en)  

[![Foo](https://img.shields.io/badge/ПОДПИСАТЬСЯ-НА%20ОБНОВЛЕНИЯ-brightgreen.svg?style=social&logo=telegram&color=blue)](https://t.me/GyverLibs)

# GyverIO
Быстрые функции для работы с пинами AVR (полный список смотри в gio_avr.h), ESP8266, ESP32
- Быстрая реализация функций shiftIn/shiftOut
- Классы для быстрого управления пином
- Отдельная обработка случаев константных и неконстантных пинов для AVR

### GPIO
#### us (MHz)
|         | digitalWrite | gio_write | gio_toggle | digitalRead | gio_read   | pinMode      | gio_mode   |
|---------|--------------|-----------|------------|-------------|------------|--------------|------------|
| AVR     | 1.47 (0.68)  | 0.06 (16) | 0.06 (16)  | 3.23 (0.31) | 0.13 (8)   | 3.33 (0.3)   | 0.13 (8)   |
| ESP8266 | 1.56 (0.64)  | 0.08 (12) | 0.5 (2)    | 0.56 (1.8)  | 0.21 (4.8) | 1.43 (0.7)   | 0.67 (1.5) |
| ESP32   | 0.33 (3)     | 0.04 (24) | 0.13 (8)   | 1.67 (0.6)  | 0.08 (12)  | 16.67 (0.06) | 0.08 (12)  |
| ESP32C3 | 0.92 (1)     | 0.06 (16) | 0.19 (5.2) | 0.25 (4)    | 0.04 (24)  | 20 (0.05)    | 0.04 (24)  |

### Shift
#### us byte (MHz)
|         | shiftOut     | gio_send | gio_send (non-const) |
|---------|--------------|----------|----------------------|
| AVR     | 100 (0.075)  | 6 (1.3)  | 11 (0.7)             |

### Совместимость
Совместима со всеми Arduino платформами (используются Arduino-функции)
- Для esp8266 и esp32 быстрый `pinMode()` (`mode()`) работает только на режимы `INPUT`/`OUTPUT`!

## Содержание
- [Установка](#install)
- [Документация](#docs)
- [Использование](#usage)
- [Версии](#versions)
- [Баги и обратная связь](#feedback)

<a id="install"></a>
## Установка
- Библиотеку можно найти по названию **GyverIO** и установить через менеджер библиотек в:
    - Arduino IDE
    - Arduino IDE v2
    - PlatformIO
- [Скачать библиотеку](https://github.com/GyverLibs/GyverIO/archive/refs/heads/main.zip) .zip архивом для ручной установки:
    - Распаковать и положить в *C:\Program Files (x86)\Arduino\libraries* (Windows x64)
    - Распаковать и положить в *C:\Program Files\Arduino\libraries* (Windows x32)
    - Распаковать и положить в *Документы/Arduino/libraries/*
    - (Arduino IDE) автоматическая установка из .zip: *Скетч/Подключить библиотеку/Добавить .ZIP библиотеку…* и указать скачанный архив
- Читай более подробную инструкцию по установке библиотек [здесь](https://alexgyver.ru/arduino-first/#%D0%A3%D1%81%D1%82%D0%B0%D0%BD%D0%BE%D0%B2%D0%BA%D0%B0_%D0%B1%D0%B8%D0%B1%D0%BB%D0%B8%D0%BE%D1%82%D0%B5%D0%BA)
### Обновление
- Рекомендую всегда обновлять библиотеку: в новых версиях исправляются ошибки и баги, а также проводится оптимизация и добавляются новые фичи
- Через менеджер библиотек IDE: найти библиотеку как при установке и нажать "Обновить"
- Вручную: **удалить папку со старой версией**, а затем положить на её место новую. "Замену" делать нельзя: иногда в новых версиях удаляются файлы, которые останутся при замене и могут привести к ошибкам!


<a id="docs"></a>
## Документация
### gio
Быстые функции для работы с пинами

```cpp
int gio::read(int P);
void gio::high(int P);
void gio::low(int P)
void gio::write(int P, int V);

//
void gio::toggle(int P);

// режим пина. Для esp8266/esp32 только INPUT/OUTPUT!
void gio::mode(int P, int V);

// нужно вызывать для esp8266/esp32 при инициализации пина
// иначе mode() не будет работать!
void gio::init(int P);
```

### gio::shift
Быстрый аналог shiftIn/shiftOut (отправка данных с клоком)

```cpp
// прочитать пакет
void gio::shift::read(uint8_t dat_pin, uint8_t clk_pin, uint8_t order, uint8_t* data, uint16_t len, uint8_t delay = 0);

// прочитать байт
uint8_t gio::shift::read_byte(uint8_t dat_pin, uint8_t clk_pin, uint8_t order, uint8_t delay = 0);

// прочитать пакет + cs пин
void gio::shift::read_cs(uint8_t dat_pin, uint8_t clk_pin, uint8_t cs_pin, uint8_t order, uint8_t* data, uint16_t len, uint8_t delay = 0);

// прочитать байт + cs пин
uint8_t gio::shift::read_cs_byte(uint8_t dat_pin, uint8_t clk_pin, uint8_t cs_pin, uint8_t order, uint8_t delay = 0);

// отправить пакет
void gio::shift::send(uint8_t dat_pin, uint8_t clk_pin, uint8_t order, uint8_t* data, uint16_t len, uint8_t delay = 0);

// отправить байт
void gio::shift::send_byte(uint8_t dat_pin, uint8_t clk_pin, uint8_t order, uint8_t data, uint8_t delay = 0);

// отправить пакет + cs пин
void gio::shift::send_cs(uint8_t dat_pin, uint8_t clk_pin, uint8_t cs_pin, uint8_t order, uint8_t* data, uint16_t len, uint8_t delay = 0);

// отправить байт + cs пин
void gio::shift::send_cs_byte(uint8_t dat_pin, uint8_t clk_pin, uint8_t cs_pin, uint8_t order, uint8_t data, uint8_t delay = 0);
```

#### Примечание
- `delay` в микросекундах, позволяет уменьшить скорость передачи. Например 1 мкс ограничит скорость до ~1 МГц, 2 мкс до ~500 кГц
- Пины нужно сконфигурировать как `OUTPUT` самостоятельно до отправки (при запуске программы например)

### Настройки компиляции
```cpp
#define GIO_USE_ARDUINO     // отключить быстрые функции (заменить на стандартные)
#define GIO_NO_MASK         // отключить быстрый IO на основе маски для AVR (в классе PinIO и всех shift)
```

<a id="usage"></a>
## Использование

```cpp
gio::write(3, 1);   // включить пин 3

// отправить данные по пинам 3 и 4
uint8_t data[] = {34, 63, 231, 9};
gio::shift::send(3, 4, MSBFIRST, data, 4);
```

<a id="versions"></a>
## Версии
- v1.0

<a id="feedback"></a>
## Баги и обратная связь
При нахождении багов создавайте **Issue**, а лучше сразу пишите на почту [alex@alexgyver.ru](mailto:alex@alexgyver.ru)  
Библиотека открыта для доработки и ваших **Pull Request**'ов!

При сообщении о багах или некорректной работе библиотеки нужно обязательно указывать:
- Версия библиотеки
- Какой используется МК
- Версия SDK (для ESP)
- Версия Arduino IDE
- Корректно ли работают ли встроенные примеры, в которых используются функции и конструкции, приводящие к багу в вашем коде
- Какой код загружался, какая работа от него ожидалась и как он работает в реальности
- В идеале приложить минимальный код, в котором наблюдается баг. Не полотно из тысячи строк, а минимальный код