# Speak out loud
*Озвучивание текстов из узлов ROS*

<p align="center">

<!-- <add logo> -->

<a href="LICENSE"><img src="https://img.shields.io/badge/License-GPLv2-green.svg" alt="License" /></a>
<a><img src="https://img.shields.io/badge/ROS-Melodic-blue" alt="ros_version_melodic" /></a>
<a><img src="https://img.shields.io/badge/ROS-Noetic-blue" alt="ros_version_noetic" /></a>
</p>

*README доступно на нескольких языках: [Английский](README.md), [Русский](README.ru.md)*

*[Тестовые сценарии](TESTS.md)*

## Описание
speak_out_loud (SoL) - ROS пакет для генерации речи из текста, который поддерживает русский язык.

Пакет основан на генераторе речи RHVoice и высокоуровневом аудио интерфейсе Speech Dispatcher.

Чтобы что-то сказать, достаточно отправить текст, упакованный в сообщение типа SpeakGoal, в топик *sol/texts*.

Продвинутое управление воспроизведением включает в себя:
* приоритеты, которые позволяют задавать очерёдность воспроизведения сообщений;
* whitelist  - список узлов, от которых воспроизводятся тексты, и только от них; 
* blacklist  - список узлов, воспроизведение от которых блокируется;
* режим debug для воспроизведения дополнительной информации.

Автор:  Александр Малышев <asanmalyshev AT gmail DOT com>               
Youtube видео с некоторыми примерами (на русском): https://youtu.be/uVPauu7p71E


## Зависимости
Пакет зависит от [RHVoice](https://github.com/Olga-Yakovleva/RHVoice/) и [Speech Dispatcher (spd)](https://github.com/brailcom/speechd).
За полной документацией по установке и решению проблем обращаться по ссылкам.
Краткая инструкция по установке на Ubuntu описана далее.


## Инструкция по установке пакета и зависимостей для Ubuntu 
Работоспособность проверена для Ubuntu 18.04 (ROS Melodic) и Ubuntu 20.04 (ROS Noetic)

### Rhvoice
```shell
sudo add-apt-repository ppa:linvinus/rhvoice
sudo apt-get update
sudo apt-get install rhvoice rhvoice-english rhvoice-russian 
```
Для добавления других языков установить пакет rhvoice-*language_name*. 

### Speech dispatcher
```shell
sudo apt install speech-dispatcher speech-dispatcher-rhvoice speech-dispatcher-audio-plugins libspeechd-dev python3-speechd
```

### Пакет speak_out_loud
- скопировать репозиторий github:
```shell
cd ~/catkin_ws/src #change [catkin_ws] on your own ws name
git clone https://github.com/asanmalyshev/speak_out_loud
```
- собрать пакет, используя, например, `catin_make` или `catkin_tools`:
```shell
catkin_make 
```
или 
```shell
catkin build speak_out_loud
```

## Запуск
```shell
roslaunch speak_out_loud speak_out_loud.launch
```
Для настройки сервера доступно несколько параметров launch-файла.

### Параметры
| Параметр | Описание | Значения | Значения по умолчания
| --- | --- | --- | ---
| ___output___ | Вывод log-файлов | screen/log | screen 
| ___default_voice___ | используемый голос по умолчанию | некоторые: elena/aleksndr | elena 
| ___whitelist___ | whitelist узлов | список значений | [ ] 
| ___blacklist___ | blacklist узлов | список значений | [ ] 
| ___debug___ | использование отладочного режима | True/False | False
| ___use_action_interface___ | использование action-интерфейса| True/False | False

Полный список голосов (при условии скачивания соответствующих пакетов) можно найти [тут](https://github.com/RHVoice/RHVoice/wiki/Latest-version).

Русские голоса хорошо работают и с английскими текстами.
Можно посылать смешанные тексты - голосовой синтезатор их воспроизведёт.

Пользователь может задавать whitelist/blacklist узлов от которых надо озвучивать текст.
Подробнее в [секции](#фильтрация).

### Проблемы со звуком
Иногда вместе с голосом появляется шум.
Чтобы избавиться от него, необходимо сделать изменение в настройках PulseAudio.
В файле `/etc/pulse/default.pa/` надо найти сточку `load-module module-udev-detect`
и заменить её на `load-module module-udev-detect tsched=0`.
После перезагрузки системы, шумы должны исчезнуть.

### Текст произносится не до конца
Speech dispatcher по умолчанию выключается по истечении некотрого времени после отключения последнего клиента.
Так как SoL сервер после отправки текста отключается от speech dispatcher, возможна потеря части произносимого текста.
Чтобы избежать подобных ситуаций, необходимо поправить конфигурационный файл `/etc/speech-dispatcher/speechd.conf`:
Найдите параметр `Timeout`, раскоментируйте его и измените количество секунд, которое speech dispatcher будет ждать после отключения клиента, или же установите значение параметра =0, чтобы снять ограничение на время ожидания. 


## Тестирование
Несколько вариантов тестирования:
- командой в bash;
- запуск launch-файла с тестовым узлом. 
Оба варианта требует запуск сервера.
```shell
roslaunch speak_out_loud speak_out_loud.launch
```

### Через Bash
```shell
rostopic pub /sol/texts speak_out_loud/SpeakGoal "{sender_node: '', text: 'Привет', voice: '', priority: 1, debug: false}"
```

### Launch-файл
```shell
roslaunch speak_out_loud speak_out_loud_client_example.launch 
```
Набрать пару слов, задать приоритет.
#### Client emulator launch parameters
| Параметр | Описание | Значения | Значения по умолчания
| --- | --- | --- | ---
| ___output___ | Вывод log-файлов | screen/log | screen 
| ___node_name___ | Имя узла |  | speak_out_loud_client
<!-- | ___debug___ | использование отладочного топика | True/False | False -->


## Узлы
___/sol/sol_server____
Сервер озвучивания текстов. Получает тексты и возпроизводит их.

## Топики
___Публикации___
| Топик | Тип | Описание 
| --- | --- | --- 
___sol/do_i_say___ | ([std_msgs/Bool](http://docs.ros.org/en/api/std_msgs/html/msg/Bool.html)) | True, если какой-то текст будет проговариваться, False когда, проговаривание текста закончено 

___Подписки___
| Топик | Тип | Описание 
| --- | --- | --- 
___sol/texts___ | ([speak_out_loud/SpeakGoal](action/Speak.action)) | топик для сообщений
___/sol/action_iface/goal___ | ([speak_out_loud/SpeakGoal](action/Speak.action)) | Если ипользуется Action интерфейс: топик цели
___/sol/action_iface/cancel___ | ([actionlib_msgs/GoalID](http://docs.ros.org/en/api/actionlib_msgs/html/msg/GoalID.html)) | Если ипользуется Action интерфейс: топик отмены 
___/sol/action_iface/feedback___ | ([speak_out_loud/SpeakFeedback](action/Speak.action)) | Если ипользуется Action интерфейс: топик обратной связи
___/sol/action_iface/result___ | ([speak_out_loud/SpeakResult](action/Speak.action)) | Если ипользуется Action интерфейс: топик результата
___/sol/action_iface/status___ | ([actionlib_msgs/GoalStatusArray](http://docs.ros.org/en/api/actionlib_msgs/html/msg/GoalStatusArray.html)) | Если ипользуется Action интерфейс: топик статуса


## Интерфейс взаимодействия клиента и сервера
Клиенты общаются с сервером посредством сообщений из [action/Speak.action](action/Speak.action).
```
# goal
string  sender_node   # sender node name 
string  text          # text to read
string  voice         # voice to speak with
uint8   priority      # priority of text
bool    debug         # debug message
bool    use_ssml      # use ssml markup
---
# result
uint8 smsg_id         # result of reading text (not used in current version) 
uint8 IMPORTANT     = 1
uint8 MESSAGE       = 2
uint8 TEXT          = 3
uint8 NOTIFICATION  = 4
uint8 PROGRESS      = 5
---
# feedback
int32 msg_id          # id of queued message
uint8 msg_status      # status of queued message
string msg            # msg to say
string mark           # mark name
uint8 SPD_BEGIN       = 1
uint8 SPD_END         = 2
uint8 SPD_INDEX_MARKS = 3
uint8 SPD_CANCEL      = 4
uint8 SPD_PAUSE       = 5
uint8 SPD_RESUME      = 6
```
Поля из Goal заполняются на пользовательской стороне.
- ___sender_node___ - имя узла-отправителя. Поле заполняется вручную. Не должно быть пустым, если используются whitelist/blacklist. По умолчанию поле пустое. 
- ___text___ содержит текст для воспроизведения __*)__;
- ___voice___ - голос для воспроизведения. Если не задан, то используется голос по умолчанию;
- ___priority___ - приоритет, число в диапазоне [1,5]. Для удобства эти значения связаны с константами, описанными в [msg/Priority.msg](msg/Priority.msg). Правила приоритизации описаны в [Приоритеты сообщений](#приоритеты-сообщений). Значение по умолчанию: Priority.TEXT;
- ___debug___ - сообщение, побликуемое в режиме debug;
- ___use_ssml___ - использовать разметку ssml;
> ___*)___ - обязательные поля

## Приоритеты сообщений
На реальных системах тексты могут приходить от большого числа узлов.

![common_schema](doc/images/common_schema.png)

Для организации очереди на прочтение используются приоритеты. В speech dispatcher их определено пять. 
В пакете они продублированы в [msg/Priority.msg](msg/Priority.msg). 
| Приоритет | Значение 
| --- | :---: 
 IMPORTANT | 1 |
 MESSAGE | 2 |
 TEXT | 3 |
 NOTIFICATION | 4 |
 PROGRESS | 5 |

Также в Priorty.msg заданы значения MIN=0 и MAX=6, которые помогают определить границы значений приоритетов.

О приоритетах подробнее в [spd документация / описание протокола (SSIP)](https://htmlpreview.github.io/?https://github.com/brailcom/speechd/blob/master/doc/ssip.html#Message-Priority-Commands).

На изображении ниже инфографика о том, как работают приоритеты разного типа:
![spd_prioiries](doc/images/spd_priorities_explanation.png)

Есть очередь (Queue, Q) из текстов для прочтения и текущий читаемый текст (current reading text, CRT).
Очередь состоит из полей, каждое из которых имеет приоритет.
Текстов с приоритетами IMPORTANT и MESSAGE может быть много (образуют подочередь), 
текстов с приоритетами TEXT и PROGRESS - по одному, 
очередь не содержит текстов типа NOTIFICATION.

При работе с очередью, над пришедшим текстом выполняется операция отмены, добавления или замены.
Новые IMPORTANT и MESSAGE добавляются в конец соответствующей подочереди,
в то время как остальные типы заменяют хранимые значения.

В каждый момент времени Q и CRT могут быть:
- [частично] заполнены текстами (цветные элементы) - в очереди есть тексты для прочтения / текст, читаемый в настоящий момент; 
- пустыми (пустые элементы) - ничего нет для прочтения и ничего не читается.

В дальнейшем словом "текст" обозначается текст, содержащийся во входящем сообщении.

1) IMPORTANT сообщение пришло:
- все читаемые тексты, кроме приоритета IMPORTANT, отменяются;
- если CRT пуста, текст отправляется в неё, иначе добавляется в конец подочереди IMPORTANT в Q;
- PROGRESS в Q отменяется.

2) MESSAGE сообщение пришло:
- все читаемые тексты, кроме приоритета IMPORTANT и MESSAGE, отменяются;
- если CRT пуста, текст отправляется в неё, иначе добавляется в конец подочереди MESSAGE в Q;
- TEXT и PROGRESS в Q отменяется.

3) TEXT сообщение пришло:
- если читаются тексты приоритетов PROGRESS, NOTIFICATION и TEXT, то они отменяются;
- если CRT пуста, текст отправляется в неё, иначе заменяет значение TEXT в Q;
- PROGRESS в Q отменяется.

4) NOTIFICATION сообщение пришло:
- если Q не пусто, то отменяется;
- если читается сообщение NOTIFICATION, читаемое отменяется, а новое воспроизводится. 

5) PROGRESS сообщение пришло:
- если читается сообщение NOTIFICATION, оно отменяется, а пришедшее воспроизводится; 
- если Q не пусто или NOTIFICATION в CRT, текст заменяет значение, хранимое в Q;
- если сообщений этого типа долго не поступает, то PROGRESS исполняется с приоритетом MESSAGE.


## Настраивание узлов

### Режим отладки
Для отладки используется специальное поле debug в SpeakGoal. При выключенном режиме отладки, сообщения со значением True в поле debug игнорируются. Они воспроизводятся только если сервер запущен с параметром _debug=True_.

###  Фильтрация
Для уменьшения количества топиков, от которых приходят тексты для прочтения, используются launch параметры whitelist и blacklist.
Whitelist имеет приоритет выше. Это значит, что если оба фильтрующих списка определены, данные фильтруются только whitelist.
По умолчанию, оба списка пустые, обрабатываются все входящие сообщения.
Обоими списками фильтрации можно управлять через ros сервисы /sol/whitelist_control и /sol/blacklist_control, оба имеющих тип [speak_out_loud/SpeakFilter](srv/SpeakFilter).
```
string name     # name to operate
bool operation  # operation
---
string[] nameslist   # current list
bool    status       # True - filtering is on, False - filtering is off
```
_Запрос_
| Поле | Тип | Описание 
| --- | --- | --- 
___name___ | ([std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html)) | имя фильтруемого узла
___operation___ | ([std_msgs/Bool](http://docs.ros.org/en/api/std_msgs/html/msg/Bool.html)) | True - добавить в список, False - удалить из списка

_Ответ_
| Поле | Тип | Описание 
| --- | --- | --- 
___nameslist___ | ([std_msgs/String](http://docs.ros.org/en/api/std_msgs/html/msg/String.html)[]) | текущий список
___status___ | ([std_msgs/Bool](http://docs.ros.org/en/api/std_msgs/html/msg/Bool.html)) | True - фильтрация включена, False - фильтрация выключена

Пользователю необходимо отправить запрос с name='' и operation=True если фильтрацию нужно включить, operation=False - если выключить.
Отправка непустого поля name добавляет (operation=True) или удаляет (operation=False) топик из списка фильтрации.

Если __whitelist__ содержит элементы, проверяется есть ли имя отправителя (просматривается поле в сообщении) в списке, и если есть, то __перенаправляет сообщение на сервер,__ в противном случае сообщение __игнорируется__.
```xml
<!-- your awesome launch file -->
<launch>
  <!-- ... -->
  <include file="$(find speak_out_loud)/launch/speak_out_loud.launch">
    <arg name="whitelist" value="[/speak_out_loud_client]"/>
  </include>
  <!-- ... -->
</launch>
```
Если __blacklist__  содержит элементы, проверяется есть ли имя отправильтеля в списке, и если есть - __сообщение игнориуется__, иначе - __перенаправляется__.
```xml
<!-- your even more awesome launch file -->
<launch>
  <!-- ... -->
  <include file="$(find speak_out_loud)/launch/speak_out_loud.launch">
    <arg name="blacklist" value="[/speak_out_loud_client]"/>
  </include>
  <!-- ... -->
</launch>
```
Заданные whitelist/blacklist могут быть дополнены без перезапуска (но только для запущенного экземпляра). Для этого необходимо отправить имя узла в соответствующий [топик](#топик).
При этом автоматически включается фильтрация соответствующего типа.

Также whitelisting/blacklisting могут быть включены и выключены во время работы сервера. Для этого надо отрпавить True/False в соответствующий [топик](#топик).
При включении одного фильтра, другой автоматически выключается. Выключение фильтров не удаляет список фильтраций.

## Форматирование текста
### Использование меток
При работе с SoL в тексте можно вставлять специальные метки. 
При их достижении в ходе прочтения сообщения в feedback топик посылается сообщение с указанием имени достигнутной метки (поле feedback.mark).
Метка - набор символов, заключённый между '<' и '>':
``` 
# текст без метки:
Привет! 

# текст c меткой:
Привет<hi>! 
```
Для использования режима меток в отправляемом в SoL сообщении необходимо установить поле use_ssml = True
Этот функционал доступен только в режиме action-интерфейса.

## Назначение приоритетов

В целом, предпочтительным является использование приоритетов MESSAGE и TEXT. Какой именно использовать - зависит от нагрузки на возспроизводящую систему.
Для систем с редкими сообщениями приоритета TEXT достаточно. 
Если сообщений очень много, то дополнительно лучше использовать MESSAGE.

Использовать IMPORTANT рекомендуется в экстренных случаях, делая текст максимально коротким.
Например: потеря соединения с интернетом, низкий уровень заряда батареи, возгорание робота, потеря колёс, гусениц, иных обычно важных частей.

Так как нет гарантии воспроизведения сообщений приоритета NOTIFICATION,
для робототехники выделить прикладую пользу сложно. Разве что для пугания ночью случайных прохожих в неопределённое время.

Из-за своего сложного повдения, приоритет PROGRESS также не рекомендуется использовать.
В разделе тестирования PROGRESS рассматривается только на базовом уровне. Для сложных сценариев он не используется.


## Тестирование

Раздел *[тестирования](TESTS.md)* вынесен в отдельный файл 


## Лицензия

Пакет распространяется под лицензией GPLv2 or later.

RHVoice использует [LGPLv2.1](https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html). 
При компиляции используется MAGE, разработанная под [GPLv3](https://www.gnu.org/licenses/gpl-3.0.html). 
Для сохранения пакета и его ответвлений под GPLv2 необходимо перекомпилировать пакет без MAGE. 
Подробности в разделе о лицензии пакета.

Speech dispatcher содержит модули, приложения, бибилиотеки, разрабатывавшиеся под [LGPLv2.1](https://www.gnu.org/licenses/old-licenses/lgpl-2.1.html), [GPLv2](https://www.gnu.org/licenses/old-licenses/gpl-2.0.html) и GPLv2-or-later. 
Подробности в разделе о лицензии пакета.
