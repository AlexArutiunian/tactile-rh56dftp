# RH56DFTP tactile recording notes

Дата фиксации: 2026-05-15

Рабочая папка:

```bash
cd ~/rh56_tactile_toolkit
```

## Железо

Использовалась левая рука Inspire Hand RH56DFTP.

Основной рабочий IP для новой руки:

```text
192.168.123.211
```

Проблемная рука:

```text
192.168.123.210
```

На руке `.210` тактильные каналы большого пальца в проверках и в записях оставались нулевыми:

```text
thumb_tip_3x3      ZERO
thumb_tip_12x8     ZERO
thumb_middle_3x3   ZERO
thumb_pad_12x8     ZERO
palm_8x14          ZERO
```

То есть это была не ошибка визуализации: большой палец реально не отдавал tactile данные в той записи/на той руке.

На руке `.211` большой палец работает:

```text
thumb_tip_3x3      OK_TOUCH
thumb_tip_12x8     OK_TOUCH
thumb_middle_3x3   READS_RAW_NO_DELTA или OK_TOUCH, зависит от касания
thumb_pad_12x8     OK_TOUCH
```

Один канал все еще выглядел проблемным даже на `.211`:

```text
middle_tip_3x3     ZERO
```

## Что означают значения сенсоров

`raw_max`, `max_delta`, `force_delta`, tactile raw - это не Ньютоны и не миллиметры.

Для tactile это сырые значения датчиков/регистров. Их надо использовать как:

- факт касания;
- активная площадь контакта;
- относительная сила/изменение внутри одного и того же запуска;
- облако точек для восстановления формы.

Значения около `65280` / `65281` / `65284` похожи на насыщение uint16 или специальное высокое значение регистра. Их нельзя напрямую считать физической силой.

## Калибровка

В API есть регистр:

```text
GESTURE_FORCE_CALIB
address: 1009
description: force sensor calibration
```

По описанию API и официального мануала RH56DFTP это калибровка датчиков силы, а не tactile матриц:

```text
Запуск калибровки при записи 1. Рука должна быть раскрыта, пальцы не должны касаться объектов.
```

В мануале этот же регистр обозначен как `GESTURE_FORCE_CLB` по адресу `1009`.
В установленном Python API он называется `GESTURE_FORCE_CALIB`.

Проверенные места:

- `/home/al/.local/lib/python3.10/site-packages/Register/config/configFTP/ftp_registers.py`;
- `/home/al/.local/lib/python3.10/site-packages/Register/RegisterKey/ftp_registers_keys.py`;
- `/home/al/.local/lib/python3.10/site-packages/RH56DFTP/RH56DFTP_TCP.py`;
- `packages/plusml_rh56dftp-0.1.7-py3-none-any.whl`;
- официальный мануал RH56DFTP.
- официальный support page Inspire Robots: там есть Windows PC software для Dexterous Hands;
- `INSPIRE ROBOTS-THE Dexterous Hand PC Instructions`.

В наших запусках аппаратную калибровку через `GESTURE_FORCE_CALIB` мы не делали.

Что мы делали вместо этого:

- software baseline для `FORCE_ACT(*)`;
- baseline frames для tactile raw;
- потом считали `force_delta = forces - force_baseline`;
- tactile raw сохраняли полностью, без аппаратного zero/calib.

Если нужна аппаратная калибровка силы перед новой записью, команда будет такая:

```bash
python3 - <<'PY'
from RH56DFTP.RH56DFTP_TCP import RH56DFTP_TCP

client = RH56DFTP_TCP("192.168.123.211", 6000)
print("Open hand, remove all contacts, then press Enter")
input()
print("Start force calibration:", client.set("GESTURE_FORCE_CALIB", 1))
client.close()
PY
```

После этого лучше заново снять baseline и проверить `FORCE_ACT(*)`.

Важно: это не починит tactile канал, который реально все время `ZERO`.

Для tactile матриц отдельного регистра калибровки не найдено. В карте регистров tactile данные идут как read-only блоки:

```text
3000 small finger tactile    R
3370 ring finger tactile     R
3740 middle finger tactile   R
4110 index finger tactile    R
4480 thumb tactile           R
4900 palm tactile            R
```

То есть через публичный Modbus/API можно читать tactile raw, но не видно команды аппаратной tactile calibration/zero.

### Windows PC software

На официальной странице поддержки Inspire есть:

```text
The Dexterous Hands PC Software
https://en.inspire-robots.com/wp-content/uploads/2023/11/The-Dexterous-Hands-PC-Software.rar
```

И инструкция:

```text
INSPIRE ROBOTS-THE Dexterous Hand PC Instructions
https://en.inspire-robots.com/wp-content/uploads/2024/02/INSPIRE-ROBOTS-THE-Dexterous-Hand-PC-Instructions.pdf
```

В этой инструкции написано, что main screen показывает real-time angle/current/force/temperature/error, и если force applied to a finger is not accurate, нужно калибровать force sensor.

Явной tactile calibration в этой инструкции не найдено. Это похоже на UI-обертку вокруг тех же открытых регистров, где калибруется force через `GESTURE_FORCE_CLB/CALIB`, а tactile только отображается/читается.

### C++/ROS/другие SDK

По публичным результатам нашлись C++/DDS проекты под Inspire/Unitree, но они в основном про joint control/state для RH56DFX/RH56DFTP, без найденной команды tactile calibration.

Отдельный найденный C++ SDK с функцией `calibrate_touch_sensor` относится к другой руке/платформе `STARK`, не к Inspire RH56DFTP, поэтому его нельзя переносить на эту руку без подтверждения производителя.

## Проверка всех tactile сенсоров

Базовая команда для проверки руки `.211`:

```bash
python3 scripts/check_all_tactile_sensors.py \
  --baseline-frames 10 \
  --duration 30 \
  --threshold 80 \
  --ip 192.168.123.211
```

Как проводить:

1. Во время baseline не трогать руку.
2. После сообщения `Baseline ready` нажимать все зоны: каждый палец, tip/pad, отдельно большой палец и ладонь.
3. Смотреть `TACTILE SENSOR CHECK SUMMARY`.

Файл результата:

```text
scripts/tactile_sensor_check_summary.json
```

Статусы:

```text
OK_TOUCH            канал увидел изменение выше threshold
READS_RAW_NO_DELTA  канал читает ненулевые raw, но заметного касания не увидел
ZERO                канал все время ноль
```

## Датасет новой руки

Новый root для записей с рабочей рукой `.211`:

```text
scripts/tactile_grasp_dataset_new_hand
```

Каждая сессия пишет:

```text
metadata.json
baseline_raw.jsonl
frames_raw.jsonl
grasp_trace.csv
```

В `frames_raw.jsonl` сохраняются:

- `positions`;
- `forces`;
- `force_delta`;
- `sensors_raw`;
- `phase`;
- для manual сегментов еще `segment`.

## Grasp static

Первый сильный хват стаканчика был слишком сильный:

```bash
python3 scripts/adaptive_grasp_force_record_session.py \
  --ip 192.168.123.211 \
  --object-name cup_small \
  --session-name cup_small_grasp_01 \
  --mode grasp_static \
  --out-root scripts/tactile_grasp_dataset_new_hand \
  --thumb-rot-pos 2000 \
  --hold-sec 10 \
  --baseline-sec 3
```

Более слабый хват, который оказался нормальнее:

```bash
python3 scripts/adaptive_grasp_force_record_session.py \
  --ip 192.168.123.211 \
  --object-name cup_small \
  --session-name cup_small_grasp_01_very_small \
  --mode grasp_very_small \
  --out-root scripts/tactile_grasp_dataset_new_hand \
  --thumb-rot-pos 2000 \
  --force-threshold 60 \
  --grasp-step 40 \
  --hold-sec 10 \
  --baseline-sec 3
```

Замечание: в этом запуске большой палец механически почти не двигался, поэтому для slide/contact отдельно сделали ручной режим без автоматического закрытия.

## Manual slide/contact

Скрипт:

```text
scripts/record_tactile_slide_session.py
```

Он не закрывает руку автоматически. Это режим для ручного контакта: объект рукой человека катается/прижимается по tactile зонам.

Уже записанный manual slide:

```bash
python3 scripts/record_tactile_slide_session.py \
  --ip 192.168.123.211 \
  --object-name cup_small \
  --session-name cup_small_slide_manual_01 \
  --mode slide_manual \
  --out-root scripts/tactile_grasp_dataset_new_hand \
  --thumb-rot-pos 0 \
  --record-sec 25 \
  --record-hz 15 \
  --baseline-sec 3
```

Проверка этой записи показала:

```text
baseline_raw.jsonl  45 frames
frames_raw.jsonl    375 frames
grasp_trace.csv     375 data rows + header
```

В этой записи thumb tactile есть, в том числе `thumb_tip_12x8` и `thumb_pad_12x8`.

## Типы касаний

Для восстановления формы стаканчика/окружности лучше писать сегментами:

```text
bottom  - донышко стакана, прижимать/катать нижнюю окружность
rim     - горлышко стакана, прижимать/катать верхнюю окружность
side    - боковая стенка стакана, катать цилиндрическую часть
slide   - общий ручной slide/roll по датчикам без разделения
grasp   - статический хват рукой
roll    - объект прокатывается, чтобы разные участки окружности прошли по tactile зоне
```

Команда для отдельной записи окружностей донышка и горлышка:

```bash
python3 scripts/record_tactile_slide_session.py \
  --ip 192.168.123.211 \
  --object-name cup_small \
  --session-name cup_small_circle_contacts_01 \
  --mode contact_circle_bottom_rim \
  --out-root scripts/tactile_grasp_dataset_new_hand \
  --thumb-rot-pos 0 \
  --record-hz 15 \
  --baseline-sec 3 \
  --segments bottom:10,rim:10,side:10
```

Порядок работы:

1. На baseline не трогать руку.
2. Перед каждым сегментом скрипт ждет Enter.
3. `bottom`: прижимать/катать донышко.
4. `rim`: прижимать/катать горлышко.
5. `side`: прижимать/катать боковую стенку.

Если нужно только донышко и горлышко:

```bash
python3 scripts/record_tactile_slide_session.py \
  --ip 192.168.123.211 \
  --object-name cup_small \
  --session-name cup_small_circle_contacts_02 \
  --mode contact_circle_bottom_rim \
  --out-root scripts/tactile_grasp_dataset_new_hand \
  --thumb-rot-pos 0 \
  --record-hz 15 \
  --baseline-sec 3 \
  --segments bottom:12,rim:12
```

## Визуализация

Основной генератор:

```text
generate_tactile_shape_report.py
```

Для новых данных с руки `.211` не надо включать искусственный thumb contact, потому что thumb tactile работает.

Команда для визуализации одной новой сессии:

```bash
python3 generate_tactile_shape_report.py \
  --session scripts/tactile_grasp_dataset_new_hand/cup_small_slide_manual_01 \
  --out-root tactile_shape_cup_new_hand
```

Для будущей записи окружностей:

```bash
python3 generate_tactile_shape_report.py \
  --session scripts/tactile_grasp_dataset_new_hand/cup_small_circle_contacts_01 \
  --out-root tactile_shape_cup_new_hand
```

Старые финальные визуализации лежат тут:

```text
tactile_shape_final
```

Старые варианты с искусственным контактом большого пальца использовались только для старой руки/старых записей, где thumb tactile был нулевой:

```bash
python3 generate_tactile_shape_report.py \
  --session <old_session> \
  --out-root <old_output_folder> \
  --assume-thumb-final-contact
```

Для `.211` этот флаг использовать не надо.

## Что важно для анализа формы

Для формы по tactile точкам лучше не использовать один статический кадр, если объект только один раз сжали. Надежнее:

- писать manual `bottom/rim/side` сегменты;
- отделять точки по `segment`;
- фитить окружность отдельно по `bottom` и `rim`;
- использовать `side` для проверки цилиндрической стенки;
- не трактовать raw значения как физическую силу.

Для стаканчика ожидаемый следующий анализ:

```text
bottom contacts -> окружность донышка
rim contacts    -> окружность горлышка
side contacts   -> боковая поверхность / цилиндр
```
