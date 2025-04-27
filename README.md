# ZMK PMW3360 Driver

Этот репозиторий содержит драйвер для оптического сенсора PMW3360, часто используемого в компьютерных мышах. Драйвер разработан для использования с прошивкой ZMK.

## Возможности

- Поддержка оптического сенсора PMW3360
- Настраиваемые параметры CPI/DPI
- Режимы энергосбережения
- Различные режимы ввода: перемещение, прокрутка и точный режим (snipe)
- Настраиваемая частота опроса

## Использование

### Пример PMW3360

Добавьте следующее в файл overlay устройства (board.overlay):

```dts
&spi1 {
  status = "okay";
    
  cs-gpios = <&gpio0 17 GPIO_ACTIVE_LOW>;
    
  pmw3360: pmw3360@0 {
    compatible = "pixart,pmw3360";
    status = "okay";
    reg = <0>;
    spi-max-frequency = <2000000>;
    irq-gpios = <&gpio0 10 (GPIO_ACTIVE_LOW | GPIO_PULL_UP)>;
    scroll-layers = <1>;
    /* Вы можете добавить:
    snipe-layers = <2>;  
    */
  };
};
```

## Конфигурация

Настройте параметры сенсора в файле `prj.conf` вашего проекта:

```
CONFIG_PMW3360=y
CONFIG_PMW3360_CPI=1600
CONFIG_PMW3360_SCROLL_CPI=800
CONFIG_PMW3360_SNIPE_CPI=400
```

Смотрите файл `Kconfig.pmw3360` для всех доступных опций конфигурации.

## Интеграция с ZMK

Этот драйвер разработан для использования как модуль в ZMK. Добавьте его в ваш репозиторий конфигурации ZMK, включив его как подмодуль:

```
west config manifest.project-filter -- "+[^pmw3360]"
git submodule add https://github.com/yourusername/zmk-pmw3360-driver.git modules/pmw3360-driver
```

Затем добавьте его в ваш манифест `west.yml`:

```yaml
manifest:
  remotes:
    - name: zmk
      url-base: https://github.com/zmkfirmware
    - name: pmw3360-driver
      url-base: https://github.com/yourusername
  projects:
    - name: zmk
      remote: zmk
      revision: main
      import: app/west.yml
    - name: pmw3360-driver
      remote: pmw3360-driver
      revision: main
  self:
    path: config
```

## Лицензия

Этот проект лицензирован под лицензией MIT - см. файл LICENSE для деталей.
