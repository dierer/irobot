namespace irobot {
    export enum ScreenDirection {
        //% blockId="Vertical" block="竖屏"
        Vertical,
        //% blockId="Horizontal" block="横屏"
        Horizontal
    }

    export enum StringSize {
        //% blockId="SmallSize" block="16*16"
        SmallSize,
        //% blockId="MediumSize" block="24*24"
        MediumSize,
        //% blockId="LargeSize" block="32*32"
        LargeSize
    }

    export enum Pics {
        //% blockId="UpPic" block="向上图片"
        UpPic,
        //% blockId="RightPic" block="向右图片"
        RightPic,
        //% blockId="DownPic" block="向下图片"
        DownPic,
        //% blockId="LeftPic" block="向左图片"
        LeftPic,
        //% blockId="RightUpPic" block="右上图片"
        RightUpPic,
        //% blockId="RightDownPic" block="右下图片"
        RightDownPic,
        //% blockId="LeftUpPic" block="左上图片"
        LeftUpPic,
        //% blockId="LeftDownPic" block="左下图片"
        LeftDownPic,
        //% blockId="WifiPic" block="WIFI图片"
        WifiPic,
        //% blockId="LightingPic" block="打雷图片"
        LightingPic,
        //% blockId="CottaPic" block="短袖图片"
        CottaPic,
        //% blockId="WindPic" block="风图片"
        WindPic,
        //% blockId="DistancePic" block="距离图片"
        DistancePic,
        //% blockId="BluetoothPic" block="蓝牙图片"
        BluetoothPic,
        //% blockId="GetUpPic" block="起床图片"
        GetUpPic,
        //% blockId="SunnyPic" block="晴天图片"
        SunnyPic,
        //% blockId="SoundPic" block="声音图片"
        SoundPic,
        //% blockId="HumidityPic" block="湿度图片"
        HumidityPic,
        //% blockId="SleepPic" block="睡觉图片"
        SleepPic,
        //% blockId="WechatPic" block="微信图片"
        WechatPic,
        //% blockId="TemperaturePic" block="温度图片"
        TemperaturePic,
        //% blockId="SnowPic" block="雪图片"
        SnowPic,
        //% blockId="NightPic" block="夜间图片"
        NightPic,
        //% blockId="RainPic" block="雨天图片"
        RainPic,
        //% blockId="TShirtPic" block="长袖图片"
        TShirtPic
    }

    export enum Motor {
        //% blockId="LeftUpMotor" block="左前电机"
        LeftUpMotor,
        //% blockId="LeftDownMotor" block="左后电机"
        LeftDownMotor,
        //% blockId="RightUpMotor" block="右前电机"
        RightUpMotor,
        //% blockId="RightDownMotor" block="右后电机"
        RightDownMotor
    }

    export enum PS2Button {
        //% blockId="Left" block="向左方向键"
        Left,
        //% blockId="Down" block="向下方向键"
        Down,
        //% blockId="Right" block="向右方向键"
        Right,
        //% blockId="Up" block="向上方向键"
        Up,
        //% blockId="Start" block="开始(Start)按键"
        Start,
        //% blockId="Analog_Left" block="左侧摇杆按下"
        AnalogLeft,
        //% blockId="Analog_Right" block="右侧摇杆按下"
        AnalogRight,
        //% blockId="Select" block="选择(Select)按键"
        Select,
        //% blockId="Square" block="正方形(□)按键"
        Square,
        //% blockId="Cross" block="叉型(×)按键"
        Cross,
        //% blockId="Circle" block="圆型(○)按键"
        Circle,
        //% blockId="Triangle" block="三角形(△)按键"
        Triangle,
        //% blockId="R1" block="R1按键"
        R1,
        //% blockId="L1" block="L1按键"
        L1,
        //% blockId="R2" block="R2按键"
        R2,
        //% blockId="L2" block="L2按键"
        L2,
        //% blockId="Buttons" block="按键(空缺)"
        Buttons,
        //% blockId="RX" block="右侧摇杆X的值"
        RX,
        //% blockId="RY" block="右侧摇杆Y的值"
        RY,
        //% blockId="LX" block="左侧摇杆x的值"
        LX,
        //% blockId="LY" block="左侧摇杆Y的值"
        LY,
    };

    // I2C address for sht20
    const SHT20_ADDRESS = 0x40;
    // register address for sht20
    const SHT20_SOFT_RESET = 0xFE;
    const SHT20_T_HOLD = 0xE3;
    const SHT20_RH_HOLD = 0xE5;

    // initialzie flag for sht20
    let sht20Initialize = false;

    // initialzie flag for lcd
    let lcdInitialize = false;

    // I2C address for pca9685
    const PCA9685_ADDRESS = 0x40;
    // register address for pca9685
    const MODE1 = 0x00;
    const MODE2 = 0x01;
    const PRESCALE = 0xFE;
    const LED0_ON_L = 0x06;

    // initialzie flag for pca9685
    let pca9685Initialize = false;

    // initialize flag for ps2
    let ps2Initialize = false;
    let ps2Connect = false;
    // response buffer for ps2
    let respBuf = pins.createBuffer(6);

    // write register through i2c
    function i2cWriteReg(addr: number, reg: number, value: number): void {
        let buf = pins.createBuffer(2);
        buf[0] = reg;
        buf[1] = value;
        pins.i2cWriteBuffer(addr, buf);
    }

    // read register through i2c
    function i2cReadReg(addr: number, reg: number): number {
        pins.i2cWriteNumber(addr, reg, NumberFormat.UInt8BE);
        let value = pins.i2cReadNumber(addr, NumberFormat.UInt8BE);
        return value;
    }

    // init sht20
    function initSht20(): void {
        pins.i2cWriteNumber(SHT20_ADDRESS, SHT20_SOFT_RESET, NumberFormat.UInt8BE);
        basic.pause(10);
        sht20Initialize = true;
    }

    // read temperature from sht20
    //% blockId="irobot_temperature" block="温度测量值(℃)"
    //% weight=100
    //% group="机器人传感"
    export function readTemerature(): number {
        if (!sht20Initialize) {
            initSht20();
        }
        pins.i2cWriteNumber(SHT20_ADDRESS, SHT20_T_HOLD, NumberFormat.UInt8BE);
        basic.pause(90);
        let data = pins.i2cReadNumber(SHT20_ADDRESS, NumberFormat.UInt16BE);
        let temperature = data / 65536.0 * 175.72 - 46.85;
        return temperature;
    }

    // read humidity from sht20
    //% blockId="irobot_humidity" block="湿度测量值(\\%)"
    //% weight=99
    //% group="机器人传感"
    export function readHumidity(): number {
        if (sht20Initialize == false) {
            initSht20();
        }
        pins.i2cWriteNumber(SHT20_ADDRESS, SHT20_RH_HOLD, NumberFormat.UInt8BE);
        basic.pause(30);
        let data2 = pins.i2cReadNumber(SHT20_ADDRESS, NumberFormat.Int16BE);
        let humidity = data2 / 65536.0 * 125.0 - 6.0;
        return humidity;
    }

    // read distance from ultrasonic
    //% blockId="irobot_distance" block="超声波测距值(cm)"
    //% weight=98
    //% group="机器人传感"
    export function sonar(): number {
        let maxDistance = 500;
        pins.setPull(DigitalPin.P1, PinPullMode.PullNone);
        pins.digitalWritePin(DigitalPin.P1, 0);
        control.waitMicros(2);
        pins.digitalWritePin(DigitalPin.P1, 1);
        control.waitMicros(10);
        pins.digitalWritePin(DigitalPin.P1, 0);
        const data3 = pins.pulseIn(DigitalPin.P2, PulseValue.High, maxDistance * 58);
        return Math.idiv(data3, 58);
    }

    // init lcd
    function initLCD(): void {
        serial.redirect(SerialPin.P16, SerialPin.P12, BaudRate.BaudRate115200);
        basic.pause(10);
        lcdInitialize = true;
    }

    // clear screen use colour
    //% blockId="irobot_clear_lcd" block="清除屏幕: 颜色 %colour"
    //% weight=100
    //% colour.min=0 colour.max=63
    //% group="机器人屏幕"
    export function clearLCD(colour: number): void {
        if (!lcdInitialize) {
            initLCD();
        }
        serial.writeLine("CLR(" + colour.toString() + ");");
        basic.pause(40);
    }

    // change screen direction
    //% blockId="irobot_direction_lcd" block="改变屏幕方向: 方向 %direction"
    //% weight=99
    //% group="机器人屏幕"
    export function directionLCD(direction: ScreenDirection): void {
        if (!lcdInitialize) {
            initLCD();
        }
        let screenDir = "0";
        switch (direction) {
            case ScreenDirection.Vertical:
                screenDir = "0";
                break;
            case ScreenDirection.Horizontal:
                screenDir = "1";
                break;
        }
        serial.writeLine("DIR(" + screenDir + ");");
        basic.pause(20);
    }

    // change screen bright
    //% blockId="irobot_bright_lcd" block="改变屏幕亮度: 亮度 %brightness"
    //% weight=98
    //% brightness.min=0 brightness.max=255
    //% group="机器人屏幕"
    export function brightLCD(brightness: number): void {
        if (!lcdInitialize) {
            initLCD();
        }
        serial.writeLine("BL(" + brightness.toString() + ");");
        basic.pause(20);
    }

    // change screen background colour
    //% blockId="irobot_backround_lcd" block="改变屏幕背景色: 颜色 %colour"
    //% weight=97
    //% colour.min=0 colour.max=63
    //% group="机器人屏幕"
    export function backgroundLCD(colour: number): void {
        if (!lcdInitialize) {
            initLCD();
        }
        serial.writeLine("SBC(" + colour.toString() + ");");
        basic.pause(15);
    }

    // draw point on screen
    //% blockId="irobot_point_lcd" block="绘制单点: x %px|y %py|颜色 %colour"
    //% weight=96
    //% colour.min=0 colour.max=63
    //% group="机器人屏幕"
    export function pointLCD(x: number, y: number, colour: number): void {
        if (!lcdInitialize) {
            initLCD();
        }
        serial.writeLine("PS(" + x.toString() + "," + y.toString() + "," + colour.toString() + ");");
        basic.pause(25);
    }

    // draw line on screen
    //% blockId="irobot_line_lcd" block="绘制线条: xstart %xstart|ystart %ystart|xend %xend|yend %yend|颜色 %colour"
    //% weight=95
    //% colour.min=0 colour.max=63
    //% group="机器人屏幕"
    export function lineLCD(xstart: number, ystart: number, xend: number, yend: number, colour: number): void {
        if (!lcdInitialize) {
            initLCD();
        }
        serial.writeLine("PL(" + xstart.toString() + "," + ystart.toString() + ","
            + xend.toString() + "," + yend.toString() + "," + colour.toString() + ");");
        basic.pause(30);
    }

    // draw box on screen
    //% blockId="irobot_box_lcd" block="绘制矩形: xstart %xstart|ystart %ystart|xend %xend|yend %yend|颜色 %colour|填充 %brush"
    //% weight=94
    //% colour.min=0 colour.max=63
    //% group="机器人屏幕"
    export function boxLCD(xstart: number, ystart: number, xend: number, yend: number, colour: number, brush: boolean): void {
        if (!lcdInitialize) {
            initLCD();
        }
        if (!brush) {
            serial.writeLine("BOX(" + xstart.toString() + "," + ystart.toString() + ","
                + xend.toString() + "," + yend.toString() + "," + colour.toString() + ");");
        } else {
            serial.writeLine("BOXF(" + xstart.toString() + "," + ystart.toString() + ","
                + xend.toString() + "," + yend.toString() + "," + colour.toString() + ");");
        }
        basic.pause(35);
    }

    // draw circle on screen
    //% blockId="irobot_circle_lcd" block="绘制圆形: x %px|y %py|radius %r|颜色 %colour|填充 %brush"
    //% weight=93
    //% colour.min=0 colour.max=63
    //% group="机器人屏幕"
    export function circleLCD(x: number, y: number, r: number, colour: number, brush: boolean): void {
        if (!lcdInitialize) {
            initLCD();
        }
        if (!brush) {
            serial.writeLine("CIR(" + x.toString() + "," + y.toString() + ","
                + r.toString() + "," + colour.toString() + ");");
        } else {
            serial.writeLine("CIRF(" + x.toString() + "," + y.toString() + ","
                + r.toString() + "," + colour.toString() + ");");
        }
        basic.pause(35);
    }

    // draw string on screen
    //% blockId="irobot_string_lcd" block="显示字符: x %x|y %y|str %str|颜色 %colour|大小 %size|透明 %lucency"
    //% weight=92
    //% colour.min=0 colour.max=63
    //% group="机器人屏幕"
    export function stringLCD(x: number, y: number, str: string, colour: number, size: StringSize, lucency: boolean): void {
        if (!lcdInitialize) {
            initLCD();
        }
        if (size == StringSize.SmallSize) {
            if (lucency) {
                serial.writeLine("DC16(" + x.toString() + "," + y.toString() + ",'"
                    + str + "'," + colour.toString() + "," + ");");
            } else {
                serial.writeLine("DCV16(" + x.toString() + "," + y.toString() + ",'"
                    + str + "'," + colour.toString() + "," + ");");
            }
            basic.pause(35);
        } else if (size == StringSize.MediumSize) {
            if (lucency) {
                serial.writeLine("DC24(" + x.toString() + "," + y.toString() + ",'"
                    + str + "'," + colour.toString() + "," + ");");
            } else {
                serial.writeLine("DCV24(" + x.toString() + "," + y.toString() + ",'"
                    + str + "'," + colour.toString() + "," + ");");
            }
            basic.pause(40);
        } else if (size == StringSize.LargeSize) {
            if (lucency) {
                serial.writeLine("DC32(" + x.toString() + "," + y.toString() + ",'"
                    + str + "'," + colour.toString() + "," + ");");
            } else {
                serial.writeLine("DCV32(" + x.toString() + "," + y.toString() + ",'"
                    + str + "'," + colour.toString() + "," + ");");
            }
            basic.pause(50);
        }
    }

    // draw pic on screen
    //% blockId="irobot_pic_lcd" block="显示图片: x %x|y %y|pic %Pics|透明: %lucency"
    //% weight=91
    //% group="机器人屏幕"
    export function picLCD(x: number, y: number, pic: Pics, lucency: boolean): void {
        if (!lcdInitialize) {
            initLCD();
        }
        let picAddr = "";
        switch (pic) {
            case Pics.UpPic:
                picAddr = "2097152";
                break;
            case Pics.RightPic:
                picAddr = "2099200";
                break;
            case Pics.DownPic:
                picAddr = "2101248";
                break;
            case Pics.LeftPic:
                picAddr = "2103296";
                break;
            case Pics.RightUpPic:
                picAddr = "2105344";
                break;
            case Pics.RightDownPic:
                picAddr = "2107392";
                break;
            case Pics.LeftUpPic:
                picAddr = "2109440";
                break;
            case Pics.LeftDownPic:
                picAddr = "2111488";
            case Pics.WifiPic:
                picAddr = "2113536";
                break;
            case Pics.LightingPic:
                picAddr = "2115584";
                break;
            case Pics.CottaPic:
                picAddr = "2117632";
                break;
            case Pics.WindPic:
                picAddr = "2119680";
                break;
            case Pics.DistancePic:
                picAddr = "2121728";
                break;
            case Pics.BluetoothPic:
                picAddr = "2123776";
                break;
            case Pics.GetUpPic:
                picAddr = "2125824";
                break;
            case Pics.SunnyPic:
                picAddr = "2127872";
                break;
            case Pics.SoundPic:
                picAddr = "2129920";
                break;
            case Pics.HumidityPic:
                picAddr = "2131968";
                break;
            case Pics.SleepPic:
                picAddr = "2134016";
                break;
            case Pics.WechatPic:
                picAddr = "2136064";
                break;
            case Pics.TemperaturePic:
                picAddr = "2138112";
                break;
            case Pics.SnowPic:
                picAddr = "2140160";
                break;
            case Pics.NightPic:
                picAddr = "2142208";
                break;
            case Pics.RainPic:
                picAddr = "2144256";
                break;
            case Pics.TShirtPic:
                picAddr = "2146304";
                break;
        }
        if (lucency) {
            serial.writeLine("FSIMG(" + picAddr + "," + x.toString() + "," + y.toString() + ",32,32,1);");
        } else {
            serial.writeLine("FSIMG(" + picAddr + "," + x.toString() + "," + y.toString() + ",32,32,0);");
        }
    }

    // set frequency for pca9685
    function setFreqPca9685(freq: number): void {
        let prescale = 25000000 / 4096;
        prescale /= freq;
        prescale -= 1;
        let oldmode = i2cReadReg(PCA9685_ADDRESS, MODE1);
        let newmode = (oldmode & 0x7F) | 0x10;
        i2cWriteReg(PCA9685_ADDRESS, MODE1, newmode);
        i2cWriteReg(PCA9685_ADDRESS, PRESCALE, prescale);
        i2cWriteReg(PCA9685_ADDRESS, MODE1, oldmode);
        basic.pause(5);
        i2cWriteReg(PCA9685_ADDRESS, MODE1, oldmode | 0xA1);
    }

    // set channel pwm for pca9685
    function setPwmPca9685(channel: number, on: number, off: number): void {
        if (channel < 0 || channel > 15) {
            return;
        }
        let buf2 = pins.createBuffer(5);
        buf2[0] = LED0_ON_L + 4 * channel;
        buf2[1] = on & 0xff;
        buf2[2] = (on >> 8) & 0xff;
        buf2[3] = off & 0xff;
        buf2[4] = (off >> 8) & 0xff;
        pins.i2cWriteBuffer(PCA9685_ADDRESS, buf2);
    }

    // init pca9685
    function initPca9685(): void {
        i2cWriteReg(PCA9685_ADDRESS, MODE1, 0x20);
        setFreqPca9685(50);
        for (let idx = 0; idx < 16; idx++) {
            setPwmPca9685(idx, 0, 0);
        }
        pca9685Initialize = true;
    }

    // run motor for irobot
    //% blockId="irobot_run_motor" block="动作: 电机 %motor|速度 %speed"
    //% weight=100
    //% speed.min=-100 speed.max=100
    //% group="机器人运动"
    export function runMotor(motor: Motor, speed: number): void {
        if (!pca9685Initialize) {
            initPca9685();
        }
        // convert and limit speed
        speed = speed * 40;
        if (speed >= 4095) {
            speed = 4095;
        }
        if (speed <= -4095) {
            speed = -4095;
        }
        // run motor
        switch (motor) {
            case Motor.LeftUpMotor:
                if (speed >= 0) {
                    setPwmPca9685(2, 4095, 0);
                    setPwmPca9685(1, 0, 4095);
                    setPwmPca9685(0, 0, speed);
                } else {
                    setPwmPca9685(2, 0, 4095);
                    setPwmPca9685(1, 4095, 0);
                    setPwmPca9685(0, 0, -1 * speed);
                };
                break;
            case Motor.LeftDownMotor:
                if (speed >= 0) {
                    setPwmPca9685(3, 4095, 0);
                    setPwmPca9685(4, 0, 4095);
                    setPwmPca9685(5, 0, speed);
                } else {
                    setPwmPca9685(3, 0, 4095);
                    setPwmPca9685(4, 4095, 0);
                    setPwmPca9685(5, 0, -1 * speed);
                };
                break;
            case Motor.RightUpMotor:
                if (speed >= 0) {
                    setPwmPca9685(6, 4095, 0);
                    setPwmPca9685(7, 0, 4095);
                    setPwmPca9685(8, 0, speed);
                } else {
                    setPwmPca9685(6, 0, 4095);
                    setPwmPca9685(7, 4095, 0);
                    setPwmPca9685(8, 0, -1 * speed);
                };
                break;
            case Motor.RightDownMotor:
                if (speed >= 0) {
                    setPwmPca9685(9, 4095, 0);
                    setPwmPca9685(10, 0, 4095);
                    setPwmPca9685(11, 0, speed);
                } else {
                    setPwmPca9685(9, 0, 4095);
                    setPwmPca9685(10, 4095, 0);
                    setPwmPca9685(11, 0, -1 * speed);
                };
                break;
        }
    }

    // stop motor for irobot
    //% blockId="irobot_stop_motor" block="停止(全部)"
    //% weight=99
    //% group="机器人运动"
    export function stopMotor(): void {
        if (!pca9685Initialize) {
            initPca9685();
        }
        // stop left up motor
        setPwmPca9685(2, 0, 4095);
        setPwmPca9685(1, 4095, 0);
        setPwmPca9685(0, 0, 0);
        // stop left down motor
        setPwmPca9685(3, 0, 4095);
        setPwmPca9685(4, 4095, 0);
        setPwmPca9685(5, 0, 0);
        // stop right up motor
        setPwmPca9685(6, 0, 4095);
        setPwmPca9685(7, 4095, 0);
        setPwmPca9685(8, 0, 0);
        // stop right down motor
        setPwmPca9685(9, 0, 4095);
        setPwmPca9685(10, 4095, 0);
        setPwmPca9685(11, 0, 0);
    }

    // init ps2
    function initPs2(): void {
        pins.digitalWritePin(DigitalPin.P0, 1);
        pins.spiPins(DigitalPin.P14, DigitalPin.P15, DigitalPin.P13);
        pins.spiFormat(8, 3);
        pins.spiFrequency(250000);
        ps2Initialize = true;
    }

    // send command for ps2
    function sendCommand(): Buffer {
        let command = hex`804200000000000000`;
        let cmdBuf = pins.createBuffer(command.length);
        pins.digitalWritePin(DigitalPin.P0, 0);
        for (let n = 0; n < command.length; n++) {
            cmdBuf[n] = pins.spiWrite(command[n]);
        }
        pins.digitalWritePin(DigitalPin.P0, 1);
        return cmdBuf;
    }

    // 手柄按键轮询函数
    function poll(): void {
        if (!ps2Initialize) {
            initPs2();
        }
        let cmdBuf2 = sendCommand();
        if (cmdBuf2[2] != 0x5A) {
            ps2Connect = false;
        } else {
            for (let o = 0; o < 6; o++) {
                respBuf[o] = cmdBuf2[o + 3];
            }
            ps2Connect = true;
        }
    }

    // judge button from ps2
    //% blockId=irobot_ps2 block="设置PS2手柄: %b|按下"
    //% weight=98
    //% group="机器人遥控"
    export function buttonPressed(button: PS2Button): number {
        if (!ps2Connect) {
            return 0x00;
        }
        switch (button) {
            case PS2Button.Left:
                return respBuf[0] & 0x01 ? 0 : 1;
            case PS2Button.Down:
                return respBuf[0] & 0x02 ? 0 : 1;
            case PS2Button.Right:
                return respBuf[0] & 0x04 ? 0 : 1;
            case PS2Button.Up:
                return respBuf[0] & 0x08 ? 0 : 1;
            case PS2Button.Start:
                return respBuf[0] & 0x10 ? 0 : 1;
            case PS2Button.AnalogLeft:
                return respBuf[0] & 0x20 ? 0 : 1;
            case PS2Button.AnalogRight:
                return respBuf[0] & 0x40 ? 0 : 1;
            case PS2Button.Select:
                return respBuf[0] & 0x80 ? 0 : 1;
            case PS2Button.Square:
                return respBuf[1] & 0x01 ? 0 : 1;
            case PS2Button.Cross:
                return respBuf[1] & 0x02 ? 0 : 1;
            case PS2Button.Circle:
                return respBuf[1] & 0x04 ? 0 : 1;
            case PS2Button.Triangle:
                return respBuf[1] & 0x08 ? 0 : 1;
            case PS2Button.R1:
                return respBuf[1] & 0x10 ? 0 : 1;
            case PS2Button.L1:
                return respBuf[1] & 0x20 ? 0 : 1;
            case PS2Button.R2:
                return respBuf[1] & 0x40 ? 0 : 1;
            case PS2Button.L2:
                return respBuf[1] & 0x80 ? 0 : 1;
            case PS2Button.RX:
                return respBuf[2] - 0x80;
            case PS2Button.RY:
                return respBuf[3] - 0x80;
            case PS2Button.LX:
                return respBuf[4] - 0x80;
            case PS2Button.LY:
                return respBuf[5] - 0x80;
        }
        return 0x00;
    }

    // forever run for ps2
    basic.forever(function () {
        poll();
    })
}
