# 🌸 PetalPID

## Lightweight universal PID controller library with Ziegler–Nichols auto tuning and variable cycle time

----------

## Perfect PID for your project

**PetalPID** is a floating-point based PID library that can be used on Arduino as well as on any other platform. Even in regular C++ code (it is possible to convert it to pure C). Unlike other libraries, **PetalPID** uses variable cycle time and the same function for the main and tuning loop, making it very easy to implement.

> 🚧 This's not the release version of PetalPID! Please be careful!

----------

## 🏗️ Getting started

> Example of auto-tuning process (this graph is produced by `examples/TemperatureController`)
![Auto-tunning PID](assets/Snapshot%20[02:02:11].svg)

1. Download or include this repo as a library

    Example for PlatformIO:

    ```ini
    lib_deps =
        https://github.com/F33RNI/PetalPID.git
    ```

2. Include header file in your project

    ```cpp
    #include <PetalPID.h>
    ```

3. Create PetalPID instance

    ```cpp
    PetalPID pid = PetalPID();

    // Or
    PetalPID pid = PetalPID(1.f, 2.f, 3.f, 0.f, 255.f);
    ```

4. Setup limits and gains (if you're not starting with auto-tune)

    ```cpp
    pid.set_min_max_output(0.f, 255.f);
    pid.set_min_max_integral(-1000.f, 1000.f);
    
    pid.set_gains(1.f, 2.f, 3.f);
    ```

5. In a main loop...

    ```cpp
    // Without any delays. Call as frequent as possible
    float pid_output = pid.calculate(measured_value, setpoint, micros());
    ```

6. Starting auto-tuning

    ```cpp
    // TYPE_P, TYPE_PI, TYPE_PD, TYPE_CLASSIC_PID, TYPE_PESSEN_INTEGRAL_RULE, TYPE_SOME_OVERSHOOT, TYPE_NO_OVERSHOOT
    // 50 on-off cycles
    pid.start_auto_tune(TYPE_NO_OVERSHOOT, 50);
    ```

7. Getting results

    ```cpp
    Serial.print(pid.get_p());
    Serial.print("\t");
    Serial.print(pid.get_i());
    Serial.print("\t");
    Serial.println(pid.get_d());
    ```

**More info in `📄 Documentation` section and `examples/`**

----------

## 📄 Documentation

You can build Doxygen documentation using provided `Doxygen` file

Just clone repo and run:

```shell
doxygen
```

This will generate HTML and LaTeX docs. Open `docs/index.html` file to view it

----------

## 💡 Projects using PetalPID

- [in17clock](https://github.com/F33RNI/in17clock) by Fern Lane
    > IN17 Nixie tube clock with internal DC-DC converter, random melody alarm and temperature/humidity sensor

Does your project also use **PetalPID**? Add it to this list by creating a pull request or issue!

----------

## ✨ Contribution

1. Clone this repo

  ```shell
  git clone https://github.com/F33RNI/PetalPID
  ```

2. Follow the `.clang-format` style while editing the code
3. Follow [Conventional Commits](https://www.conventionalcommits.org/en/v1.0.0/#specification>) while creating commits
4. Create pull request
