# Clone Project
```
git clone --recurse-submodules git@github.com:ZivLow/simple_step.git
```

## Fix for include path problems
[Configuration of c_cpp_properties.json file](https://github.com/espressif/vscode-esp-idf-extension/blob/master/docs/C_CPP_CONFIGURATION.md)
1. Press F1
2. "Add vscode configuration folder"

## Update submodules to latest version of remote
1. Navigate to the submodule directory:
```
cd components/as5600
```
```
cd components/TMC-API
```
2. Push updates to remote submodule:
```
git switch master
git fetch
git add .
git commit -S -m "update submodule"
git push
cd ../..
git submodule update --remote
git fetch
git status
```
3. Update main repository:
```
git add .
git commit -S -m "update submodule"
git push
git pull
git status
```

# ESP-IDF useful commands
## Create sdkconfig.defaults
Make changes after using menuconfig, then
```
idf.py save-defconfig
```

## Test environment
- ESP32 chip
- ESP-IDF v5.1
