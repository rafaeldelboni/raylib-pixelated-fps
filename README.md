# raylib-pixalated-fps

Testing raylib with some pixalated shader.

## Compiling / Building

### Installing Dependencies

#### Arch Linux
```bash
yay -Sy clang cmake lcov glfw libx11 libxcursor libxinerama libxrandr vulkan-headers xorg-server-devel xorg-xinput
```

#### Ubuntu
```bash
sudo apt-get install clang cmake lcov libasound2-dev mesa-common-dev libx11-dev libxrandr-dev libxi-dev xorg-dev libgl1-mesa-dev libglu1-mesa-dev
```

## Usage

### Setup
This will build cmake files and download dependencies
```bash
make setup
```

### Build
```bash
# Debug build
make debug
# Release build
make release
```

### Run Tests
This will build and run unity tests
```bash
make test
```

#### Code Coverage Checks
This will build and run unity tests and generate reports for coverage (depends on lcov)
```bash
make coverage
# This part is optional: Generates a html with more coverage details
genhtml build/coverage/coverage.info --output-directory build/coverage/out
```

### Clean
This will delete generated files for debug and release
```bash
make clean
```

### Run Binary
This will open an Raylib white window with the unlicense logo on it.
```bash
# Debug bin
./build/debug/bin/example_app
# Release bin
./build/release/bin/example_app
```

## License
This is free and unencumbered software released into the public domain.  
For more information, please refer to <http://unlicense.org/>
