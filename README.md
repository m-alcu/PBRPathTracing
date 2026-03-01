# 3D Rendering Engine

Software 3D engine using SDL3, Dear ImGui, and modern CMake.

## Dependencias
- **SDL3** (descargado automáticamente via CMake FetchContent).
- **rapidobj** (descargado automáticamente via CMake FetchContent) - Parser de archivos OBJ.
- **yaml-cpp** (descargado automáticamente via CMake FetchContent) - Parser de archivos YAML para escenas.
- **Dear ImGui** (incluido en `src/vendor/imgui/`).
- **stb_image** (incluido en `src/vendor/stb/`) - Carga de imágenes (PNG, JPG, BMP, TGA, etc.).
- **Google Test** (descargado via CMake FetchContent para tests).
- **CMake** ≥ 3.28.
- Toolchain C++17 (GCC/Clang/MSVC o `emcc`).
- Opcional: Emscripten SDK para builds WebAssembly.

## Configuración y compilación (desktop)
1. Generar archivos de construcción:
   ```bash
   cmake -S . -B build
   ```
2. Compilar el binario:
   ```bash
   cmake --build build
   ```
3. Ejecutar el motor:
   ```bash
   ./build/bin/3DEngine
   ```

Ejecuta el binario desde la raíz del repositorio para que los paths relativos de `resources/` funcionen correctamente.

## Tests

El proyecto incluye tests unitarios usando Google Test. Para compilar y ejecutar los tests:

```bash
# Configurar (incluye tests por defecto)
cmake -S . -B build

# Compilar tests específicos
cmake --build build --target test_math --config Release
cmake --build build --target test_ecs --config Release

# Ejecutar tests directamente
./build/bin/Release/test_math.exe   # Windows
./build/bin/test_math               # Linux/macOS
./build/bin/Release/test_ecs.exe    # Windows
./build/bin/test_ecs                # Linux/macOS

# O usar CTest (todos los tests)
cd build && ctest -C Release --output-on-failure
```

Para desactivar la compilación de tests:
```bash
cmake -S . -B build -DBUILD_TESTS=OFF
```

## Escenas disponibles
Seleccionables desde el combo "Scene" en la ventana de ImGui:
- **Torus**, **Tetrakis**, **Icosahedron**, **Cube**, **Knot**, **Star**: primitivas/solids paramétricos con rotación automática opcional.
- **Amiga**: escena inspirada en el clásico logo/banda Amiga.
- **World**: escena que carga geometría desde recursos externos.

## Skybox / Cubemap

El motor soporta fondos de tipo skybox mediante cubemaps de 6 caras. Se pueden seleccionar desde el combo "Background" en ImGui o configurar en el archivo YAML de la escena:

```yaml
scene:
  background: skybox
  skybox:
    px: "resources/skybox/px.png"
    nx: "resources/skybox/nx.png"
    py: "resources/skybox/py.png"
    ny: "resources/skybox/ny.png"
    pz: "resources/skybox/pz.png"
    nz: "resources/skybox/nz.png"
```

Las caras usan nombres de ejes: `px`/`nx` = +X/-X, `py`/`ny` = +Y/-Y, `pz`/`nz` = +Z/-Z. El skybox rota con la cámara.

Se pueden descargar texturas de skybox gratuitas desde: https://freestylized.com/all-skybox/

## Controles principales
- **Movimiento estilo Descent**: Flechas o keypad para pitch/yaw, `Q`/`E` (o keypad 7/9) para roll, `A`/`Z` (o keypad ±) para avanzar/retroceder.
- **Órbita con el ratón**: mantener clic derecho y arrastrar para orbitar; rueda del ratón para acercar/alejar. Se desactiva el modo vuelo libre mientras se orbita.
- **ImGui**: ajustar velocidad y sensibilidad de cámara, sombreado de los sólidos, tipo de fondo y la escena activa.
- **Escape**: salir.

## Problemas comunes
- **Assets no encontrados**: asegúrate de ejecutar el binario desde la raíz del proyecto para que los paths relativos apunten a `resources/`. Si lo lanzas desde otro directorio, usa `--workdir` o ajusta las rutas de recursos en el código.
- **SDL3 no detectado**: verifica que los headers y la librería estén en las rutas de tu toolchain (`CMAKE_PREFIX_PATH`, `SDL3_DIR`, o variables de entorno como `PKG_CONFIG_PATH`).
- **Compilador antiguo**: se requiere un compilador con soporte C++17; actualiza GCC/Clang o usa el toolchain provisto por tu SDK.

## Build en Raspberry Pi (Raspberry Pi OS / Debian)

SDL3 requiere varias bibliotecas de desarrollo X11 y del sistema para compilar en Raspberry Pi. Instálalas antes de ejecutar CMake.

### Instalación mínima (X11 + gráficos básicos)

```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential cmake pkg-config git \
    libx11-dev libxext-dev libxrender-dev \
    libxrandr-dev libxcursor-dev libxfixes-dev \
    libxi-dev libxss-dev libxtst-dev libxkbcommon-dev \
    libgl1-mesa-dev libegl1-mesa-dev libgles2-mesa-dev \
    libdrm-dev libgbm-dev \
    libudev-dev libdbus-1-dev
```

### Instalación recomendada (X11 + Wayland + Audio)

```bash
sudo apt-get update
sudo apt-get install -y \
    build-essential cmake pkg-config git \
    libx11-dev libxext-dev libxrender-dev \
    libxrandr-dev libxcursor-dev libxfixes-dev \
    libxi-dev libxss-dev libxtst-dev libxkbcommon-dev \
    libwayland-dev wayland-protocols libdecor-0-dev \
    libegl1-mesa-dev libgles2-mesa-dev libgl1-mesa-dev \
    libdrm-dev libgbm-dev \
    libasound2-dev libpulse-dev \
    libudev-dev libdbus-1-dev \
    libraspberrypi-dev
```

### Detalle de paquetes X11

| Paquete | Propósito |
|---------|-----------|
| `libx11-dev` | Biblioteca principal X11 |
| `libxext-dev` | Extensiones X11 |
| `libxrender-dev` | Extensión de renderizado X11 |
| `libxrandr-dev` | Gestión de resolución y pantallas |
| `libxcursor-dev` | Soporte avanzado de cursor |
| `libxfixes-dev` | Correcciones misceláneas X11 |
| `libxi-dev` | XInput 2 (multi-touch, dispositivos de entrada) |
| `libxss-dev` | Extensión de salvapantallas |
| `libxtst-dev` | Extensión XTest |
| `libxkbcommon-dev` | Mapeo de teclado (X11 y Wayland) |
| `libdrm-dev` | Direct Rendering Manager (KMS/DRM) |
| `libgbm-dev` | Generic Buffer Management (KMS/DRM) |
| `libegl1-mesa-dev` | Abstracción gráfica EGL |
| `libgles2-mesa-dev` | OpenGL ES 2.0 |
| `libgl1-mesa-dev` | OpenGL |
| `libudev-dev` | Enumeración de dispositivos (joysticks, input) |
| `libdbus-1-dev` | Mensajería D-Bus del sistema |
| `libraspberrypi-dev` | Acceso GPU VideoCore de Raspberry Pi |

### Compilación en Raspberry Pi

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --parallel $(nproc)
./build/bin/3DEngine
```

## Build para WebAssembly (Emscripten)
1. Activa el entorno de Emscripten (`source /path/to/emsdk_env.sh`).
2. Genera con la toolchain de Emscripten (puedes usar `emcmake` para simplificar las rutas):
   ```bash
   emcmake cmake -S . -B build-wasm -DCMAKE_BUILD_TYPE=Release
   ```
3. Compila:
   ```bash
   cmake --build build-wasm
   ```
4. Sirve los artefactos generados (HTML/JS/WASM) con un servidor web y abre el HTML principal en el navegador. Asegúrate de habilitar el preloading de recursos en el script de despliegue si necesitas texturas o escenas externas.
