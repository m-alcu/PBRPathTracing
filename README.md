# PBR Path Tracer

A physically-based CPU path tracer written in C++17, rendered progressively into an SDL3 streaming texture with a Dear ImGui overlay.

---

## Features

- **Unbiased Monte Carlo path tracing** with cosine-weighted importance sampling
- **Lambertian (diffuse) BRDF** with correct throughput accounting
- **Russian roulette** path termination
- **Binary BVH** (bounding volume hierarchy) over triangle meshes for fast ray–mesh intersection
- **Analytical sphere** intersection in the same scene
- **Reinhard tone mapping** + gamma-2 correction
- **Progressive accumulation**: each frame adds one sample per pixel; the image refines indefinitely
- **Multi-threaded rendering**: one `std::thread` per CPU core, row-striped (no contention)
- **SDL3 streaming texture**: the float accumulation buffer is tone-mapped and uploaded to the GPU every frame via `SDL_UpdateTexture`
- **YAML scene files**: define camera, materials (albedo + emission), OBJ meshes and spheres
- **Interactive orbit camera**: left-drag to orbit, scroll to zoom — resets accumulation on change
- **Dear ImGui panel**: live SPP counter, FPS, thread count, scene switcher

---

## Theory

### 1. The Rendering Equation

The foundation of the renderer is Kajiya's rendering equation (1986):

```math
L_o(\mathbf{x},\,\omega_o) = L_e(\mathbf{x},\,\omega_o) + \int_{\Omega} f_r(\mathbf{x},\,\omega_i,\,\omega_o)\,L_i(\mathbf{x},\,\omega_i)\,(\omega_i \cdot \mathbf{n})\,d\omega_i
```

| Symbol | Meaning |
|--------|---------|
| `Lo(x, ωo)` | Outgoing radiance at surface point `x` in direction `ωo` |
| `Le(x, ωo)` | Emitted radiance (non-zero only for light sources) |
| `fr(x, ωi, ωo)` | Bidirectional Reflectance Distribution Function (BRDF) |
| `Li(x, ωi)` | Incoming radiance from direction `ωi` |
| `ωi · n` | Cosine of the angle between incident direction and surface normal |
| `Ω` | Hemisphere above the surface point |

The integral has no closed form for general scenes, so we solve it with Monte Carlo estimation.

---

### 2. Monte Carlo Estimation

A Monte Carlo estimator for an integral `∫ f(x) dx` is:

```math
\hat{I} \approx \frac{1}{N} \sum_{i=1}^{N} \frac{f(x_i)}{p(x_i)}
```

where $x_i$ are samples drawn from probability density $p$. Applied to the rendering equation:

```math
L_o \approx L_e + \frac{1}{N} \sum_{i=1}^{N} \frac{f_r(\omega_i)\,L_i(\omega_i)\,(\omega_i \cdot \mathbf{n})}{p(\omega_i)}
```

Each sample traces one random path through the scene. After `N` samples the estimator converges to the true solution; variance decreases as `1/√N`.

---

### 3. Lambertian BRDF

The Lambertian (perfectly diffuse) BRDF is:

```math
f_r(\mathbf{x},\,\omega_i,\,\omega_o) = \frac{\rho}{\pi}
```

It is constant — scattering is equal in all directions — and $\rho \in [0,1]^3$ is the albedo (fraction of light reflected per colour channel). The factor $1/\pi$ normalises energy conservation:

```math
\int_{\Omega} f_r\,(\omega_i \cdot \mathbf{n})\,d\omega_i = \int_{\Omega} \frac{\rho}{\pi}\cos\theta\,d\omega_i = \rho
```

---

### 4. Cosine-Weighted Hemisphere Sampling

Naïve uniform hemisphere sampling has high variance because the `cos(θ)` term in the integrand approaches zero near the horizon. **Cosine-weighted** sampling draws `ωi` proportional to `cos(θ)`, exactly matching that factor:

```math
p(\omega_i) = \frac{\cos\theta}{\pi}
```

Substituting into the Monte Carlo estimator for a Lambertian surface:

```math
\frac{f_r\,(\omega_i \cdot \mathbf{n})}{p(\omega_i)} = \frac{(\rho/\pi)\cos\theta}{\cos\theta/\pi} = \rho
```

The $\cos\theta$ and $\pi$ factors cancel exactly, so the **throughput update per bounce is simply**:

```math
\beta \leftarrow \beta \otimes \rho \qquad (\otimes = \text{component-wise multiply})
```

This is what `tracePath()` does — no extra cosine evaluation needed.

#### Sampling formula (Malley's method)

Given two uniform random numbers $u_1, u_2 \in [0,1)$:

```math
r = \sqrt{u_1}, \qquad \varphi = 2\pi u_2
```

```math
x = r\cos\varphi, \quad y = r\sin\varphi, \quad z = \sqrt{1 - u_1}
```

where $z$ is the "up" axis, aligned with the surface normal.

This samples the unit disk uniformly and projects it up onto the hemisphere, producing the cosine-weighted distribution. Implemented in `sampleCosineHemisphere()`.

---

### 5. Orthonormal Basis (ONB) Construction

The sampled direction `(x, y, z)` is in a local frame where `z = n̂`. To transform it to world space we build a tangent frame `(T, B, N)` using:

```
up = (|n.z| < 0.999) ? (0,0,1) : (0,1,0)    ← avoid parallel case
T  = normalize(up × n)
B  = n × T
```

Then the world-space bounce direction is:

```math
\omega_i = \mathrm{normalize}(T\,x + B\,y + \mathbf{n}\,z)
```

Implemented in `makeONB()`.

---

### 6. Path Throughput and Recursive Estimator

The path tracer unrolls the recursive rendering equation into a loop. It maintains a **throughput** vector `β` that accumulates the product of BRDFs and sampling weights along the path:

```math
\beta_0 = (1,1,1), \qquad \beta_{k+1} = \beta_k \otimes \rho
```

At each bounce, the emitted radiance is accumulated:

```math
L \mathrel{+}= \beta \cdot L_e
```

When the path escapes the scene (no intersection), the sky radiance is added:

```math
L \mathrel{+}= \beta \cdot L_{\mathrm{sky}}(\omega_i)
```

where the sky is a simple gradient:

```math
L_{\mathrm{sky}}(\mathbf{d}) = (1-t)\,(1,1,1) + t\,(0.5,\,0.7,\,1.0), \qquad t = \tfrac{1}{2}(d_y + 1)
```

---

### 7. Russian Roulette Path Termination

Paths that contribute little energy waste computation. **Russian roulette** terminates a path with probability `(1 − p)` and, if it survives, boosts the throughput to keep the estimator unbiased:

```math
p = \mathrm{clamp}\!\left(\max(\beta_r,\,\beta_g,\,\beta_b),\;0.05,\;0.95\right)
```

```
if rand() > p: terminate path
```

```math
\beta \leftarrow \frac{\beta}{p}
```

Applied from `depth ≥ 3`, this eliminates low-contribution paths while maintaining an unbiased estimate (the expected value of `β/p` equals `β`).

---

### 8. Ray–Triangle Intersection (Möller–Trumbore)

Given ray `r(t) = o + t·d` and triangle vertices `v0, v1, v2`:

```
e1 = v1 − v0
e2 = v2 − v0
h  = d × e2
a  = e1 · h               ← if |a| < ε: ray is parallel to triangle

f  = 1/a
s  = o − v0
u  = f · (s · h)          ← barycentric u; reject if u ∉ [0,1]

q  = s × e1
v  = f · (d · q)          ← barycentric v; reject if v < 0 or u+v > 1

t  = f · (e2 · q)         ← ray parameter; reject if t < ε (behind or self-hit)
```

The hit point is $\mathbf{p} = \mathbf{o} + t\,\mathbf{d}$. The shading normal is interpolated from per-vertex normals:

```math
\mathbf{n} = \mathrm{normalize}\!\left(\mathbf{n}_0(1-u-v) + \mathbf{n}_1\,u + \mathbf{n}_2\,v\right)
```

---

### 9. Ray–Sphere Intersection

For sphere centre $\mathbf{c}$ and radius $r$, substitute $\mathbf{r}(t)$ into $|\mathbf{p} - \mathbf{c}|^2 = r^2$:

```math
\mathbf{oc} = \mathbf{o} - \mathbf{c}, \quad b = \mathbf{oc} \cdot \mathbf{d}, \quad \Delta = b^2 - \left(|\mathbf{oc}|^2 - r^2\right)
```

If $\Delta < 0$: miss. Otherwise:

```math
t = -b - \sqrt{\Delta} \quad \text{(near root)}; \quad \text{if } t < \varepsilon,\; t = -b + \sqrt{\Delta}
```

Surface normal at hit: $\mathbf{n} = \mathrm{normalize}(\mathbf{p} - \mathbf{c})$.

---

### 10. Binary BVH

Triangle meshes are accelerated with a top-down **binary BVH** built by recursive spatial median splitting:

1. **Compute AABB** of the current triangle range.
2. **Choose split axis**: the dimension with the largest extent (`max(dx, dy, dz)`).
3. **Sort** triangle centroids along that axis.
4. **Split** at the median; recurse left and right.
5. **Leaf** when ≤ 4 triangles remain.

Traversal uses the **slab test** for AABB intersection:

```
for each axis i:
    t0 = (min[i] − o[i]) / d[i]
    t1 = (max[i] − o[i]) / d[i]
    if d[i] < 0: swap(t0, t1)
    tMin = max(tMin, t0)
    tMax = min(tMax, t1)
    if tMax ≤ tMin: miss
```

Early exit uses the current best hit `t` as `tMax`, discarding nodes that cannot improve the result.

---

### 11. Tone Mapping and Display

The accumulation buffer stores HDR linear radiance as `Vec3`. Each frame:

1. **Average** $N$ samples: $c = \text{accum}[i] / N$
2. **Reinhard** per-channel: $c' = c\,/\,(1+c)$ — maps $[0,\infty) \to [0,1)$
3. **Gamma-2** (sRGB approximation): $c'' = \sqrt{c'}$
4. **Pack** to ARGB8888: `(0xFF << 24) | (r << 16) | (g << 8) | b`
5. **Upload** via `SDL_UpdateTexture` and present with `SDL_RenderTexture`

---

## Project Structure

```
src/
  main.cpp                  SDL3 window + ImGui + progressive render loop
  constants.hpp             SCREEN_WIDTH, SCREEN_HEIGHT, SCENES_PATH
  pt/
    math.hpp                Vec3 (all ops + operator[]), Ray
    material.hpp            Material {albedo, emission},  Hit {t, p, n, matId}
    rng.hpp                 PCG32 random number generator
    scene.hpp               Camera · Triangle · Sphere · AABB · BVHNode · PBRScene
    tracer.hpp              sky() · sampleCosineHemisphere() · makeONB() · tracePath()
    scene_loader.hpp/.cpp   PBRSceneLoader::loadFromFile() — yaml-cpp + tinyobjloader
  vendor/
    imgui/                  Dear ImGui + SDL3 backends
    nothings/               stb_image
    tinyobjloader/          tinyobjloader
  slib.hpp/cpp              Legacy math library (kept for unit tests)
  smath.hpp/cpp             Legacy matrix math  (kept for unit tests)

resources/
  scenes/
    spheres.yaml            3 coloured spheres + ground + area light
    cornell.yaml            Cornell box approximated with spheres
    suzanne.yaml            Blender's Suzanne mesh
    bunny.yaml              Stanford bunny
  objs/                     OBJ mesh files

tests/
  test_math.cpp             Unit tests for legacy math library
```

---

## Scene File Format

```yaml
scene:
  name: "My Scene"

  camera:
    position: [0.0, 1.0, 4.0]   # world-space eye position
    target:   [0.0, 0.0, 0.0]   # look-at point
    fov: 45.0                    # vertical field of view (degrees)

  materials:
    - name: red_diffuse
      albedo:   [0.8, 0.1, 0.1]  # diffuse reflectance [0..1] per channel
      emission: [0.0, 0.0, 0.0]  # emitted radiance (set > 0 for area lights)

    - name: warm_light
      albedo:   [0.0, 0.0, 0.0]
      emission: [12.0, 10.0, 8.0]

  objects:
    - type: sphere
      center:   [0.0, 0.0, 0.0]
      radius:   0.5
      material: red_diffuse

    - type: obj
      file:     "resources/objs/suzanne.obj"
      material: red_diffuse
```

---

## Controls

| Input | Action |
|-------|--------|
| Left-drag | Orbit camera (resets SPP) |
| Scroll wheel | Zoom in/out (resets SPP) |
| Scene combo (ImGui) | Switch scene |
| Reset button (ImGui) | Clear accumulation buffer |
| Escape | Quit |

---

## Build

```bash
# Configure
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release

# Build (parallel)
cmake --build build --parallel $(nproc)

# Run from the project root so resources/ paths resolve correctly
./build/bin/PBRPathTracing
```

### Dependencies (fetched automatically by CMake)

| Library | Version | Purpose |
|---------|---------|---------|
| SDL3 | 3.4.0 | Window, input, streaming texture |
| yaml-cpp | 0.9.0 | Scene file parsing |
| Google Test | 1.14.0 | Unit tests (optional, `BUILD_TESTS=OFF` to skip) |

Vendored (in `src/vendor/`): Dear ImGui, stb_image, tinyobjloader.

### System dependencies (Linux / Raspberry Pi)

```bash
sudo apt-get install -y \
    build-essential cmake pkg-config git \
    libx11-dev libxext-dev libxrender-dev libxrandr-dev \
    libxcursor-dev libxfixes-dev libxi-dev libxss-dev \
    libxkbcommon-dev libwayland-dev wayland-protocols \
    libegl1-mesa-dev libgles2-mesa-dev libgl1-mesa-dev \
    libdrm-dev libgbm-dev libudev-dev libdbus-1-dev
```

### Unit tests

```bash
cmake --build build --target test_math
./build/bin/test_math
# or via CTest
cd build && ctest --output-on-failure
```
