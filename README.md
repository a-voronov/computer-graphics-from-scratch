# Computer Graphics from Scratch

C++ implementation of [Computer Graphics from Scratch](https://gabrielgambetta.com/computer-graphics-from-scratch/) book for self-studying purposes.

The book is using JavaScript and HTML Canvas to implement logic and display results.
It seems to me that the easiest implementation in C++ without using any libraries, is to render results straight into the bmp files. So I've added a quick [bmp implementation](/examples/bmp.h).

## Run Examples

```
mkdir -p bin results

clang++ -std=c++17 examples/<example>.cc -o bin/<example>

bin/<example> && open results/<example>.bmp
```

## Results

### Raytracing

| [01 - Basic Raytracing](/examples/01-basic-raytracing.cc) | [02 - Diffuse Reflection](/examples/02-diffuse-reflection.cc) | [03 - Specular Reflection](/examples/03-specular-reflection.cc) |
|---|---|---|
| ![Basic Raytracing](/results/01-basic-raytracing.bmp) | ![Diffuse Reflection](/results/02-diffuse-reflection.bmp) | ![Specular Reflection](/results/03-specular-reflection.bmp) |

| [04 - Shadows](/examples/04-shadows.cc) | [05 - Reflections](/examples/05-reflections.cc) | [06 - Camera](/examples/06-camera.cc) |
|---|---|---|
| ![Shadows](/results/04-shadows.bmp) | ![Reflections](/results/05-reflections.bmp) | ![Camera](/results/06-camera.bmp) |

Here I had to introduce few changes to the code.

First, a "shadow acne" appeared on the yellow ball when I've added a reflections computation.
We're recursively firing rays that are very close to the surface, and a floating-point error can be accumulated after each bounce.
The book examples don't have it since, they are using JS language where numbers have double-precision, while here I'm using a float which has a single-precision.

Another thing I've noticed is that when I was using `uint8_t` values to represent a color ranging from 0 to 255, I was getting a bit different results with lights and reflections.
So I switched to `float` with 0 to 1 range, which allowed me to represent more variety of colors during computations. I also didn't need to clamp values anymore until the very last moment when converting the image data into the bmp file format.

| Shadow Acne | Color uint8_t | Color float |
| --- | --- | --- |
| ![Shadow Acne](/results/05-shadow-acne.bmp) | ![Color uint8_t](/results/05-old-colors.bmp) | ![Color float](/results/05-reflections.bmp) |

### Extending the Raytracer

Here I've implemented shadow optimization and bounding volume hierarchy tree (BVH) myself as an extra exploration of the topic.
I've also added a subsampled option to see what effect it has on quality and performance.

| Original | BVH & Shadow Optimizations | Subsampling |
| --- | --- | --- |
| ![Original](/results/06-camera.bmp) | ![BVH & Shadow Optimizations](/results/07-optimizations.bmp) | ![Subsampling](/results/07-optimizations-subsampling.bmp) |

Next, based on the existing BVH tree and Object abstraction, I've added Triangles to the scene.

| Other Primitives | Constructive Solid Geometry | Transparency |
| --- | --- | --- |
| ![Other Primitives](/results/08-triangles.bmp) | ![Constructive Solid Geometry](/results/09-constructive-solid-geometry.bmp) | ![Transparency](/results/10-transparency.bmp) |