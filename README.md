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

| [01 - Basic Raytracing](/examples/01-basic-raytracing.cc) | 02 - Diffuse Reflection | 03 - Specular Reflection |
|---|---|---|
| ![Basic Raytracing](/results/01-basic-raytracing.bmp) |  |  |
