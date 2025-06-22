# Computer Graphics from Scratch

C++ implementation of [Computer Graphics from Scratch](https://gabrielgambetta.com/computer-graphics-from-scratch/) book.

The book is using JavaScript and HTML Canvas to implement logic and display results.
It seems to me that the easiest implementation in C++ without using any libraries, is to render results into the bmp files.

## Run Examples

```
mkdir -p bin gen

clang++ -std=c++17 examples/<example>.cc -o bin/<example>

bin/<example> && open gen/<example>.bmp
```
