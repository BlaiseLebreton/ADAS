echo "Compiling main 3.4.7"
g++ -ggdb rs232.c liaison.cpp main.cpp -o output `pkg-config --cflags --libs opencv` -latomic -Wno-write-strings && echo "Done"
