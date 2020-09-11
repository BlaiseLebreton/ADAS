
echo "Compiling main 4.4"
g++ -ggdb main.cpp -o adas `pkg-config --cflags --libs opencv4` -latomic && echo "Done"
