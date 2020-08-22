
echo "Compiling main 3.4.7"
g++ -ggdb main.cpp -o adas `pkg-config --cflags --libs opencv` -latomic && echo "Done"
