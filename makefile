
echo "Compiling main 4.4"
g++ -ggdb main.cpp -o output `pkg-config --cflags --libs opencv4` -latomic && echo "Done"
echo "Nana was here"
