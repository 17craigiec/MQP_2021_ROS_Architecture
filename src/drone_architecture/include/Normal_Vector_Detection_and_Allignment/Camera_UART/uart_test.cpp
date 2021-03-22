#include <fstream>
#include <iostream>

using namespace std;

int main(int argc, char const *argv[])
{
    ifstream myfile ("/dev/ttyAMA0");
    string line;

    while(true)
    {
        getline(myfile, line);
        cout << line << endl;

        int timer = 0;
        while(timer < 100000000)
        {
            timer++;
        }
    }
    
    return 0;
}
