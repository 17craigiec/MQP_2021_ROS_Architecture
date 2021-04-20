#include <string.h>
#include <iostream>
#include <fstream> 

using namespace std;

struct BoxInfo
{
    int x = -1;
    int y = -1;
    double height = -1;
    double width = -1;
};


BoxInfo readBoxInfo();

void coutBoxInfo(BoxInfo bi);

bool isBoxValid(BoxInfo bi);
