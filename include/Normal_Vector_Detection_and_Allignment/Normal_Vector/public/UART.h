#include <iostream>
#include <fstream>
#include <string>
#include <thread>
#include <mutex>

using namespace std;


class UART
{
private:
    string m_current_line = "";
    ifstream m_file;
    mutex m_mutex;
    thread m_thread;

    void readNewLine();

public:
    UART(string file_name);
    ~UART();
    
    string returnCurrentLine();
    void join();
};
