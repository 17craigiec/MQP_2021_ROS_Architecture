#include "../public/UART.h"

UART::UART(string file_name)
{
    m_file = ifstream("/dev/ttyAMA0");
    m_thread = thread(&UART::readNewLine, this);
}

UART::~UART()
{
    m_thread.join();
}

void UART::join()
{
    m_thread.join();
}

void UART::readNewLine()
{
    string line;

    while(true)
    {
        getline(m_file, line);
        // cout << line << endl;

        m_mutex.lock();
        m_current_line = line;
        m_mutex.unlock();
    }
}

string UART::returnCurrentLine()
{
    string tmp_line;

    m_mutex.lock();
    tmp_line = m_current_line;
    m_mutex.unlock();

    return m_current_line;
}