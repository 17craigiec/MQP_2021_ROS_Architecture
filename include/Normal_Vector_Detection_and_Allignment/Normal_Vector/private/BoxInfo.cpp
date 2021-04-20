#include "../public/BoxInfo.h"

using namespace std;

bool isLineValid(string line)
{
    bool is_valid = true;
    string valid_chars = "1234567890 (,)";

    for(int i = 0; i < line.length(); i++)
    {
        is_valid = is_valid && (valid_chars.find(line[i]) != string::npos);
    }
    
    return is_valid;
}

BoxInfo readBoxInfo(string line)
{
    // objects constructed for use in for loop
    BoxInfo box_info;
    string nums = "0123456789";
    int num_comma = 0;
    string curr_num = "";


    // if(~isLineValid(line))
    // {
    //     cout << "Invalid Line Detected...  \"" << line << "\"" << endl;
    //     return box_info;
    // }

    for(int i = 0; i < line.length(); i++)
    {
        if(line[i] == ',' || line[i] == ')')
        {
            try
            {
                 switch(num_comma)
                {
                    case 0:
                        box_info.x = stoi(curr_num);
                        break;
                    case 1:
                        box_info.y = stoi(curr_num);
                        break;
                    case 2:
                        box_info.width = stoi(curr_num);
                        break;
                    case 3:
                        box_info.height = stoi(curr_num);
                        break;
                    
                    default:
                        cout << "ERROR: failure in readBoxInfo switch case    Input Line: \"" << line << "\"    num_comma: " << num_comma << endl;
                        break;
                }
            }
            catch(const exception& e)
            {
                cerr << e.what() << '\n';
            }
            
            curr_num = "";
            num_comma++;
        }
        
        if(nums.find(line[i]) != string::npos)
        {
            // A number has been found
            curr_num.push_back(line[i]);
        }
        
    }

    // Optional Print
    // coutBoxInfo(box_info);
    return box_info;
    
}

void coutBoxInfo(BoxInfo bi)
{
    cout << "X:" << bi.x << "  Y:" << bi.y << "  H:" << bi.height << "  W:" << bi.width << endl << endl;
}

bool isBoxValid(BoxInfo bi)
{
    return (bi.height != -1 && bi.width != -1 && bi.x != -1 && bi.y != -1);
}