
#include <fstream>
#include <iostream>

int main(int argc, char const *argv[])
{
    std::ofstream csv_record("data.csv");
    // std::ofstream csv_record("src/my_ros_package/src/data.csv");

    if (!csv_record.is_open())
    {
        std::cout << "打开失败" << std::endl;
    }


////*******************写入数据到data.csv文件************************////
    for (long int i = 0; i < 10000; i++)
    {
        for (int j = 0; j < 10000; j++)
        {
        csv_record << i << "\t" << i + 1 << "\n";
        }
        

        csv_record <<"-------------------------" << "\n";
    }
////**************************************************************////
  
    csv_record.close();

    return 0;
}




