#include <iostream>
#include <string>

using namespace std;

int main() {
    int number;
    
    while (true) {
        cout << "请输入一个1-10之间的数字: ";
        cin >> number;
        
        if (number == 0) {
            cout << "程序结束。" << endl;
            break;
        }
        
        if (number < 1 || number > 10) {
            cout << "无效，请输入1-10之间的数字。" << endl;
            continue;
        }
        
        for (int i = 0; i < number; i++) {
            cout << "hello robomaster";
        }
        cout << endl;
    }
    
    return 0;
}