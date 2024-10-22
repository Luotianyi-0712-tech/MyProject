#include <iostream>
#include <vector>
#include <fstream>
#include <algorithm>
#include <string>
using namespace std;

class ImageMatrix {
private:
    vector<vector<int>> matrix;
    int width;
    int height;

public:
    // 构造函数
    ImageMatrix() : width(0), height(0) {}
    ImageMatrix(int w, int h) : width(w), height(h) {
        matrix = vector<vector<int>>(h, vector<int>(w, 0));
    }

    // 判断图像是否为空
    bool isEmpty() const {
        return width == 0 || height == 0;
    }

    // 从文件读取图像
    void readFromFile(const string& filename) {
        ifstream file(filename);
        if (!file.is_open()) {
            cerr << "无法打开文件: " << filename << endl;
            return;
        }

        matrix.clear();
        string line;
        while (getline(file, line)) {
            vector<int> row;
            int value;
            size_t pos = 0;
            while ((pos = line.find(' ')) != string::npos) {
                value = stoi(line.substr(0, pos));
                row.push_back(value);
                line.erase(0, pos + 1);
            }
            value = stoi(line);
            row.push_back(value);
            matrix.push_back(row);
        }

        height = matrix.size();
        width = matrix[0].size();

        file.close();
    }

    // 保存图像到文件
    void saveToFile(const string& filename) const {
        ofstream file(filename);
        if (!file.is_open()) {
            cerr << "无法打开文件: " << filename << endl;
            return;
        }

        for (const auto& row : matrix) {
            for (int value : row) {
                file << value << " ";
            }
            file << endl;
        }

        file.close();
    }

    // 显示图像
    void display() const {
        for (const auto& row : matrix) {
            for (int value : row) {
                cout << value << " ";
            }
            cout << endl;
        }
    }

    // 二值化显示
    void displayBinary() const {
        for (const auto& row : matrix) {
            for (int value : row) {
                cout << (value > 0 ? "1 " : "0 ");
            }
            cout << endl;
        }
    }

    // 修改像素值
    void setPixel(int x, int y, int value) {
        if (x >= 0 && x < width && y >= 0 && y < height) {
            matrix[y][x] = value;
        }
    }

    // 绘制矩形
    void drawRectangle(int x, int y, int w, int h, int value) {
        for (int i = x; i < x + w; ++i) {
            setPixel(i, y, value);
            setPixel(i, y + h - 1, value);
        }
        for (int j = y; j < y + h; ++j) {
            setPixel(x, j, value);
            setPixel(x + w - 1, j, value);
        }
    }

    // 阈值化
    void threshold(int thr) {
        for (auto& row : matrix) {
            for (int& value : row) {
                if (value <= thr) {
                    value = 0;
                }
            }
        }
    }

    // 水平翻转
    void flipHorizontal() {
        for (auto& row : matrix) {
            reverse(row.begin(), row.end());
        }
    }

    // 顺时针旋转90度
    void rotate90() {
        vector<std::vector<int>> rotated(width, vector<int>(height));
        for (int i = 0; i < height; ++i) {
            for (int j = 0; j < width; ++j) {
                rotated[j][height - 1 - i] = matrix[i][j];
            }
        }
        matrix = rotated;
        swap(width, height);
    }

    // 垂直翻转
    void flipVertical() {
        reverse(matrix.begin(), matrix.end());
    }
};

int main() {
    // 任务1
    ImageMatrix img(5, 7);
    for (int i = 0; i < 5; ++i) {
        for (int j = 0; j < 7; ++j) {
            if ((i == 0 || i == 4 || j == 0 || j == 6) && !(i == 0 && j == 0)) {
                img.setPixel(i, j, 255);
            }
        }
    }
    cout << "任务1 创建的5x7图像：" << endl;
    img.display();

    // 任务2
    img.flipVertical();
    cout << "\n任务2 垂直翻转后的图像：" << endl;
    img.display();
    //通过把文件写入代码中，创造性的解决了文件打不开的问题
    ofstream outFile("complex_image.txt");
    if (outFile.is_open()) {
        outFile << "33 23 32 93 33 77 34 35 42 11 15 16 20\n"
                << "21 138 0 12 43 83 81 73 68 52 51 115 49\n"
                << "25 243 155 132 142 137 140 130 211 205 102 121 0\n"
                << "64 232 54 76 47 42 140 89 67 63 37 187 6\n"
                << "19 111 57 43 34 74 109 171 45 33 32 45 8\n"
                << "32 152 56 42 52 71 255 95 198 243 41 11 91\n"
                << "76 15 171 165 121 158 45 19 0 51 153 134 90\n"
                << "88 1 74 11 89 37 34 25 8 2 77 191 5\n"
                << "54 6 90 15 84 32 1 9 86 55 42 8 91\n";
        outFile.close();
        cout << "成功创建complex_image.txt文件" << endl;
    } else {
        cerr << "无法创建complex_image.txt文件" << endl;
        return 1;
    }

    ImageMatrix complex;
    complex.readFromFile("complex_image.txt");
    cout << "\n任务3  原始复杂图像：" << endl;
    complex.display();

    complex.threshold(100);
    complex.flipHorizontal();
    complex.rotate90();
    complex.rotate90();
    complex.rotate90();

    cout << "\n任务3 - 处理后的复杂图像（二值化显示）：" << endl;
    complex.displayBinary();

    return 0;
}