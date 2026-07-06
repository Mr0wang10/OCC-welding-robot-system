// weldwire_identfy.cpp : 焊丝末端中心像素坐标识别
// 从 input/ 读取图片(不带后缀), 识别区域 (620,150)-(700,250)
// 焊丝可能沿任意方向延伸, 输出标记图和坐标txt到 result/ 文件夹
#define NOMINMAX
#include <opencv2/opencv.hpp>
#include <iostream>
#include <string>
#include <vector>
#include <fstream>
#include <windows.h>

using namespace std;
using namespace cv;

static string trim(const string& s) {
    string r = s;
    while (!r.empty() && (r.back() == ' ' || r.back() == '\r' || r.back() == '\n' || r.back() == '\t')) r.pop_back();
    while (!r.empty() && (r.front() == ' ' || r.front() == '\r' || r.front() == '\n' || r.front() == '\t')) r.erase(r.begin());
    return r;
}

Point findWireTipInRoi(const Mat& roi) {
    if (roi.empty()) return Point(-1, -1);

    Mat gray;
    if (roi.channels() == 3) cvtColor(roi, gray, COLOR_BGR2GRAY);
    else gray = roi.clone();

    int w = gray.cols, h = gray.rows;

    // ---------------------------------------------------------------
    // 策略: 使用多次阈值 + 直线检测找焊丝
    // ---------------------------------------------------------------

    // 1. 低阈值提取暗色区域 (焊丝+喷嘴+阴影)
    Mat darkLow, darkHigh;
    threshold(gray, darkLow, 110, 255, THRESH_BINARY_INV);
    threshold(gray, darkHigh, 70, 255, THRESH_BINARY_INV);

    // 2. 用 Canny 提取边缘
    Mat edges;
    Canny(gray, edges, 30, 100);

    // 3. Hough 直线检测 — 在暗色区域 + 边缘上检测直线段
    //    (焊丝是细长线状结构, 在Canny边缘 + 暗色掩码中都会显现)
    vector<Vec4i> lines;
    HoughLinesP(edges, lines, 1, CV_PI / 180, 30, 20, 10);

    // 4. 筛选直线: 找 ROI 中最长的、穿过暗色区域的直线
    int bestLen = 0;
    Vec4i bestLine(-1, -1, -1, -1);

    for (const auto& line : lines) {
        int x1 = line[0], y1 = line[1], x2 = line[2], y2 = line[3];
        double len = sqrt((double)(x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));

        // 太短的线忽略
        if (len < 15) continue;

        // 检查该直线是否穿过暗色区域（焊丝是暗的）
        int midX = (x1 + x2) / 2, midY = (y1 + y2) / 2;
        midX = max(0, min(w - 1, midX));
        midY = max(0, min(h - 1, midY));

        // 线段的中间点应该是暗的
        int midVal = gray.at<uchar>(midY, midX);
        if (midVal > 110) continue; // 不是暗色, 跳过

        // 检查线上多个点是否都在暗色区域
        int darkPts = 0, totalCheck = 0;
        for (float t = 0; t <= 1; t += 0.1f) {
            int px = cvRound(x1 + t * (x2 - x1));
            int py = cvRound(y1 + t * (y2 - y1));
            px = max(0, min(w - 1, px));
            py = max(0, min(h - 1, py));
            totalCheck++;
            if (gray.at<uchar>(py, px) < 110) darkPts++;
        }
        if (darkPts < totalCheck * 0.6) continue; // 线上大多数点必须是暗的

        // 选择最长的有效线段
        if (len > bestLen) {
            bestLen = (int)len;
            bestLine = line;
        }
    }

    // 5. 如果 Hough 找到了焊丝, 取其最低点作为末端
    if (bestLen >= 15) {
        int x1 = bestLine[0], y1 = bestLine[1], x2 = bestLine[2], y2 = bestLine[3];
        int tipY = max(y1, y2);
        int tipX = (tipY == y1) ? x1 : x2;

        // 在末端附近细化定位
        int refineStart = max(0, tipY - 5);
        int refineEnd = min(h - 1, tipY);

        int sumX = 0, cnt = 0;
        for (int y = refineEnd; y >= refineStart; y--) {
            for (int x = max(0, tipX - 10); x <= min(w - 1, tipX + 10); x++) {
                if (gray.at<uchar>(y, x) < 110) {
                    sumX += x; cnt++;
                    break;
                }
            }
        }
        if (cnt > 0) tipX = sumX / cnt;

        return Point(tipX, tipY);
    }

    // ---------------------------------------------------------------
    // 备用方案: 纵向搜索近邻暗像素路径 (无Hough直线时)
    // 从喷嘴位置(ROI右上)开始, 沿暗像素路径追踪到最远端
    // ---------------------------------------------------------------
    // 找起点: ROI顶部水平居中
    Point start(-1, -1);
    // 从 ROI 顶部开始，在水平中央区域寻找焊丝起点
    for (int y = 0; y < h / 2; y++) {
        for (int x = cvRound(w * 0.3); x < cvRound(w * 0.7); x++) {
            if (gray.at<uchar>(y, x) < 110) {
                start = Point(x, y);
                break;
            }
        }
        if (start.x >= 0) break;
    }

    // 如果没找到起点, 从ROI最暗的行开始
    if (start.x < 0) {
        double minVal;
        Point minPt;
        minMaxLoc(gray, &minVal, nullptr, &minPt);
        start = minPt;
    }

    // 从起点沿暗像素路径向下追踪
    // 允许向左右各偏移 trackWidth 像素
    const int trackWidth = 30;
    int bestY = start.y;
    int bestX = start.x;
    int prevX = start.x;

    for (int y = start.y + 1; y < h; y++) {
        bool found = false;
        // 从上一行的X位置开始, 向左右扩展搜索
        for (int dx = -trackWidth; dx <= trackWidth; dx++) {
            int x = prevX + dx;
            if (x < 0 || x >= w) continue;
            if (gray.at<uchar>(y, x) < 110) {
                if (!found) {
                    bestX = x; bestY = y; found = true;
                    prevX = x;
                }
            }
        }
        if (!found) {
            // 连续5行找不到暗像素, 认为焊丝已结束
            if (y - bestY > 5) break;
        }
    }

    // 如果追踪到的路径太短, 回退到最下方的暗像素
    if (bestY - start.y < 10) {
        for (int y = h - 1; y >= 0; y--) {
            for (int x = 0; x < w; x++) {
                if (gray.at<uchar>(y, x) < 90) {
                    return Point(x, y);
                }
            }
        }
        return Point(-1, -1);
    }

    return Point(bestX, bestY);
}

int main(int argc, char* argv[]) {
    // 读取文件名
    string name;
    if (argc >= 2) name = trim(argv[1]);
    else { cout << "请输入图片文件名(不带后缀): "; getline(cin, name); name = trim(name); }

    // 加载图片
    string inputDir = "input/";
    Mat src;
    string fullPath;
    for (const auto& ext : vector<string>{ ".png", ".jpg", ".jpeg", ".bmp", ".tiff" }) {
        fullPath = inputDir + name + ext;
        src = imread(fullPath);
        if (!src.empty()) break;
    }
    if (src.empty()) { cerr << "错误: 无法读取图片" << endl; return -1; }
    cout << "图片加载成功: " << fullPath << " (" << src.cols << "x" << src.rows << ")" << endl;

    // 识别区域: (620,150)-(700,250)
    Rect roiRect(620, 150, 80, 100);
    roiRect = roiRect & Rect(0, 0, src.cols, src.rows);
    Mat roi = src(roiRect).clone();
    cout << "识别区域: (" << roiRect.x << "," << roiRect.y << ")-("
        << (roiRect.x + roiRect.width) << "," << (roiRect.y + roiRect.height) << ")" << endl;

    // 检测
    Point tipLocal = findWireTipInRoi(roi);

    // 创建 result 文件夹
    CreateDirectoryA("result", NULL);

    Point wireTip;
    if (tipLocal.x >= 0 && tipLocal.y >= 0) {
        wireTip.x = tipLocal.x + roiRect.x;
        wireTip.y = tipLocal.y + roiRect.y;
        cout << "焊丝末端中心像素坐标: (" << wireTip.x << ", " << wireTip.y << ")" << endl;

        // ---- 保存标记图 ----
        Mat result = src.clone();
        rectangle(result, roiRect, Scalar(255, 255, 0), 1);
        circle(result, wireTip, 3, Scalar(0, 0, 255), -1);
        drawMarker(result, wireTip, Scalar(0, 255, 0), MARKER_CROSS, 20, 1);
        putText(result, "(" + to_string(wireTip.x) + "," + to_string(wireTip.y) + ")",
            Point(wireTip.x + 10, wireTip.y - 10),
            FONT_HERSHEY_SIMPLEX, 0.5, Scalar(0, 255, 255), 2);

        string imgPath = "result/" + name + "_result.png";
        imwrite(imgPath, result);
        cout << "标记图: " << imgPath << endl;

        // ---- 保存坐标 txt ----
        string txtPath = "result/" + name + "_coord.txt";
        ofstream ofs(txtPath);
        if (ofs.is_open()) {
            ofs << wireTip.x << " " << wireTip.y << endl;
            ofs.close();
            cout << "坐标文件: " << txtPath << " (内容: " << wireTip.x << " " << wireTip.y << ")" << endl;
        }
    }
    else {
        cout << "未检测到焊丝末端" << endl;
        string txtPath = "result/" + name + "_coord.txt";
        ofstream ofs(txtPath);
        if (ofs.is_open()) {
            ofs << "-1 -1" << endl;
            ofs.close();
        }
    }

    return 0;
}
