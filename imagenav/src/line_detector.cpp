#include "imagenav/line_detector.hpp"

#include "cmath"

namespace imagenav{

LineDetector::LineDetector()
{
}

cv::Mat LineDetector::detectLine(const cv::Mat& cv_img)
{
    cv::Mat bev_img, hsl_img, img, edges;
    std::vector<cv::Mat> channel;

    // 鳥瞰図変換<-パラメータチューニングがすごく難しい
    bev_img = toBEV(cv_img);

    cv::cvtColor(cv_img, hsl_img, cv::COLOR_BGR2HLS);
    cv::split(hsl_img, channel);
    img = channel[1];
    cv::GaussianBlur(img, img, cv::Size(5, 5), 0);
    cv::Mat grad_x, grad_y;
    cv::Mat abs_grad_x, abs_grad_y;

    // ソーベルフィルタ<-エッジの強調処理
    cv::Sobel(img, grad_x, CV_16S, 1, 0, 5);
    cv::convertScaleAbs(grad_x, abs_grad_x);
    cv::Sobel(img, grad_y, CV_16S, 0, 1, 5);
    cv::convertScaleAbs(grad_y, abs_grad_y);

    cv::addWeighted(abs_grad_x, 0.5, abs_grad_y, 0.5, 0, edges);

    return edges;
}

// 白線の開始位置（x座標）を推定<-あとで複数レーン検出できるように拡張
std::vector<int> LineDetector::estimateLinePosition(const cv::Mat& img)
{
    int height = img.rows;
    int width = img.cols;
    const int threshold = 50;

    int estimate_height = height / 5;

    cv::Mat cat_img = img(cv::Rect(0, estimate_height, width, height - estimate_height));

    std::vector<int> white_pixel_counts(width, 0);
    for(int y = 0; y < cat_img.rows; ++y)
    {
        const uchar *pLine = img.ptr<uchar>(y);
        for(int x = 0; x < img.cols; ++x)
        {
            if(pLine[x] > 128)
                white_pixel_counts[x]++;
        }
    }

    std::vector<int> peaks;
    for (int x = 1; x < width - 1; ++x)
    {
        if (white_pixel_counts[x] > white_pixel_counts[x - 1] &&
            white_pixel_counts[x] > white_pixel_counts[x + 1] &&
            white_pixel_counts[x] > 10) // ピークの最小閾値
        {
            peaks.push_back(x);
        }
    }

    if(peaks.size() <= 1)
    {
        return prev_start_xs;
    }

    // ピークを降順にソート
    std::sort(peaks.begin(), peaks.end(), [&](int a, int b)
    {
        return white_pixel_counts[a] > white_pixel_counts[b];
    });

    std::vector<int> start_xs;
    if (peaks.size() > 0) start_xs.push_back(peaks[0]);
    if (peaks.size() > 1) start_xs.push_back(peaks[1]);

    // 以前の位置と比較して誤検出を防ぐ<-雑な処理になっているが, 厳密に計算するのであれば姿勢の情報を使うのが良さそう.
    for (size_t i = 0; i < start_xs.size(); ++i)
    {
        if (prev_start_xs[i] != -1 && abs(start_xs[i] - prev_start_xs[i]) > threshold && start_xs[i] <= img.cols && start_xs[i] >= 0)
        {
            start_xs[i] = prev_start_xs[i];
        }

    }

    // 現在の位置を記録
    prev_start_xs = start_xs;

    return start_xs;
}

// スライドウィンドウ法
std::vector<cv::Point> LineDetector::SlideWindowMethod(const cv::Mat& img, const int start_x)
{
    const int window_width = 50;
    const int window_height = 40;
    std::vector<cv::Point> window_position;
    std::vector<int> white_pixel_counts(window_width, 0);
    int estimate_x = start_x; // 値渡しだけど良いのかな<-あとでポインタに修正?
    int prev_estimate_x = start_x;
    int prev_delta_x = 0;

    for(int height = img.rows; height > window_height; height-=window_height)
    {
        std::fill(white_pixel_counts.begin(), white_pixel_counts.end(), 0);

        for(int y = height; y < img.rows + window_height && y < img.rows; ++y)
        {
            const uchar *pLine = img.ptr<uchar>(y);
            for(int x = estimate_x-(window_width/2); x < estimate_x+(window_width/2); ++x)
            {
                if(pLine[x] > 128)
                {
                    int relative_x = x - (estimate_x - (window_width/2));
                    white_pixel_counts[relative_x]++;
                }
            }
        }

        // ウィンドウ内の白ピクセル中央値を計算
        std::vector<int> white_pixel_indices;
        for (int x = 0; x < window_width; ++x)
        {
            if (white_pixel_counts[x] > 0)
            {
                for (int i = 0; i < white_pixel_counts[x]; ++i)
                {
                    white_pixel_indices.push_back(x);
                }
            }
        }

        int median_relative_x = 0;
        if (!white_pixel_indices.empty())
        {
            size_t n = white_pixel_indices.size() / 2;
            std::nth_element(white_pixel_indices.begin(), white_pixel_indices.begin() + n, white_pixel_indices.end());
            median_relative_x = white_pixel_indices[n];

            estimate_x = estimate_x - (window_width / 2) + median_relative_x;
        }else{
            estimate_x = prev_estimate_x + prev_delta_x;
        }

        prev_delta_x = estimate_x - prev_estimate_x;
        prev_estimate_x = estimate_x;

        // スライドウィンドウの中心座標<-window_position
        window_position.push_back(cv::Point{estimate_x, height - window_height/2});
    }

    return window_position;
}

cv::Mat LineDetector::toBEV(const cv::Mat& img)
{
    // 俯瞰画像の変換座標 480x300
    cv::Point2f src_points[4] = { {-400, 300}, {100, 175}, {380, 175}, {880, 300} };
    cv::Point2f dst_points[4] = { {0, 480}, {0, 0}, {480, 0}, {480, 480} };

    // cv::Point2f src_points[4] = { {0, 300}, {200, 160}, {280, 160}, {480, 300} };
    // cv::Point2f dst_points[4] = { {0, 480}, {0, 0}, {480, 0}, {480, 480} };

    cv::Mat trans_mat = cv::getPerspectiveTransform(src_points, dst_points);

    cv::warpPerspective(img, img, trans_mat, img.size());

    return img;
}

cv::Mat LineDetector::PointVisualizar(cv::Mat& img, const std::vector<cv::Point>& points)
{
    for(const auto& point : points)
    {
        cv::circle(img, point, 5, cv::Scalar(0, 0, 255), 3);
    }

    return img;
}

cv::Mat LineDetector::draw_lines(const cv::Mat& cv_img ,const std::vector<cv::Vec4i>& line)
{
    const int length = 100;
    const int thickness = 3;

    for(size_t i = 0; i < line.size(); i++)
    {
        if(line[i][0] != line[i][2])
        {
            double slope = (line[i][3] - line[i][1])/(line[i][2] - line[i][0]);
            // 傾きから角度を求める
            double theta = std::atan(slope);
            if(slope > 0)
            {
                if(line[i][2] > line[i][0])
                {
                    int x = line[i][2] + length*std::cos(theta);
                    int y = line[i][3] + length*std::sin(theta);
                    cv::line(cv_img, cv::Point(x, y), cv::Point(line[i][0], line[i][1]), cv::Scalar(0, 0, 255), thickness);
                }else
                {
                    int x = line[i][0] + length*std::cos(theta);
                    int y = line[i][1] + length*std::sin(theta);
                    cv::line(cv_img, cv::Point(x, y), cv::Point(line[i][2], line[i][3]), cv::Scalar(0, 0, 255), thickness);
                }
            }else{
                if(line[i][2] > line[i][0])
                {
                    int x = line[i][0] + length*std::cos(theta);
                    int y = line[i][1] + length*std::sin(theta);
                    cv::line(cv_img, cv::Point(x, y), cv::Point(line[i][2], line[i][3]), cv::Scalar(0, 0, 255), thickness);
                }else{
                    int x = line[i][2] + length*std::cos(theta);
                    int y = line[i][3] + length*std::sin(theta);
                    cv::line(cv_img, cv::Point(x, y), cv::Point(line[i][0], line[i][1]), cv::Scalar(0, 0, 255), thickness);
                }
            }
        }
    }
    return cv_img;
}

} // namespace