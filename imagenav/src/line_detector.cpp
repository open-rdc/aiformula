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

std::vector<int> LineDetector::estimateLinePosition(const cv::Mat& img)
{
    const int window_height = 40;
    const int threshold = 10;

    int max_white_x = 0;
    int max_x = 0;
    std::vector<int> start_xs;

    for(int line=0; line < 2; line++)
    {
        std::vector<int> white_pixel_counts(window_width[line], 0);

        for(int y = img.rows - window_height; y < img.rows; ++y)
        {
            const uchar *pLine = img.ptr<uchar>(y);
            for(int x = 0; x < window_width[line]; ++x)
            {
                int screen_x = prev_start_xs[line] - window_width[line]/2 + x;

                if(pLine[screen_x] > 128)
                    white_pixel_counts[x]++;
            }
        }

        for(int x = 0; x < window_width[line]; ++x)
        {
            if(white_pixel_counts[x] > white_pixel_counts[x - 1])
            {
                max_white_x = prev_start_xs[line] - window_width[line]/2 + x;
                max_x = x;
            }
        }

        start_xs.push_back(max_white_x);

        if(start_xs[line] >= img.cols){
            start_xs[line] = 400;
        }else if(start_xs[line] <= 0){
            start_xs[line] = 100;
        }

        if(abs(prev_start_xs[line] - start_xs[line]) > 10)
        {
            start_xs[line] = prev_start_xs[line];
        }

        // 白ピクセルが規定量を下回ったらウィンドウ拡大
        if(white_pixel_counts[max_x] <= threshold){
            if(window_width[line] <= 200)
                window_width[line] += 3;
        }else{
            window_width[line] = 50;
        }

        prev_start_xs[line] = start_xs[line];

        // 検出が被ったらウィンドウを初期位置にリセット
        if(abs(start_xs[0] - start_xs[1]) <= 20){
            std::cerr << "window reset" << std::endl;
            start_xs = {100, 400};
        }
    }

    return start_xs;
}

// スライドウィンドウ法
std::vector<cv::Point> LineDetector::SlideWindowMethod(const cv::Mat& img, const int start_x, const int line)
{
    const int window_width = 100;
    const int window_height = 40;
    const int threshold = window_width/2;
    std::vector<cv::Point> window_position;
    std::vector<int> white_pixel_counts(window_width, 0);
    int estimate_x = start_x;
    int prev_delta_x = 0;

    for(int height = img.rows - 1; height > window_height; height-=window_height)
    {
        std::fill(white_pixel_counts.begin(), white_pixel_counts.end(), 0);

        for(int y = height; y > height - window_height; --y)
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
            if (white_pixel_counts[x] > 20)
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
            estimate_x = prev_window_position[line][window_position.size()].x + prev_delta_x;
        }

        // 検出したウィンドウが前回の位置と離れすぎていたら前回の位置を保持
        if(abs(estimate_x - prev_window_position[line][window_position.size()].x) >= threshold && prev_window_position[line][window_position.size()].x != 0)
        {
            estimate_x = prev_window_position[line][window_position.size()].x;
            // 一つ前のウィンドウと離れていたら前のウィンドウの位置を保持
            if(window_position.size() > 0 && abs(estimate_x - window_position[window_position.size()-1].x) > window_width/2)
            {
                estimate_x = window_position[window_position.size()-1].x;
            }
        }

        // 分岐路検出
        if(window_position.size() > 0 && abs(estimate_x - window_position[window_position.size()-1].x) > window_width/5)
        {
            if(JunctionDetectWindow(img, cv::Point{estimate_x, height}))
            {
                for(int y=height-window_height/2; y > window_height; y-=window_height)
                {
                    window_position.push_back(cv::Point{estimate_x, y});
                }
                
                return window_position;
            }
        }

        prev_delta_x = estimate_x - prev_window_position[line][window_position.size()].x;
        // スライドウィンドウの中心座標<-window_position
        window_position.push_back(cv::Point{estimate_x, height - window_height/2});
    }
    
    for(int i=0; i<window_position.size(); i++)
    {
        prev_window_position[line][i] = window_position[i];
    }

    return window_position;
}

bool LineDetector::JunctionDetectWindow(const cv::Mat& img, const cv::Point pos)
{
    const int window_width=100;
    const int window_height=40;
    const int threshold=10;
    std::vector<cv::Point> window_position;
    std::vector<int> white_pixel_counts(window_width, 0);

    cv::Mat visualize_img = img;
    for(int height=pos.y-window_height; height > window_height; height-=window_height)
    {
        std::fill(white_pixel_counts.begin(), white_pixel_counts.end(), 0);

        for(int y=height; y>height - window_height; y--)
        {
            const uchar *pLine = img.ptr<uchar>(y);
            for(int x=pos.x-(window_width/2); x<=pos.x+(window_width/2); x++)
            {
                if(pLine[x] > 128)
                {
                    int relative_x = x - (pos.x - (window_width/2));
                    white_pixel_counts[relative_x]++;
                }
            }
        }
        window_position.push_back(cv::Point{pos.x, height});
    }

    visualize_img = WindowVisualizar(visualize_img, window_position, 0, cv::Scalar(255, 0, 255));

    for(int i=0; i<white_pixel_counts.size(); i++)
    {
        if(white_pixel_counts[i] >= threshold)
            return true;
    }

    return false;
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

cv::Mat LineDetector::WindowVisualizar(cv::Mat& img, const std::vector<cv::Point>& points, const int line, const cv::Scalar color)
{
    const int BOX_SIZE_height = 40;

    for (const auto& point : points)
    {
        cv::Point top_left(point.x - window_width[line] / 2, point.y - BOX_SIZE_height / 2);
        cv::Point bottom_right(point.x + window_width[line] / 2, point.y + BOX_SIZE_height / 2);
  
        // 四角形の描画
        cv::rectangle(img, top_left, bottom_right, color, 2);
    }

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