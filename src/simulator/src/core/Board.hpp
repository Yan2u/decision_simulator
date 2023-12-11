#ifndef SIMULATOR_BOARD_HPP
#define SIMULATOR_BOARD_HPP

#include <thread>
#include <vector>

#include "fmt/format.h"
#include "opencv2/opencv.hpp"

#include "Area.hpp"
#include "Robot.hpp"

namespace simulator {
class Board {
   private:
    const cv::Scalar AREA_COLORS[4] = {cv::Scalar(45, 45, 45),
                                       cv::Scalar(64, 164, 238),
                                       cv::Scalar(218, 136, 118),
                                       cv::Scalar(125, 197, 116)};

    const cv::Scalar CAMP_FOREGROUND_COLORS[2] = {cv::Scalar(255, 0, 0), cv::Scalar(0, 0, 255)};

    const cv::Scalar FOREGROUND_COLOR = cv::Scalar(0, 0, 0);

    const cv::Scalar BACKGROUND_COLOR = cv::Scalar(255, 255, 255);

    const std::vector<Robot>* m_robots_ptr;
    const std::vector<Area>* m_areas_ptr;
    int m_size_n, m_size_m;
    int m_width, m_height;

    cv::Mat m_canvas;
    cv::Mat m_canvas_text;
    cv::Mat m_img_robot_0;
    cv::Mat m_img_robot_1;

    int rotate_image(const cv::Mat& src, cv::Mat& dst, const double angle, const int mode) {
        // mode = 0 ,Keep the original image size unchanged
        // mode = 1, Change the original image size to fit the rotated scale, padding with zero

        if (src.empty()) {
            std::cout << "Damn, the input image is empty!\n";
            return -1;
        }

        if (mode == 0) {
            cv::Point2f center((src.cols - 1) / 2.0, (src.rows - 1) / 2.0);
            cv::Mat rot = cv::getRotationMatrix2D(center, angle, 1.0);
            cv::warpAffine(src, dst, rot, src.size());  // the original size
        } else {
            double alpha = -angle * CV_PI / 180.0;  // convert angle to radian format

            cv::Point2f srcP[3];
            cv::Point2f dstP[3];
            srcP[0] = cv::Point2f(0, src.rows);
            srcP[1] = cv::Point2f(src.cols, 0);
            srcP[2] = cv::Point2f(src.cols, src.rows);

            // rotate the pixels
            for (int i = 0; i < 3; i++)
                dstP[i] = cv::Point2f(
                    srcP[i].x * cos(alpha) - srcP[i].y * sin(alpha),
                    srcP[i].y * cos(alpha) + srcP[i].x * sin(alpha));
            double minx, miny, maxx, maxy;
            minx = std::min(std::min(std::min(dstP[0].x, dstP[1].x), dstP[2].x), float(0.0));
            miny = std::min(std::min(std::min(dstP[0].y, dstP[1].y), dstP[2].y), float(0.0));
            maxx = std::max(std::max(std::max(dstP[0].x, dstP[1].x), dstP[2].x), float(0.0));
            maxy = std::max(std::max(std::max(dstP[0].y, dstP[1].y), dstP[2].y), float(0.0));

            int w = maxx - minx;
            int h = maxy - miny;

            // translation
            for (int i = 0; i < 3; i++) {
                if (minx < 0)
                    dstP[i].x -= minx;
                if (miny < 0)
                    dstP[i].y -= miny;
            }

            cv::Mat warpMat = cv::getAffineTransform(srcP, dstP);
            cv::warpAffine(
                src,
                dst,
                warpMat,
                cv::Size(w, h),
                1,
                0,
                BACKGROUND_COLOR);  // extend size

        }  // end else

        return 0;
    }

    void draw_image_on(cv::Mat& src, cv::Mat& dst, cv::Rect position) {
        cv::Mat roi = src(position);
        cv::resize(dst, dst, position.size());
        cv::Mat mask;
        cv::cvtColor(dst, mask, cv::COLOR_BGR2GRAY);
        assert(mask.channels() == 1);
        dst.copyTo(roi, mask);
    }

    cv::Rect2d get_unit_rect(int idx, int idy, double padding = 0.0) {
        return cv::Rect2d(
            cv::Point2d(
                (double)m_width / m_size_m * idy + padding,
                (double)m_height / m_size_n * idx + padding),
            cv::Point2d(
                (double)m_width / m_size_m * (idy + 1) - padding,
                (double)m_height / m_size_n * (idx + 1) - padding));
    }

    void draw_grid() {
        m_canvas = cv::Mat(cv::Size(m_width, m_height), CV_8UC3, BACKGROUND_COLOR);

        // draw outmost border
        cv::rectangle(
            m_canvas,
            cv::Rect2d(cv::Point2d(0.0, 0.0), cv::Size(m_width, m_height)),
            FOREGROUND_COLOR,
            1);

        // draw splitter
        for (int i = 1; i < m_size_n; ++i) {
            cv::line(
                m_canvas,
                cv::Point2d(0.0, (double)m_height / m_size_n * i),
                cv::Point2d(m_width, (double)m_height / m_size_n * i),
                FOREGROUND_COLOR,
                1);
        }
        for (int i = 1; i < m_size_m; ++i) {
            cv::line(
                m_canvas,
                cv::Point2d((double)m_width / m_size_m * i, 0.0),
                cv::Point2d((double)m_width / m_size_m * i, m_height),
                FOREGROUND_COLOR,
                1);
        }

        // draw areas
        if (m_areas_ptr) {
            int n_areas = m_areas_ptr->size();
            for (int i = 0; i < n_areas; ++i) {
                auto&& area = (*m_areas_ptr)[i];
                auto&& area_color = AREA_COLORS[(int)area.type];

                for (int j = area.x1; j <= area.x2; ++j) {
                    for (int k = area.y1; k <= area.y2; ++k) {
                        cv::rectangle(m_canvas, get_unit_rect(j, k, 1.0), area_color, -1);
                    }
                }
            }
        }

        // draw robots
        int n_robots = m_robots_ptr->size();
        for (int i = 0; i < n_robots; ++i) {
            cv::Mat robot_img_rotate;
            auto&& robot = (*m_robots_ptr)[i];

            cv::Mat& target_img = robot.camp == 0 ? m_img_robot_0 : m_img_robot_1;
            if (rotate_image(target_img, robot_img_rotate, 45 * robot.direction - 90, 0) == -1) {
                robot_img_rotate = target_img.clone();
                fmt::println("failed to rotate!");
            }

            cv::Rect rect = get_unit_rect(robot.pos_x, robot.pos_y, 5.0);
            draw_image_on(m_canvas, robot_img_rotate, rect);
        }
    }

    void draw_status() {
        int n_robots = m_robots_ptr->size();
        std::vector<cv::Mat> texts;
        for (int i = 0; i < n_robots; ++i) {
            auto&& robot = (*m_robots_ptr)[i];
            cv::Mat text(cv::Size(600, 50), CV_8UC3, BACKGROUND_COLOR);
            cv::putText(
                text,
                fmt::format(
                    "Robot({}) - ATK: {:.2f}, DEF: {:.2f}, POS: ({}, {})",
                    robot.id,
                    robot.attack,
                    robot.defense,
                    robot.pos_x,
                    robot.pos_y),
                cv::Point(5, 25),
                cv::FONT_HERSHEY_PLAIN,
                1.0,
                FOREGROUND_COLOR,
                1);
            int baseline;
            cv::Size text_hp_size =
                cv::getTextSize("HP: ", cv::FONT_HERSHEY_PLAIN, 1.0, 2, &baseline);
            cv::putText(
                text,
                "HP: ",
                cv::Point(5, 40),
                cv::FONT_HERSHEY_PLAIN,
                1.0,
                FOREGROUND_COLOR,
                1);

            cv::Rect hp_bar_rect =
                cv::Rect(cv::Point(10 + text_hp_size.width, 30), cv::Point(595, 45));
            cv::rectangle(text, hp_bar_rect, FOREGROUND_COLOR, 1);

            int end_x = 11 + text_hp_size.width + (double)(594 - (11 + text_hp_size.width)) * (std::max(robot.health, 0.0) / robot.base_health);

            cv::rectangle(
                text,
                cv::Rect(cv::Point(11 + text_hp_size.width, 31), cv::Point(end_x, 44)),
                CAMP_FOREGROUND_COLORS[robot.camp],
                -1);

            text_hp_size = cv::getTextSize(
                fmt::format("{:.2f} / {:.2f}", robot.health, robot.base_health),
                cv::FONT_HERSHEY_PLAIN,
                1.0,
                2,
                &baseline);

            cv::putText(
                text,
                fmt::format("{:.2f} / {:.2f}", robot.health, robot.base_health),
                cv::Point(
                    hp_bar_rect.x + 0.5 * hp_bar_rect.width - 0.5 * text_hp_size.width,
                    hp_bar_rect.y + 0.5 * hp_bar_rect.height + 0.5 * text_hp_size.height),
                cv::FONT_HERSHEY_PLAIN,
                1.0,
                FOREGROUND_COLOR,
                1);
            texts.push_back(text);
        }

        cv::vconcat(texts, m_canvas_text);
    }

   public:
    void show() {
        draw_grid();
        cv::imshow("Board", m_canvas);
        cv::waitKey(5);

        draw_status();
        cv::imshow("Status", m_canvas_text);
        cv::waitKey(5);
    }

    simulator::Board& operator=(const simulator::Board& board_copy) {
        this->m_size_n = board_copy.m_size_n;
        this->m_size_m = board_copy.m_size_m;
        this->m_robots_ptr = board_copy.m_robots_ptr;
        this->m_areas_ptr = board_copy.m_areas_ptr;
        this->m_width = board_copy.m_width;
        this->m_height = board_copy.m_height;
        return *this;
    }

    Board()
        : Board(1, 1, 0, 0, nullptr, nullptr) {}

    Board(int n, int m, int width, int height)
        : Board(n, m, width, height, nullptr, nullptr) {}

    Board(
        int n,
        int m,
        int width,
        int height,
        const std::vector<Robot>* robots_ptr,
        const std::vector<Area>* areas_ptr)
        : m_robots_ptr(robots_ptr),
          m_areas_ptr(areas_ptr),
          m_size_n(n),
          m_size_m(m),
          m_width(width),
          m_height(height) {
        // load robot images from /assets
        m_img_robot_0 =
            cv::imread("/home/yan2u/learn_ros/decision_simulator/src/simulator/assets/robot_0.png");
        m_img_robot_1 =
            cv::imread("/home/yan2u/learn_ros/decision_simulator/src/simulator/assets/robot_1.png");
        assert(m_img_robot_0.channels() == 3);
        assert(m_img_robot_1.channels() == 3);
    }
};
};  // namespace simulator

#endif