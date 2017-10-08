//
// Created by nikitas on 27.03.16.
//

#ifndef NAOMECH_COREFUNTIONS_H
#define NAOMECH_COREFUNTIONS_H

#include "opencv2/core/core.hpp"
//#include <flatbuffers/flatbuffers.h>

namespace ant {
    namespace vision_utils {
        
        struct CameraParameters {
            /*
             * Intrinsic calibration parameters (camera to image coordinates transform)
             */
            float f, a, s, cx, cy;
            
            /*
             * Extrinsic calibration parameters (world to camera coordinates transform) 
             * Rot is 3x3 rotation matrix
             * Trans is 3x1 translation vector
             */
            cv::Mat Rot, Trans; 
            
            /*
             * f = focal length (Vrep default is 2.0 (units?))
             * a = aspect ratio
             * s = skew
             * cx = offset x
             * cx = offset y
             */
            CameraParameters(float f, float a = 1.0f, float s = 0.0f, float cx = 0.0f, float cy = 0.0f)
            {
                SetIntrinsicParameters(f, a, s, cx, cy);
            }
            
            CameraParameters(cv::Mat& R, cv::Mat& t, float f, float a = 1.0f, float s = 0.0f, float cx = 0.0f, float cy = 0.0f)
            {
                SetExtrinsicParameters(R, t);
                SetIntrinsicParameters(f, a, s, cx, cy);
            }
            
            void SetIntrinsicParameters(float f, float a = 1.0f, float s = 0.0f, float cx = 0.0f, float cy = 0.0f)
            {
                this->f = f;
                this->a = a;
                this->s = s;
                this->cx = cx;
                this->cy = cy;
            }
            
            void SetExtrinsicParameters(cv::Mat& Rot, cv::Mat& Trans)
            {
                if (Rot.rows != 3 || Rot.rows != 3) {
                    throw std::runtime_error{"Rotation matrix must be 3x3"};
                }
                
                if (Trans.rows != 3 || Trans.cols != 1) {
                    throw std::runtime_error{"Translation vector must be 3x1"};
                }
                
                this->Rot = Rot.clone();
                this->Trans = Trans.clone();
            }
            
            /*
             * Returns 3x3 intrinsic calibration matrix 
             */
            cv::Mat GetIntCalibrationMatrix33() const
            {
                return (cv::Mat_<float>(3, 3) << f, s, cx, 0, a*f, cy, 0, 0, 1);
            } 
            
            /*
             * Returns 3x4 intrinsic calibration matrix 
             */
            cv::Mat GetIntCalibrationMatrix34() const
            {
                return (cv::Mat_<float>(3, 4) << f, s, cx, 0, 0, a*f, cy, 0, 0, 0, 1, 0);
            }
            
            /*
             * Returns 3x4 extrinsic calibration matrix
             */
            cv::Mat GetExtCalibrationMatrix34() const
            {
                cv::Mat T;
                cv::hconcat(Rot, Trans, T);
                return T;
            }
            
            /*
             * Returns 4x4 extrinsic calibration matrix
             */
            cv::Mat GetExtCalibrationMatrix44() const
            {
                cv::Mat T, pad;
                pad = (cv::Mat_<float>(1, 4) << 0, 0, 0, 1);
                cv::hconcat(Rot, Trans, T);
                cv::vconcat(T, pad, T);
                return T;
            }
        };

        template<class _Tp, int m, int n>
        inline
        float norm(const cv::Matx<_Tp, m, n> &M) {
            float sum = 0.0f;
            for (int i = 0; i < m; i++) {
                for (int j = 0; j < n; j++) {
                    sum += M(i, j) * M(i, j);
                }
            }
            return std::sqrt(sum);
        }


        template<class _Tp>
        inline
        cv::Vec<_Tp, 2> getVector(const cv::Vec4i &segment) {
            return cv::Vec<_Tp, 2>(segment(2) - segment(0), segment(3) - segment(1));
        }


        inline float getAngle(const cv::Vec2f &vec1, const cv::Vec2f &vec2) {
            const float cosAlpha = vec1.dot(vec2) / (vision_utils::norm(vec1) * vision_utils::norm(vec2));
            return std::acos(cosAlpha);
        }


        inline float getAltitude(const cv::Point &a, const cv::Point &b, const cv::Point &c) {
            const float dividend = std::abs((b.y - c.y) * a.x + (c.x - b.x) * a.y +
                                            (b.x * c.y - c.x * b.y));
            const float sum = (c.x - b.x) * (c.x - b.x) + (b.y - c.y) * (b.y - c.y);
            const float divider = std::sqrt(sum);
            return dividend / divider;
        }

        inline
        bool cmp(const cv::Point &p1, const cv::Point &p2) {
            return cv::norm(p1) < cv::norm(p2);
        }


        inline
        void operator+=(cv::Vec4i &line1, const cv::Vec4i &line2) {
            using vision_utils::cmp;

            const cv::Point a(line1(0), line1(1)), b(line1(2), line1(3));
            const cv::Point c(line2(0), line2(1)), d(line2(2), line2(3));

            if (cv::norm(c - d) < 0.00001) {
                return;
            }
            if (cv::norm(a - b) < 0.00001) {
                line1 = line2;
                return;
            }

            const cv::Point min = std::min(a, std::min(b, std::min(c, d, cmp), cmp), cmp);
            const cv::Point max = std::max(a, std::max(b, std::max(c, d, cmp), cmp), cmp);

            line1 = cv::Vec4i(min.x, min.y, max.x, max.y);
            return;
        }

//        inline cv::Mat reconstructCvMat(const flatbuffers::Vector<uint8_t> *frame,
//                                        int cols,
//                                        int rows,
//                                        int type){
//            return cv::Mat(rows,cols,type,(void*)frame->Data());
//        }

        inline void rot90(cv::Mat img, int degrees)
        {
            // O(1) operations
            if (degrees == 90) {
                transpose(img, img);
                flip(img, img, 1); //transpose+flip(1)=CW
            }
            else if (degrees == 270) {
                transpose(img, img);
                flip(img, img, 0); //transpose+flip(0)=CCW
            }
            else if (degrees == 180) {
                flip(img, img, -1); //flip(-1)=180
            }
        }
    }
}

#endif //NAOMECH_COREFUNTIONS_H
