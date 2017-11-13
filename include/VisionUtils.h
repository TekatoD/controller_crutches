//
// Created by nikitas on 27.03.16.
//

#ifndef NAOMECH_COREFUNTIONS_H
#define NAOMECH_COREFUNTIONS_H

#include "opencv2/core/core.hpp"
//#include <flatbuffers/flatbuffers.h>

namespace ant {
    namespace vision_utils {
        
        struct camera_parameters_t {
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
            camera_parameters_t(float f, float a = 1.0f, float s = 0.0f, float cx = 0.0f, float cy = 0.0f)
            {
                set_intrinsic_parameters(f, a, s, cx, cy);
            }
            
            camera_parameters_t(cv::Mat& R, cv::Mat& t, float f, float a = 1.0f, float s = 0.0f, float cx = 0.0f, float cy = 0.0f)
            {
                set_extrinsic_parameters(R, t);
                set_intrinsic_parameters(f, a, s, cx, cy);
            }
            
            void set_intrinsic_parameters(float f, float a = 1.0f, float s = 0.0f, float cx = 0.0f, float cy = 0.0f)
            {
                this->f = f;
                this->a = a;
                this->s = s;
                this->cx = cx;
                this->cy = cy;
            }
            
            void set_extrinsic_parameters(cv::Mat &Rot, cv::Mat &Trans)
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
            cv::Mat get_intrinsic_calibration_matrix33() const
            {
                return (cv::Mat_<float>(3, 3) << f, s, cx, 0, a*f, cy, 0, 0, 1);
            } 
            
            /*
             * Returns 3x4 intrinsic calibration matrix 
             */
            cv::Mat get_intrinsic_calibration_matrix34() const
            {
                return (cv::Mat_<float>(3, 4) << f, s, cx, 0, 0, a*f, cy, 0, 0, 0, 1, 0);
            }
            
            /*
             * Returns 3x4 extrinsic calibration matrix
             */
            cv::Mat get_extrinsic_calibration_matrix34() const
            {
                cv::Mat T;
                cv::hconcat(Rot, Trans, T);
                return T;
            }
            
            /*
             * Returns 4x4 extrinsic calibration matrix
             */
            cv::Mat get_extrinsic_calibration_matrix44() const
            {
                cv::Mat T, pad;
                pad = (cv::Mat_<float>(1, 4) << 0, 0, 0, 1);
                cv::hconcat(Rot, Trans, T);
                cv::vconcat(T, pad, T);
                return T;
            }
        };
        
        class camera_projection_t {
        public:
            camera_projection_t(camera_parameters_t cameraParams) : m_cameraParams(cameraParams) {}
            
            void set_camera_parameters(camera_parameters_t cameraParams)
            {
                m_cameraParams = cameraParams;
            }
            
            /*
             * World coords to camera coords
             * Result is 3x1 vector in homogeneous coordinates 
             */
            cv::Mat world_to_image(float x, float y, float z)
            {
                return camera_to_image(world_to_camera(x, y, z));
            }
            
            cv::Mat world_to_image(cv::Mat WorldPoint)
            {
                return camera_to_image(world_to_camera(WorldPoint));
            }
            
            
            /*
             * Transform point in world coordinates to camera coordinates
             * Result is 4x1 vector in homogeneous coordinates
             */
            cv::Mat world_to_camera(float x, float y, float z)
            {
                cv::Mat WorldPoint = (cv::Mat_<float>(4, 1) << x, y, z, 1);
                return m_cameraParams.get_extrinsic_calibration_matrix44() * WorldPoint;
            }
            
            cv::Mat world_to_camera(cv::Mat WorldPoint)
            {
                return m_cameraParams.get_extrinsic_calibration_matrix44() * WorldPoint;
            }
            
            
            /*
             * Project a point in camera coordinates to camera projective plane (image coordinates)
             * Result is 3x1 vector in homogeneous coordinates
             */
            
            cv::Mat camera_to_image(float x, float y, float z)
             {
                cv::Mat CameraPoint = (cv::Mat_<float>(4, 1) << x, y, z, 1);
                return m_cameraParams.get_intrinsic_calibration_matrix34() * CameraPoint;
            }
            
            cv::Mat camera_to_image(cv::Mat CameraPoint)
            {
                return m_cameraParams.get_intrinsic_calibration_matrix34() * CameraPoint;
            }
            
            /*
             * Point on image to camera coordinates. Result is a ray in H.C.
             * Result is 3x1 vector in homogeneous coordinates (sx, sy, s)
             * (Ray from camera COP through the x, y point on the camera projective plane)
             */
            cv::Mat image_to_camera(float x, float y)
            {
                cv::Mat ImagePoint = (cv::Mat_<float>(3, 1) << x, y, 1);
                return m_cameraParams.get_intrinsic_calibration_matrix33().inv() * ImagePoint;
            }
            
            cv::Mat image_to_camera(cv::Mat ImagePoint)
            {
                return m_cameraParams.get_intrinsic_calibration_matrix33().inv() * ImagePoint;
            }
            
            /*
             * Transform a point in camera coordinates to world coordinates
             * Result is 4x1 vector of world coordinates in homogeneous coordinates (x, y, z, w)
             */
            cv::Mat camera_to_world(float x, float y, float z)
            {
                cv::Mat CameraPoint = (cv::Mat_<float>(4, 1) << x, y, z, 1);
                return m_cameraParams.get_extrinsic_calibration_matrix44().inv() * CameraPoint;
            }
            
            cv::Mat camera_to_world(cv::Mat CameraPoint)
            {
                return m_cameraParams.get_extrinsic_calibration_matrix44().inv() * CameraPoint;
            }
            
            cv::Mat image_to_image(cv::Mat ImagePoint)
            {
                cv::Mat H, Ext, Proj, r1, r2, t;
                
                Ext = m_cameraParams.get_extrinsic_calibration_matrix34();
                
                H = cv::Mat::zeros(3, 3, CV_32F);
                
                r1 = Ext(cv::Range::all(), cv::Range(0, 1));
                r2 = Ext(cv::Range::all(), cv::Range(1, 2));
                t = Ext(cv::Range::all(), cv::Range(3, 4));
                
                cv::hconcat(r1, r2, H);
                cv::hconcat(H, t, H);
                
                cv::Mat Ht = m_cameraParams.get_intrinsic_calibration_matrix33() * H;
                Ht /= t.at<float>(2, 0);
                
                Proj = Ht * ImagePoint;
                Proj /= Proj.at<float>(2, 0);
                
                return Proj;
            } 
            
            cv::Mat image_to_world_explicit(const cv::Mat &ImagePoint, const cv::Mat &R, const cv::Mat &t, float Width,
                                            float Height, float FovDeg) {
                // Same functionality as in CamGeom in naomech behaviour
                
                float f = m_cameraParams.f;
                float ix, iy;
                ix = ImagePoint.at<float>(0, 0);
                iy = ImagePoint.at<float>(1, 0);
                // Magic number: height in pixels
                // From top left origin (y downwards) to bottom left origin (y upwards)
                //iy = -iy + 240.0;
                
                //ix /= 320.0f;
                //iy /= 240.0f;
                
                float cx = Width / 2.0f;
                float cy = Height / 2.0f;
                float FovRad = FovDeg * (M_PI/180.0f);
                float focalDist = (0.5 * sqrt(Width*Width + Height*Height)) / tan(0.5 * FovRad);
                
                
                // Optical center x axis shift
                // line iy is (168)
                // 0.25 ix (0, 319); gx -1682, 1334
                // 0.30 ix (0, 319); gx -1641, 1440
                // 0.35 ix (0, 319); gx -1531, 1485
                // 0.375 ix (0, 319);gx -1493, 1523
                float shiftCoeff = 0.375;
                cx -= (Width / 2.0f) * shiftCoeff;
                
                //f = 239.53f;
                // Focal distance "magicNumber" = scaling number for correct value
                // Distance to line with y = 168 and x = (0, 319)
                // Pick correct distance to "left" point
                // treat "right-left" x coord (in world frame) difference as an error for sensors
                // 2.0 iy (168); lx 2094 rx 2626
                // 1.75 iy (168); lx 2412 rx 2944
                // 1.65 iy (168); lx 2620 rx 3164 - for penalty nearest horizontal penalty line
                // 1.45 for correct close measurements
                
                float magicNumber = 1.65f;
                focalDist = focalDist / magicNumber;
                //std::cout << "FocalDist: " << focalDist << std::endl;
                cv::Mat Pixel = (cv::Mat_<float>(3, 1) << focalDist, -ix + cx, -iy + cy);
                
                //std::cout << "Pixel: " << std::endl << Pixel << std::endl;
                
                
                Pixel = R * Pixel;
                Pixel *= (-t.at<float>(2, 0) / Pixel.at<float>(2, 0));
                Pixel += t;
                
                return Pixel;
            }
        private:
            camera_parameters_t m_cameraParams;
        };
        
        /*
         * Cross product matrix
         * instead of using cross product operator a x b
         * use matrix multiplication a_cross * b
         */
        inline cv::Mat xproduct_matrix33(float x, float y, float z)
        {
            return (cv::Mat_<float>(3, 3) << 0, -z, y, z, 0, -x, y, x, 0);
        }

        inline cv::Mat xproduct_matrix33(cv::Mat vec)
        {
            float x, y, z;
            
            x = vec.at<float>(0, 0);
            y = vec.at<float>(1, 0);
            z = vec.at<float>(2, 0);
            
            return (cv::Mat_<float>(3, 3) << 0, -z, y, z, 0, -x, y, x, 0);
        }
        

        /*
         * P1, P2 - points on the line
         * Returns line in Plucker coordinates (6 points)
         */
        inline cv::Mat plucker_line(cv::Mat P1, cv::Mat P2)
        {
            int rows = P1.rows;
            cv::Mat p1, p2;
            float d1, d2;
            
            p1 = P1(cv::Range(0, rows-1), cv::Range::all());
            d1 = P1.at<float>(rows-1, 0);
            p2 = P2(cv::Range(0, rows-1), cv::Range::all());
            d2 = P2.at<float>(rows-1, 0);
            
            cv::Mat PLine = cv::Mat_<float>(p1.rows * 2, 1);
            
            PLine(cv::Range(0, p1.rows), cv::Range::all()) = d1 * p2 - d2 * p1;
            PLine(cv::Range(p1.rows, PLine.rows), cv::Range::all()) = xproduct_matrix33(p1) * p2;
            
            return PLine;
        }
        
        /*
         * W - plane in H.C.
         * L - line in Plucker coordinates 
         *  https://math.stackexchange.com/questions/400268/equation-for-a-line-through-a-plane-in-homogeneous-coordinates
         */
        inline cv::Mat plane_ray_intersection(cv::Mat W, cv::Mat L)
        {
            // Plane parameters
            // Normal
            cv::Mat w = W(cv::Range(0, W.rows-1), cv::Range::all());
            // Distance
            float eps = W.at<float>(W.rows-1, 0);
            
            // line_t parameters
            cv::Mat l, m;
            l = L(cv::Range(0, L.rows/2), cv::Range::all());
            m = L(cv::Range(L.rows/2, L.rows), cv::Range::all());

            // meet operator
            cv::Mat meet, last;
            cv::hconcat((-eps) * cv::Mat::eye(3, 3, CV_32F), xproduct_matrix33(w), meet);
            cv::hconcat(w.t(), cv::Mat::zeros(1, 3, CV_32F), last);
            cv::vconcat(meet, last, meet);
            
            // Intersection point of plane and ray
            return meet * L;
        }
        
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
