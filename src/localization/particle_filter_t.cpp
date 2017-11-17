#include <cmath>
#include <chrono>
#include <random>
#include "localization/particle_filter_t.h"

using namespace drwn;

particle_filter_t::particle_filter_t()
{

}

void particle_filter_t::initialize()
{
    m_fieldWorld.enable_debug(m_debug);
    m_fieldWorld.initialize_field();

    float poseX = m_config.init_x;
    float poseY = m_config.init_y;
    float poseTheta = m_config.init_theta;
    int num_particles = m_config.num_particles;
    int random_particles = m_config.random_particles;

    //std::cout << poseX << " " << poseY << " " << poseTheta << " " << num_particles << " " << random_particles << std::endl;
    
    if (random_particles > 0) {
        float min_x, min_y, min_theta, max_x, max_y, max_theta;
        min_x = m_config.min_x;
        min_y = m_config.min_y;
        min_theta = m_config.min_theta;
        max_x = m_config.max_x;
        max_y = m_config.max_y;
        max_theta = m_config.max_theta;
        
        init_particles(min_x, max_x, min_y, max_y, min_theta, max_theta, num_particles);
    } else {
        init_particles(drwn::pose2d_t(poseX, poseY, poseTheta), num_particles);
    }
}

void particle_filter_t::predict(const Eigen::Vector3f& command, const Eigen::Vector3f& noise)
{
    assert(m_particle.size() != 0);

    for (auto& particle : m_particles) {
        particle.pose = odometry_sample(particle.pose, command, noise);
    }
}

std::tuple<field_map_t::line_type_t, point2d_t> particle_filter_t::calc_expected_measurement(float rx, float ry, float rtheta, float measured_range, float measured_bearing)
{
    // Predicted measurement
    // Cast a line from robot position with same bearing
    float epx, epy;
    // Need to normalize angles everywhere
    pose2d_t normalizer(0.0, 0.0, measured_bearing + rtheta);

    epx = rx + field_map_t::MAX_DIST * cos(normalizer.theta());
    epy = ry + field_map_t::MAX_DIST * sin(normalizer.theta());
    
    // Accept intersections with distance from robot to intersection point greater than minDist 
    return m_fieldWorld.intersect_with_field(
            line_t(rx, ry, epx, epy), measured_range * 0.7
    );
}

void particle_filter_t::correct(const measurement_bundle& measurements, const Eigen::Vector3f& noise)
{
    assert(m_particles.size() != 0);

    float weight_normalizer = 0.0f;
    for (auto& particle : m_particles) {
        if (measurements.size() < 1) {
            weight_normalizer = 1.0f;
            break;
        }
        
        float rx, ry, rtheta, vrange, vbearing;
        rx = particle.pose.x(); ry = particle.pose.y(); rtheta = particle.pose.theta();
        
        vrange = noise(0); vbearing = noise(1);
        Eigen::MatrixXf Qt(4, 4);
        Qt << vrange, 0.0f, 0.0f, 0.0f,
              0.0f, vbearing, 0.0f, 0.0f,
              0.0f, 0.0f, vrange, 0.0f,
              0.0f, 0.0f, 0.0f, vbearing;
        
        pose2d_t normalizer;
        float range1, bearing1, range2, bearing2;
        float dx1, dy1, dx2, dy2;
        
        float new_weight = particle.weight;
        for (const auto& reading : measurements) {
            // Received measurement range bearing to 2 line points
            // Format: range1, bearing1, range2, bearing2
            range1 = reading(0); bearing1 = reading(1);
            range2 = reading(2); bearing2 = reading(3);
            
            auto test_line1 = calc_expected_measurement(rx, ry, rtheta, range1, bearing1);
            field_map_t::line_type_t ret_type1 = std::get<0>(test_line1);
            point2d_t isec_point1 = std::get<1>(test_line1);
            
            auto test_line2 = calc_expected_measurement(rx, ry, rtheta, range2, bearing2);
            field_map_t::line_type_t ret_type2 = std::get<0>(test_line2);
            point2d_t isec_point2 = std::get<1>(test_line2);
            
            if (ret_type1 == field_map_t::line_type_t::NONE || ret_type2 == field_map_t::line_type_t::NONE) {
                continue;
            }
            
            dx1 = isec_point1.X - rx;
            dy1 = isec_point1.Y - ry;
            float expected_range1 = std::sqrt(dx1*dx1 + dy1*dy1);
            if (expected_range1 > field_map_t::MAX_DIST) {
                continue;
            }
            normalizer.set_theta(std::atan2(dy1, dx1) - rtheta);
            float expected_bearing1 = normalizer.theta();
            
            dx2 = isec_point2.X - rx;
            dy2 = isec_point2.Y - ry;
            float expected_range2 = std::sqrt(dx2*dx2 + dy2*dy2);
            if (expected_range2 > field_map_t::MAX_DIST) {
                continue;
            }
            normalizer.set_theta(std::atan2(dy2, dx2) - rtheta);
            float expected_bearing2 = normalizer.theta();
            
            float bdiff1, bdiff2;
            normalizer.set_theta(expected_bearing1 - bearing1);
            bdiff1 = normalizer.theta();
            normalizer.set_theta(expected_bearing2 - bearing2);
            bdiff2 = normalizer.theta();
            Eigen::Vector4f diff = {
                expected_range1 - range1,
                bdiff1,
                expected_range2 - range2,
                bdiff2,
            };
            
            float det = (2.0f * M_PI * Qt).determinant();
            float temp = (diff.transpose() * Qt.inverse() * diff)(0);
            new_weight *= det * std::exp((-1.0f / 2.0f) * temp);
            
            // Penalize the particle if isec points are not on the same line
            if (ret_type1 != ret_type2) {
                new_weight = new_weight * 0.1; 
            }
                
            
            if (std::isnan(new_weight)) {
                new_weight = 0.0f;
            }
        }
        
        weight_normalizer = weight_normalizer + new_weight;
        particle.weight = new_weight;
    }
    
    // While debugging
    if (weight_normalizer < 0.00001) {
        std::cout << "!!! weight normalizer is too close to 0 !!!" << std::endl;
    }
    
    float highestWeight = 0.0f;
    std::size_t highestWeightIndex = 0;
    for (std::size_t index = 0; index < m_particles.size(); index++) {
        particle_t& particle = m_particles[index];
        
        if (fabs(weight_normalizer) < 0.00001 || std::isnan(weight_normalizer) || std::isnan(particle.weight)) {
            // Reset all weights
            particle.weight = 1.0f / m_particles.size();
        } else {
            particle.weight = particle.weight / weight_normalizer;
            
        }
        
        if (particle.weight > highestWeight) {
            highestWeight = particle.weight;
            highestWeightIndex = index;
        }
    }
    m_topParticleIndex = highestWeightIndex;
}

void particle_filter_t::resample()
{
    low_variance_resampling();
    calc_pose_mean_cov();
}

void particle_filter_t::low_variance_resampling()
{
    std::vector<particle_t> new_particles;
    
    float Jinv = 1.0f / m_particles.size();
    
    std::default_random_engine generator;
    std::uniform_real_distribution<float> dist(0.0, Jinv);
    float r = dist(generator);
    
    float c = m_particles[0].weight;
    int i = 0;
    particle_t new_particle;
    for (int j = 0; j < m_particles.size(); j++) {
       float U = r + (j-1) * Jinv; 
       while (U > c) {
           i++;
           c += m_particles[i].weight;
       }
       new_particle.weight = 1.0f / m_particles.size();
       new_particle.pose = m_particles[i].pose;
       new_particles.push_back(new_particle);
    }
    
    if (new_particles.size() != m_particles.size()) {
        throw std::length_error("Length error in low_variance_resampling");
    }
    
    m_particles.swap(new_particles);
}

Eigen::Vector4f particle_filter_t::get_line_range_bearing(pose2d_t robotPose, float x1, float y1, float x2, float y2)
{
    float rx, ry, rtheta;

    rx = robotPose.x();
    ry = robotPose.y();
    rtheta = robotPose.theta();

    float gx1, gy1, gx2, gy2;
    gx1 = (x1-rx)*cos(rtheta) + (y1-ry)*sin(rtheta);
    gy1 = -(x1-rx)*sin(rtheta) + (y1-ry)*cos(rtheta);
    gx2 = (x2-rx)*cos(rtheta) + (y2-ry)*sin(rtheta);
    gy2 = -(x2-rx)*sin(rtheta) + (y2-ry)*cos(rtheta);

    // Convert to range-bearing
    // format is (id, range, bearing)
    // id is unknown, so it's -1
    float dx1, dy1, dx2, dy2;
    float range1, range2, bearing1, bearing2;
    dx1 = gx1 - rx;
    dy1 = gy1 - ry;
    drwn::pose2d_t normalizer(0.0, 0.0, atan2(dy1, dx1) - rtheta);
    range1 = sqrt(dx1*dx1 + dy1*dy1);
    bearing1 = normalizer.theta();

    dx2 = gx2 - rx;
    dy2 = gy2 - ry;
    normalizer.set_theta(atan2(dy2, dx2) - rtheta);

    range2 = sqrt(dx2*dx2 + dy2*dy2);
    bearing2 = normalizer.theta();

    Eigen::Vector4f lineRangeBearing = {
            range1,
            bearing1,
            range2,
            bearing2
    };

    return lineRangeBearing;
}

void particle_filter_t::calc_pose_mean_cov()
{
    pose2d_t meanAccum;
    float pw, px, py, ptheta;
    for (const auto& particle : m_particles) {
        px = particle.pose.x();
        py = particle.pose.y();
        ptheta = particle.pose.theta();
        
        // For correct mean calculation
        pw = 1.0 / m_particles.size();
        meanAccum += pose2d_t(px*pw, py*pw, ptheta*pw);
    }
    
    pose2d_t devAccum, angleNormalizer;
    float mx, my, mtheta;
    mx = meanAccum.x();
    my = meanAccum.y();
    mtheta = meanAccum.theta();
    for (const auto& particle : m_particles) {
        px = particle.pose.x();
        py = particle.pose.y();
        ptheta = particle.pose.theta();
        
        // for correct stddev calculation
        // square root of mean of (xi-x.mean)**2 for all i
        pw = 1.0 / m_particles.size();
        angleNormalizer.set_theta(ptheta-mtheta);
        devAccum += pose2d_t(pw*pow(px-mx, 2), pw*pow(py-my, 2), pw*pow(angleNormalizer.theta(), 2));
    }
    // sqrt(mean((xi - x.mean())**2))
    devAccum.set_x(sqrt(devAccum.x()));
    devAccum.set_y(sqrt(devAccum.y()));
    devAccum.set_theta(sqrt(devAccum.theta()));
    
    m_poseMean = meanAccum;
    m_poseDev = devAccum;
    
    // DEBUG
    // Sometimes reset particles around current mean
    pose2d_t nmz;
    if (m_poseDev.x() > 200.0f || m_poseDev.y() > 200.0f || m_poseDev.theta() > 10.0f) {
        std::cout << "======================== REGENERATING PARTICLES ===========================" << std::endl;
        float min_x, min_y, min_theta, max_x, max_y, max_theta;
        min_x = m_poseMean.x() - m_poseDev.x();
        max_x = m_poseMean.x() + m_poseDev.x();
        min_y = m_poseMean.y() - m_poseDev.y();
        max_y = m_poseMean.y() + m_poseDev.y();
        nmz.set_theta(m_poseMean.theta() - (m_poseDev.theta() * (M_PI / 180.0f)));
        min_theta = nmz.theta();
        nmz.set_theta(m_poseMean.theta() + (m_poseDev.theta() * (M_PI / 180.0f)));
        max_theta = nmz.theta();
        init_particles(min_x, max_x, min_y, max_y, min_theta, max_theta, m_particles.size());
    }    
    
}


void particle_filter_t::init_particles(const pose2d_t& pose, int num_particles)
{
    float defaultWeight = 1.0f / num_particles;
    m_topParticleIndex = 0;
    
    m_particles.resize(num_particles);
    for (auto& particle : m_particles) {
        particle.pose = pose;
        particle.weight = defaultWeight;
    }
}

void particle_filter_t::init_particles(float min_x, float max_x, float min_y, float max_y, float min_theta, float max_theta, int num_particles)
{
    float defaultWeight = 1.0f / num_particles;
    m_topParticleIndex = 0;
    
    m_particles.resize(num_particles);
    
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::uniform_real_distribution<> dist_x(min_x, max_x);
    std::uniform_real_distribution<> dist_y(min_y, max_y);
    std::uniform_real_distribution<> dist_theta(min_theta, max_theta);
    
    for (auto& particle : m_particles) {
        particle.pose = pose2d_t(dist_x(generator), dist_y(generator), dist_theta(generator));
        particle.weight = defaultWeight;
    }
}

float particle_filter_t::sample_normal_distribution(float variance)
{
    unsigned int seed = std::chrono::system_clock::now().time_since_epoch().count();
    std::default_random_engine generator(seed);
    std::normal_distribution<float> normal(0.0f, variance);
    
    return normal(generator);
}

pose2d_t particle_filter_t::odometry_sample(pose2d_t pose, Eigen::Vector3f command, Eigen::Vector3f noise)
{
    float rot1, trans, rot2;
    rot1 = command(0); trans = command(1); rot2 = command(2);
    
    float r1_noise, t_noise, r2_noise;
    r1_noise = noise(0); t_noise = noise(1), r2_noise = noise(2);
    
    pose.normalize_theta();
    float theta_old = pose.theta();
    
    float rot1_h, trans_h, rot2_h;
    rot1_h = rot1 - sample_normal_distribution(r1_noise);
    trans_h = trans - sample_normal_distribution(t_noise);
    rot2_h = rot2 - sample_normal_distribution(r2_noise);
    
    pose2d_t newPose;
    newPose.set_theta(theta_old + rot1_h);
    float normalized = newPose.theta();
    
    newPose.set_x(trans_h * std::cos(normalized));
    newPose.set_y(trans_h * std::sin(normalized));
    newPose.set_theta(rot1_h + rot2_h);
    
    return pose + newPose;
}

Eigen::Vector3f particle_filter_t::get_odometry_command(pose2d_t prevPose, pose2d_t currPose)
{
    float rot1, trans, rot2;
    float dx, dy;
    dx = currPose.x() - prevPose.x();
    dy = currPose.y() - prevPose.y();
    
    trans = sqrt(dx*dx + dy*dy);
    pose2d_t normalizer(0.0f, 0.0f, atan2(dy, dx) - prevPose.theta());
    rot1 = normalizer.theta();
    normalizer.set_theta(currPose.theta() - prevPose.theta() - rot1);
    rot2 = normalizer.theta();
    
    return Eigen::Vector3f {
        rot1,
        trans,
        rot2
    };
}

bool particle_filter_t::is_debug_enabled() const {
    return m_debug;
}

void particle_filter_t::enable_debug(bool debug) {
    m_debug = debug;
    m_fieldWorld.enable_debug(debug);
}

